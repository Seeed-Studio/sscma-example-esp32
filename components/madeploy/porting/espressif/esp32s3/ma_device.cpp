
#include <core/ma_compiler.h>
#include <core/ma_debug.h>

#include <porting/ma_device.h>

#include <cstring>


#include "ma_camera_esp32.h"
#include "ma_config_board.h"
#include "ma_storage_lfs.h"
#include "ma_transport_console.h"
#include "ma_transport_serial.h"
#include "ma_transport_i2c.h"
#include "ma_transport_spi.h"
#include "ma_transport_mqtt.h"
#include "ma_pm.h"

#include <esp_efuse.h>
#include <esp_efuse_chip.h>
#include <esp_efuse_table.h>
#include <esp_partition.h>
#include <hal/efuse_hal.h>
#include <spi_flash_mmap.h>
#include <driver/gpio.h>


extern "C" {

void ma_invoke_pre_hook(void*) {}

void ma_invoke_post_hook(void*) {}

uint8_t* _ma_static_tensor_arena;
}

static inline uint32_t _device_id_from_efuse() {
    char id_full[16]{};
    esp_err_t err = esp_efuse_read_field_blob(ESP_EFUSE_OPTIONAL_UNIQUE_ID, id_full, 16u << 3);

    if (err != ESP_OK) [[unlikely]]
        return 0ul;

    // Fowler–Noll–Vo hash function
    uint32_t hash  = 0x811c9dc5;
    uint32_t prime = 0x1000193;
    for (size_t i = 0; i < 16; ++i) {
        uint8_t value = id_full[i];
        hash          = hash ^ value;
        hash *= prime;
    }

    return hash;
}

namespace ma {

void __ma_device_background_task() {
    ma_trigger_pm_ctrl();
}

bool __ma_is_trigger_gpio(int gpio) {
    static int trigger_gpios[] = {1, 2, 3, 21, 41, 42};
    for (auto t : trigger_gpios) {
        if (t == gpio) {
            return true;
        }
    }  
    return false;
}

void __ma_init_gpio_level(int gpio, int level) {
    gpio_config_t io_conf;
    io_conf.intr_type    = GPIO_INTR_DISABLE;
    io_conf.mode         = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << gpio;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)gpio, level);
    MA_LOGV(MA_TAG, "Initializing GPIO %d to %d", gpio, level);
}

void __ma_set_gpio_level(int gpio, int level) {
    MA_LOGV(MA_TAG, "Setting GPIO %d to %d", gpio, level);
    gpio_set_level((gpio_num_t)gpio, level);
}


Device::Device() {
    MA_LOGD(MA_TAG, "Initializing device: %s", MA_BOARD_NAME);
    { 
        m_name = MA_BOARD_NAME; 
        ma_init_pm_ctrl();
    }

    MA_LOGD(MA_TAG, "Initializing device version");
    { m_version = "XIAO(ESP32S3)"; }

    MA_LOGD(MA_TAG, "Initializing device ID");
    { m_id = std::to_string((size_t)_device_id_from_efuse()); }

    MA_LOGD(MA_TAG, "Initializing storage");
    {
        m_storage = get_storage_instance();

        {
            MA_STORAGE_GET_POD(m_storage, "ma#bootcount", m_bootcount, 0);
            ++m_bootcount;
            MA_STORAGE_NOSTA_SET_POD(m_storage, "ma#bootcount", m_bootcount);
        }
    }

    MA_LOGD(MA_TAG, "Initializing transports");
    {
        static Console console;
        auto ret = console.init(nullptr);
        if (ret != MA_OK) {
            MA_LOGE(MA_TAG, "Failed to initialize console transport: %d", ret);
        }
        m_transports.push_back(&console);

        static Serial serial;
        ret = serial.init(nullptr);
        if (ret != MA_OK) {
            MA_LOGE(MA_TAG, "Failed to initialize serial transport: %d", ret);
        }
        m_transports.push_back(&serial);

        static I2C i2c;
        ret = i2c.init(nullptr);
        if (ret != MA_OK) {
            MA_LOGE(MA_TAG, "Failed to initialize i2c transport: %d", ret);
        }
        m_transports.push_back(&i2c);

        static SPI spi;
        ret = spi.init(nullptr);
        if (ret != MA_OK) {
            MA_LOGE(MA_TAG, "Failed to initialize spi transport: %d", ret);
        }
        m_transports.push_back(&spi);

        static MQTT mqtt;
        {
            ma_mqtt_topic_config_t config;
            using namespace std::string_literals;
            if (!MA_STORAGE_EXISTS(m_storage, MA_STORAGE_KEY_MQTT_PUB_TOPIC) || !MA_STORAGE_EXISTS(m_storage, MA_STORAGE_KEY_MQTT_SUB_TOPIC)) {
                std::string topic_prefix = "sscma/"s + MA_AT_API_VERSION + "/" + PRODUCT_NAME_PREFIX + "_"s + PRODUCT_NAME_SUFFIX + "_"s + m_id;
                std::string topic_pub    = topic_prefix + "/tx";
                std::string topic_sub    = topic_prefix + "/rx";
                MA_STORAGE_SET_STR(ret, m_storage, MA_STORAGE_KEY_MQTT_PUB_TOPIC, topic_pub);
                MA_STORAGE_SET_STR(ret, m_storage, MA_STORAGE_KEY_MQTT_SUB_TOPIC, topic_sub);
                MA_STORAGE_SET_RVPOD(ret, m_storage, MA_STORAGE_KEY_MQTT_PUB_QOS, 0);
                MA_STORAGE_SET_RVPOD(ret, m_storage, MA_STORAGE_KEY_MQTT_SUB_QOS, 0);
            }
        }
        ret = mqtt.init(nullptr);
        if (ret != MA_OK) {
            MA_LOGE(MA_TAG, "Failed to initialize mqtt transport: %d", ret);
        }
        m_transports.push_back(&mqtt);
    }

    MA_LOGD(MA_TAG, "Initializing sensors");
    {
        static CameraESP32 camera(0);
        m_sensors.push_back(&camera);
    }

    _ma_static_tensor_arena = (uint8_t*)heap_caps_malloc(MA_ENGINE_TFLITE_TENSOE_ARENA_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    MA_LOGD(MA_TAG, "Initializing models");
    {
        const esp_partition_t* partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_UNDEFINED, "models");
        if (partition) {
            static const uint8_t* mmap = nullptr;
            static uint32_t handler    = 0;
            esp_err_t ret              = spi_flash_mmap(partition->address, partition->size, SPI_FLASH_MMAP_DATA, reinterpret_cast<const void**>(&mmap), &handler);

            if (ret != ESP_OK) {
                MA_LOGE(MA_TAG, "Failed to map models partition");
                return;
            }

            const size_t size = partition->size;
            size_t id         = 0;
            for (size_t i = 0; i < size; i += 4096) {
                const void* data = mmap + i;
                if (ma_ntohl(*(static_cast<const uint32_t*>(data) + 1)) == 0x54464C33) {  // 'TFL3'
                    ma_model_t model;
                    model.id   = ++id;
                    model.type = MA_MODEL_TYPE_UNDEFINED;
                    model.name = mmap;
                    model.addr = data;
                    m_models.push_back(model);
                }
            }
        } else {
            MA_LOGE(MA_TAG, "Failed to find models, using default");
        }
    }
}

Device* Device::getInstance() {
    static Device device{};
    return &device;
}

Device::~Device() {
    // NVIC_SystemReset();
}

}  // namespace ma