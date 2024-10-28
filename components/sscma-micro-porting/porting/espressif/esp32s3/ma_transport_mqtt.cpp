#include <ma_config_board.h>

#include "core/ma_debug.h"
#include "ma_transport_mqtt.h"

#include <chrono>
#include <cstring>
#include <iostream>

#include "esp_event.h"
#include "esp_netif_sntp.h"
#include "esp_tls.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "mdns.h"
#include "mqtt_client.h"
#include "nvs_flash.h"

#include "core/utils/ma_ringbuffer.hpp"
#include "ma_storage_lfs.h"
#include "porting/ma_osal.h"

typedef enum {
    NETWORK_LOST = 0,
    NETWORK_IDLE,
    NETWORK_JOINED,
    NETWORK_CONNECTED,
} ma_net_sta_t;

volatile int _net_sta = NETWORK_LOST;

static esp_netif_t* _esp_netif               = nullptr;
static esp_mqtt_client_handle_t _mqtt_client = nullptr;

ma::Mutex _net_sync_mutex{};

static ma_mqtt_config_t _mqtt_server_config{};
static ma_mqtt_topic_config_t _mqtt_topic_config{};

in4_info_t _in4_info{};
in6_info_t _in6_info{};

static ma::SPSCRingBuffer<char>* _rb_rx = nullptr;

#define MDNS_ITEM_PROTOCAL       "protocol"
#define MDNS_ITEM_SERVER         "server"
#define MDNS_ITEM_PORT           "port"
#define MDNS_ITEM_DEST           "dest"
#define MDNS_ITEM_AUTH           "auth"
#define SSCMA_MDNS_HOST_NAME_LEN 64
#define SSCMA_MDNS_DEST_NAME_LEN 128
#define SSCMA_MDNS_SERV_NAME_LEN 128
#define SSCMA_MDNS_PORT          3141

typedef struct mdns_record {
    char host_name[SSCMA_MDNS_HOST_NAME_LEN];
    char server[SSCMA_MDNS_SERV_NAME_LEN];
    char destination[SSCMA_MDNS_DEST_NAME_LEN];
    char protocol[8];
    char authentication[8];
    uint16_t port;
} mdns_record_t;

static mdns_record_t _mdns_record{};

static void _emit_mdns() {
    if (_net_sta != NETWORK_CONNECTED) {
        return;
    }

    mdns_init();
    mdns_hostname_set(PRODUCT_NAME_PREFIX "_" PRODUCT_NAME_SUFFIX);
    mdns_instance_name_set(PRODUCT_NAME_PREFIX "_" PRODUCT_NAME_SUFFIX);

    static std::string port_str = std::to_string((int)_mdns_record.port);

    mdns_txt_item_t srv_data[] = {
        {MDNS_ITEM_PROTOCAL, _mdns_record.protocol},
        {MDNS_ITEM_SERVER, _mdns_record.server},
        {MDNS_ITEM_PORT, port_str.c_str()},
        {MDNS_ITEM_DEST, _mdns_record.destination},
        {MDNS_ITEM_AUTH, _mdns_record.authentication},
    };

    mdns_service_add(PRODUCT_NAME_PREFIX, "_sscma", "_tcp", SSCMA_MDNS_PORT, srv_data, 5);
}

static void _wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {

    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)data;
        MA_LOGD(MA_TAG, "joined to AP");

        {
            ma::Guard lock(_net_sync_mutex);
            memcpy(&_in4_info.ip.addr, &event->ip_info.ip, 4);
            memcpy(&_in4_info.netmask.addr, &event->ip_info.netmask, 4);
            memcpy(&_in4_info.gateway.addr, &event->ip_info.gw, 4);
        }

        _net_sta = NETWORK_JOINED;
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        MA_LOGD(MA_TAG, "disconnected from AP");
        _net_sta = NETWORK_IDLE;

        esp_wifi_clear_fast_connect();

        if (esp_wifi_connect() != ESP_OK) {
            MA_LOGE(MA_TAG, "failed to reconnect to AP");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

static void _mqtt_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)data;

    switch ((esp_mqtt_event_id_t)id) {
        case MQTT_EVENT_CONNECTED: {
            _net_sta = NETWORK_CONNECTED;

            do {
                int cnt = esp_mqtt_client_subscribe(_mqtt_client, _mqtt_topic_config.sub_topic, _mqtt_topic_config.sub_qos);
                if (cnt >= 0) {
                    break;
                }
                MA_LOGD(MA_TAG, "Failed to subscribe to topic %d", cnt);
                vTaskDelay(100 / portTICK_PERIOD_MS);
            } while (_net_sta == NETWORK_CONNECTED);

            {
                const char* discover_topic = "/ma/" MA_AT_API_VERSION "/discover";
                std::string payload        = "{\"pub_topic\":\"";
                payload += _mqtt_topic_config.pub_topic;
                payload += "\",\"pub_qos\":";
                payload += std::to_string((int)_mqtt_topic_config.pub_qos);
                payload += ",\"sub_topic\":\"";
                payload += _mqtt_topic_config.sub_topic;
                payload += "\",\"sub_qos\":";
                payload += std::to_string((int)_mqtt_topic_config.sub_qos);
                payload += ",\"client_id\":\"";
                payload += _mqtt_server_config.client_id;
                payload += "\"}";

                esp_mqtt_client_publish(_mqtt_client, discover_topic, payload.c_str(), payload.size(), _mqtt_topic_config.pub_qos, 0);
            }

            {
                std::strncpy(_mdns_record.host_name, _mqtt_server_config.client_id, sizeof _mdns_record.host_name - 1);
                std::strncpy(_mdns_record.server, _mqtt_server_config.host, sizeof _mdns_record.server - 1);
                std::strncpy(_mdns_record.authentication, std::strlen(_mqtt_server_config.username) ? "y" : "n", sizeof _mdns_record.protocol - 1);
                std::strncpy(_mdns_record.protocol, _mqtt_server_config.use_ssl ? "mqtts" : "mqtt", sizeof _mdns_record.authentication - 1);
                std::string topic = _mqtt_topic_config.sub_topic;
                {
                    auto pos = topic.find_last_of('/');
                    if (pos != std::string::npos) {
                        topic = topic.substr(0, pos);
                    }
                }
                std::strncpy(_mdns_record.destination, topic.c_str(), sizeof _mdns_record.destination - 1);
                _mdns_record.port = _mqtt_server_config.port;

                _emit_mdns();
            }

            _emit_mdns();
        } break;

        case MQTT_EVENT_DISCONNECTED:
            _net_sta = NETWORK_JOINED;
            break;

        case MQTT_EVENT_DATA:
            if (_rb_rx) {
                MA_LOGD(MA_TAG, "Received data from mqtt %d:%d", event->topic_len, event->data_len);
                static int sub_topic_len = std::strlen(_mqtt_topic_config.sub_topic);
                if (event->topic_len == sub_topic_len && std::strncmp(event->topic, _mqtt_topic_config.sub_topic, event->topic_len) == 0) {
                    _rb_rx->push(event->data, event->data_len);
                }
            }
            break;

        default:
            break;
    }
}

static TaskHandle_t _mqtt_serivice_thread_handler{};

static void _mqtt_serivice_thread(void*) {
    while (1) {
        switch (_net_sta) {
            case NETWORK_LOST: {
                MA_LOGD(MA_TAG, "Network lost, reconnecting");
                esp_err_t ret = nvs_flash_init();
                if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
                    nvs_flash_erase();
                    ret = nvs_flash_init();
                    if (ret != ESP_OK) {
                        MA_LOGE(MA_TAG, "Failed to init nvs");
                        return;
                    }
                }

                ret = esp_netif_init();
                if (ret != ESP_OK) {
                    MA_LOGE(MA_TAG, "Failed to init netif");
                    return;
                }
                ret = esp_event_loop_create_default();
                if (ret != ESP_OK) {
                    MA_LOGE(MA_TAG, "Failed to create event loop");
                    return;
                }

                static esp_netif_inherent_config_t esp_netif_config;
                esp_netif_config            = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
                esp_netif_config.if_desc    = "sscma";
                esp_netif_config.route_prio = 128;
                _esp_netif                  = esp_netif_create_wifi(WIFI_IF_STA, &esp_netif_config);
                esp_wifi_set_default_wifi_sta_handlers();

                static wifi_init_config_t init_cfg;
                init_cfg = WIFI_INIT_CONFIG_DEFAULT();

                ret = esp_wifi_init(&init_cfg);
                if (ret != ESP_OK) {
                    MA_LOGE(MA_TAG, "Failed to init wifi");
                    return;
                }

                ret = esp_wifi_set_storage(WIFI_STORAGE_RAM);
                if (ret != ESP_OK) {
                    MA_LOGE(MA_TAG, "Failed to set wifi storage");
                    return;
                }

                ret = esp_wifi_set_mode(WIFI_MODE_STA);
                if (ret != ESP_OK) {
                    MA_LOGE(MA_TAG, "Failed to set wifi mode");
                    return;
                }

                ret = esp_wifi_start();
                if (ret != ESP_OK) {
                    MA_LOGE(MA_TAG, "Failed to start wifi");
                    return;
                }

                _net_sta = NETWORK_IDLE;
            }
                [[fallthrough]];

            case NETWORK_IDLE: {
                MA_LOGD(MA_TAG, "Network idle, joining to wifi");

                esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &_wifi_event_handler, nullptr);
                esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &_wifi_event_handler, nullptr);

                static wifi_config_t cfg = {
                    .sta =
                        {
                            .ssid        = {},
                            .password    = {},
                            .scan_method = WIFI_ALL_CHANNEL_SCAN,
                            .sort_method = WIFI_CONNECT_AP_BY_SIGNAL,
                            .threshold =
                                {
                                    .rssi     = -127,
                                    .authmode = WIFI_AUTH_OPEN,
                                },
                        },
                };


                if (cfg.sta.ssid[0] == 0) {
                    auto storage = ma::get_storage_instance();
                    if (!storage) {
                        MA_LOGE(MA_TAG, "Failed to get storage instance");
                        return;
                    }

                    MA_STORAGE_GET_CSTR(storage, MA_STORAGE_KEY_WIFI_SSID, (char*)cfg.sta.ssid, 32, "");
                    if (cfg.sta.ssid[0] == 0) {
                        MA_STORAGE_GET_CSTR(storage, MA_STORAGE_KEY_WIFI_BSSID, (char*)cfg.sta.ssid, 32, "");
                    }
                    if (cfg.sta.ssid[0] == 0) {
                        MA_LOGD(MA_TAG, "No wifi ssid found");
                        return;
                    }

                    MA_STORAGE_GET_CSTR(storage, MA_STORAGE_KEY_WIFI_PWD, (char*)cfg.sta.password, 64, "");
                    int8_t security;
                    MA_STORAGE_GET_POD(storage, MA_STORAGE_KEY_WIFI_SECURITY, security, 0);

                    std::vector<wifi_auth_mode_t> sec_map = {
                        WIFI_AUTH_WPA_WPA2_PSK,
                        WIFI_AUTH_OPEN,
                        WIFI_AUTH_WEP,
                        WIFI_AUTH_WPA_WPA2_PSK,
                        WIFI_AUTH_WPA2_WPA3_PSK,
                        WIFI_AUTH_WPA3_PSK,
                    };
                    cfg.sta.threshold.authmode = security < sec_map.size() ? sec_map[security] : (wifi_auth_mode_t)security;
                }

                if (esp_wifi_set_config(WIFI_IF_STA, &cfg) != ESP_OK) {
                    MA_LOGE(MA_TAG, "Failed to set wifi config");
                    return;
                }

                esp_wifi_clear_fast_connect();

                if (esp_wifi_connect() != ESP_OK) {
                    MA_LOGE(MA_TAG, "Failed to join to wifi");
                    esp_wifi_disconnect();
                    break;
                }

                while (_net_sta != NETWORK_JOINED) {
                    vTaskDelay(100 / portTICK_PERIOD_MS);
                }
            }
                [[fallthrough]];

            case NETWORK_JOINED: {
                MA_LOGD(MA_TAG, "Network joined, connecting to mqtt");

                if (_mqtt_server_config.host[0] == 0) {
                    auto storage = ma::get_storage_instance();
                    if (!storage) {
                        MA_LOGE(MA_TAG, "Failed to get storage instance");
                        return;
                    }

                    MA_STORAGE_GET_ASTR(storage, MA_STORAGE_KEY_MQTT_HOST, _mqtt_server_config.host, "");
                    if (_mqtt_server_config.host[0] == 0) {
                        MA_LOGD(MA_TAG, "No mqtt host found");
                        return;
                    }

                    MA_STORAGE_GET_POD(storage, MA_STORAGE_KEY_MQTT_PORT, _mqtt_server_config.port, 0);
                    if (_mqtt_server_config.port == 0) {
                        _mqtt_server_config.port = 1883;
                    }

                    MA_STORAGE_GET_ASTR(storage, MA_STORAGE_KEY_MQTT_CLIENTID, _mqtt_server_config.client_id, "");
                    if (_mqtt_server_config.client_id[0] == 0) {
                        MA_LOGD(MA_TAG, "No mqtt client id found");
                        return;
                    }

                    MA_STORAGE_GET_ASTR(storage, MA_STORAGE_KEY_MQTT_USER, _mqtt_server_config.username, "");
                    if (_mqtt_server_config.username[0] == 0) {
                        MA_LOGD(MA_TAG, "No mqtt username found");
                    }

                    MA_STORAGE_GET_ASTR(storage, MA_STORAGE_KEY_MQTT_PWD, _mqtt_server_config.password, "");
                    if (_mqtt_server_config.password[0] == 0) {
                        MA_LOGD(MA_TAG, "No mqtt password found");
                    }

                    MA_STORAGE_GET_POD(storage, MA_STORAGE_KEY_MQTT_SSL, _mqtt_server_config.use_ssl, 0);

                    {
                        MA_STORAGE_GET_ASTR(storage, MA_STORAGE_KEY_MQTT_PUB_TOPIC, _mqtt_topic_config.pub_topic, "");
                        if (_mqtt_topic_config.pub_topic[0] == 0) {
                            MA_LOGD(MA_TAG, "No mqtt pub topic found");
                            return;
                        }

                        MA_STORAGE_GET_ASTR(storage, MA_STORAGE_KEY_MQTT_SUB_TOPIC, _mqtt_topic_config.sub_topic, "");
                        if (_mqtt_topic_config.sub_topic[0] == 0) {
                            MA_LOGD(MA_TAG, "No mqtt sub topic found");
                            return;
                        }

                        MA_STORAGE_GET_POD(storage, MA_STORAGE_KEY_MQTT_PUB_QOS, _mqtt_topic_config.pub_qos, 0);
                        MA_STORAGE_GET_POD(storage, MA_STORAGE_KEY_MQTT_SUB_QOS, _mqtt_topic_config.sub_qos, 0);
                    }
                }

                if (!_mqtt_client) {

                    static esp_mqtt_client_config_t esp_mqtt_cfg = {
                        .broker =
                            {
                                .address =
                                    {
                                        .hostname  = _mqtt_server_config.host,
                                        .transport = _mqtt_server_config.use_ssl ? MQTT_TRANSPORT_OVER_SSL : MQTT_TRANSPORT_OVER_TCP,
                                        .port      = (uint16_t)_mqtt_server_config.port,
                                    },
                            },
                        .credentials =
                            {
                                .username  = _mqtt_server_config.username,
                                .client_id = _mqtt_server_config.client_id,
                                .authentication =
                                    {
                                        .password = _mqtt_server_config.password,
                                    },
                            },
                    };

                    MA_LOGD(MA_TAG, "Connecting to mqtt %s:%d", esp_mqtt_cfg.broker.address.hostname, esp_mqtt_cfg.broker.address.port);
                    MA_LOGD(MA_TAG, "Client id: %s", esp_mqtt_cfg.credentials.client_id);
                    MA_LOGD(MA_TAG, "Username: %s", esp_mqtt_cfg.credentials.username);
                    MA_LOGD(MA_TAG, "Password: %s", esp_mqtt_cfg.credentials.authentication.password);
                    MA_LOGD(MA_TAG, "Use SSL: %d", _mqtt_server_config.use_ssl);

                    if (_mqtt_server_config.use_ssl) {
                        auto storage = ma::get_storage_instance();
                        static std::string custom_ca;

                        if (storage) {
                            MA_STORAGE_GET_STR(storage, MA_STORAGE_KEY_MQTT_SSL_CA, custom_ca, "");
                        }
                        if (custom_ca.size()) {
                            custom_ca.push_back('\0');
                            MA_LOGD(MA_TAG, "Using custom CA:\n%s", custom_ca.c_str());
                            esp_mqtt_cfg.broker.verification.certificate     = custom_ca.c_str();
                            esp_mqtt_cfg.broker.verification.certificate_len = custom_ca.size();
                        } else {
                            auto ret = esp_tls_init_global_ca_store();
                            if (ret != ESP_OK) {
                                MA_LOGE(MA_TAG, "Failed to init global ca store");
                                return;
                            }
                            esp_mqtt_cfg.broker.verification.use_global_ca_store = true;
                        }
                        static const char* servers[] = {
                            "pool.ntp.org",
                            "ntp.ntsc.ac.cn",
                            "time.windows.com",
                        };
                        esp_sntp_config_t ntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("");
                        ntp_config.server_from_dhcp  = true;
                        ntp_config.num_of_servers    = sizeof servers / sizeof servers[0];
                        for (size_t i = 0; i < ntp_config.num_of_servers; ++i) {
                            ntp_config.servers[i] = servers[i];
                        }
                        esp_netif_sntp_deinit();
                        auto ret = esp_netif_sntp_init(&ntp_config);
                        if (ret != ESP_OK) {
                            MA_LOGE(MA_TAG, "Failed to init sntp");
                            break;
                        }
                        while (esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000)) != ESP_OK) {
                            MA_LOGD(MA_TAG, "Failed to sync time with NTP");
                        }
                        std::time_t end_time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
                        MA_LOGD(MA_TAG, "Current Time and Date: %s", std::ctime(&end_time));
                    }

                    _mqtt_client = esp_mqtt_client_init(&esp_mqtt_cfg);

                    esp_mqtt_client_register_event(_mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, _mqtt_event_handler, nullptr);
                    esp_mqtt_client_start(_mqtt_client);
                }
            } break;

            case NETWORK_CONNECTED:
                break;

            default:
                break;
        }

        MA_LOGD(MA_TAG, "Network status: %d", _net_sta);
        MA_LOGD(MA_TAG, "Minimum free heap: %d KB", int(esp_get_minimum_free_heap_size() / 1024));

        vTaskDelay(3000 / portTICK_PERIOD_MS);
    }
}

static void _mqtt_serivice_thread_wrapper(void*) {
    _mqtt_serivice_thread(nullptr);
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

namespace ma {

MQTT::MQTT() noexcept : Transport(MA_TRANSPORT_MQTT) {}

MQTT::~MQTT() noexcept {}

ma_err_t MQTT::init(void const* config) {
    if (m_initialized) {
        return MA_OK;
    }

    (void)config;

    if (_rb_rx == nullptr) {
        _rb_rx = new SPSCRingBuffer<char>(4096);
    }

    auto ret = xTaskCreatePinnedToCore(_mqtt_serivice_thread_wrapper, "mqtt_service", 6144, nullptr, 3, &_mqtt_serivice_thread_handler, 1);

    if (ret != pdPASS) {
        MA_LOGE(MA_TAG, "Failed to create mqtt service thread");
        return MA_FAILED;
    }

    m_initialized = true;

    return MA_OK;
}

void MQTT::deInit() {
    if (!m_initialized) {
        return;
    }

    if (_rb_rx) {
        delete _rb_rx;
        _rb_rx = nullptr;
    }

    if (_mqtt_serivice_thread_handler) {
        vTaskDelete(_mqtt_serivice_thread_handler);
        _mqtt_serivice_thread_handler = nullptr;
    }

    esp_mqtt_client_unsubscribe(_mqtt_client, _mqtt_topic_config.sub_topic);
    esp_mqtt_client_disconnect(_mqtt_client);
    esp_mqtt_client_stop(_mqtt_client);

    m_initialized = false;
}

size_t MQTT::available() const {
    return _rb_rx->size();
}

size_t MQTT::send(const char* data, size_t length) {
    if (!m_initialized) {
        return 0;
    }

    if (_net_sta != NETWORK_CONNECTED) {
        return 0;
    }

    MA_LOGD(MA_TAG, "Publishing data to mqtt %d", length);
    if (esp_mqtt_client_publish(_mqtt_client, _mqtt_topic_config.pub_topic, data, length, _mqtt_topic_config.pub_qos, 0) < 0) {
        MA_LOGD(MA_TAG, "Failed to publish data to mqtt");
        return 0;
    }

    return length;
}

size_t MQTT::flush() {
    return 0;
}

size_t MQTT::receive(char* data, size_t length) {
    if (!m_initialized) {
        return 0;
    }

    return _rb_rx->pop(data, length);
}

size_t MQTT::receiveIf(char* data, size_t length, char delimiter) {
    if (!m_initialized) {
        return 0;
    }

    return _rb_rx->popIf(data, length, delimiter);
}

}  // namespace ma
