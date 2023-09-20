

#include <stdio.h>

#include "core/edgelab.h"

struct some_config_t {
    int val;
};

extern "C" void app_main()
{
    using namespace edgelab;
    using namespace edgelab::utility;

    printf("Data storage demo:\n");

    printf("Geting storage handler from data delegate...\n");
    auto* storage = new edgelab::Storage();

    printf("Init storage...\n");
    storage->init();

    printf("Quering storage keys ->\n");
    for (const auto& k : *storage)
        printf("\t%s\n", k);

    printf("Clearing storage...\n");
    storage->clear();

    printf("Quering storage keys ->\n");
    for (const auto& k : *storage)
        printf("\t%s\n", k);

    {
        printf("Emplace KV to storage ->\n");

        auto kv = el_make_storage_kv("key_1", 1);
        bool is_ok = storage->emplace(kv);
        printf("\tstore KV (%s, %d), %s\n", kv.key, kv.value, is_ok ? "ok" : "fail");

        int value = 2;
        is_ok = storage->emplace(el_make_storage_kv("key_2", value));
        printf("\tstore KV (key_2, 2), %s\n", is_ok ? "ok" : "fail");

        kv = el_make_storage_kv("key_3", 3);
        *storage << kv << el_make_storage_kv("key_4", 4);
        printf("\tstore KV (key_3, 3), (key_4, 4)\n");

        char str[] = "Hello EdgeLab!";
        *storage << el_make_storage_kv("key_5", str);
        printf("\tstore KV (key_5, %s)\n", str);

        some_config_t some_config = some_config_t{.val = 42};
        auto kv_1 = el_make_storage_kv_from_type(some_config);
        *storage << kv_1;
        printf("\tstore type KV (%s, some_config_t{ int val = %d; })\n", kv_1.key, kv_1.value.val);
    }

    {
        printf("Get KV from storage ->\n");

        auto kv = el_make_storage_kv("key_1", 0);
        bool is_ok = storage->get(kv);
        printf("\tget KV (%s, %d), %s\n", kv.key, kv.value, is_ok ? "ok" : "fail");

        int value = storage->get_value<int>("key_2");
        printf("\tget KV (key_2, %d), %s\n", value, value == 2 ? "ok" : "fail");

        storage->get(el_make_storage_kv("key_3", value));
        printf("\tget KV (key_3, %d), %s\n", value, value == 3 ? "ok" : "fail");

        *storage >> el_make_storage_kv("key_4", value);
        printf("\tget KV (key_4, %d), %s\n", value, is_ok ? "ok" : "fail");

        char str[32] = "";
        *storage >> el_make_storage_kv("key_5", str);
        printf("\tget KV (key_5, %s)\n", str);

        some_config_t some_config = some_config_t{.val = 42};
        auto kv_2 = el_make_storage_kv_from_type(some_config);
        *storage >> kv_2;
        printf("\tget type KV (%s, some_config_t{ int val = %d; })\n", kv_2.key, kv_2.value.val);
    }

    printf("Quering storage keys ->\n");
    for (const auto& k : *storage)
        printf("\t%s\n", k);

    {
        printf("Other storage API ->\n");

        size_t size = storage->get_value_size("key_1");
        printf("\tvalue size of key (%s, %d)\n", "key_1", size);

        bool has_key = storage->contains("key_2");
        printf("\tstorage %s the key (key_2)\n", has_key ? "has" : "doesn't have");

        has_key = storage->contains("some_key");
        printf("\tstorage %s the key (some_key)\n", has_key ? "has" : "doesn't have");

        auto kv = el_make_storage_kv("key_2", 2);
        bool is_emplaced = storage->try_emplace(kv);
        printf("\t%s KV (%s, %d)\n",
               is_emplaced ? "emplaced" : "not emplaced existing",
               kv.key,
               kv.value);
    }

    // TODO: move freeRTOS, ESP related function call to EdgeLab
    for (int i = 1000; i >= 0; --i) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now...\n");
    fflush(stdout);
    esp_restart();
}
