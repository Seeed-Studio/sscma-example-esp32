#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <functional>
#include <memory>
#include <queue>
#include <utility>

class TaskExecutor {
   public:
    TaskExecutor(size_t worker_stack_size = configMINIMAL_STACK_SIZE, size_t worker_priority = configMAX_PRIORITIES - 1)
        : __task_queue_lock(xSemaphoreCreateCounting(1, 1)),
          __task_stop_requested(false),
          __worker_thread_stop_requested(false),
          __worker_ret(),
          __worker_handler(),
          __worker_name(new char[configMAX_TASK_NAME_LEN]),
          __worker_stack_size(worker_stack_size),
          __worker_priority(worker_priority) {
        static uint16_t worker_id = 0;
        sprintf(__worker_name, "task_executor_%2X", worker_id++);
    }

    ~TaskExecutor() {
        stop();
        vSemaphoreDelete(__task_queue_lock);
    }

    void start() {
        __worker_ret = xTaskCreate(
          &TaskExecutor::c_run, __worker_name, __worker_stack_size, this, __worker_priority, &__worker_handler);
    }

    void stop() {
        __worker_thread_stop_requested.store(true, std::memory_order_relaxed);
        if (__worker_ret == pdPASS) [[likely]]
            vTaskDelete(__worker_handler);
    }

    void add_task(std::function<void(std::atomic<bool>&)> task) {
        m_lock();
        __task_queue.push(task);
        __task_stop_requested.store(true, std::memory_order_relaxed);
        m_unlock();
    }

    const char* get_worker_name() { return __worker_name; }

   protected:
    inline void m_lock() { xSemaphoreTake(__task_queue_lock, portMAX_DELAY); }
    inline void m_unlock() { xSemaphoreGive(__task_queue_lock); }

   private:
    SemaphoreHandle_t __task_queue_lock;
    std::atomic<bool> __task_stop_requested;
    std::atomic<bool> __worker_thread_stop_requested;

    BaseType_t   __worker_ret;
    TaskHandle_t __worker_handler;
    char*        __worker_name;
    size_t       __worker_stack_size;
    size_t       __worker_priority;

    std::queue<std::function<void(std::atomic<bool>&)>> __task_queue;

    static void c_run(void* this_pointer) { static_cast<TaskExecutor*>(this_pointer)->run(); }

    void run() {
        while (!__worker_thread_stop_requested.load(std::memory_order_relaxed)) {
            std::function<void(std::atomic<bool>&)> task;
            {
                m_lock();
                if (!__task_queue.empty()) {
                    task = std::move(__task_queue.front());
                    __task_queue.pop();
                    if (__task_queue.empty()) [[likely]]
                        __task_stop_requested.store(false, std::memory_order_seq_cst);
                    else
                        __task_stop_requested.store(true, std::memory_order_seq_cst);
                }
                m_unlock();
            }
            if (task) task(__task_stop_requested);
            vTaskDelay(15 / portTICK_PERIOD_MS);
        }
    }
};
