
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include <atomic>
#include <functional>
#include <memory>
#include <queue>
#include <thread>
#include <utility>

class TaskExecutor {
   public:
    TaskExecutor()
        : __task_queue_lock(xSemaphoreCreateCounting(1, 1)),
          __task_stop_requested(false),
          __worker_thread_stop_requested(false) {}

    ~TaskExecutor() {
        stop();
        vSemaphoreDelete(__task_queue_lock);
    }

    void start() { __worker_thread = std::thread(&TaskExecutor::run, this); }

    void stop() {
        __worker_thread_stop_requested.store(true, std::memory_order_relaxed);
        __worker_thread.join();
    }

    void add_task(std::function<void(std::atomic<bool>&)> task) {
        m_lock();
        __task_queue.push(task);
        __task_stop_requested.store(true, std::memory_order_relaxed);
        m_unlock();
    }

   protected:
    inline void m_lock() { xSemaphoreTake(__task_queue_lock, portMAX_DELAY); }
    inline void m_unlock() { xSemaphoreGive(__task_queue_lock); }

   private:
    std::queue<std::function<void(std::atomic<bool>&)>> __task_queue;
    std::thread                                         __worker_thread;

    SemaphoreHandle_t __task_queue_lock;
    std::atomic<bool> __task_stop_requested;
    std::atomic<bool> __worker_thread_stop_requested;

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
