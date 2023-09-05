/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Seeed Technology Co.,Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include "el_repl_executor.hpp"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <cstdio>
#include <functional>
#include <memory>
#include <queue>
#include <utility>

#include "el_debug.h"

namespace edgelab::repl {

ReplExecutor::ReplExecutor(size_t worker_stack_size, size_t worker_priority)
    : _task_queue_lock(xSemaphoreCreateCounting(1, 1)),
      _task_stop_requested(false),
      _worker_thread_stop_requested(false),
      _worker_ret(),
      _worker_handler(),
      _worker_name(new char[configMAX_TASK_NAME_LEN]{}),
      _worker_stack_size(worker_stack_size),
      _worker_priority(worker_priority) {
    static uint16_t worker_id = 0;
    volatile size_t length    = configMAX_TASK_NAME_LEN - 1;
    std::snprintf(_worker_name, length, "task_executor_%2X", worker_id++);

    EL_ASSERT(_task_stop_requested.is_lock_free());
    EL_ASSERT(_worker_thread_stop_requested.is_lock_free());
}

ReplExecutor::~ReplExecutor() {
    stop();
    vSemaphoreDelete(_task_queue_lock);
}

void ReplExecutor::start() {
    _worker_ret =
      xTaskCreate(&ReplExecutor::c_run, _worker_name, _worker_stack_size, this, _worker_priority, &_worker_handler);
}

void ReplExecutor::stop() {
    _worker_thread_stop_requested.store(true, std::memory_order_relaxed);
    if (_worker_ret == pdPASS) [[likely]]
        vTaskDelete(_worker_handler);
}

void ReplExecutor::add_task(types::el_repl_task_t task) {
    m_lock();
    _task_queue.push(task);
    _task_stop_requested.store(true, std::memory_order_relaxed);
    m_unlock();
}

const char* ReplExecutor::get_worker_name() const { return _worker_name; }

inline void ReplExecutor::m_lock() const { xSemaphoreTake(_task_queue_lock, portMAX_DELAY); }
inline void ReplExecutor::m_unlock() const { xSemaphoreGive(_task_queue_lock); }

void ReplExecutor::run() {
    while (!_worker_thread_stop_requested.load(std::memory_order_relaxed)) {
        types::el_repl_task_t task;
        {
            m_lock();
            if (!_task_queue.empty()) {
                task = std::move(_task_queue.front());
                _task_queue.pop();
                if (_task_queue.empty()) [[likely]]
                    _task_stop_requested.store(false, std::memory_order_seq_cst);
                else
                    _task_stop_requested.store(true, std::memory_order_seq_cst);
            }
            m_unlock();
        }
        if (task) task(_task_stop_requested);
        vTaskDelay(15 / portTICK_PERIOD_MS);  // TODO: use yield
    }
}

void ReplExecutor::c_run(void* this_pointer) { static_cast<ReplExecutor*>(this_pointer)->run(); }

}  // namespace edgelab::repl
