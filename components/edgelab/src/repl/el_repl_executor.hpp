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

#ifndef _EL_REPL_EXECUTOR_HPP_
#define _EL_REPL_EXECUTOR_HPP_

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include <atomic>
#include <functional>
#include <queue>

#define CONFIG_EL_REPL_EXECUTOR_STACK_SIZE 40960
#define CONFIG_EL_REPL_EXECUTOR_PRIO       5

namespace edgelab::repl {

namespace types {

typedef std::function<void(std::atomic<bool>&)> el_repl_task_t;

}

class ReplExecutor {
   public:
    ReplExecutor(size_t worker_stack_size = CONFIG_EL_REPL_EXECUTOR_STACK_SIZE,
                 size_t worker_priority   = CONFIG_EL_REPL_EXECUTOR_PRIO);
    ~ReplExecutor();

    void start();
    void stop();

    void add_task(types::el_repl_task_t task);

    const char* get_worker_name() const;

   protected:
    void m_lock() const;
    void m_unlock() const;

    void        run();
    static void c_run(void* this_pointer);

   private:
    mutable SemaphoreHandle_t _task_queue_lock;
    std::atomic<bool>         _task_stop_requested;
    std::atomic<bool>         _worker_thread_stop_requested;

    BaseType_t   _worker_ret;
    TaskHandle_t _worker_handler;
    char*        _worker_name;
    size_t       _worker_stack_size;
    size_t       _worker_priority;

    std::queue<types::el_repl_task_t> _task_queue;
};

}  // namespace edgelab::repl

#endif
