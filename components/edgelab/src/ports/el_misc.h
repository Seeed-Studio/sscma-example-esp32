/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2023 Hongtai Liu (Seeed Technology Inc.)
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

#ifndef _EL_MISC_H_
#define _EL_MISC_H_

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
namespace edgelab {
#endif  // defined(__cplusplus)

/*
 * @brief: sleep for ms
 * @param: ms
 * @return: void
 */
void el_sleep(uint32_t ms);

/*
 * @brief: get current time in ms
 * @param: none
 * @return: uint64_t
 */
uint64_t el_get_time_ms(void);

/*
 * @brief: get current time in us
 * @param: none
 * @return: uint64_t
 */
uint64_t el_get_time_us(void);

/*
 * @brief: wrapper of malloc
 * @param: size
 * @return: void*
 */
void* el_malloc(size_t size);

/*
 * @brief: wrapper of calloc
 * @param: nmemb
 * @param: size
 * @return: void*
 */
void* el_calloc(size_t nmemb, size_t size);

/*
 * @brief: free memory
 * @param: ptr
 * @return: void
 */
void el_free(void* ptr);

/*
 * @brief: wrapper of printf
 * @param: format
 * @param: ...
 * @return: int
 */
int el_printf(const char* format, ...);

/*
 * @brief: wrapper of putchar
 * @param: c
 * @return: int
 */
int el_putchar(char c);

/*
 * @brief: wrapper of getline
 * @param: lineptr
 * @param: n
 * @param: stream
 * @return: size_t
 * @note: this function is not thread safe
 */
size_t el_getline(char** lineptr, size_t* n, FILE* stream);

/*
 * @brief: reset the system
 * @param: none
 * @return: void
 */
void el_reset(void);

#if defined(__cplusplus) == 1
}  // namespace edgelab
}
#endif  // defined(__cplusplus)

#endif  // _EL_MISC_H_