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

#ifndef _EL_TYPES_H_
#define _EL_TYPES_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "el_compiler.h"

#ifdef __cplusplus
extern "C" {
namespace edgelab {
#endif

/*------------------------------------------------------------------*/
/* CONSTANTS
 *------------------------------------------------------------------*/
enum {
    EL_OK = 0,       // success
    EL_LOGE = -1,   // general error
    EL_ETIMOUT = -2, // timeout
    EL_EIO = -3,     // IO error
    EL_EINVAL = -4,  // invalid argument
    EL_ENOMEM = -5,  // out of memory
    EL_EBUSY = -6,   // busy
    EL_ENOTSUP = -7, // not supported
    EL_EPERM = -8,   // operation not permitted
    EL_EAGAIN = -9,  // try again
};

typedef int EL_ERR;

/*------------------------------------------------------------------*/
/* STRUCTURES
 *------------------------------------------------------------------*/

typedef struct EL_ATTR_PACKED {
    size_t size;
    int *dims;
} el_shape_t;

typedef struct EL_ATTR_PACKED {
    float scale;
    int32_t zero_point;
} el_quant_param_t;

typedef struct EL_ATTR_PACKED {
    void *pool;
    size_t size;
} el_memory_pool_t;

typedef struct EL_ATTR_PACKED {
    uint16_t x;
    uint16_t y;
    uint16_t w;
    uint16_t h;
    uint8_t score;
    uint8_t target;
} el_box_t;

typedef struct EL_ATTR_PACKED {
    uint16_t score;
    uint16_t target;
} el_class_t;

typedef struct EL_ATTR_PACKED {
    size_t index;
    void *data;
    size_t size;
} el_tensor_t;

typedef struct EL_ATTR_PACKED {
    el_tensor_t *tensor;
    size_t length;
} el_input_t;

typedef enum {
    EL_PIXEL_FORMAT_RGB888 = 0,
    EL_PIXEL_FORMAT_RGB565,
    EL_PIXEL_FORMAT_YUV422,
    EL_PIXEL_FORMAT_GRAYSCALE,
    EL_PIXEL_FORMAT_JPEG,
    EL_PIXEL_FORMAT_UNKNOWN,
} el_pixel_format_t;

typedef enum {
    EL_PIXEL_ROTATE_0 = 0,
    EL_PIXEL_ROTATE_90,
    EL_PIXEL_ROTATE_180,
    EL_PIXEL_ROTATE_270,
    EL_PIXEL_ROTATE_UNKNOWN,
} el_pixel_rotate_t;

typedef struct EL_ATTR_PACKED {
    uint8_t *data;
    size_t size;
    uint32_t width;
    uint32_t height;
    el_pixel_format_t format;
    el_pixel_rotate_t rotate;
} el_img_t;

typedef struct EL_ATTR_PACKED {
    uint32_t width;
    uint32_t height;
} el_res_t;

#ifdef __cplusplus
}
}
#endif

#endif /* _EL_TYPES_H_ */