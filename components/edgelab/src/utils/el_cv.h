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

#ifndef _EL_CV_H_
#define _EL_CV_H_

#include <stdint.h>

#include "el_common.h"

#ifdef __cplusplus
extern "C" {
namespace edgelab {
#endif

/**
 * @brief Convert image format
 *
 * @param src
 * @param dst
 * @return EL_ERR
 */
EL_ERR el_img_convert(const el_img_t* src, el_img_t* dst);

/**
 * @brief Crop image
 *
 * @param src
 * @param dst
 * @param box
 * @return void
 */
void el_crop_image(const el_img_t* src, el_img_t* dst, const el_box_t* box);

/**
 * @brief Resize image
 *
 * @param src
 * @param dst
 * @param dst_width
 * @param dst_height
 * @param keep_ratio
 * @return void
 */
void el_resize_image(const el_img_t* src,
                     el_img_t* dst,
                     uint32_t dst_width,
                     uint32_t dst_height,
                     bool keep_ratio = false);

/**
 * @brief draw box
 *
 * @param img
 * @param box
 * @param show_text
 * @return void
 */
void el_draw_box(el_img_t* img, const el_box_t* box, bool show_text = false);

/**
 * @brief draw box
 *
 * @param img
 * @param x
 * @param y
 * @param w
 * @param h
 * @param color
 * @return void
 */
void el_draw_rect(el_img_t* img,
                  uint16_t x,
                  uint16_t y,
                  uint16_t w,
                  uint16_t h,
                  uint32_t color,
                  uint16_t thickness = 1);




#ifdef __cplusplus
} // namespace edgelab
}
#endif

#endif /* _EL_CV_H_ */