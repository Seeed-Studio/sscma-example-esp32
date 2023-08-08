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

#ifndef _EL_NMS_H
#define _EL_NMS_H

#include <forward_list>

#include "el_common.h"
#include "el_types.h"

#ifdef __cplusplus
extern "C" {
namespace edgelab {
#endif

/**
 * @brief Non-maximum suppression
 *
 * @param boxes
 * @param nms_iou_thresh
 * @param nms_score_thresh
 * @param nms_box_num
 * @param soft_nms
 * @param multi_label
 * @return int
 */
int el_nms(std::forward_list<el_box_t>& boxes,
           uint8_t                      nms_iou_thresh,
           uint8_t                      nms_score_thresh,
           bool                         soft_nms     = false,
           bool                         multi_target = false);

#ifdef __cplusplus
}  // namespace edgelab
}
#endif

#endif /* EL_NMS_H */
