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

#ifndef _EDGELAB_H
#define _EDGELAB_H

#include "el_common.h"
#include "el_misc.h"
#include "el_algorithm.h"
#include "el_inference.h"
#include "el_device.h"
#include "el_camera.h"
#include "el_display.h"
#include "el_repl.h"
#include "el_cv.h"
#include "el_nms.h"
#include "el_base64.h"

#ifdef CONFIG_EL_TFLITE
#include "el_inference_tflite.h"
#endif

#ifdef CONFIG_EL_ALGORITHM_YOLO
#include "el_algorithm_yolo.h"
#endif

#ifdef __cplusplus
using namespace edgelab;
#endif

#endif /* EDGELAB_H */