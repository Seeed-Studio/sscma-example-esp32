#ifndef _MA_CONFIG_BOARD_H_
#define _MA_CONFIG_BOARD_H_

#ifdef MA_PORTING_ESPRESSIF_ESP32S3_EYE
#include "boards/ma_board_esp32s3_eye.h"
#elif defined(MA_PORTING_SEEED_XIAO_ESP32S3)
#include "boards/ma_board_xiao_s3.h"
#define MA_SERVER_RUN_DEVICE_BACKGROUND_TASK             1
#define MA_SERVER_RUN_DEVICE_BACKGROUND_TASK_INTERVAL_MS 1000
#define MA_SERVER_RUN_DEVICE_BACKGROUND_TASK_PRIORITY    1
#define MA_SERVER_RUN_DEVICE_BACKGROUND_TASK_STACK_SIZE  10240
#else
#error "Please define the board type"
#endif

#define MA_BOARD_NAME                        PORT_DEVICE_NAME

#define MA_USE_ENGINE_TFLITE                 1
#define MA_ENGINE_TFLITE_TENSOE_ARENA_SIZE   (2048 * 1024)
#define MA_USE_ENGINE_TENSOR_INDEX           1
#define MA_USE_STATIC_TENSOR_ARENA           1

#define MA_DEBUG_LEVEL                       2

#define MA_TRIGGER_USE_REAL_GPIO             1

#define MA_USE_EXTERNAL_WIFI_STATUS          1

#define MA_USE_LIB_JPEGENC                   1

#define MA_TFLITE_OP_CONV_2D                 1
#define MA_TFLITE_OP_RESHAPE                 1
#define MA_TFLITE_OP_SHAPE                   1
#define MA_TFLITE_OP_PACK                    1
#define MA_TFLITE_OP_PAD                     1
#define MA_TFLITE_OP_PADV2                   1
#define MA_TFLITE_OP_SUB                     1
#define MA_TFLITE_OP_ADD                     1
#define MA_TFLITE_OP_RELU                    1
#define MA_TFLITE_OP_MAX_POOL_2D             1
#define MA_TFLITE_OP_SPLIT                   1
#define MA_TFLITE_OP_CONCATENATION           1
#define MA_TFLITE_OP_FULLY_CONNECTED         1
#define MA_TFLITE_OP_RESIZE_NEAREST_NEIGHBOR 1
#define MA_TFLITE_OP_QUANTIZE                1
#define MA_TFLITE_OP_TRANSPOSE               1
#define MA_TFLITE_OP_LOGISTIC                1
#define MA_TFLITE_OP_MUL                     1
#define MA_TFLITE_OP_SPLIT_V                 1
#define MA_TFLITE_OP_STRIDED_SLICE           1
#define MA_TFLITE_OP_MEAN                    1
#define MA_TFLITE_OP_SOFTMAX                 1
#define MA_TFLITE_OP_DEPTHWISE_CONV_2D       1
#define MA_TFLITE_OP_LEAKY_RELU              1
#define MA_TFLITE_OP_RMAU6                   1

#define MA_INVOKE_ENABLE_RUN_HOOK            1

#define MA_FILESYSTEM_LITTLEFS               1
#define MA_STORAGE_LFS_USE_FLASHBD           1

#define MA_OSAL_FREERTOS                     1
#define MA_OSAL_FREERTOS_CXX_ALLOC_OVERRIDE  0
#define MA_SEVER_AT_EXECUTOR_STACK_SIZE      (20 * 1024)
#define MA_SEVER_AT_EXECUTOR_TASK_PRIO       2


#endif  // _MA_CONFIG_BOARD_H_