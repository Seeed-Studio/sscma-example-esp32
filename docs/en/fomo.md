
# Train and deploy a fomo model on ESP32-S3

## Introduction

This tutorial shows how to train and deploy a meter reading detection model on ESP32-S3.

## Prerequisites

### Hardware
- PC with Linux or Mac OS, WSL2 is also supported.
- ESP32-S3 development board
- USB cable

### Software
- [EdgeLab](https://edgelab.readthedocs.io/zh_CN/latest/)
- [ESP-IDF](./get_start.md#how-to-install)  
- [edgelab-example-esp32](https://github.com/Seeed-Studio/edgelab-example-esp32)

```{note}
Plesae make sure that you have installed the latest version of EdgeLab & ESP-IDF.
```

## Train the model

### Prepare the dataset

Use the provided dataset to train the model.

```bash
cd EdgeLab
mkdir -p datasets
cd datasets
wget https://files.seeedstudio.com/wiki/Edgelab/coco_mask.zip
unzip coco_mask.zip
```

### Prepare the configuration file

```python
_base_ = '../_base_/default_runtime.py'

num_classes=2
model = dict(
    type='Fomo',
    backbone=dict(type='MobileNetV2', widen_factor=0.35, out_indices=(2, )),
    head=dict(
        type='Fomo_Head',
        input_channels=16,
        num_classes=num_classes,
        middle_channels=[96, 32],
        act_cfg='ReLU6',
        loss_cls=dict(type='BCEWithLogitsLoss',
                      reduction='none',
                      pos_weight=40),
        loss_bg=dict(type='BCEWithLogitsLoss', reduction='none'),
        cls_weight=40,
    ),
)

# dataset settings
dataset_type = 'FomoDatasets'
data_root = './work_dirs/datasets/coco_mask'
height=96
width=96
batch_size=16
workers=4


train_pipeline = [
    dict(type='RandomResizedCrop', height=height, width=width, scale=(0.90, 1.1),
         p=1),
    dict(type='Rotate', limit=20),
    dict(type='RandomBrightnessContrast',
         brightness_limit=0.2,
         contrast_limit=0.2,
         p=0.5),
    dict(type='HorizontalFlip', p=0.5),
]
test_pipeline = [dict(type='Resize', height=height, width=width, p=1)]

data = dict(samples_per_gpu=batch_size,
            workers_per_gpu=workers,
            train_dataloader=dict(collate=True),
            val_dataloader=dict(collate=True),
            train=dict(type=dataset_type,
                       data_root=data_root,
                       ann_file='train/_annotations.coco.json',
                       img_prefix='train',
                       pipeline=train_pipeline),
            val=dict(type=dataset_type,
                     data_root=data_root,
                     test_mode=True,
                     ann_file='valid/_annotations.coco.json',
                     img_prefix='valid',
                     pipeline=test_pipeline),
            test=dict(type=dataset_type,
                      data_root=data_root,
                      test_mode=True,
                      ann_file='valid/_annotations.coco.json',
                      img_prefix='valid',
                      pipeline=test_pipeline))

# optimizer
lr=0.001
epochs=70
optimizer = dict(type='Adam', lr=lr, weight_decay=0.0005)

optimizer_config = dict(grad_clip=dict(max_norm=35, norm_type=2))
# learning policy
lr_config = dict(policy='step',
                 warmup='linear',
                 warmup_iters=30,
                 warmup_ratio=0.000001,
                 step=[100, 200, 250])
# runtime settings
evaluation = dict(interval=1, metric=['mAP'], fomo=True)
find_unused_parameters = True
```

Save the configuration file as `fomo_mobnetv2_x8_custom.py` in the `configs/fomo` directory.

### Train the model

#### Download the pre-trained model.
```bash
cd EdgeLab
mkdir -p work_dirs/pretrain/ && cd work_dirs/pretrain
wget  https://github.com/Seeed-Studio/EdgeLab/releases/download/model_zoo/fomo_mv2n_96.pth 
```

#### Train the model.
```bash
cd EdgeLab
conda activate edgelab
python tools/train.py mmdet configs/fomo/fomo_mobnetv2_x8_custom.py --cfg-options load_from=./work_dirs/pretrain/fomo_mv2n_96.pth 
```

## Convert the model
Convert models to TensorFlow Lite.

```bash
cd EdgeLab
conda activate edgelab
python tools/torch2tflite.py mmdet  configs/fomo/fomo_mobnetv2_x8_custom.py --weights work_dirs/fomo_mobnetv2_x8_custom/exp1/latest.pth --tflite_type int8 
```

```{note}
Note: The exp1 in the path is generated in the first training. If you train multiple times, expx will be added one by one.
```

## Deploy the model
Convert the model to a C file and put it in the `components/modules/model` directory of `edgelab-example-esp32`.
```bash
cd edgelab-example-esp32
conda activate edgelab
python tools/tflite2c.py --input work_dirs/fomo_mobnetv2_x8_custom/exp1/latest.tflite --name fomo --output_dir ./components/modules/model --classes='("unmask", "mask")'
```

Compile and flash the program to the ESP32-S3 development board.

```bash
cd edgelab-example-esp32/examples/meter
idf.py set-target esp32s3
idf.py build
```

### Run 

![fomo_mask](../_static/esp32/images/fomo_mask.gif)