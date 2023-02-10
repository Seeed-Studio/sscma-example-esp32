# 训练和部署一个模拟读表检测模型

## 概述

本教程将指导您如何使用EdgeLab进行模拟读表检测模型的训练以及将其部署在ESP32-S3芯片上。

## 准备工作

### 硬件
- 一台Linux或者Mac电脑
- 一块ESP32-S3开发板
- 一根USB线



### 软件
- [EdgeLab](https://edgelab.readthedocs.io/zh_CN/latest/)
- [ESP-IDF](./get_start.md#环境安装)
- [edgelab-example-esp32](https://github.com/Seeed-Studio/edgelab-example-esp32)

```{note}
请根据对应软件的文档进行安装。
```

## 训练模型
### 准备数据集
使用我们提供的数据集来训练模型。

```bash
cd EdgeLab
mkdir -p datasets
cd datasets
wget https://files.seeedstudio.com/wiki/Edgelab/meter.zip
unzip meter.zip
```

### 准备配置文件
```python
_base_ = '../_base_/default_runtime.py'

num_classes=1
model = dict(type='PFLD',
             backbone=dict(type='PfldMobileNetV2',
                           inchannel=3,
                           layer1=[16, 16, 16, 16, 16],
                           layer2=[32, 32, 32, 32, 32, 32],
                           out_channel=16),
             head=dict(
                 type='PFLDhead',
                 num_point=num_classes,
                 input_channel=16,
             ),
             loss_cfg=dict(type='PFLDLoss'))


# dataset settings
dataset_type = 'MeterData'

data_root = '~/datasets/meter'
height=112
width=112
batch_size=32
workers=4

train_pipeline = [
    dict(type="Resize", height=height, width=width, interpolation=0),
    dict(type='ColorJitter', brightness=0.3, p=0.5),
    # dict(type='GaussNoise'),
    dict(type='MedianBlur', blur_limit=3, p=0.3),
    dict(type='HorizontalFlip'),
    dict(type='VerticalFlip'),
    dict(type='Rotate'),
    dict(type='Affine', translate_percent=[0.05, 0.1], p=0.6)
]

val_pipeline = [dict(type="Resize", height=height, width=width)]



data = dict(
    samples_per_gpu=batch_size,
    workers_per_gpu=workers,
    train=dict(type=dataset_type,
               data_root=data_root,
               index_file=r'train/annotations.txt',
               pipeline=train_pipeline,
               test_mode=False),
    val=dict(type=dataset_type,
             data_root=data_root,
             index_file=r'val/annotations.txt',
             pipeline=val_pipeline,
             test_mode=True),
    test=dict(type=dataset_type,
              data_root=data_root,
              index_file=r'val/annotations.txt',
              pipeline=val_pipeline,
              test_mode=True
              # dataset_info={{_base_.dataset_info}}
              ))


lr=0.0001
epochs=300
evaluation = dict(save_best='loss')
optimizer = dict(type='Adam', lr=lr, betas=(0.9, 0.99), weight_decay=1e-6)
optimizer_config = dict(grad_clip=dict(max_norm=35, norm_type=2))
# learning policy
lr_config = dict(policy='step',
                 warmup='linear',
                 warmup_iters=400,
                 warmup_ratio=0.0001,
                 step=[400, 440, 490])
# lr_config = dict(
#     policy='OneCycle',
#     max_lr=0.0001,
#     # steps_per_epoch=388,
#     # epoch=1500,
#     pct_start=0.1)
# runtime settings
# runner = dict(type='EpochBasedRunner', max_epochs=30)
# evaluation = dict(interval=1, metric=['bbox'])
total_epochs = epochs
find_unused_parameters = True
```

### 训练模型

```bash
cd EdgeLab
conda activate edgelab
tools/train.py mmpose configs/pfld/pfld_mv2n_112.py --gpus=1 --cfg-options total_epochs=50
```

### 模型转换
将模型转换为TensorFlow Lite。
```bash
cd EdgeLab
conda activate edgelab
python tools/export.py configs/pfld/pfld_mv2n_112.py --weights work_dirs/pfld_mv2n_112/exp1/latest.pth --data ~/datasets/meter/train/images
```

## 部署模型

将模型转换为C语言文件，然后将其放入到`edgelab-example-esp32`的`components/modules/model`目录下。
```bash
cd edgelab-example-esp32
conda activate edgelab
python tools/tflite2c.py --input work_dirs/pfld_mv2n_112/exp1/latest.tflite --model_name pfld_meter --output_dir ./components/modules/model
```

编译并烧录程序到ESP32-S3开发板。

```bash
cd edgelab-example-esp32/examples/meter
idf.py set-target esp32s3
idf.py build
```

## 运行示例

![meter](../_static/esp32/images/meter_reading.gif)

