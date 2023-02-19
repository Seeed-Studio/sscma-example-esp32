# EdgeLab Deployment on Espressif Chipsets


<div align="center">
  <img width="100%" src="https://user-images.githubusercontent.com/20147381/206665275-feceede2-c68c-4259-a4db-541b3bd25b2f.png">
  <h3> <a href="https://edgelab.readthedocs.io/en/latest/"> Documentation </a> | <a href="https://edgelab.readthedocs.io/zh_CN/latest/"> 中文文档 </a>  </h3>
</div>

English | [简体中文](README_zh-CN.md)

- [Introduction](#introduction)
- [How to Install](#how-to-install)
  * [Install the ESP IDF](#install-the-esp-idf)
- [Build the example](#build-the-example)

## Introduction

The project provides examples of how to deploy models from EdgeLab toEspressif chipsets. It is based on the [ESP-IDF](https://github.com/espressif/esp-idf) and [tensorflow lite micro](https://github.com/tensorflow/tflite-micro). 


## How to Install

### Install the ESP IDF

Follow the instructions of the
[ESP-IDF get started guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)
to setup the toolchain and the ESP-IDF itself.

The next steps assume that this installation is successful and the
[IDF environment variables are set](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html#step-4-set-up-the-environment-variables). Specifically,
* the `IDF_PATH` environment variable is set
* the `idf.py` and Xtensa-esp32 tools (e.g., `xtensa-esp32-elf-gcc`) are in `$PATH`

### get submodules
cd to the root directory of the project and run the following command to get the submodules:

```
git submodule init
git submodule update examples/esp32/compoents/esp-nn
git submodule update examples/esp32/compoents/esp32-camera
```


## Build the example

Go to example directory (`examples/<example_name>`) and build the example.

Set the IDF_TARGET (For ESP32-S3 target, IDF version `release/v4.4` is needed)

```
idf.py set-target esp32s3
```

To build this, run:

```
idf.py build
```

### Load and run the example

To flash (replace `/dev/ttyUSB0` with the device serial port):
```
idf.py --port /dev/ttyUSB0 flash
```

Monitor the serial output:
```
idf.py --port /dev/ttyUSB0 monitor
```

Use `Ctrl+]` to exit.

The previous two commands can be combined:
```
idf.py --port /dev/ttyUSB0 flash monitor
```

```{tips}
Please follow example READMEs for more details.
```


### Performance 

The performance of the EdgeLab-related models, measured on different chipsets, is summarized in the following table.

| Target | Model | Dataset | Input Resolution | Peak RAM |Inferencing  Time | F1 Score|Link|
| ---- | -----| ---| ---| -----------| --------| --------| --------|
| ESP32-S3 |          Meter         | [custom](https://files.seeedstudio.com/wiki/Edgelab/meter.zip)|112x112 (RGB)| 320KB |     380ms    |  97% |[pfld_meter_int8.tflite](https://github.com/Seeed-Studio/edgelab-example-esp32/blob/main/model_zoo/pfld_meter_int8.tflite)|
| ESP32-S3  |          Fomo          | [custom]()|96x96 (GRAY)| 244KB |    150ms    |  99.5%|[fomo_mask_int8.tflite](https://github.com/Seeed-Studio/edgelab-example-esp32/blob/main/model_zoo/fomo_mask_int8.tflite)|


### Demonstration

#### Meter Reading

![meter_reading](./docs/_static/images/meter_reading.gif)


## Contributing
- If you find an issue in these examples, or wish to submit an enhancement request, please use the Issues section on Github.
- For ESP-IDF related issues please use [esp-idf](https://github.com/espressif/esp-idf) repo.
- For TensorFlow related information use [tflite-micro](https://github.com/tensorflow/tflite-micro) repo.
- For EdgeLab use [EdgeLab](https://github.com/seeed-studio/EdgeLab) repo.

## License

These examples are covered under MIT License.

These examples use the ESP-IDF which is covered under Apache License 2.0.

TensorFlow library code and third_party code contains their own license specified under respective [repos](https://github.com/tensorflow/tflite-micro).