# SSCMA Deployment on Espressif Chipsets

<div align="center">
  <img width="100%" src="https://cdn.jsdelivr.net/gh/Seeed-Studio/SSCMA-Micro@dev/docs/images/sscma.png">
</div>



English | [简体中文](README_zh-CN.md)

## Introduction

The project provides examples of how to deploy models from SSCMA to Espressif chipsets. It is based on the [ESP-IDF](https://github.com/espressif/esp-idf) and [TFLite-Micro](https://github.com/tensorflow/tflite-micro).


## Getting Started

### Install ESP IDF

Follow instructions in this guide
[ESP-IDF - Get Started](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)
to setup the built toolchain used by SSCMA examples. Currently we're using the latest version `v5.1`.

### Clone and Setup the Repository

1. Clone our repository.

    ```sh
    git clone https://github.com/Seeed-Studio/sscma-example-esp32
    ```

2. Go to `sscma-example-esp32` folder.

    ```sh
    cd sscma-example-esp32
    ```

3. Fetch the submodules.

    ```sh
    git submodule update --init
    ```

### Build and Run Examples

1. Go to examples folder and list all available examples.

    ```sh
    cd examples && \
    ls
    ```

2. Choose a `<demo>` and enter its folder.

    ```sh
    cd '<demo>'
    ```

3. Generate build config using ESP-IDF.

    Currently we develop and test on [XIAO-ESP32S3](https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html) module.

    ```sh
    # set build target
    idf.py set-target esp32s3
    ```

    You can change a display driver, enable or disable a TFLite operator if it need using menuconfig.

    ```sh
    # change device or demo specific configuration
    idf.py menuconfig
    ```

4. Build the demo firmware.

    ```sh
    idf.py build
    ```

5. Flash the demo firmware to device and Run.

    To flash (the target serial port may vary depend on your operating system, please replace `/dev/ttyACM0` with your device serial port).

    ```
    idf.py --port /dev/ttyACM0 flash
    ```

    Monitor the serial output.

    ```
    idf.py --port /dev/ttyACM0 monitor
    ```

#### Tip

- Use `Ctrl+]` to exit monitor.

- The previous two commands can be combined.

    ```sh
    idf.py --port /dev/ttyACM0 flash monitor
    ```


### Supported Models and Performance

Please refer to [SSCMA Model Zoo](https://github.com/Seeed-Studio/sscma-model-zoo) for details.


## Contributing

- If you find any issue in using these examples, or wish to submit an enhancement request, please use the raise a [Issue](https://github.com/Seeed-Studio/sscma-example-esp32/issues) or submit a [Pull Request](https://github.com/Seeed-Studio/sscma-example-esp32/pulls).


## License

```
These examples are covered under MIT License.

These examples use the ESP-IDF which is covered under Apache License 2.0.

TensorFlow, FlashDB, JPEGENC and other third-party libraries are distribute under their own License.
```
