# SSCMA 在 Espressif 芯片上的部署

<div align="center">
  <img width="100%" src="https://user-images.githubusercontent.com/20147381/206665275-feceede2-c68c-4259-a4db-541b3bd25b2f.png">
</div>

[English](README.md) | 简体中文

## 简介

该项目提供了如何将 SSCMA 中的模型部署到 Espressif 芯片组的示例。它基于 [ESP-IDF](https://github.com/espressif/esp-idf) 和 [TFLite-Micro](https://github.com/tensorflow/tflite-micro)。

## 开始使用

### 安装 ESP IDF

按照这个指南中的说明进行操作：[ESP-IDF - 入门指南](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)，以设置 SSCMA 示例使用的构建工具链。目前我们使用的是最新版本 `v5.3`。

### 克隆并设置仓库

1. 克隆我们的仓库。

    ```sh
    git clone https://github.com/Seeed-Studio/sscma-example-esp32
    ```

2. 进入 `sscma-example-esp32` 文件夹。

    ```sh
    cd sscma-example-esp32
    ```

3. 获取子模块。

    ```sh
    git submodule update --init
    ```

### 构建和运行示例

1. 进入示例文件夹并列出所有可用的示例。

    ```sh
    cd examples && \
    ls
    ```

2. 选择一个 `<demo>` 并进入其文件夹。

    ```sh
    cd '<demo>'
    ```

3. 使用 ESP-IDF 生成构建配置。

    目前我们在 [XIAO-ESP32S3](https://www.seeedstudio.com/XIAO-ESP32S3-p-5627.html) 模块上进行开发和测试。

    ```sh
    # 设置构建目标
    idf.py set-target esp32s3
    ```

    您可以使用 Menuconfig 更改显示驱动程序，启用或禁用 TFLite 运算符，如果需要的话。

    ```sh
    # 更改设备或演示特定的配置
    idf.py menuconfig
    ```

4. 构建示例固件。

    ```sh
    idf.py build
    ```

5. 将示例固件烧录到设备并运行。

    要烧录固件（目标串口可能因操作系统而异，请用您的设备串口替换 `/dev/ttyACM0`）。

    ```
    idf.py --port /dev/ttyACM0 flash
    ```

    监视串口输出。

    ```
    idf.py --port /dev/ttyACM0 monitor
    ```

#### 提示

- 使用 `Ctrl+]` 退出监视。

- 前面两个命令可以合并。

    ```sh
    idf.py --port /dev/ttyACM0 flash monitor
    ```

### 支持的模型和性能

请参阅 [SSCMA 模型仓库](https://github.com/Seeed-Studio/sscma-model-zoo) 获取详细信息。

## 贡献

- 如果您在使用这些示例时遇到任何问题，或希望提交增强请求，请使用 [问题](https://github.com/Seeed-Studio/sscma-example-esp32/issues) 或提交 [拉取请求](https://github.com/Seeed-Studio/sscma-example-esp32/pulls)。

## 许可证

```
本项目根据 Apache 2.0 许可证进行许可。请参阅 LICENSE 了解详细信息。
```
