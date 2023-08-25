# AT Protocol Specification v0.0.2

## Link

 - USART Serial

## Stracture

### Command

- Command header: `AT+`
- Command tag: `{String}`
- Command body: `{String}`
- Command terminator: `\n`

#### Tagging Policy

All `AT` commands support tagging with a `|` delimiter.

```
AT+{Tag:String}|{Body:String}\n
```

Example request: `AT+10|ID?\n`

Response:

```json
\r{
  "10|ID?": "B63E3DA5"
}\n
```

## Types

### Command Types

- Read-only operation: `AT+{String}?\n`
- Execute operation: `AT+{String}={String},{String}...\n`
- Config operation: `AT+T{String}\n`
- Reserved operation: `AT+{String}\n`

### Response Tyeps

- Operation reply: `\r{Json}\n`
- Event reply: `\r{Json}\n`
- Logging reply: `{String}`
- Normal reply: `> {String}\n`

### Rules

1. **Read-only operation** must have a sync/async **Operation reply**.
1. **Execute operation** must have a sync/async **Operation reply**, may have multiple async **Logging reply** and **Event reply**, **Logging reply** should be off in production case.
1. **Config operation** has no reply.
1. **Reserved operation** may have sync/async replies with no specified reply types. 
1. You may recieve **Logging reply** when monitor the device.

## Intereaction

### Read-only operation

#### Get device ID

Request: `AT+ID?\n`

Response:

```json
\r{
  "ID?": "B63E3DA5"
}\n
```

#### Get device name

Request: `AT+NAME?\n`

Response:

```json
\r{
  "NAME?": "Seeed Studio XIAO (ESP32-S3)"
}\n
```

#### Get device status

Request: `AT+STAT?\n`

Response:

```json
\r{
  "STAT?": {
    "status": "ok",
    "boot_count": 594,
    "current_model_id": 2,
    "current_sensor_id": 1
  }
}\n
```

#### Get version deatils

Request: `AT+VER?\n`

Response:

```json
\r{
  "VER?": {
    "edgelab_cpp_sdk": "0.0.2",
    "chip_revision": "1"
  }
}\n
```

#### Get available algorithms

Request: `AT+ALGO?\n`

Response:

```json
\r{
  "ALGO?": [
    {
      "type": "IMCLS",
      "categroy": "classification",
      "input_from": "camera"
    },
    {
      "type": "YOLO",
      "categroy": "detection",
      "input_from": "camera"
    },
    {
      "type": "PFLD",
      "categroy": "pose",
      "input_from": "camera"
    },
    {
      "type": "FOMO",
      "categroy": "detection",
      "input_from": "camera"
    }
  ]
}\n
```

#### Get available algorithms

Request: `AT+ALGO?\n`

Response:

```json
\r{
  "ALGO?": [
    {
      "type": "IMCLS",
      "categroy": "classification",
      "input_from": "camera"
    },
    {
      "type": "YOLO",
      "categroy": "detection",
      "input_from": "camera"
    },
    {
      "type": "PFLD",
      "categroy": "pose",
      "input_from": "camera"
    },
    {
      "type": "FOMO",
      "categroy": "detection",
      "input_from": "camera"
    }
  ]
}\n
```

#### Get available models

Request: `AT+MODEL?\n`

Response:

```json
\r{
  "MODEL?": [
    {
      "id": 2,
      "type": "YOLO",
      "address": "0x500000",
      "size": "0x41310"
    }
  ]
}\n
```

#### Get available sensors

Request: `AT+SENSOR?\n`

Response:

```json
\r{
  "SENSOR?": [
    {
      "id": 1,
      "type": "camera",
      "state": "available"
    }
  ]
}\n
```

### Execute operation

#### Load a model by model ID

Pattern: `AT+MODEL=<SENSOR_ID>\n`

Request: `AT+MODEL=2\n`

Response:

```json
\r{
  "MODEL": {
    "id": 2,
    "status": "ok"
  }
}\n
```

####  Set a default sensor by sensor ID

Pattern: `AT+SENSOR=<SENSOR_ID,ENABLE/DISABLE>\n`

Request: `AT+SENSOR=1,1\n`

Response:

```json
\r{
  "SENSOR": {
    "id": 1,
    "status": "ok"
  }
}\n
```

####  Sample data from current sensor

Pattern: `AT+SAMPLE=<N_TIMES>\n`

Request: `AT+SAMPLE=1\n`

Response:

```json
\r{
  "SAMPLE": {
    "status": "ok",
    "sensor_id": 1
  }
}\n
```

Events:

```json
\r{
  "event": {
    "from": "SAMPLE",
    "status": "ok",
    "contents": {
      "jpeg": "{BASE64JPEG:String}"
    }
  },
  "timestamp": 1867724
}\n
```

####  Invoke for N times

Pattern: `AT+INVOKE=<N_TIMES,RESULT_ONLY>\n`

Request: `AT+INVOKE=1,1\n`

Response:

```json
\r{
  "INVOKE": {
    "status": "ok",
    "model_id": 2,
    "model_type": "YOLO",
    "algorithm_category": "detection",
    "algorithm_config": {
      "score_threshold": 60,
      "iou_threshold": 45
    },
    "sensor_id": 1,
    "sensor_type": "camera",
    "sensor_state": "available"
  }
}\n
```

Events:

```json
\r{
  "event": {
    "from": "INVOKE",
    "status": "ok",
    "contents": {
      "model_id": 2,
      "sensor_id": 1,
      "preprocess_time": 14,
      "run_time": 368,
      "postprocess_time": 1,
      "boxes": [
        {
          "x": 75,
          "y": 73,
          "w": 27,
          "h": 45,
          "target": 0,
          "score": 81
        }
      ],
      "roi": [
        240,
        240
      ]
    }
  },
  "timestamp": 2155621
}\n
```


### Config operation

#### Set score threshold

Pattern: `AT+TSCORE=<SCORE_THRESHOLD>\n`

Request: `AT+TSCORE=60\n`

No-reply.

Note: Available while invoking using a specified algorithm.

#### Set IoU threshold

Pattern: `AT+TIOU=<IOU_THRESHOLD>\n`

Request: `AT+TIOU=45\n`

No-reply.

Note: Available while invoking using a specified algorithm.


### Reserved operation

#### List available commands

Request: `AT+HELP\n`

Response:

```
Command list:
  AT+INVOKE=<N_TIMES,RESULT_ONLY>
    Invoke for N times (-1 for infinity loop)
  {String}...
```

#### Reboot device

Request: `AT+RST\n`

No-reply.

#### Stop all running tasks

Request: `AT+BREAK\n`

No-reply.
