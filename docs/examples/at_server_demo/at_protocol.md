# AT Protocol Specification v2023.9.1


## Link

 - USART Serial (Stateless)


## Design

### Command Lexical Format

- Command header: `AT+`
- Command tag: `{String}`
- Command body: `{String}`
- Command terminator: `\n`

Note: Each character should be a ASCII `char8_t`.

### Command Types

- Read-only operation: `AT+{String}?\n`
- Execute operation: `AT+{String}!` or `AT+{String}={Any},{Any}...\n`
- Config operation: `AT+T{String}={Any}\n`
- Reserved operation: `AT+{String}\n` or `AT+{String}={Any}\n`

### Response Lexical Format

- Response header: `\r`
- Response body: `{JSON:String}`
- Response terminator: `\n`

Note: Each character should be a ASCII `char8_t`.

### Response Types

#### Normal Reply

- Operation response: `\r{JSON:String}\n`
- Event response: `\r{JSON:String}\n`
- Logging response: `\r{JSON:String}\n`

#### Unhandled Reply

- System stdout: `{String}\n...`

### Response Format

```json
{
  "type": {Type:Unsigned},
  "name": "{String}",
  "code": {Code:Integer},
  "data": {Any...}
}   
```

#### Type

| Key | Value              |
|-----|--------------------|
| `0` | Operation response |
| `1` | Event response     |
| `2` | Logging response   |

#### Code

| Key | Value                   |
|-----|-------------------------|
| `0` | Success                 |
| `1` | Try again               |
| `2` | Logic error             |
| `3` | Timeout                 |
| `4` | IO error                |
| `5` | Invalid argument        |
| `6` | Out of memory           |
| `7` | Busy                    |
| `8` | Not supported           |
| `9` | Operation not permitted |


## Policy

### Tagging

All `AT` commands support tagging with a `@` delimiter.

```
AT+{Tag:String}@{Body:String}\n
```

Example request: `AT+10@ID?\n`

Response:

```json
\r{
  "type": 0,
  "name": "10@ID?",
  "code": 0,
  "data": "B63E3DA5"
}\n
```

Note: Using interger or alpha as `Tag` is recommand, conatining any control characters or `"` may break output json format.


### Intereaction Rules

1. **Read-only operation**:
    - Must have a sync/async **Operation reply**.
1. **Execute operation** or **Config operation**:
    - Must have a sync/async **Operation reply**
    - May have a sync/async **Event reply**.
    - Must have a sync **Logging reply** when error occured before the execution.
1. **Reserved operation**:
    - May have sync/async replies with no specified reply types.
1. **Non-Operation** while monitoring the device outputs:
    - May recieve **Event reply**.
    - May recieve **Unhandled reply**.


## Intereaction Examples

### Read-only operation

#### Get device ID

Request: `AT+ID?\n`

Response:

```json
\r{
  "type": 0,
  "name": "ID?",
  "code": 0,
  "data": "B63E3DA5"
}\n
```

#### Get device name

Request: `AT+NAME?\n`

Response:

```json
\r{
  "type": 0,
  "name": "NAME?",
  "code": 0,
  "data": "Seeed Studio XIAO (ESP32-S3)"
}\n
```

#### Get device status

Request: `AT+STAT?\n`

Response:

```json
\r{
  "type": 0,
  "name": "STAT?",
  "code": 0,
  "data": {
    "boot_count": 1520,
    "model": {
      "id": 2,
      "type": 3,
      "address": "0x500000",
      "size": "0x41310"
    },
    "sensor": {
      "id": 1,
      "type": 1,
      "state": 1
    }
  }
}\n
```

#### Get version deatils

Request: `AT+VER?\n`

Response:

```json
\r{
  "type": 0,
  "name": "VER?",
  "code": 0,
  "data": {
    "software": "2023.9.1",
    "hardware": "1"
  }
}\n
```

#### Get available algorithms

Request: `AT+ALGO?\n`

Response:

```json
\r{
  "type": 0,
  "name": "ALGOS?",
  "code": 0,
  "data": [
    {
      "type": 4,
      "categroy": 3,
      "input_from": 1
    },
    {
      "type": 3,
      "categroy": 1,
      "input_from": 1
    },
    {
      "type": 2,
      "categroy": 2,
      "input_from": 1
    },
    {
      "type": 1,
      "categroy": 1,
      "input_from": 1
    }
  ]
}\n
```

#### Get available algorithms

Request: `AT+ALGOS?\n`

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

Request: `AT+MODELS?\n`

Response:

```json
\r{
  "type": 0,
  "name": "MODELS?",
  "code": 0,
  "data": [
    {
      "id": 2,
      "type": 3,
      "address": "0x500000",
      "size": "0x41310"
    },
    {
      "id": 5,
      "type": 3,
      "address": "0x400000",
      "size": "0x41310"
    }
  ]
}\n
```

#### Get available sensors

Request: `AT+SENSORS?\n`

Response:

```json
\r{
  "type": 0,
  "name": "SENSORS?",
  "code": 0,
  "data": [
    {
      "id": 1,
      "type": 1,
      "state": 1
    }
  ]
}\n
```

### Execute operation

#### Load a model by model ID

Pattern: `AT+MODEL=<MODEL_ID>\n`

Request: `AT+MODEL=2\n`

Response:

```json
\r{
  "type": 0,
  "name": "MODEL",
  "code": 0,
  "data": {
    "model": {
      "id": 2,
      "type": 3,
      "address": "0x500000",
      "size": "0x41310"
    }
  }
}\n
```

####  Set a default sensor by sensor ID

Pattern: `AT+SENSOR=<SENSOR_ID,ENABLE/DISABLE>\n`

Request: `AT+SENSOR=1,1\n`

Response:

```json
\r{
  "type": 0,
  "name": "SENSOR",
  "code": 0,
  "data": {
    "sensor": {
      "id": 1,
      "type": 1,
      "state": 1
    }
  }
}\n
```

####  Sample data from current sensor

Pattern: `AT+SAMPLE=<N_TIMES>\n`

Request: `AT+SAMPLE=1\n`

Response:

```json
\r{
  "type": 0,
  "name": "SAMPLE",
  "code": 0,
  "data": {
    "sensor": {
      "id": 1,
      "type": 1,
      "state": 1
    }
  }
}\n
```

Events:

```json
\r{
  "type": 1,
  "name": "SAMPLE",
  "code": 0,
  "data": {
    "jpeg": "{BASE64JPEG:String}"
  }
}\n
```

#### Invoke for N times

Pattern: `AT+INVOKE=<N_TIMES,RESULT_ONLY>\n`

Request: `AT+INVOKE=1,1\n`

Response:

```json
\r{
  "type": 0,
  "name": "INVOKE",
  "code": 0,
  "data": {
    "model": {
      "id": 2,
      "type": 3,
      "address": "0x500000",
      "size": "0x41310"
    },
    "algorithm": {
      "type": 3,
      "category": 1,
      "config": {
        "score_threshold": 60,
        "iou_threshold": 50
      }
    },
    "sensor": {
      "id": 1,
      "type": 1,
      "state": 1
    }
  }
}\n
```

Events:

```json
\r{
  "type": 1,
  "name": "INVOKE",
  "code": 0,
  "data": {
    "perf": [
      8,
      365,
      0
    ],
    "boxes": [
      [
        87,
        83,
        77,
        65,
        0,
        70
      ]
    ]
  }
}\n
```
#### Set a condition action trigger

Pattern: `AT+ACTION=<"COND","TRUE_CMD","FALSE_OR_EXCEPTION_CMD">\n`

Request: `AT+ACTION="count(id,0)>=3","LED=1","LED=0"\n`

Response:

```json
\r{
  "type": 0,
  "name": "ACTION",
  "code": 0,
  "data": {
    "cond": "count(id,0)>=3",
    "true": "LED=1",
    "false_or_exception": "LED=0"
  }
}\n
```
Events:

```json
\r{
  "type": 1,
  "name": "ACTION",
  "code": 0,
  "data": {
    "true": "AT+LED=1"
  }
}\n
```

Note: Only have events reply when condition evaluation is `true`.

### Config operation

#### Set score threshold

Pattern: `AT+TSCORE=<SCORE_THRESHOLD>\n`

Request: `AT+TSCORE=60\n`

Response:

```json
\r{
  "type": 0,
  "name": "TSCORE",
  "code": 0,
  "data": 60
}\n
```

Note: Available while invoking using a specified algorithm.

#### Set IoU threshold

Pattern: `AT+TIOU=<IOU_THRESHOLD>\n`

Request: `AT+TIOU=55\n`

Response:

```json
\r{
  "type": 0,
  "name": "TIOU",
  "code": 0,
  "data": 55
}\n
```

Note: Available while invoking using a specified algorithm.

### Reserved operation

#### Set LED status

Pattern: `AT+LED=<ENABLE/DISABLE>\n`

Request: `AT+LED=1\n`

No-reply.

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

#### Yield I/O task for 10ms

Request: `AT+YIELD\n`

No-reply.

## Patterns

### Status

```json
"{Key:String}": "{Value:String}"
```

Key: `status`

Values:

- `ok`
- `try again`
- `logic error`
- `timeout`
- `input/output error`
- `invalid argument`
- `out of memory`
- `busy`
- `not supported`
- `operation not permitted`
- `unknown`

### Algorithm Types

```json
"{Key:String}": "{Value:String}"
```

Key: `type`

Values:

- `FOMO`
- `PFLD`
- `YOLO`
- `IMCLS`
- `undefined`

### Algorithm Categories

```json
"{Key:String}": "{Value:String}"
```

Key: `categroy`

Values:

- `detection`
- `pose`
- `classification`
- `undefined`

### Sensor Types

```json
"{Key:String}": "{Value:String}"
```

Key: `type`

Values:

- `camera`
- `undefined`

### Sensor State

```json
"{Key:String}": "{Value:String}"
```

Key: `state`

Values:

- `registered`
- `available`
- `locked`
- `unknown`

### Image Types

```json
"{Key:String}": "{Data:String}"
```

Key:

- `grayscale`
- `jpeg`
- `rgb565`
- `rgb888`
- `yuv422`
- `undefined`

Data: `BASE64`

### Box Type

```json
"{Key:String}": ["{Value:JSON}"]
```

Key: `boxes`

Value:

```json
{
    "x": 75,
    "y": 73,
    "w": 27,
    "h": 45,
    "target": 0,
    "score": 81
}
```

### Point Type

```json
"{Key:String}": ["{Value:JSON}"]
```

Key: `points`

Value:

```json
{
    "x": 75,
    "y": 73,
    "target": 0
}
```

### Class Type

```json
"{Key:String}": ["{Value:JSON}"]
```

Key: `classes`

Value:

```json
{
    "target": 0,
    "score": 81
}
```
