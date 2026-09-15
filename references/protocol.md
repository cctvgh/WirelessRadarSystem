---
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: 'fb770e5b-8f00-4d1e-bbb1-88ac91fa3b59'
  PropagateID: 'fb770e5b-8f00-4d1e-bbb1-88ac91fa3b59'
  ReservedCode1: 'ccf5ac3a-8de8-4e19-a242-96e7e08e9d3e'
  ReservedCode2: 'ccf5ac3a-8de8-4e19-a242-96e7e08e9d3e'
---

# RD-03D 数据协议与 UDP 封包格式

本文件定义无线毫米波雷达智能感知系统 V1.0 的完整数据链路协议，供二次开发与排障参考。

## 1. RD-03D 雷达串口协议

RD-03D（Ai-Thinker，24GHz FMCW/多普勒毫米波雷达）通过 UART 输出目标数据帧，波特率 **256000**。

### 1.1 数据帧结构

| 段 | 长度 | 说明 |
|----|------|------|
| 帧头 | 4 字节 | `AA FF 03 00` |
| 载荷 | 26 字节 | 3 组目标数据（每组 8 字节）+ 2 字节尾校验 |
| 帧尾 | 2 字节 | `55 CC` |

### 1.2 单目标数据（8 字节）

| 偏移 | 长度 | 字段 | 说明 |
|------|------|------|------|
| 0-1 | 2B | X | 横向坐标（毫米），有符号 16 位补码 |
| 2-3 | 2B | Y | 纵向距离（毫米），有符号 16 位补码 |
| 4-5 | 2B | SPD | 速度（毫米/秒），有符号 |
| 6-7 | 2B | 预留 | 恒为 0 |

**空目标**：8 字节全零表示该目标槽无有效数据。

**解码规则**（与固件 `decodeSigned` 一致）：

```
raw = lo | (hi << 8)
value = raw & 0x8000 ? (int16_t)(raw | 0xFFFF0000) : (int16_t)raw   # 补码符号扩展
```

**坐标说明**：X 为水平偏移（左负右正），Y 为径向距离（正前方）。距离过滤：仅接受 **100mm < 距离 ≤ 8000mm** 的有效目标。

### 1.3 指令帧（配置雷达）

| 指令 | 内容 | 用途 |
|------|------|------|
| 启用 | `FD FC FB FA 04 00 FF 00 01 00 04 03 02 01` | 启用雷达 |
| 多目标 | `FD FC FB FA 02 00 90 00 04 03 02 01` | 进入多目标模式 |
| 结束 | `FD FC FB FA 02 00 FE 00 04 03 02 01` | 结束指令序列 |

发射端固件在启动时依次下发三条指令，此后每 60 秒重复下发一次，防止雷达退出多目标模式。

## 2. UDP 无线传输协议

### 2.1 网络拓扑

```
发射节点 (XIAO ESP32-S3)  ——WiFi客户端——►  接收端热点 (AP)
   UDP 发送端口 = 4210          ◄——网关 IP 推断——  SSID: RadarNet
                                  密码: radar12345
```

发射端作为 STA 连接接收端建立的 AP 热点，**接收端 IP 通过 DHCP 网关地址自动获取**（`WiFi.gatewayIP()`）。

### 2.2 数据包（41 字节, packed, little-endian）

```
偏移    类型      字段          说明
0      uint32    magic         协议魔数 0x0D03DA7A（校验用）
4      uint32    nodeId        节点编号（1 起，多节点组网区分）
8      uint32    frameCount    帧计数
12     uint32    uptimeMs      节点运行毫秒
16     int16     t0.x          目标0 X
18     int16     t0.y          目标0 Y
20     int16     t0.spd        目标0 速度
22     uint8     t0.valid      目标0 有效标记
23     int16     t1.x          目标1 X
25     int16     t1.y          目标1 Y
27     int16     t1.spd        目标1 速度
29     uint8     t1.valid      目标1 有效标记
30     int16     t2.x          目标2 X
32     int16     t2.y          目标2 Y
34     int16     t2.spd        目标2 速度
36     uint8     t2.valid      目标2 有效标记
37     uint8     flags         状态位（bit0=节点在线）
38-40  uint8[3]  reserved      预留
```

Python 端解包模板（与 `radar_server.py` 一致）：

```python
PACKET_FMT = "<IIIIhhhBhhhBhhhBB3s"
```

### 2.3 心跳机制

发射端在超过 2 秒无目标数据时发送 **心跳包**（目标全零、`flags=0x01`），使接收端能持续感知节点在线状态。接收端超过 3 秒未收到任何数据包即判定节点离线。

## 3. 警戒区判定

警戒区为雷达坐标系中的矩形区域（毫米）：

```
X ∈ [-4000, 4000]    （横向）
Y ∈ [200, 4000]     （纵向：200mm 起避免贴脸误报）
```

任一台法目标落入矩形即触发警戒告警；目标全部离开后告警复位。判定逻辑在 Giga 固件与 PC 服务端中一致实现，事件以 JSON 上报：

```json
{"event":"alarm_enter","node":1}
{"event":"alarm_exit","node":1}
{"event":"node_lost","node":2}
```

## 4. 自定义扩展指引

- 新增传感器类型：在 `Target` 结构后扩展字段，同步更新两端 `struct` 定义与大小校验
- 调整警戒区：修改 `ALARM_ZONE_*` 宏（固件）与 `ALARM_ZONE_*` 常量（server.py）
- 修改组网凭据：修改 `WIFI_SSID`/`WIFI_PASSWORD`（两端固件），并保持一致