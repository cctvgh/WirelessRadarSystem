---
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: '7a103c28-e777-4f04-b889-d0c2404a3aee'
  PropagateID: '7a103c28-e777-4f04-b889-d0c2404a3aee'
  ReservedCode1: '4de7e3fe-066c-43c6-97f3-1e9d7269c627'
  ReservedCode2: '4de7e3fe-066c-43c6-97f3-1e9d7269c627'
---

# 硬件物料清单与接线指南

本文档为无线毫米波雷达智能感知系统 V1.0 的完整硬件部署说明。系统分为两个节点：

- **雷达节点（发射端）**：XIAO ESP32-S3 + RD-03D 雷达，电池供电，无线发射目标数据
- **显示节点（接收端）**：Arduino GIGA R1 + 显示扩展板，或 PC 运行服务端

## 1. 物料清单

### 雷达节点（发射端）

| 数量 | 物料 | 说明 |
|------|------|------|
| 1 | Seeed Studio XIAO ESP32-S3 | 主控，WiFi/BLE 双模 |
| 1 | RD-03D 雷达模块（安信可） | 24GHz 毫米波，3 目标，8m 探测 |
| 1 | 3.7V 锂聚合物电池 602560（约 1300mAh） | 雷达侧供电 |
| 1 | 拨动开关 | 电源开关 |
| 1 | MT3608 升压模块 | 3.7V → 5V 供雷达 |
| 若干 | 杜邦线/焊接线 | 连接线 |

> 雷达节点可直接用 XIAO 的 USB-C 充电，无需额外充电板。

### 显示节点（接收端，Giga R1 方案）

| 数量 | 物料 | 说明 |
|------|------|------|
| 1 | Arduino GIGA R1 | 主控，自带 WiFi（R4 无线模组） |
| 1 | GIGA Display Shield | 3.97 寸触摸屏 800x480 |
| 1 | 3.7V 锂电 1160100 ≥ 5000mAh | 显示侧供电 |
| 1 | 开关 | 电源开关 |
| 1 | MT3608 升压模块 | 3.7V → 6V 供 GIGA VIN |
| 1 | BW4056 USB-C 充电板 | 显示侧充电 |
| 1 | 无源压电蜂鸣器模块 | 距离感知/告警提示音 |
| 1 | Mini 电量指示板（1S Li-ion） | 电池电量显示（可选） |

## 2. 接线说明

### 2.1 雷达节点接线（XIAO ESP32-S3 ↔ RD-03D）

| RD-03D | XIAO ESP32-S3 | 说明 |
|--------|---------------|------|
| TX | D0（GPIO1） | UART1 RX（波特率 256000） |
| RX | D1（GPIO2） | UART1 TX |
| VCC | 5V | 由升压模块供电 |
| GND | GND | 共地 |

> **警告**：不要使用 D6/D7（GPIO43/44），它们被 USB-Serial（UART0）占用。

雷达节点供电链路：`锂电池 → 开关 → MT3608(5V) → XIAO 5V 与雷达 VCC`。

### 2.2 显示节点供电

Giga R1 侧：`LiPo(5000mAh) → 开关 → 升压(6V) → GIGA VIN`。

### 2.3 蜂鸣器接线（显示节点）

| 蜂鸣器 | GIGA R1 |
|--------|---------|
| VCC | 5V |
| Signal | D9 |
| GND | GND |

> 必须使用 **无源** 压电蜂鸣器。有源蜂鸣器自带振荡器，无法响应固件发送的频率参数。

### 2.4 网络参数（默认值）

| 参数 | 值 |
|------|-----|
| AP SSID | `RadarNet` |
| AP 密码 | `radar12345` |
| UDP 端口 | `4210` |
| GIGA IP | `192.168.4.1`（AP 网关） |

## 3. 固件烧录

### 3.1 安装 Arduino IDE 依赖

**显示节点（Giga R1）**：
1. 开发板管理器安装 `Arduino Mbed OS GIGA Boards`
2. 库管理器安装 `Arduino_GigaDisplay_GFX`、`Arduino_GigaDisplayTouch`

**雷达节点（XIAO ESP32-S3）**：
1. 开发板管理器安装 `esp32`（Espressif）
2. 选择板型 `XIAO ESP32-S3`（Seeed 支持包）

### 3.2 烧录步骤

```
1. 打开 firmware/rd03d_receiver/rd03d_receiver.ino，烧录到 GIGA R1
2. 打开 firmware/rd03d_transmitter/rd03d_transmitter.ino，烧录到 XIAO ESP32-S3
3. 若 XIAO 为全新板，需先按住 BOOT 键插入 USB 进入下载模式
```

### 3.3 上电顺序（关键）

1. **先给 GIGA R1 上电**，等待热点 `RadarNet` 出现（约 8 秒）
2. 再给雷达节点上电，节点自动连接热点并开始上报

> 顺序颠倒会导致 Wi-Fi 连接失败，需重启节点。

## 4. 3D 打印外壳

原项目提供 4 个 STL 文件（位于原始仓库 `CAD files/`，本技能发布包已包含）：

| 文件 | 尺寸 (W×D×H) | 用途 |
|------|-------------|------|
| `body display.stl` | 31 × 83 × 109 mm | 显示端机身 |
| `top display.stl` | — | 显示端顶盖 |
| `bottom radarmodule.stl` | 29 × 67 × 30 mm | 雷达端底壳 |
| `top radarmodule.stl` | 29 × 64 × 27 mm | 雷达端顶盖 |

## 5. 安全注意事项

- 锂电池接线务必核对极性，充电时关闭电源开关，充电过程有人值守
- 24GHz 频段按所在地无线电管理要求使用（RD-03D 已取得 CE 认证）
- 雷达可穿透薄墙/门探测运动目标，部署时注意隐私边界
- 本项目按 MIT 许可提供，无任何明示或默示担保，风险自担