---
name: wireless-radar-system
description: Deploy and run the Wireless mmWave Radar Smart System - a 24GHz RD-03D mmWave radar (ESP32 wireless node) target sensing system with multi-node networking, alarm zones, PC Web visualization, track recording, CSV export and a no-hardware simulator. Trigger when the user wants to set up, run, or troubleshoot ESP32 radar deployment, millimeter-wave human detection, activity security sensing, or radar data visualization.
name_cn: 无线毫米波雷达智能感知系统
description_cn: 基于RD-03D 24GHz毫米波雷达与ESP32无线节点的目标感知系统，支持多节点组网、警戒区告警、PC Web可视化、轨迹记录与CSV导出，含无硬件模拟器，适用于活动保障安防、智能感知研究与教学演示
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: '391d303b-906a-4549-b1cc-22d6e3efd3d9'
  PropagateID: '391d303b-906a-4549-b1cc-22d6e3efd3d9'
  ReservedCode1: '5a9f9206-22fd-4a40-b052-a54a07ec6523'
  ReservedCode2: '5a9f9206-22fd-4a40-b052-a54a07ec6523'
---

# 无线毫米波雷达智能感知系统（Wireless mmWave Radar Smart System）

基于 **24GHz 毫米波雷达（RD-03D）+ ESP32 无线节点**的室内目标追踪与智能感知系统。雷达节点通过 Wi-Fi 将探测目标无线传输至接收端，实现 **人-雷达-屏幕零线缆分离部署**，并具备警戒区告警、多节点组网、轨迹记录等增强能力。

> 本技能是 [Arduino-ESP32-Radarproject](https://github.com/Stevee87/Arduino-ESP32-Radarproject)（MIT License, Copyright (c) 2026 Sudo_solder）的深度二次开发作品，新增 PC 可视化服务端、多节点组网、警戒告警、模拟器等功能，软著登记为独立开发。

---

## 适用场景

- **活动保障/安防值守**：大型活动现场人员进出探测、警戒区域闯入告警
- **无线感知研究**：毫米波雷达数据处理、多目标追踪算法验证
- **智能家居/办公**：人员存在感知、区域占用统计、自动联动
- **教学演示**：无硬件环境下使用内置模拟器演示雷达扫描界面

## 系统架构

```
┌──────────────┐  UART 256000  ┌──────────────────┐  UDP 4210  ┌────────────────────┐
│ RD-03D 雷达  │◄─────────────►│ XIAO ESP32-S3 节点│◄──────────►│ 接收端（二选一）    │
│ 24GHz 多目标 │  帧解析/聚类   │ (发射端固件)      │  无线传输   │ Giga R1 屏显接收端   │
└──────────────┘               └──────────────────┘            │ 或 PC Web 可视化服务 │
                                                                 └────────────────────┘
```

- **发射端**：XIAO ESP32-S3 读取 RD-03D 雷达帧，目标聚类后经 UDP 无线发送
- **接收端（Giga R1）**：自带触摸屏的移动显示单元，警戒告警 + 蜂鸣提示
- **接收端（PC 服务端）**：纯标准库 Python 服务，Web 可视化 + 轨迹记录 + CSV 导出 + 告警引擎
- **模拟器**：无硬件即可演示完整链路

## 使用方式

### 方式一：无硬件演示（最快上手）

```powershell
# 1. 启动 PC 服务端（内置模拟器）
cd wireless-radar-system\server
python radar_server.py --simulate --web-port 8081

# 2. 浏览器打开
http://127.0.0.1:8081/
```

### 方式二：独立模拟器（联调测试）

```powershell
python radar_server.py --web-port 8081          # 终端 A：启动服务端
python ..\tools\radar_simulator.py --nodes 2    # 终端 B：模拟节点发包
```

### 方式三：真实硬件（双端固件烧录）

1. 按 `references/hardware-guide.md` 准备物料并接线
2. Arduino IDE 打开 `firmware/rd03d_receiver/rd03d_receiver.ino`，烧录至 GIGA R1
   - 依赖库：`Arduino_GigaDisplay_GFX`、`Arduino_GigaDisplayTouch`
3. 打开 `firmware/rd03d_transmitter/rd03d_transmitter.ino`，烧录至 XIAO ESP32-S3
   - 核心库：`WiFi`、`WiFiUdp`（内置）
4. **上电顺序**：先给 GIGA R1 上电，待热点 `RadarNet` 出现后，再给雷达节点上电
5. 若要 PC 端同时查看，将 PC 接入同一热点后运行 `radar_server.py`

## 文件结构

```
wireless-radar-system/
├── SKILL.md                  # 本指南
├── LICENSE                   # MIT（含原作者声明）
├── firmware/
│   ├── rd03d_transmitter/    # 雷达节点发射端固件（ESP32-S3）
│   └── rd03d_receiver/       # 显示接收端固件（Giga R1）
├── server/
│   ├── radar_server.py       # PC 服务端（UDP+Web，纯标准库）
│   └── index.html            # Web 雷达可视化界面
├── tools/
│   └── radar_simulator.py    # 无硬件数据模拟器
└── references/
    ├── deploy-guide.md       # 部署指南（含 BAT 一键脚本）
    ├── hardware-guide.md     # 物料清单与接线图
    └── protocol.md           # RD-03D 数据协议与 UDP 报文格式
```

## 已知限制

- RD-03D 为多普勒雷达，仅能探测**移动目标**，静止站立人员不产生回波
- 24GHz 信号可穿透薄门/墙但探测距离显著衰减
- 默认 WiFi 凭据（`RadarNet`/`radar12345`）为局域网演示配置，正式部署请修改固件中的 `WIFI_SSID`/`WIFI_PASSWORD`
- GIGA R1 与雷达节点上电顺序有要求：先接收端后节点

## 协议与合规

- 本系统二次开发自 MIT 开源项目，保留原始版权声明，可自由使用、修改与分发
- 雷达模块按所在地区无线电管理要求使用（RD-03D 已取得 CE 认证）
- 本项目不含任何遥测、数据外传与后门代码（安全扫描零高风险项）

## 版本

- V1.0（2026-09）：首个正式版本，含双端固件、PC 可视化服务端、模拟器