---
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: 'b8762ebd-76c9-41ca-a2bb-6e1f6f4c271a'
  PropagateID: 'b8762ebd-76c9-41ca-a2bb-6e1f6f4c271a'
  ReservedCode1: '906948f1-5d60-43a9-a1cf-7321b16a049e'
  ReservedCode2: '906948f1-5d60-43a9-a1cf-7321b16a049e'
---

# 无线毫米波雷达智能感知系统 V1.0（Wireless mmWave Radar Smart System）

基于 **24GHz 毫米波雷达（RD-03D）+ ESP32 无线节点**的室内目标感知系统。雷达节点通过 Wi-Fi 将探测目标无线传输至接收端，实现人-雷达-屏幕**零线缆分离部署**，具备警戒区告警、多节点组网、PC Web 可视化、轨迹记录与 CSV 导出等能力，附带**无硬件模拟器**。

> 本仓库是 [Arduino-ESP32-Radarproject](https://github.com/Stevee87/Arduino-ESP32-Radarproject)（MIT License, Copyright (c) 2026 Sudo_solder）的**深度二次开发作品**：在保留原始版权声明的前提下完成代码重构与原创功能增强。

## 功能特性

- **多目标追踪**：RD-03D 雷达最多同时追踪 3 个移动目标，探测距离 8 米、120° 视场
- **多节点组网**：UDP 协议支持最多 8 个雷达节点接入，节点独立分色显示
- **警戒区告警**：矩形警戒区实时判定，目标闯入触发告警（屏幕闪烁 + 蜂鸣 + 事件上报）
- **PC Web 可视化**：零依赖 Python 服务端 + 原生 HTML/Canvas 雷达界面，轨迹回放
- **数据导出**：一键导出 CSV（时间/节点/目标/坐标/速度），支持 Excel 二次分析
- **无硬件演示**：内置数据模拟器，秒级启动全链路演示
- **双端固件**：XIAO ESP32-S3 发射端 + Arduino GIGA R1 触摸屏接收端，随附 3D 打印外壳

## 快速开始（无硬件，1 分钟）

```powershell
cd server
python radar_server.py --simulate --web-port 8081
# 浏览器打开 http://127.0.0.1:8081/
```

有硬件（雷达+ESP32）请阅读 [references/hardware-guide.md](references/hardware-guide.md)。

## 系统架构

```
┌──────────────┐  UART 256000  ┌──────────────────┐  UDP 4210  ┌────────────────────┐
│ RD-03D 雷达  │◄─────────────►│ XIAO ESP32-S3 节点│ ─────────► │ 接收端（二选一）    │
│ 24GHz 多目标 │  帧解析/聚类   │ (发射端固件)      │  无线传输   │ Giga R1 屏显接收端   │
└──────────────┘               └──────────────────┘            │ 或 PC Web 可视化服务 │
                                                                 └────────────────────┘
```

## 目录结构

```
wireless-radar-system/
├── SKILL.md                  # TeleAgent 技能定义
├── LICENSE                   # MIT（含原作者声明）
├── firmware/                 # 双端固件（Arduino）
├── server/                   # PC 服务端 + Web 可视化
├── tools/                    # 无硬件数据模拟器
├── CAD/                      # 3D 打印外壳（4 个 STL）
└── references/               # 部署 / 硬件 / 协议文档
```

## 许可

MIT License — 二次开发声明详见 [LICENSE](LICENSE)。原始项目版权归 Sudo_solder 所有。

## 支持

- 问题反馈：GitHub Issues
- 部署指南：`references/deploy-guide.md`
- 数据协议：`references/protocol.md`