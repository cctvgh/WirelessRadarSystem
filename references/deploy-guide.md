---
AIGC:
  ContentProducer: '001191110102MAD55U9H0F10002'
  ContentPropagator: '001191110102MAD55U9H0F10002'
  Label: '1'
  ProduceID: '3113197a-09b9-4b55-b35b-a33c7c303491'
  PropagateID: '3113197a-09b9-4b55-b35b-a33c7c303491'
  ReservedCode1: 'ce95500b-d343-4c40-bd1f-807c65dc4cb3'
  ReservedCode2: 'ce95500b-d343-4c40-bd1f-807c65dc4cb3'
---

# 部署指南

本文档覆盖三种部署形态：**纯 PC 演示**（无硬件）、**硬件双节点**、**PC 接收端与硬件共存**，并提供一键启动脚本。

## 1. 环境要求

| 项 | 要求 |
|----|------|
| Python | 3.8+（Windows/Linux/macOS 均可） |
| 依赖 | **零第三方依赖**（仅标准库） |
| 浏览器 | Chrome / Edge / Firefox（支持 Canvas 与 Fetch） |
| 硬件（可选） | XIAO ESP32-S3 + RD-03D、Arduino GIGA R1（需烧录固件） |

> 无 Python 环境时也可直接双击 `server/index.html` 查看界面，但无实时数据。

## 2. 方式一：纯 PC 演示（无需任何硬件）

### 2.1 启动服务端（内置模拟器）

```powershell
cd wireless-radar-system\server
python radar_server.py --simulate --web-port 8081
```

启动后终端显示：

```
============================================================
 无线毫米波雷达智能感知系统 V1.0 - PC 服务端
============================================================
 UDP 监听端口 : 4210
 Web 界面     : http://127.0.0.1:8081/
```

### 2.2 访问可视化界面

浏览器打开 `http://127.0.0.1:8081/`，可看到：

- 雷达扫描动画、120° 视场、距离圈（0.5m–8m）
- 2 个模拟节点、4 个运动目标及轨迹
- 警戒区（黄色虚线矩形）与目标闯入告警（红色横幅 + 蜂鸣）
- 节点状态、目标数、帧率实时统计
- 「导出 CSV」下载全部轨迹数据

## 3. 方式二：有硬件双节点部署

### 3.1 烧录固件

按 `references/hardware-guide.md` 完成接线与烧录。

### 3.2 上电

1. 先给 GIGA R1（接收端）上电，等待热点 `RadarNet` 出现
2. 再给 XIAO ESP32-S3（雷达节点）上电，节点自动连接并上报
3. GIGA 触摸屏显示雷达界面，默认蜂鸣器开启，点击「BUZZER AN」静音

### 3.3 可选：PC 接收

若希望 PC 端同时接收与记录，将 PC 接入同一 WiFi（`RadarNet`），然后：

```powershell
python radar_server.py --web-port 8081
```

PC 通过热点直接接收节点 UDP 数据（默认端口 4210）。

## 4. 方式三：独立模拟器联调

终端 A（服务端）与终端 B（模拟器）分开运行：

```powershell
# 终端 A
python radar_server.py --web-port 8081

# 终端 B
python ..\tools\radar_simulator.py --host 127.0.0.1 --port 4210 --nodes 2 --targets 3
```

模拟器参数：

| 参数 | 默认 | 说明 |
|------|------|------|
| `--host` | 127.0.0.1 | 服务端 IP |
| `--port` | 4210 | UDP 端口 |
| `--nodes` | 2 | 模拟节点数（1-8） |
| `--targets` | 2 | 每节点活动目标数 |
| `--fps` | 10 | 发送帧率 |

## 5. 一键启动脚本（Windows）

创建 `启动雷达演示.bat`（UTF-8 编码）到项目根目录：

```bat
@echo off
chcp 65001 >nul
title 无线毫米波雷达智能感知系统
cd /d "%~dp0server"
start "radar-server" python radar_server.py --simulate --web-port 8081
timeout /t 5 >nul
start "" "http://127.0.0.1:8081/"
echo 雷达演示已启动，浏览器自动打开...
pause
```

停止脚本 `停止雷达.bat`：

```bat
@echo off
taskkill /IM python.exe /FI "WINDOWTITLE eq radar-server" /T /F >nul 2>&1
echo 已停止
pause
```

## 6. 常见问题

| 现象 | 排查 |
|------|------|
| Web 页面打不开 | 确认服务端已启动；检查端口是否被占用（`netstat -ano | findstr 8081`） |
| 界面无目标数据 | 确认模拟器/节点已发送数据；检查 UDP 端口一致 |
| 节点状态一直「离线」 | 检查节点 WiFi 连接、热点 SSID/密码、上电顺序 |
| 告警不触发 | 确认警戒区参数与目标坐标；模拟目标是否进入 `Y≥200` 区域 |
| 8080 被占用 | 加 `--web-port` 改用其他端口（如 8081） |

## 7. 数据导出

「导出CSV」生成文件包含列：

```
ts,node,target,x_mm,y_mm,speed_mm_s,distance_mm
```

可用于 Excel 二次分析（进入/离开次数、逗留时长、热力统计）。