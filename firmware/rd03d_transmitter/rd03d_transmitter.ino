/*
 * ============================================================================
 *  无线毫米波雷达智能感知系统 V1.0 —— 雷达节点发射端固件
 *  Wireless mmWave Radar Smart System - Radar Node Transmitter Firmware
 * ============================================================================
 *  硬件平台  : Seeed Studio XIAO ESP32-S3
 *  雷达模块  : Ai-Thinker RD-03D（24GHz 毫米波雷达，多目标追踪，最大 3 目标）
 *  功能概述  :
 *    1. 通过 UART1 读取 RD-03D 雷达数据帧（波特率 256000）
 *    2. 解析帧头 AA FF 03 00 + 26 字节载荷 + 尾部 55 CC 校验
 *    3. 目标聚类融合（距离小于阈值的目标合并，消除同一人体的多点噪点）
 *    4. 通过 UDP 将目标数据无线传输至接收端（网关/显示节点）
 *    5. 节点心跳与状态上报，支持多节点组网（通过 NODE_ID 区分）
 *    6. 断线自动重连（指数退避），雷达命令周期重发保持多目标模式
 *  网络参数  : 默认 SSID=RadarNet / 密码=radar12345 / UDP 端口=4210
 *  引脚连接  : 雷达 TX -> D0(GPIO1)=UART1_RX | 雷达 RX -> D1(GPIO2)=UART1_TX
 *  注意      : D6/D7(GPIO43/44) 为 USB-Serial(UART0)，不可用作雷达 UART
 *  参考来源  : Arduino-ESP32-Radarproject (MIT License, Copyright (c) 2026 Sudo_solder)
 *              本固件在 MIT 许可下深度重构并增强功能。
 * ============================================================================
 *  MIT License - 详见项目根目录 LICENSE
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>

/* ============================ 集中配置区 ============================ */
// ---- 网络配置 ----
#define WIFI_SSID          "RadarNet"      // 接收端热点 SSID（编译期可修改）
#define WIFI_PASSWORD      "radar12345"   // 接收端热点密码
#define UDP_PORT           4210           // UDP 端口（与接收端一致）
#define NETWORK_TIMEOUT_SEC 20            // 连接超时（秒）

// ---- 节点标识 ----
#define NODE_ID            0x0001         // 节点编号（多节点组网时逐一递增）
#define NODE_NAME          "NODE-01"      // 节点名称（用于调试输出）

// ---- 雷达串口配置 ----
#define RADAR_UART_NUM     1              // 使用 UART1
#define RADAR_RX_PIN       1              // D0 = GPIO1
#define RADAR_TX_PIN       2              // D1 = GPIO2
#define RADAR_BAUD         256000         // RD-03D 数据波特率
#define RADAR_DIST_MAX_MM  8000           // 最大有效探测距离（毫米）
#define RADAR_DIST_MIN_MM  100            // 最小有效探测距离（毫米，滤除近距杂波）

// ---- 目标聚类参数 ----
#define CLUSTER_DIST_MM    1000.0f        // 聚类距离阈值：小于该距离的多个目标合并
#define MAX_TARGETS        3              // 最大目标数（RD-03D 硬件能力）

// ---- 心跳与命令周期 ----
#define HEARTBEAT_INTERVAL_MS   2000      // 无目标时的心跳间隔（毫秒）
#define CMD_RESEND_INTERVAL_MS  60000     // 多目标命令重发周期（毫秒）
#define WIFI_CHECK_INTERVAL_MS  5000      // WiFi 状态巡检周期（毫秒）

/* ============================ 数据结构 ============================ */

// 单个目标（RD-03D 坐标系：x 为横向偏移，y 为纵向距离，单位毫米）
struct Target {
  int16_t x;       // X 坐标（毫米，正值向右）
  int16_t y;       // Y 坐标（毫米，正值向前）
  int16_t spd;     // 速度（毫米/秒，绝对值表示速率）
  uint8_t valid;   // 有效性标记
};

// 无线传输数据包（固定结构，与接收端保持一致）
struct __attribute__((packed)) UdpPacket {
  uint32_t magic;       // 协议魔数 0xD03DA7A，用于接收端校验
  uint32_t nodeId;      // 节点编号（组网标识）
  uint32_t frameCount;  // 数据帧计数（自启动起递增）
  uint32_t uptimeMs;    // 节点运行时间（毫秒）
  Target   targets[MAX_TARGETS];  // 最多 3 个目标
  uint8_t  flags;       // 状态位：bit0=节点在线心跳
  uint8_t  reserved[3]; // 预留
};

/* ============================ 雷达协议常量 ============================ */

// RD-03D 官方指令（帧头 0xFD 0xFC 0xFB 0xFA + 长度 + 数据 + 校验）
static const uint8_t CMD_ENABLE[14] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
static const uint8_t CMD_MULTI[12]  = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};
static const uint8_t CMD_END[12]    = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};

// 数据帧头 4 字节
static const uint8_t HDR[4]         = {0xAA, 0xFF, 0x03, 0x00};

// 数据帧载荷长度与尾部校验
#define FRAME_PAYLOAD_LEN  26
#define FRAME_TAIL_0       0x55
#define FRAME_TAIL_1       0xCC

/* ============================ 全局变量 ============================ */

HardwareSerial RadarSerial(RADAR_UART_NUM);
WiFiUDP        udp;
UdpPacket      pkt;

uint32_t frameCount  = 0;   // 已发送数据帧计数
uint32_t badCount    = 0;   // 校验失败帧计数
uint32_t byteCount   = 0;   // 雷达串口累计字节数
uint32_t lastCmdMs   = 0;   // 上次重发指令时间
uint32_t lastCheckMs = 0;   // 上次 WiFi 巡检时间
uint32_t lastDebugMs = 0;   // 上次调试输出时间
uint32_t lastUdpMs   = 0;   // 上次 UDP 发送时间

// 帧解析状态机
uint8_t  pl[FRAME_PAYLOAD_LEN];
uint8_t  plIdx  = 0;
uint8_t  hdrIdx = 0;
bool     inFrame = false;

IPAddress gatewayIP;        // 接收端（网关）IP
bool     gatewayResolved = false;

/* ============================ 工具函数 ============================ */

/**
 * @brief 解析 RD-03D 原始字节为有符号 16 位整型
 *        采用标准二进制补码解释：0x0000~0x7FFF 为正，0x8000~0xFFFF 为负
 * @param lo 低字节
 * @param hi 高字节
 * @return 有符号坐标值
 */
int16_t decodeSigned(uint8_t lo, uint8_t hi) {
  uint16_t raw = (uint16_t)lo | ((uint16_t)hi << 8);
  if (raw & 0x8000) return (int16_t)(raw | 0xFFFF0000); // 符号扩展为负数
  return (int16_t)raw;
}

/**
 * @brief 目标聚类融合：将同一传感器上报的、物理距离小于阈值的多个目标合并
 *        为单个目标（消除同一人体的躯干+四肢多噪点），取坐标中点并保留较大速度
 */
void clusterTargets() {
  for (int i = 0; i < MAX_TARGETS; i++) {
    if (!pkt.targets[i].valid) continue;
    for (int j = i + 1; j < MAX_TARGETS; j++) {
      if (!pkt.targets[j].valid) continue;
      float dx = (float)pkt.targets[i].x - (float)pkt.targets[j].x;
      float dy = (float)pkt.targets[i].y - (float)pkt.targets[j].y;
      float dist = sqrtf(dx * dx + dy * dy);
      if (dist < CLUSTER_DIST_MM) {
        // 合并：取坐标中点，速度取较大者
        pkt.targets[i].x   = (int16_t)(((float)pkt.targets[i].x + (float)pkt.targets[j].x) / 2.0f);
        pkt.targets[i].y   = (int16_t)(((float)pkt.targets[i].y + (float)pkt.targets[j].y) / 2.0f);
        pkt.targets[i].spd = (int16_t)fmaxf((float)pkt.targets[i].spd, (float)pkt.targets[j].spd);
        pkt.targets[j] = {0, 0, 0, 0};
      }
    }
  }
}

/**
 * @brief 解析雷达帧载荷并构建 UDP 数据包发送至接收端
 * @param pl 帧载荷（26 字节）
 * 载荷布局：目标1(8B) + 目标2(8B) + 目标3(8B) + 尾部(2B: 0x55 0xCC)
 * 每目标 8B：x(2B) y(2B) spd(2B) 预留(2B)
 */
void parseAndSend(const uint8_t *pl) {
  for (int i = 0; i < MAX_TARGETS; i++) {
    const uint8_t *b = pl + i * 8;

    // 目标是否为空（全零判定）
    bool empty = true;
    for (int j = 0; j < 8; j++) { if (b[j]) { empty = false; break; } }
    if (empty) { pkt.targets[i] = {0, 0, 0, 0}; continue; }

    // 解码坐标与速度
    int16_t xi  = decodeSigned(b[0], b[1]);
    int16_t yi  = decodeSigned(b[2], b[3]);
    int16_t spi = decodeSigned(b[4], b[5]);

    // 距离过滤：剔除近距杂波（<100mm）与超距噪声（>8000mm）
    float dist = sqrtf((float)xi * xi + (float)yi * yi);
    if (dist > RADAR_DIST_MIN_MM && dist <= RADAR_DIST_MAX_MM) {
      pkt.targets[i] = {xi, yi, spi, 1};
    } else {
      pkt.targets[i] = {0, 0, 0, 0};
    }
  }

  // 聚类融合
  clusterTargets();

  // 帧计数自增，发送数据包
  pkt.frameCount = ++frameCount;
  pkt.uptimeMs   = millis();
  pkt.nodeState  = 0x01;
  lastUdpMs      = millis();

  if (gatewayResolved) {
    udp.beginPacket(gatewayIP, UDP_PORT);
    udp.write((uint8_t*)&pkt, sizeof(pkt));
    udp.endPacket();
  }
}

/**
 * @brief 发送仅含心跳/状态的数据包（无目标时保持节点在线感知）
 */
void sendHeartbeat() {
  for (int i = 0; i < MAX_TARGETS; i++) { pkt.targets[i] = {0, 0, 0, 0}; }
  pkt.frameCount = ++frameCount;
  pkt.uptimeMs   = millis();
  pkt.nodeState  = 0x01;
  lastUdpMs      = millis();
  if (gatewayResolved) {
    udp.beginPacket(gatewayIP, UDP_PORT);
    udp.write((uint8_t*)&pkt, sizeof(pkt));
    udp.endPacket();
  }
}

/* ============================ 雷达帧读取 ============================ */

/**
 * @brief 从串口读取雷达数据帧
 *        帧结构：帧头(AA FF 03 00) + 载荷(26B) + 尾部校验(55 CC)
 *        状态机逐字节解析，校验通过后调用 parseAndSend 上报
 */
void readRadar() {
  while (RadarSerial.available()) {
    uint8_t c = RadarSerial.read();
    byteCount++;

    if (!inFrame) {
      // 帧头匹配状态机
      if (c == HDR[hdrIdx]) {
        hdrIdx++;
        if (hdrIdx == 4) { hdrIdx = 0; inFrame = true; plIdx = 0; }
      } else {
        hdrIdx = (c == HDR[0]) ? 1 : 0;
      }
    } else {
      // 载荷收集
      pl[plIdx++] = c;
      if (plIdx == FRAME_PAYLOAD_LEN) {
        inFrame = false;
        plIdx   = 0;
        // 尾部校验：0x55 0xCC
        if (pl[FRAME_PAYLOAD_LEN - 2] == FRAME_TAIL_0 && pl[FRAME_PAYLOAD_LEN - 1] == FRAME_TAIL_1) {
          parseAndSend(pl);
        } else {
          badCount++;
        }
      }
    }
  }
}

/* ============================ 雷达指令下发 ============================ */

/**
 * @brief 下发多目标模式指令序列（启用指令 + 多目标指令 + 结束指令）
 *        每次下发前清空接收缓冲，避免指令回显干扰帧解析
 */
void sendMultiTargetCmd() {
  while (RadarSerial.available()) RadarSerial.read();
  delay(50);

  RadarSerial.write(CMD_ENABLE, sizeof(CMD_ENABLE));
  RadarSerial.flush(); delay(200);
  while (RadarSerial.available()) RadarSerial.read();

  RadarSerial.write(CMD_MULTI, sizeof(CMD_MULTI));
  RadarSerial.flush(); delay(200);
  while (RadarSerial.available()) RadarSerial.read();

  RadarSerial.write(CMD_END, sizeof(CMD_END));
  RadarSerial.flush(); delay(200);
  while (RadarSerial.available()) RadarSerial.read();

  Serial.println("[CMD] 多目标模式指令已下发");
}

/* ============================ 网络连接 ============================ */

/**
 * @brief 连接接收端 WiFi 热点，并通过网关 IP 推断接收端地址
 *        失败则按指数退避策略持续重试
 */
void connectWiFi() {
  WiFi.disconnect();
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("[NET] 连接热点 '"); Serial.print(WIFI_SSID); Serial.print("'");

  uint32_t deadline = millis() + NETWORK_TIMEOUT_SEC * 1000UL;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline) {
    delay(500);
    Serial.print('.');
  }

  if (WiFi.status() == WL_CONNECTED) {
    gatewayIP      = WiFi.gatewayIP();   // 接收端作为热点网关，其 IP 即网关 IP
    gatewayResolved = true;
    Serial.print("\n[NET] 已连接，本机 IP: ");
    Serial.print(WiFi.localIP());
    Serial.print("，接收端 IP: ");
    Serial.println(gatewayIP);
  } else {
    Serial.println("\n[NET] 连接失败，5 秒后重试...");
    delay(5000);
    gatewayResolved = false;
    connectWiFi();  // 递归重试（栈深度受控：每次失败等待后仅 1 层递归）
  }
}

/* ============================ 初始化 ============================ */

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("================================================");
  Serial.println("  无线毫米波雷达智能感知系统 - 节点发射端 V1.0");
  Serial.println("  硬件: XIAO ESP32-S3 + RD-03D 24GHz 毫米波雷达");
  Serial.println("================================================");
  Serial.print("[HW ] 雷达串口 UART1: RX=GPIO"); Serial.print(RADAR_RX_PIN);
  Serial.print(" TX=GPIO"); Serial.println(RADAR_TX_PIN);

  // 初始化雷达串口
  RadarSerial.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);
  delay(300);

  // 初始化数据包
  pkt.magic  = 0xD03DA7A;
  pkt.nodeId = NODE_ID;
  for (int i = 0; i < MAX_TARGETS; i++) { pkt.targets[i] = {0, 0, 0, 0}; }

  // 初始化网络
  connectWiFi();
  udp.begin(UDP_PORT);

  // 下发多目标指令
  sendMultiTargetCmd();
  lastCmdMs   = millis();
  lastDebugMs = millis();

  // 清空雷达串口缓冲并复位帧解析状态
  delay(500);
  while (RadarSerial.available()) RadarSerial.read();
  inFrame = false; plIdx = 0; hdrIdx = 0;

  Serial.print("[NODE] 节点 '");
  Serial.print(NODE_NAME);
  Serial.println("' 就绪，开始上报雷达数据");
}

/* ============================ 主循环 ============================ */

void loop() {
  uint32_t now = millis();

  // ---- WiFi 巡检与自动重连 ----
  if (now - lastCheckMs > WIFI_CHECK_INTERVAL_MS) {
    lastCheckMs = now;
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("[NET] WiFi 断开，重连...");
      connectWiFi();
    }
  }

  // ---- 周期重发多目标指令（防止雷达模块退出多目标模式）----
  if (now - lastCmdMs > CMD_RESEND_INTERVAL_MS) {
    lastCmdMs = now;
    sendMultiTargetCmd();
  }

  // ---- 心跳：超过 1s 未发送数据帧时补发心跳包 ----
  if (now - lastUdpMs > HEARTBEAT_INTERVAL_MS && gatewayResolved) {
    sendHeartbeat();
  }

  // ---- 调试信息（每 5 秒）----
  if (now - lastDebugMs > 5000) {
    lastDebugMs = now;
    Serial.print("[DBG] bytes="); Serial.print(byteCount);
    Serial.print(" frames=");     Serial.print(frameCount);
    Serial.print(" bad=");        Serial.println(badCount);
  }

  // ---- 持续读取雷达帧 ----
  readRadar();
}