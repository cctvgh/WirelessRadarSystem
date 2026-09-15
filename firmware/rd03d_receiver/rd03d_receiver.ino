/*
 * ============================================================================
 *  无线毫米波雷达智能感知系统 V1.0 —— 雷达显示接收端固件（Arduino Giga R1）
 *  Wireless mmWave Radar Smart System - Radar Display Receiver Firmware
 * ============================================================================
 *  硬件平台  : Arduino GIGA R1 + GIGA Display Shield（800x480 触摸屏）
 *  功能概述  :
 *    1. 建立 WiFi 热点（默认 SSID=RadarNet），供雷达节点接入
 *    2. UDP 接收多节点雷达数据（协议魔数校验，按 nodeId 区分节点）
 *    3. 雷达扫描界面实时渲染：扇形扫描、距离圈、目标点（多节点分色）
 *    4. 目标"幽灵"过滤：2 秒无真实位移的目标自动隐藏，抑制静止杂波
 *    5. 警戒区域告警：目标进入配置的警戒区触发报警（蜂鸣器+屏显+串口事件）
 *    6. 距离感知蜂鸣：无源压电蜂鸣器按最近目标距离缩放节拍与音调
 *    7. 触摸静音按钮：一键关闭蜂鸣器，不影响雷达可视化
 *    8. 连接状态看门狗：节点失联自动清理显示并提示
 *  网络参数  : AP SSID=RadarNet / 密码=radar12345 / UDP 端口=4210 / 自身 IP=192.168.4.1
 *  蜂鸣器    : 无源压电蜂鸣器 VCC=5V / Signal=D9 / GND=GND
 *  参考来源  : Arduino-ESP32-Radarproject (MIT License, Copyright (c) 2026 Sudo_solder)
 *              本固件在 MIT 许可下深度重构并增强功能。
 * ============================================================================
 *  MIT License - 详见项目根目录 LICENSE
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiUDP.h>
#include "Arduino_GigaDisplay_GFX.h"
#include "Arduino_GigaDisplayTouch.h"

/* ============================ 集中配置区 ============================ */

// ---- 网络配置（热点）----
#define AP_SSID        "RadarNet"        // 热点名称
#define AP_PASS        "radar12345"     // 热点密码
#define UDP_PORT       4210             // UDP 监听端口
#define MAX_NODES      4                // 最大支持节点数（多节点组网）

// ---- 显示与雷达 ----
#define SCREEN_W       800
#define SCREEN_H       480
#define RADAR_CX       400
#define RADAR_CY       480
#define RADAR_R        440
#define MAX_DIST_MM    8000.0f
#define SECTOR_HALF    60.0f
#define MAX_TARGETS    3               // 单节点最多 3 目标
#define MOVE_THRESH_MM 15.0f           // 目标位移判定阈值（毫米）

// ---- 警戒区配置（雷达坐标系，单位毫米）----
#define ALARM_ZONE_MIN_X  -4000        // 警戒区 X 下限
#define ALARM_ZONE_MAX_X   4000        // 警戒区 X 上限
#define ALARM_ZONE_MIN_Y  200          // 警戒区 Y 下限（近处）
#define ALARM_ZONE_MAX_Y  4000         // 警戒区 Y 上限（远处）

// ---- 蜂鸣器 ----
#define BUZZER_PIN       9             // 无源蜂鸣器信号引脚
#define BEEP_DUR_MS      70            // 单个蜂鸣时长
#define BEEP_DIST_MIN    300.0f        // 最近距离（毫米）
#define BEEP_DIST_MAX    8000.0f       // 最远距离（毫米）
#define BEEP_INTERVAL_MIN  120         // 最近蜂鸣间隔（毫秒）
#define BEEP_INTERVAL_MAX  900         // 最远蜂鸣间隔（毫秒）
#define BEEP_FREQ_MIN      700         // 最远音调（Hz）
#define BEEP_FREQ_MAX      1800        // 最近音调（Hz）
#define ALARM_FREQ         2200        // 警戒告警音调（Hz）
#define ALARM_INTERVAL_MS  250         // 警戒告警间隔（毫秒）

// ---- 静音按钮 ----
#define BTN_X 671
#define BTN_Y 80
#define BTN_W 128
#define BTN_H 40

// ---- 连接看门狗 ----
#define LINK_TIMEOUT_MS     3000        // 节点失联判定时间
#define BG_REFRESH_INTERVAL_MS 10000    // 全屏刷新周期（消除残影）
#define STILL_TIMEOUT_MS    2000        // 幽灵目标静止隐藏时间

/* ============================ 数据结构 ============================ */

// 单目标
struct Target {
  int16_t x;        // X 坐标（毫米）
  int16_t y;        // Y 坐标（毫米）
  int16_t spd;      // 速度（毫米/秒）
  uint8_t valid;    // 有效性
};

// 网络数据包（与发射端保持一致）
struct __attribute__((packed)) UdpPacket {
  uint32_t magic;       // 协议魔数 0xD03DA7A
  uint32_t nodeId;      // 节点编号
  uint32_t frameCount;  // 帧计数
  uint32_t uptimeMs;    // 节点运行时间
  Target   targets[MAX_TARGETS];
  uint8_t  flags;       // 状态位
  uint8_t  reserved[3];
};

// 显示目标
struct DisplayTarget {
  float x, y, spd;
  bool  valid;
  bool  inAlarmZone;    // 是否处于警戒区
};

// 节点状态
struct NodeState {
  bool     online;
  uint32_t lastPacketMs;
  uint32_t lastFrame;
  uint32_t nodeId;
};

// 目标点渲染状态
struct DotState {
  int16_t cx, cy, bx, by, bw, bh;
  bool active;
};

/* ============================ 全局变量 ============================ */

GigaDisplay_GFX          display;
Arduino_GigaDisplayTouch touch;
WiFiUDP                  udp;

NodeState     nodes[MAX_NODES];
DisplayTarget targets[MAX_NODES][MAX_TARGETS];
DisplayTarget lastDrawn[MAX_NODES][MAX_TARGETS];
DotState      dots[MAX_NODES][MAX_TARGETS];
DisplayTarget prevRaw[MAX_NODES][MAX_TARGETS];      // 幽灵过滤基准
uint32_t      stillSinceMs[MAX_NODES][MAX_TARGETS];
bool          staleNow[MAX_NODES][MAX_TARGETS];

uint32_t frameCountGlobal = 0;
bool     alarmActive      = false;   // 当前帧是否处于警戒告警
bool     alarmWasActive   = false;   // 上一帧警戒状态（用于事件沿触发）
bool     buzzerMuted      = false;

uint32_t lastBeepMs   = 0;
bool     beepActive   = false;
uint32_t beepStartMs  = 0;

bool     touchDownLast = false;
uint32_t lastToggleMs  = 0;

uint32_t lastBgRefreshMs = 0;
uint32_t lastUptimeMs    = 0;

/* ============================ 颜色定义 ============================ */

#define C_BG     0x0000
#define C_GREEN  0x07E0
#define C_GDIM   0x02E0
#define C_RED    0xF800
#define C_AMBER  0xFD20
#define C_CYAN   0x07FF
#define C_WHITE  0xFFFF
#define C_ORANGE 0xFBE0

// 节点颜色（最多 4 节点）
static const uint16_t NODE_COLS[4] = {C_RED, C_AMBER, C_CYAN, C_ORANGE};

/* ============================ 函数声明 ============================ */

bool inAlarmZone(float x, float y);

/* ============================ UDP 接收 ============================ */

/**
 * @brief 接收并解析节点 UDP 数据包
 *        校验魔数后按 nodeId 路由到对应节点的状态槽，更新目标数据
 */
void receiveUdp() {
  int sz = udp.parsePacket();
  if (sz < (int)sizeof(UdpPacket)) return;

  UdpPacket pkt;
  udp.read((uint8_t*)&pkt, sizeof(pkt));

  // 协议魔数校验
  if (pkt.magic != 0xD03DA7A) return;

  // 节点索引（nodeId 从 1 开始）
  int idx = (int)(pkt.nodeId - 1);
  if (idx < 0 || idx >= MAX_NODES) idx = 0;

  nodes[idx].online       = true;
  nodes[idx].lastPacketMs = millis();
  nodes[idx].lastFrame    = pkt.frameCount;
  nodes[idx].nodeId       = pkt.nodeId;

  frameCountGlobal = pkt.frameCount;

  // 更新目标数据
  for (int t = 0; t < MAX_TARGETS; t++) {
    if (pkt.targets[t].valid) {
      targets[idx][t].x     = (float)pkt.targets[t].x;
      targets[idx][t].y     = (float)pkt.targets[t].y;
      targets[idx][t].spd   = (float)abs(pkt.targets[t].spd);
      targets[idx][t].valid = true;
      targets[idx][t].inAlarmZone = inAlarmZone((float)pkt.targets[t].x, (float)pkt.targets[t].y);
    } else {
      targets[idx][t].valid = false;
    }
  }
}

/**
 * @brief 警戒区判定：目标坐标是否落入警戒矩形
 */
bool inAlarmZone(float x, float y) {
  return (x >= ALARM_ZONE_MIN_X && x <= ALARM_ZONE_MAX_X &&
          y >= ALARM_ZONE_MIN_Y && y <= ALARM_ZONE_MAX_Y);
}

/* ============================ 坐标变换与背景 ============================ */

/**
 * @brief 距离到像素半径的非线性映射（近处放大、远处压缩）
 */
float distToR(float d) {
  if (d <= 0)       return 0;
  if (d <= 2000.0f) return (d / 2000.0f) * 0.50f * RADAR_R;
  if (d <= 4000.0f) return (0.50f + (d - 2000.0f) / 2000.0f * 0.25f) * RADAR_R;
  if (d <= 6000.0f) return (0.75f + (d - 4000.0f) / 2000.0f * 0.15f) * RADAR_R;
  return (0.90f + (d - 6000.0f) / 2000.0f * 0.10f) * RADAR_R;
}

void targetToPixel(float x, float y, int16_t &px, int16_t &py) {
  float dist  = sqrtf(x * x + y * y);
  float angle = atan2f(x, y) * 180.0f / PI;
  float r     = distToR(dist);
  float rad   = (angle - 90.0f) * PI / 180.0f;
  px = RADAR_CX + (int16_t)(r * cosf(rad));
  py = RADAR_CY + (int16_t)(r * sinf(rad));
}

/**
 * @brief 在雷达界面上绘制警戒区轮廓（黄色矩形 + ZONE 标注）
 */
void drawAlarmZoneOutline() {
  int16_t x1, y1, x2, y2;
  targetToPixel(ALARM_ZONE_MIN_X, ALARM_ZONE_MIN_Y, x1, y1);
  targetToPixel(ALARM_ZONE_MAX_X, ALARM_ZONE_MAX_Y, x2, y2);
  display.drawRect(min(x1, x2), min(y1, y2), abs(x2 - x1), abs(y2 - y1), C_AMBER);
  display.setTextColor(C_AMBER); display.setTextSize(1);
  display.setCursor(min(x1, x2) + 2, min(y1, y2) + 2);
  display.print("ZONE");
}

/**
 * @brief 重绘指定矩形区域内的背景（用于目标点擦除，避免整体闪烁）
 */
void redrawBg(int16_t rx1, int16_t ry1, int16_t rw, int16_t rh) {
  int16_t rx2 = rx1 + rw - 1, ry2 = ry1 + rh - 1;
  if (rx1 < 0) rx1 = 0; if (ry1 < 0) ry1 = 0;
  if (rx2 >= SCREEN_W) rx2 = SCREEN_W - 1;
  if (ry2 >= SCREEN_H) ry2 = SCREEN_H - 1;
  if (rx1 > rx2 || ry1 > ry2) return;
  display.fillRect(rx1, ry1, rx2 - rx1 + 1, ry2 - ry1 + 1, C_BG);

  // 扇形扫描网格（单传感器 120° 视场，居中于 0°）
  float c = 0.0f;
  for (float a = c - SECTOR_HALF; a <= c + SECTOR_HALF; a += 0.6f) {
    float rad = (a - 90.0f) * PI / 180.0f;
    for (float r = 0; r <= RADAR_R; r += 1.5f) {
      int16_t px = RADAR_CX + (int16_t)(r * cosf(rad));
      int16_t py = RADAR_CY + (int16_t)(r * sinf(rad));
      if (px >= rx1 && px <= rx2 && py >= ry1 && py <= ry2) display.drawPixel(px, py, 0x0180);
    }
  }

  // 距离圈（0.5m~8m）
  float rd[] = {500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
  uint16_t rc[] = {0x00C0, C_GREEN, 0x00C0, 0x00C0, C_GDIM, 0x00C0, 0x00C0, 0x00C0, C_GREEN};
  for (int ri = 0; ri < 9; ri++) {
    float r = distToR(rd[ri]);
    for (float a = c - SECTOR_HALF; a <= c + SECTOR_HALF; a += 0.5f) {
      float rad = (a - 90.0f) * PI / 180.0f;
      int16_t px = RADAR_CX + (int16_t)(r * cosf(rad));
      int16_t py = RADAR_CY + (int16_t)(r * sinf(rad));
      if (px >= rx1 && px <= rx2 && py >= ry1 && py <= ry2) display.drawPixel(px, py, rc[ri]);
    }
  }

  // 角度线
  int degs[] = {-60, -30, 0, 30, 60};
  for (int i = 0; i < 5; i++) {
    float rad = (c + degs[i] - 90.0f) * PI / 180.0f;
    for (float r = 0; r <= RADAR_R; r += 1.5f) {
      int16_t px = RADAR_CX + (int16_t)(r * cosf(rad));
      int16_t py = RADAR_CY + (int16_t)(r * sinf(rad));
      if (px >= rx1 && px <= rx2 && py >= ry1 && py <= ry2)
        display.drawPixel(px, py, degs[i] == 0 ? C_GDIM : 0x0200);
    }
  }

  // 警戒区轮廓
  drawAlarmZoneOutline();

  // 雷达原点
  if (RADAR_CX >= rx1 && RADAR_CX <= rx2 && RADAR_CY >= ry1 && RADAR_CY <= ry2) {
    display.fillCircle(RADAR_CX, RADAR_CY, 5, C_GREEN);
    display.drawCircle(RADAR_CX, RADAR_CY, 9, C_GDIM);
  }
}

/**
 * @brief 全屏绘制雷达背景
 */
void drawBackground() {
  display.fillScreen(C_BG);
  float c = 0.0f;

  // 扇形网格
  for (float a = c - SECTOR_HALF; a <= c + SECTOR_HALF; a += 0.6f) {
    float rad = (a - 90.0f) * PI / 180.0f;
    for (float r = 0; r <= RADAR_R; r += 1.5f) {
      int16_t px = RADAR_CX + (int16_t)(r * cosf(rad));
      int16_t py = RADAR_CY + (int16_t)(r * sinf(rad));
      if (px >= 0 && px < SCREEN_W && py >= 0 && py < SCREEN_H) display.drawPixel(px, py, 0x0180);
    }
  }

  // 距离圈与标注
  float rd[] = {500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000};
  uint16_t rc[] = {0x00C0, C_GREEN, 0x00C0, 0x00C0, C_GDIM, 0x00C0, 0x00C0, 0x00C0, C_GREEN};
  for (int ri = 0; ri < 9; ri++) {
    float r = distToR(rd[ri]);
    for (float a = c - SECTOR_HALF; a <= c + SECTOR_HALF; a += 0.5f) {
      float rad = (a - 90.0f) * PI / 180.0f;
      int16_t px = RADAR_CX + (int16_t)(r * cosf(rad));
      int16_t py = RADAR_CY + (int16_t)(r * sinf(rad));
      if (px >= 0 && px < SCREEN_W && py >= 0 && py < SCREEN_H) display.drawPixel(px, py, rc[ri]);
    }
    display.setTextColor(ri == 1 || ri == 8 ? C_GREEN : 0x0380); display.setTextSize(1);
    char buf[8];
    if (rd[ri] < 1000) sprintf(buf, "0.5m"); else sprintf(buf, "%.0fm", rd[ri] / 1000.0f);
    float rR = (c + SECTOR_HALF - 90.0f) * PI / 180.0f;
    int16_t lx = RADAR_CX + (int16_t)((r + 4) * cosf(rR)) + 2;
    int16_t ly = RADAR_CY + (int16_t)((r + 4) * sinf(rR)) - 4;
    if (lx >= 0 && lx < SCREEN_W && ly >= 0 && ly < SCREEN_H) { display.setCursor(lx, ly); display.print(buf); }
  }

  // 边界线与角度线
  float radL = (c - SECTOR_HALF - 90.0f) * PI / 180.0f;
  float radR = (c + SECTOR_HALF - 90.0f) * PI / 180.0f;
  display.drawLine(RADAR_CX, RADAR_CY, RADAR_CX + (int16_t)(RADAR_R * cosf(radL)), RADAR_CY + (int16_t)(RADAR_R * sinf(radL)), C_GREEN);
  display.drawLine(RADAR_CX, RADAR_CY, RADAR_CX + (int16_t)(RADAR_R * cosf(radR)), RADAR_CY + (int16_t)(RADAR_R * sinf(radR)), C_GREEN);

  int degs[] = {-60, -30, 0, 30, 60};
  for (int i = 0; i < 5; i++) {
    float rad = (c + degs[i] - 90.0f) * PI / 180.0f;
    display.drawLine(RADAR_CX, RADAR_CY, RADAR_CX + (int16_t)(RADAR_R * cosf(rad)), RADAR_CY + (int16_t)(RADAR_R * sinf(rad)), degs[i] == 0 ? C_GDIM : 0x0260);
    char buf[5]; sprintf(buf, "%d", (int)(c + degs[i]));
    float lr = RADAR_R + 14.0f;
    int16_t lx = RADAR_CX + (int16_t)(lr * cosf(rad)) - 6;
    int16_t ly = RADAR_CY + (int16_t)(lr * sinf(rad)) - 4;
    if (lx >= 0 && lx < SCREEN_W && ly >= 0) { display.setTextColor(degs[i] == 0 ? C_GREEN : 0x0380); display.setTextSize(1); display.setCursor(lx, ly); display.print(buf); }
  }

  display.fillCircle(RADAR_CX, RADAR_CY, 5, C_GREEN);
  display.drawCircle(RADAR_CX, RADAR_CY, 9, C_GDIM);

  drawAlarmZoneOutline();

  // 信息框
  display.fillRect(670, 0, 130, 70, 0x0820);
  display.setTextColor(C_GDIM); display.setTextSize(1);
  display.setCursor(675, 4);  display.print("RD-03D 24GHz");
  display.setCursor(675, 14); display.print("3T 8m 120deg");
  display.setCursor(675, 24); display.print("AP: "); display.print(AP_SSID);
}

/* ============================ 目标渲染 ============================ */

void eraseDot(int ni, int ti) {
  if (!dots[ni][ti].active) return;
  redrawBg(dots[ni][ti].bx, dots[ni][ti].by, dots[ni][ti].bw, dots[ni][ti].bh);
  dots[ni][ti].active = false;
}

void drawDot(int ni, int ti) {
  int16_t tx, ty;
  targetToPixel(targets[ni][ti].x, targets[ni][ti].y, tx, ty);
  uint16_t col = NODE_COLS[ni % 4];

  // 警戒区内目标红色闪烁
  bool inZone = targets[ni][ti].inAlarmZone;
  if (inZone) col = ((millis() / 300) % 2 == 0) ? C_RED : C_WHITE;

  int16_t bx = tx - 20, by = ty - 20, bw = 112, bh = 44;
  display.drawCircle(tx, ty, 18, col);
  display.drawCircle(tx, ty, 11, col);
  display.fillCircle(tx, ty, 5, col);
  display.setTextColor(C_WHITE); display.setTextSize(1);
  float dist = sqrtf(targets[ni][ti].x * targets[ni][ti].x + targets[ni][ti].y * targets[ni][ti].y);
  char buf[14]; sprintf(buf, "%.2fm", dist / 1000.0f);
  display.setCursor(tx + 21, ty - 8); display.print(buf);
  if (targets[ni][ti].spd > 2.0f) {
    char sb[14]; sprintf(sb, "%dcm/s", (int)targets[ni][ti].spd);
    display.setCursor(tx + 21, ty + 4); display.print(sb);
  }
  if (inZone) {
    display.setTextColor(C_RED);
    display.setCursor(tx + 21, ty + 16); display.print("ZONE!");
  }
  dots[ni][ti] = {tx, ty, bx, by, bw, bh, true};
  lastDrawn[ni][ti] = targets[ni][ti];
}

/**
 * @brief 更新所有目标点显示（幽灵过滤、位移阈值、警戒告警判定与事件上报）
 */
void updateDots() {
  uint32_t now = millis();
  bool anyAlarm = false;

  for (int n = 0; n < MAX_NODES; n++) {
    for (int t = 0; t < MAX_TARGETS; t++) {
      staleNow[n][t] = false;
      if (!targets[n][t].valid) {
        stillSinceMs[n][t] = 0;
        prevRaw[n][t].valid = false;
        if (dots[n][t].active) eraseDot(n, t);
        continue;
      }

      // 真实位移判定（独立于渲染状态）
      bool movedRaw = !prevRaw[n][t].valid ||
                      fabsf(targets[n][t].x - prevRaw[n][t].x) > MOVE_THRESH_MM ||
                      fabsf(targets[n][t].y - prevRaw[n][t].y) > MOVE_THRESH_MM;
      if (movedRaw) stillSinceMs[n][t] = now;
      prevRaw[n][t] = targets[n][t];

      // 幽灵过滤：静止超过 2s 隐藏
      bool stale = (now - stillSinceMs[n][t]) > STILL_TIMEOUT_MS;
      staleNow[n][t] = stale;
      if (stale) {
        if (dots[n][t].active) eraseDot(n, t);
        continue;
      }

      // 警戒区触发
      if (targets[n][t].inAlarmZone) anyAlarm = true;

      // 位移超过阈值时重绘
      bool moved = fabsf(targets[n][t].x - lastDrawn[n][t].x) > MOVE_THRESH_MM ||
                   fabsf(targets[n][t].y - lastDrawn[n][t].y) > MOVE_THRESH_MM;
      if (dots[n][t].active && !moved && !targets[n][t].inAlarmZone) continue;
      if (dots[n][t].active) eraseDot(n, t);
      drawDot(n, t);
    }
  }

  // 警戒状态事件上报（串口 JSON，供上位机采集）
  if (anyAlarm && !alarmWasActive) {
    Serial.println("{\"event\":\"alarm_enter\"}");
  } else if (!anyAlarm && alarmWasActive) {
    Serial.println("{\"event\":\"alarm_exit\"}");
  }
  alarmWasActive = anyAlarm;
  alarmActive    = anyAlarm;

  updateInfoBox();
}

/**
 * @brief 更新信息框：在线节点、目标数、帧计数、警戒状态与节点状态灯
 */
void updateInfoBox() {
  display.fillRect(671, 34, 128, 36, 0x0820);
  display.setTextColor(C_GDIM); display.setTextSize(1);

  int online = 0;
  for (int n = 0; n < MAX_NODES; n++) if (nodes[n].online) online++;
  display.setCursor(675, 36);
  display.print("Node:"); display.print(online);

  int cnt = 0;
  for (int n = 0; n < MAX_NODES; n++)
    for (int t = 0; t < MAX_TARGETS; t++)
      if (targets[n][t].valid && !staleNow[n][t]) cnt++;
  display.print(" T:"); display.print(cnt);
  display.print(" fr:"); display.print(frameCountGlobal);

  display.setTextColor(alarmActive ? C_RED : C_GDIM);
  display.setCursor(675, 48);
  display.print(alarmActive ? "ALARM!" : "MONITOR");

  // 节点状态灯
  for (int n = 0; n < MAX_NODES; n++) {
    bool alive = nodes[n].online && (millis() - nodes[n].lastPacketMs) < LINK_TIMEOUT_MS;
    display.fillCircle(678 + n * 14, 68, 3, alive ? NODE_COLS[n % 4] : 0x39E7);
  }
}

/**
 * @brief 节点失联清理：超时未收到数据的节点标记离线并清理目标点
 */
void reapOfflineNodes() {
  if (millis() - lastUptimeMs < 1000) return;
  lastUptimeMs = millis();
  for (int n = 0; n < MAX_NODES; n++) {
    if (nodes[n].online && (millis() - nodes[n].lastPacketMs) > LINK_TIMEOUT_MS) {
      nodes[n].online = false;
      for (int t = 0; t < MAX_TARGETS; t++) {
        targets[n][t].valid = false;
        if (dots[n][t].active) eraseDot(n, t);
      }
      Serial.print("{\"event\":\"node_lost\",\"node\":");
      Serial.print(n + 1);
      Serial.println("}");
    }
  }
}

/* ============================ 蜂鸣器 ============================ */

/**
 * @brief 计算最近有效目标距离（毫米），无目标返回 -1
 */
float closestTargetDistMm() {
  float best = -1.0f;
  for (int n = 0; n < MAX_NODES; n++) {
    for (int t = 0; t < MAX_TARGETS; t++) {
      if (!targets[n][t].valid || staleNow[n][t]) continue;
      float d = sqrtf(targets[n][t].x * targets[n][t].x + targets[n][t].y * targets[n][t].y);
      if (best < 0 || d < best) best = d;
    }
  }
  return best;
}

/**
 * @brief 距离感知蜂鸣：节拍与音调随最近目标距离缩放，警戒时切换告警音
 */
void updateTrackerSound() {
  uint32_t now = millis();

  if (buzzerMuted) {
    if (beepActive) { noTone(BUZZER_PIN); beepActive = false; }
    return;
  }

  if (beepActive && (now - beepStartMs >= BEEP_DUR_MS)) beepActive = false;

  // 警戒告警音：固定高频、快节拍
  if (alarmActive) {
    if (!beepActive && (now - lastBeepMs >= ALARM_INTERVAL_MS)) {
      tone(BUZZER_PIN, ALARM_FREQ, BEEP_DUR_MS);
      beepActive  = true;
      beepStartMs = now;
      lastBeepMs  = now;
    }
    return;
  }

  float dist = closestTargetDistMm();
  if (dist < 0) return;

  float d = constrain(dist, BEEP_DIST_MIN, BEEP_DIST_MAX);
  float t = (d - BEEP_DIST_MIN) / (BEEP_DIST_MAX - BEEP_DIST_MIN);

  uint16_t interval = BEEP_INTERVAL_MIN + (uint16_t)(t * (BEEP_INTERVAL_MAX - BEEP_INTERVAL_MIN));
  uint16_t freq     = BEEP_FREQ_MAX - (uint16_t)(t * (BEEP_FREQ_MAX - BEEP_FREQ_MIN));

  if (!beepActive && (now - lastBeepMs >= interval)) {
    tone(BUZZER_PIN, freq, BEEP_DUR_MS);
    beepActive  = true;
    beepStartMs = now;
    lastBeepMs  = now;
  }
}

/* ============================ 触摸控制 ============================ */

/**
 * @brief 绘制静音按钮
 */
void drawMuteButton() {
  uint16_t border = buzzerMuted ? C_RED : C_GREEN;
  uint16_t fill   = buzzerMuted ? 0x4000 : 0x0320;
  display.fillRect(BTN_X, BTN_Y, BTN_W, BTN_H, fill);
  display.drawRect(BTN_X, BTN_Y, BTN_W, BTN_H, border);
  display.setTextColor(border); display.setTextSize(1);
  display.setCursor(BTN_X + 14, BTN_Y + 16);
  display.print(buzzerMuted ? "STUMM" : "BUZZER AN");
}

/**
 * @brief 触摸检测（边缘触发 + 消抖）：仅当从"未按下"变为"按下"时切换静音
 */
void checkTouch() {
  uint8_t contacts;
  GDTpoint_t points[5];
  contacts = touch.getTouchPoints(points);

  bool touchingButton = false;
  for (uint8_t i = 0; i < contacts; i++) {
    // 触摸原始坐标（竖屏 480x800）映射到横屏 (800x480)
    int16_t tx = points[i].y;
    int16_t ty = 480 - points[i].x;
    if (tx >= BTN_X && tx <= BTN_X + BTN_W && ty >= BTN_Y && ty <= BTN_Y + BTN_H) {
      touchingButton = true;
      break;
    }
  }

  if (touchingButton && !touchDownLast && (millis() - lastToggleMs > 300)) {
    buzzerMuted = !buzzerMuted;
    lastToggleMs = millis();
    drawMuteButton();
  }
  touchDownLast = touchingButton;
}

/* ============================ 网络初始化 ============================ */

void startAP() {
  Serial.print("[NET] 启动热点 '");
  Serial.print(AP_SSID);
  Serial.println("'...");
  WiFi.beginAP(AP_SSID, AP_PASS, 6);
  delay(5000);
  Serial.print("[NET] AP IP: ");
  Serial.println(WiFi.localIP());
  delay(1000);
  udp.begin(UDP_PORT);
  Serial.print("[NET] UDP 监听端口 ");
  Serial.println(UDP_PORT);
}

/* ============================ 周期背景刷新 ============================ */

void refreshBackground() {
  drawBackground();
  drawMuteButton();
  for (int n = 0; n < MAX_NODES; n++)
    for (int t = 0; t < MAX_TARGETS; t++) {
      dots[n][t].active = false;
      if (targets[n][t].valid) drawDot(n, t);
    }
}

/* ============================ 初始化 ============================ */

void setup() {
  Serial.begin(115200);

  display.begin();
  display.setRotation(1);
  display.fillScreen(C_BG);
  display.setTextColor(C_GREEN); display.setTextSize(2);
  display.setCursor(120, 200); display.println("mmWAVE RADAR RD-03D");
  display.setTextSize(1); display.setTextColor(C_GDIM);
  display.setCursor(120, 235); display.println("Starting Hotspot...");

  touch.begin();
  Wire1.begin();

  pinMode(BUZZER_PIN, OUTPUT);
  noTone(BUZZER_PIN);

  // 初始化状态数组
  for (int n = 0; n < MAX_NODES; n++) {
    nodes[n].online = false;
    nodes[n].lastPacketMs = 0;
    nodes[n].lastFrame = 0;
    nodes[n].nodeId = 0;
    for (int t = 0; t < MAX_TARGETS; t++) {
      targets[n][t]   = {0, 0, 0, false, false};
      lastDrawn[n][t] = {0, 0, 0, false, false};
      prevRaw[n][t]   = {0, 0, 0, false, false};
      stillSinceMs[n][t] = 0;
      staleNow[n][t]     = false;
      dots[n][t] = {0, 0, 0, 0, 0, 0, false};
    }
  }

  startAP();

  display.setCursor(120, 250);
  display.print("AP: "); display.print(AP_SSID);
  display.print("  IP: "); display.println(WiFi.localIP());
  delay(1000);

  drawBackground();
  drawMuteButton();
  Serial.println("[SYS] 接收端就绪，等待雷达节点接入...");
}

/* ============================ 主循环 ============================ */

void loop() {
  receiveUdp();
  updateDots();
  reapOfflineNodes();
  updateTrackerSound();
  checkTouch();

  if (millis() - lastBgRefreshMs >= BG_REFRESH_INTERVAL_MS) {
    lastBgRefreshMs = millis();
    refreshBackground();
  }
}