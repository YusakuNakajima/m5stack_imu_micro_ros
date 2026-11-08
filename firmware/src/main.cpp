// M5Stack Core2 (or Basic v2.6) + M5Stack IMU Unit (MPU6886) on Port A -> micro-ROS (serial).
// Port A is GPIO32/33 on the Core2 and GPIO21/22 on the Basic; M5Unified's Ex_I2C picks the
// board's pins. Build with -DIMU_INTERNAL=1 to read the Core2's built-in MPU6886 instead.
//
// Publishes m5stack_imu_msgs/msg/AccelBlock on /m5stack_imu/accel_block:
//   SAMPLES_PER_BLOCK consecutive accelerometer samples at the MPU6886's 4 kHz output
//   (ACCEL_FCHOICE_B = 1: digital low-pass bypassed, 3 dB bandwidth 1046 Hz).
//   The MPU6886 cannot sample faster than 4 kHz, so content above ~1 kHz is attenuated
//   and nothing above 2 kHz (Nyquist) is available.
//
// Two tasks:
//   imu_task (core 0): reads the MPU6886 FIFO every 2 ms and assembles blocks into a queue.
//     The FIFO (1 KB) holds only ~40 ms at 4 kHz, so nothing that can block (serial writes,
//     agent pings, time sync, the LCD) runs here.
//   loop (core 1): micro-ROS (agent connection, publishing the queued blocks, time sync, LCD).
// Lost samples are never hidden: a FIFO overflow or a full queue makes first_sample_index
// jump and flags the next block (FLAG_FIFO_OVERFLOW / FLAG_QUEUE_OVERFLOW).

#include <Arduino.h>
#include <M5Unified.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include <m5stack_imu_msgs/msg/accel_block.h>

#include <Preferences.h>
#include <esp_system.h>

#ifndef SERIAL_BAUD
#define SERIAL_BAUD 921600
#endif
#ifndef IMU_INTERNAL
#define IMU_INTERNAL 0
#endif
#ifndef ROS_DOMAIN_ID
// Used only until the host has sent one (see "ROS domain ID" below). The agent cannot set it.
#define ROS_DOMAIN_ID 0
#endif

namespace {

// ---------------------------------------------------------------- settings
constexpr uint8_t MPU_ADDR = 0x68;           // IMU Unit (AD0 low)
constexpr uint32_t I2C_FREQ_HZ = 400000;     // MPU6886 fast mode maximum
constexpr int ACCEL_FS_G = 2;                // +-2 g (as the ROS 1 M5Stack sketch); 2, 4, 8 or 16
constexpr double NOMINAL_RATE_HZ = 4000.0;   // MPU6886 accelerometer output with FCHOICE_B = 1
constexpr size_t SAMPLES_PER_BLOCK = 48;     // 12 ms per block; ~350 bytes, fits the 512 B serial MTU
constexpr size_t QUEUE_BLOCKS = 48;          // ~0.58 s of buffering between the tasks
constexpr uint32_t IMU_PERIOD_MS = 2;        // FIFO read period (8 samples at 4 kHz)
constexpr uint32_t RATE_MEASURE_MS = 2000;   // start-up measurement of the actual output rate
constexpr uint32_t TIME_SYNC_PERIOD_MS = 60000;
constexpr const char *TOPIC = "/m5stack_imu/accel_block";
constexpr double G_M_S2 = 9.80665;

// ---------------------------------------------------------------- MPU6886 registers
constexpr uint8_t REG_CONFIG = 0x1A;         // bit 6 FIFO_MODE
constexpr uint8_t REG_ACCEL_CONFIG = 0x1C;   // bits 4:3 ACCEL_FS_SEL
constexpr uint8_t REG_ACCEL_CONFIG2 = 0x1D;  // bit 3 ACCEL_FCHOICE_B
constexpr uint8_t REG_FIFO_EN = 0x23;        // bit 3 ACCEL_FIFO_EN
constexpr uint8_t REG_INT_STATUS = 0x3A;     // bit 4 FIFO_OFLOW_INT
constexpr uint8_t REG_USER_CTRL = 0x6A;      // bit 6 FIFO_EN, bit 2 FIFO_RST
constexpr uint8_t REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t REG_PWR_MGMT_2 = 0x6C;     // bits 2:0 gyro standby
constexpr uint8_t REG_FIFO_COUNTH = 0x72;
constexpr uint8_t REG_FIFO_R_W = 0x74;
constexpr uint8_t REG_WHO_AM_I = 0x75;
constexpr uint8_t WHO_AM_I_MPU6886 = 0x19;
constexpr size_t FIFO_SIZE_BYTES = 1024;

#if IMU_INTERNAL
#define IMU_BUS M5.In_I2C
#define IMU_WHERE "internal"
#else
#define IMU_BUS M5.Ex_I2C
#define IMU_WHERE "Port A"
#endif

bool reg_write(uint8_t reg, uint8_t value) {
  return IMU_BUS.writeRegister8(MPU_ADDR, reg, value, I2C_FREQ_HZ);
}

bool reg_read(uint8_t reg, uint8_t *buf, size_t len) {
  return IMU_BUS.readRegister(MPU_ADDR, reg, buf, len, I2C_FREQ_HZ);
}

int fifo_count() {
  uint8_t b[2];
  if (!reg_read(REG_FIFO_COUNTH, b, 2)) return -1;
  return ((b[0] & 0x1F) << 8) | b[1];
}

void fifo_reset() {
  reg_write(REG_USER_CTRL, 0x04);  // FIFO_RST (FIFO disabled)
  delay(1);
  uint8_t s;
  reg_read(REG_INT_STATUS, &s, 1);  // clear a pending overflow flag
  reg_write(REG_USER_CTRL, 0x40);  // FIFO_EN
}

bool mpu_init() {
  uint8_t who = 0;
  if (!reg_read(REG_WHO_AM_I, &who, 1) || who != WHO_AM_I_MPU6886) return false;
  reg_write(REG_PWR_MGMT_1, 0x80);  // device reset
  delay(50);
  reg_write(REG_PWR_MGMT_1, 0x01);  // auto-select the best clock
  delay(10);
  reg_write(REG_PWR_MGMT_2, 0x07);  // gyro standby, accelerometer on
  const uint8_t fs_sel = ACCEL_FS_G == 16 ? 3 : ACCEL_FS_G == 8 ? 2 : ACCEL_FS_G == 4 ? 1 : 0;
  reg_write(REG_ACCEL_CONFIG, fs_sel << 3);
  reg_write(REG_ACCEL_CONFIG2, 0x08);  // ACCEL_FCHOICE_B = 1: 4 kHz, 1046 Hz bandwidth
  reg_write(REG_CONFIG, 0x40);         // FIFO_MODE = 1: stop writing when full (overflow detectable)
  reg_write(REG_FIFO_EN, 0x08);        // accelerometer into the FIFO
  fifo_reset();
  return true;
}

// The FIFO record size with only the accelerometer enabled is 6 bytes on the MPU6050
// family, but some MPU6886 revisions also write the temperature (8 bytes). Measure it
// instead of assuming: bytes written in a known time / samples expected in that time.
int detect_packet_bytes() {
  fifo_reset();
  const uint32_t t0 = micros();
  delay(20);  // ~80 samples: 480-640 bytes, below the 1 KB FIFO
  const int n = fifo_count();
  const double samples = (micros() - t0) * 1e-6 * NOMINAL_RATE_HZ;
  if (n <= 0 || samples < 1) return -1;
  const double per = n / samples;
  const int candidates[] = {6, 8, 14};
  int best = -1;
  double best_err = 1e9;
  for (int c : candidates) {
    const double err = fabs(per - c) / c;
    if (err < best_err) best = c, best_err = err;
  }
  return best_err < 0.15 ? best : -1;
}

// ---------------------------------------------------------------- shared state
struct Block {
  int64_t stamp_ns;      // ROS time of the first sample
  uint64_t first_index;
  uint8_t flags;
  int16_t x[SAMPLES_PER_BLOCK], y[SAMPLES_PER_BLOCK], z[SAMPLES_PER_BLOCK];
};

int packet_bytes = 6;
double sample_rate_hz = NOMINAL_RATE_HZ;
const float m_s2_per_lsb = ACCEL_FS_G * G_M_S2 / 32768.0;
QueueHandle_t queue = nullptr;
volatile bool streaming = false;       // set by loop() when connected
volatile bool restart_request = false; // loop() -> imu_task: start a new stream (index 0)
volatile uint32_t fifo_overflows = 0, queue_overflows = 0, published = 0;

// ---------------------------------------------------------------- imu task (core 0)
double measure_rate_hz() {
  fifo_reset();
  uint64_t total = 0;
  const uint32_t t0 = micros();
  static uint8_t raw[FIFO_SIZE_BYTES];
  while (micros() - t0 < RATE_MEASURE_MS * 1000UL) {
    delay(5);  // ~20 samples per read, far below the FIFO size
    const int count = fifo_count();
    if (count >= packet_bytes) {
      const int samples = count / packet_bytes;
      if (reg_read(REG_FIFO_R_W, raw, static_cast<size_t>(samples) * packet_bytes)) total += samples;
    }
  }
  const int rest = fifo_count();  // the samples still in the FIFO were taken before now too
  if (rest > 0) total += rest / packet_bytes;
  const double elapsed_s = (micros() - t0) * 1e-6;
  return elapsed_s > 0 ? total / elapsed_s : NOMINAL_RATE_HZ;
}

void imu_task(void *) {
  static uint8_t raw[FIFO_SIZE_BYTES];
  Block blk{};
  size_t filled = 0;
  uint64_t next_index = 0;
  uint8_t pending_flags = 0;
  uint32_t last_read_us = micros();
  for (;;) {
    // always yield (vTaskDelayUntil would not if a read overran the period, starving IDLE0)
    vTaskDelay(pdMS_TO_TICKS(IMU_PERIOD_MS));
    if (restart_request) {
      fifo_reset();
      last_read_us = micros();
      next_index = 0;
      filled = 0;
      pending_flags = m5stack_imu_msgs__msg__AccelBlock__FLAG_RESYNC;
      restart_request = false;
    }
    if (!streaming) continue;

    uint8_t status = 0;
    reg_read(REG_INT_STATUS, &status, 1);
    const int count = fifo_count();
    if (count < 0) continue;
    const int64_t now_ns = rmw_uros_epoch_nanos();
    if ((status & 0x10) || count >= static_cast<int>(FIFO_SIZE_BYTES) - packet_bytes) {
      // FIFO overflow: it stopped writing at some point since the last read. Advance the
      // index by the elapsed time (approximate; the host only needs to see the jump) and
      // drop the partial block, whose samples are not contiguous with what follows.
      fifo_overflows++;
      fifo_reset();
      next_index += static_cast<uint64_t>((micros() - last_read_us) * 1e-6 * sample_rate_hz);
      filled = 0;
      pending_flags |= m5stack_imu_msgs__msg__AccelBlock__FLAG_FIFO_OVERFLOW;
      last_read_us = micros();
      continue;
    }
    const int samples = count / packet_bytes;
    if (samples <= 0) continue;
    if (!reg_read(REG_FIFO_R_W, raw, static_cast<size_t>(samples) * packet_bytes)) continue;
    last_read_us = micros();
    for (int i = 0; i < samples; ++i) {
      if (filled == 0) {
        // the newest sample in the FIFO was taken about now; sample i is (samples - 1 - i) earlier
        blk.stamp_ns = now_ns - static_cast<int64_t>((samples - 1 - i) * 1e9 / sample_rate_hz);
        blk.first_index = next_index;
      }
      const uint8_t *p = raw + i * packet_bytes;  // accel X, Y, Z first (big endian)
      blk.x[filled] = static_cast<int16_t>((p[0] << 8) | p[1]);
      blk.y[filled] = static_cast<int16_t>((p[2] << 8) | p[3]);
      blk.z[filled] = static_cast<int16_t>((p[4] << 8) | p[5]);
      filled++;
      next_index++;
      if (filled == SAMPLES_PER_BLOCK) {
        blk.flags = pending_flags;
        if (xQueueSend(queue, &blk, 0) == pdTRUE) {
          pending_flags = 0;
        } else {
          // the publisher fell behind: this block is lost, the next one shows the jump
          queue_overflows++;
          pending_flags |= m5stack_imu_msgs__msg__AccelBlock__FLAG_QUEUE_OVERFLOW;
        }
        filled = 0;
      }
    }
  }
}

// ---------------------------------------------------------------- micro-ROS (loop, core 1)
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t publisher;
m5stack_imu_msgs__msg__AccelBlock msg;
char frame_id[] = "m5stack_imu";
uint32_t last_sync_ms = 0;

enum class AgentState { Waiting, Available, Connected, Disconnected };
AgentState state = AgentState::Waiting;

// ---------------------------------------------------------------- ROS domain ID
// The micro-ROS client (this firmware), not the agent, picks the DDS domain. So that it
// follows the host's ROS_DOMAIN_ID (which changes with the PC's IP address), the host sends
//   "#ROS_DOMAIN_ID=<n>\n"   before it starts the agent, and this firmware answers
//   "#ROS_DOMAIN_ID_OK=<n>\n", keeps <n> in NVS and restarts, so that every agent session
// starts from a freshly booted micro-ROS client (the host's line can also reach the
// micro-ROS transport, which was then seen to stop answering until a reset).
// It is read only while waiting for the agent: the serial link then carries nothing else
// but the agent pings, whose bytes do not form such a line and are discarded.
constexpr int DOMAIN_ID_MAX = 232;  // highest domain ID ROS 2 accepts
constexpr const char *DOMAIN_PREFIX = "#ROS_DOMAIN_ID=";
Preferences prefs;
int domain_id = ROS_DOMAIN_ID;

// ---------------------------------------------------------------- display (core 1)
// Accelerometer statistics of the blocks published since the last refresh (m/s^2).
struct Stats {
  double sum[3] = {0, 0, 0}, sumsq[3] = {0, 0, 0};
  uint32_t n = 0;
} stats;
uint32_t blocks_since_refresh = 0;

void accumulate(const Block &blk) {
  const int16_t *axes[3] = {blk.x, blk.y, blk.z};
  for (int a = 0; a < 3; ++a) {
    for (size_t i = 0; i < SAMPLES_PER_BLOCK; ++i) {
      const double v = axes[a][i] * m_s2_per_lsb;
      stats.sum[a] += v;
      stats.sumsq[a] += v * v;
    }
  }
  stats.n += SAMPLES_PER_BLOCK;
  blocks_since_refresh++;
}

constexpr uint16_t COLOR_BG = TFT_BLACK;
constexpr double RMS_FULL_SCALE_M_S2 = 2.0;  // bar length = AC RMS / this

RTC_NOINIT_ATTR uint32_t boot_count;  // survives a software reset, not a power cycle
const char *reset_reason_text() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT WDT";
    case ESP_RST_TASK_WDT: return "TASK WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_EXT: return "external";
    default: return "other";
  }
}

// Fixed rows (320 x 240). Every row is cleared before it is drawn, and no text may be wider
// than the screen (size 1 = 6 px per character, size 2 = 12 px), or it wraps onto the next row.
constexpr int Y_INFO1 = 28, Y_INFO2 = 38, Y_STATE = 50, Y_RATE = 72, Y_SENT = 92, Y_LOST = 112;
constexpr int Y_AXIS0 = 138, AXIS_STEP = 28, Y_BOOT = 218, Y_HINT = 230;

void clear_row(int y, int text_size, uint16_t bg = COLOR_BG) {
  M5.Display.fillRect(0, y, 320, 8 * text_size, bg);
}

void text_row(int y, int text_size, uint16_t color, const char *text) {
  auto &d = M5.Display;
  clear_row(y, text_size);
  d.setTextSize(text_size);
  d.setTextColor(color, COLOR_BG);
  d.setCursor(4, y);
  d.print(text);
}

void draw_domain() {
  char line[64];
  snprintf(line, sizeof(line), "ROS domain %d  %s", domain_id, TOPIC);
  text_row(Y_INFO2, 1, TFT_LIGHTGREY, line);
}

void draw_static() {
  auto &d = M5.Display;
  d.fillScreen(COLOR_BG);
  d.fillRect(0, 0, 320, 24, TFT_NAVY);
  d.setTextSize(2);
  d.setTextColor(TFT_WHITE, TFT_NAVY);
  d.setCursor(4, 4);
  d.print("M5 IMU micro-ROS");
  char line[64];
  snprintf(line, sizeof(line), "MPU6886 on %s, +-%d g, 4 kHz", IMU_WHERE, ACCEL_FS_G);
  text_row(Y_INFO1, 1, TFT_LIGHTGREY, line);
  draw_domain();
  snprintf(line, sizeof(line), "board %d  reset: %s  boots: %lu", static_cast<int>(M5.getBoard()),
           reset_reason_text(), (unsigned long)boot_count);
  text_row(Y_BOOT, 1, TFT_LIGHTGREY, line);
  text_row(Y_HINT, 1, TFT_LIGHTGREY, "axis: mean m/s^2, vibration RMS m/s^2 (bar 0-2)");
}

// state: one line, coloured; numbers below; refreshed every DISPLAY_PERIOD_MS
void draw_dynamic(const char *state_text, uint16_t state_color, double elapsed_s) {
  char line[64];
  text_row(Y_STATE, 2, state_color, state_text);
  snprintf(line, sizeof(line), "rate %.1f Hz", sample_rate_hz);
  text_row(Y_RATE, 2, TFT_WHITE, line);
  snprintf(line, sizeof(line), "sent %.1f blk/s", elapsed_s > 0 ? blocks_since_refresh / elapsed_s : 0.0);
  text_row(Y_SENT, 2, TFT_WHITE, line);
  const bool lost = fifo_overflows || queue_overflows;
  snprintf(line, sizeof(line), "lost fifo %lu queue %lu", (unsigned long)fifo_overflows,
           (unsigned long)queue_overflows);
  text_row(Y_LOST, 2, lost ? TFT_RED : TFT_GREEN, line);

  const char *names[3] = {"x", "y", "z"};
  auto &d = M5.Display;
  for (int a = 0; a < 3; ++a) {
    const int y = Y_AXIS0 + a * AXIS_STEP;
    double mean = 0, rms = 0;
    if (stats.n > 0) {
      mean = stats.sum[a] / stats.n;
      rms = sqrt(fmax(stats.sumsq[a] / stats.n - mean * mean, 0.0));  // AC RMS (gravity removed)
    }
    if (stats.n > 0)
      snprintf(line, sizeof(line), "%s %+6.2f %5.3f", names[a], mean, rms);  // 15 chars = 180 px
    else
      snprintf(line, sizeof(line), "%s   ---    ---", names[a]);
    clear_row(y, 2);
    d.setTextSize(2);
    d.setTextColor(TFT_WHITE, COLOR_BG);
    d.setCursor(4, y);
    d.print(line);
    const int bar_x = 196, bar_w = 120;
    const int len = static_cast<int>(fmin(rms / RMS_FULL_SCALE_M_S2, 1.0) * bar_w);
    d.fillRect(bar_x, y, len, 16, TFT_ORANGE);
    d.fillRect(bar_x + len, y, bar_w - len, 16, TFT_DARKGREY);
  }
  stats = Stats();
  blocks_since_refresh = 0;
}

bool create_entities() {
  allocator = rcl_get_default_allocator();
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  if (rcl_init_options_init(&init_options, allocator) != RCL_RET_OK) return false;
  if (rcl_init_options_set_domain_id(&init_options, domain_id) != RCL_RET_OK) return false;
  if (rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator) != RCL_RET_OK)
    return false;
  if (rclc_node_init_default(&node, "m5stack_imu", "", &support) != RCL_RET_OK) return false;
  // best effort: a block that cannot be delivered is dropped (the host sees the index gap)
  if (rclc_publisher_init_best_effort(&publisher, &node,
                                      ROSIDL_GET_MSG_TYPE_SUPPORT(m5stack_imu_msgs, msg, AccelBlock),
                                      TOPIC) != RCL_RET_OK)
    return false;
  rmw_uros_sync_session(1000);
  last_sync_ms = millis();
  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);
  (void)rcl_publisher_fini(&publisher, &node);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

void publish(const Block &blk) {
  accumulate(blk);
  msg.header.stamp.sec = static_cast<int32_t>(blk.stamp_ns / 1000000000LL);
  msg.header.stamp.nanosec = static_cast<uint32_t>(blk.stamp_ns % 1000000000LL);
  msg.first_sample_index = blk.first_index;
  msg.sample_rate_hz = sample_rate_hz;
  msg.accel_m_s2_per_lsb = m_s2_per_lsb;
  msg.x.data = const_cast<int16_t *>(blk.x);
  msg.y.data = const_cast<int16_t *>(blk.y);
  msg.z.data = const_cast<int16_t *>(blk.z);
  msg.x.size = msg.y.size = msg.z.size = SAMPLES_PER_BLOCK;
  msg.x.capacity = msg.y.capacity = msg.z.capacity = SAMPLES_PER_BLOCK;
  msg.flags = blk.flags;
  if (rcl_publish(&publisher, &msg, nullptr) == RCL_RET_OK) published++;
}

void load_domain_id() {
  prefs.begin("m5imu", false);
  domain_id = prefs.getInt("domain_id", ROS_DOMAIN_ID);
}

// Reads what the host sent while no agent session is open; see "ROS domain ID" above.
void poll_domain_id() {
  static char line[32];
  static size_t len = 0;
  while (Serial.available() > 0) {
    const int c = Serial.read();
    if (c != '\n') {
      if (c == '#') len = 0;  // a command starts here (ping bytes before it are dropped)
      if (len < sizeof(line) - 1) line[len++] = static_cast<char>(c);
      continue;
    }
    line[len] = '\0';
    len = 0;
    const size_t n = strlen(DOMAIN_PREFIX);
    if (strncmp(line, DOMAIN_PREFIX, n) != 0) continue;
    char *end = nullptr;
    const long id = strtol(line + n, &end, 10);
    if (end == line + n || (*end != '\0' && *end != '\r') || id < 0 || id > DOMAIN_ID_MAX)
      continue;
    if (id != domain_id) {
      domain_id = static_cast<int>(id);
      prefs.putInt("domain_id", domain_id);
      draw_domain();
    }
    Serial.printf("#ROS_DOMAIN_ID_OK=%d\n", domain_id);
    Serial.flush();
    text_row(Y_STATE, 2, TFT_YELLOW, "restarting...");
    prefs.end();
    delay(50);
    esp_restart();
  }
}

void fatal(const char *text) {
  text_row(Y_STATE, 2, TFT_RED, text);
  while (true) delay(1000);
}

}  // namespace

void setup() {
  if (esp_reset_reason() == ESP_RST_POWERON) boot_count = 0;
  boot_count++;
  auto cfg = M5.config();
  cfg.serial_baudrate = 0;   // Serial is started below, with a larger TX buffer
  cfg.internal_imu = false;  // the MPU6886 is driven here (M5Unified must not configure it)
  M5.begin(cfg);
  // brightness is not restored by every M5Unified board path; set it explicitly
  M5.Display.wakeup();
  M5.Display.setBrightness(160);
  // micro-ROS writes ~30 KB/s; a large TX buffer keeps rcl_publish from blocking on the UART
  Serial.setTxBufferSize(8192);
  Serial.setRxBufferSize(1024);
  Serial.begin(SERIAL_BAUD);
  load_domain_id();
  draw_static();
  text_row(Y_STATE, 2, TFT_YELLOW, "starting IMU...");

#if !IMU_INTERNAL
  // M5.begin does not start Ex_I2C: on the Basic, Port A shares the internal bus and worked
  // anyway, but on the Core2 Port A is a separate bus (GPIO32/33) that stays silent without this
  M5.Ex_I2C.begin();
#endif
  if (!mpu_init()) fatal("MPU6886 not found (" IMU_WHERE ")");
  packet_bytes = detect_packet_bytes();
  if (packet_bytes < 0) fatal("FIFO packet size unknown");
  sample_rate_hz = measure_rate_hz();

  queue = xQueueCreate(QUEUE_BLOCKS, sizeof(Block));
  if (queue == nullptr) fatal("no memory for queue");
  xTaskCreatePinnedToCore(imu_task, "imu", 8192, nullptr, 5, nullptr, 0);

  set_microros_serial_transports(Serial);
  msg.header.frame_id.data = frame_id;
  msg.header.frame_id.size = strlen(frame_id);
  msg.header.frame_id.capacity = sizeof(frame_id);
  draw_static();
  draw_dynamic("waiting for agent", TFT_YELLOW, 0);
}

constexpr uint32_t DISPLAY_PERIOD_MS = 250;
#ifndef PING_FAILURES_TO_DISCONNECT
#define PING_FAILURES_TO_DISCONNECT 3
#endif

void loop() {
  static uint32_t last_ping_ms = 0, last_display_ms = 0;
  static int ping_failures = 0;
  const uint32_t now = millis();
  switch (state) {
    case AgentState::Waiting:
      poll_domain_id();
      if (now - last_ping_ms > 500) {
        last_ping_ms = now;
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) state = AgentState::Available;
      }
      if (now - last_display_ms > 1000) {
        last_display_ms = now;
        draw_dynamic("waiting for agent", TFT_YELLOW, 0);
      }
      break;
    case AgentState::Available:
      if (create_entities()) {
        xQueueReset(queue);
        restart_request = true;
        streaming = true;
        ping_failures = 0;
        state = AgentState::Connected;
        last_display_ms = now;
      } else {
        destroy_entities();
        state = AgentState::Waiting;
      }
      break;
    case AgentState::Connected: {
      Block blk;
      while (xQueueReceive(queue, &blk, 0) == pdTRUE) publish(blk);
      if (now - last_ping_ms > 2000) {
        last_ping_ms = now;
        // a busy serial link can delay one answer; the IMU task keeps buffering meanwhile
        ping_failures = rmw_uros_ping_agent(100, 1) == RMW_RET_OK ? 0 : ping_failures + 1;
        if (ping_failures >= PING_FAILURES_TO_DISCONNECT) state = AgentState::Disconnected;
      }
      if (now - last_sync_ms > TIME_SYNC_PERIOD_MS) {
        rmw_uros_sync_session(50);
        last_sync_ms = now;
      }
      if (now - last_display_ms > DISPLAY_PERIOD_MS) {
        draw_dynamic("connected", TFT_GREEN, (now - last_display_ms) * 1e-3);
        last_display_ms = now;
      }
      delay(1);
      break;
    }
    case AgentState::Disconnected:
      streaming = false;
      destroy_entities();
      state = AgentState::Waiting;
      break;
  }
}
