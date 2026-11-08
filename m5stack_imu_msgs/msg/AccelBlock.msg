# One block of consecutive accelerometer samples from the M5Stack firmware
# (firmware/, MPU6886 IMU Unit on Port A).
#
# header.stamp: estimated host ROS time of sample 0 of this block, from the micro-ROS
#   time sync (rmw_uros_sync_session) and the ESP32 clock. Good to a few ms; use
#   first_sample_index for the spacing of samples (it is exact).
# Sample k of this block is sample (first_sample_index + k) since the firmware started
# sampling; consecutive blocks continue without gaps unless FLAG_FIFO_OVERFLOW or
# FLAG_QUEUE_OVERFLOW is set (blocks lost in transport show only as an index gap).
std_msgs/Header header

float64 sample_rate_hz            # Hz, measured once at start-up against the ESP32 clock (nominal 4000)
uint64 first_sample_index
float32 accel_m_s2_per_lsb        # m/s^2 per LSB of x/y/z (full scale / 32768 * 9.80665)

# raw MPU6886 accelerometer output, sensor axes (see the IMU Unit's marking)
int16[] x
int16[] y
int16[] z

uint8 FLAG_FIFO_OVERFLOW = 1      # samples were lost before this block (FIFO overflowed)
uint8 FLAG_RESYNC = 2             # the first block after (re)starting; first_sample_index restarts at 0
uint8 FLAG_QUEUE_OVERFLOW = 4     # blocks were dropped on the M5Stack before this one (publishing fell behind)
uint8 flags
