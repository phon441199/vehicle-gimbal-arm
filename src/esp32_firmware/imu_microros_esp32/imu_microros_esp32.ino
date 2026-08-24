/**
 * imu_microros_esp32.ino  -  LOLIN D32 (ESP32)
 * ICM20948 IMU standalone over micro-ROS. ESP32/SPI port of the Teensy
 * imu_microros_test.ino (verified ROS structure reused 1:1; only the IMU
 * transport I2C->SPI and the MCU-specific reboot/LED changed).
 *
 * Publishes /imu/data (sensor_msgs/msg/Imu) @ 50Hz, frame_id=imu_link, domain=5.
 * Use with the existing visualizer:
 *   ros2 launch imu_test_visualizer imu_test.launch.py
 *
 * ---- WIRING (shared SPI bus, verified pinmap) ------------------------------
 *   SCK=18  MISO=19  MOSI=23   IMU_CS=13
 *   ICM20948 SPI: SCL=SCK SDA=MOSI AD0=MISO NCS=IMU_CS  (FSYNC->GND)
 *   ENC1_CS=21 ENC2_CS=22 parked HIGH (shared bus; harmless if encoders absent)
 *   Star ground; 1MHz SPI for the 70cm LAN run.
 * ----------------------------------------------------------------------------
 *
 * micro-ROS transport = USB serial @115200. After upload, run the agent and
 * replug if needed (ESP32 = /dev/ttyUSB*, not ttyACM):
 *   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -b 115200
 *
 * NOTE: serial is owned by micro-ROS -- no Serial.print debug here.
 *
 * Libraries: SparkFun ICM_20948, Madgwick, micro_ros_arduino
 */

#include <SPI.h>
#include <ICM_20948.h>
#include <MadgwickAHRS.h>

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>

// ─── config ────────────────────────────────────────────────────────────────
#define PIN_SCK   18
#define PIN_MISO  19
#define PIN_MOSI  23
#define ENC1_CS   21
#define ENC2_CS   22
#define IMU_CS    13
#define IMU_SPI_HZ     1000000UL      // 1MHz: margin for 70cm shared bus
#define IMU_HZ         50
#define IMU_PERIOD_MS  (1000 / IMU_HZ)
#define ROS_DOMAIN_ID  5

#ifndef LED_BUILTIN
#define LED_BUILTIN 5                 // LOLIN D32 onboard LED
#endif

// boot-time 15s figure-8 magnetometer calibration: uncomment to enable
// #define CALIBRATE_MAG
#define MAG_CAL_DURATION_MS 15000

// ─── IMU ─────────────────────────────────────────────────────────────────
ICM_20948_SPI myICM;
Madgwick filter;

float mag_offset[3] = {0, 0, 0};
float mag_scale[3]  = {1, 1, 1};

// ─── micro-ROS ─────────────────────────────────────────────────────────────
rcl_node_t          node;
rcl_allocator_t     allocator;
rclc_support_t      support;
rcl_publisher_t     imu_pub;
sensor_msgs__msg__Imu imu_msg;
static char imu_frame_id[16] = "imu_link";

unsigned long last_pub_ms = 0;
int pub_fail_count = 0;
#define PUB_FAIL_LIMIT 100

#define RCCHECK(fn) { rcl_ret_t rc = fn; if(rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
    while (true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// Park MKS gimbal driver pins OFF. This firmware uses only the IMU, but the
// PWM/EN pins are physically wired to the MKS board. Left floating they make
// the gate drivers twitch the motors (worst at the DTR/RTS reset the agent
// triggers on opening the port). Drive them LOW first thing. See
// park-wired-pins-brownout. EN LOW = driver off, PWM LOW = no drive.
static const uint8_t GIMBAL_PARK_PINS[] = {25, 26, 27, 14, 16, 17, 4, 12};
static void park_gimbal_drivers() {
    for (uint8_t i = 0; i < sizeof(GIMBAL_PARK_PINS); i++) {
        pinMode(GIMBAL_PARK_PINS[i], OUTPUT);
        digitalWrite(GIMBAL_PARK_PINS[i], LOW);
    }
}

// ─── IMU init (SPI) ─────────────────────────────────────────────────────────
bool setup_icm20948() {
    myICM.begin(IMU_CS, SPI, IMU_SPI_HZ);
    if (myICM.status != ICM_20948_Stat_Ok) return false;

    ICM_20948_fss_t fss;
    fss.a = gpm4;
    fss.g = dps500;
    myICM.setFullScale(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, fss);

    ICM_20948_dlpcfg_t dlpcfg;
    dlpcfg.a = acc_d50bw4_n68bw8;
    dlpcfg.g = gyr_d51bw2_n73bw3;
    myICM.setDLPFcfg(ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr, dlpcfg);
    myICM.enableDLPF(ICM_20948_Internal_Acc, true);
    myICM.enableDLPF(ICM_20948_Internal_Gyr, true);

    ICM_20948_smplrt_t srd;
    srd.a = 10;
    srd.g = 10;
    myICM.setSampleRate(ICM_20948_Internal_Acc, srd);
    myICM.setSampleRate(ICM_20948_Internal_Gyr, srd);

    myICM.startupMagnetometer();
    return true;
}

#ifdef CALIBRATE_MAG
void calibrate_magnetometer() {
    float mn[3] = { 1e9,  1e9,  1e9};
    float mx[3] = {-1e9, -1e9, -1e9};
    unsigned long start = millis();
    while (millis() - start < MAG_CAL_DURATION_MS) {
        if (myICM.dataReady()) {
            myICM.getAGMT();
            float m[3] = {myICM.magX(), myICM.magY(), myICM.magZ()};
            for (int i = 0; i < 3; i++) {
                if (m[i] < mn[i]) mn[i] = m[i];
                if (m[i] > mx[i]) mx[i] = m[i];
            }
        }
        digitalWrite(LED_BUILTIN, (millis() / 250) % 2);
        delay(10);
    }
    for (int i = 0; i < 3; i++) mag_offset[i] = (mx[i] + mn[i]) / 2.0f;
    float delta[3], avg = 0;
    for (int i = 0; i < 3; i++) { delta[i] = (mx[i] - mn[i]) / 2.0f; avg += delta[i]; }
    avg /= 3.0f;
    for (int i = 0; i < 3; i++) mag_scale[i] = (delta[i] > 0.01f) ? avg / delta[i] : 1.0f;
}
#endif

void eulerToQuaternion(float roll, float pitch, float yaw,
                       float &qw, float &qx, float &qy, float &qz) {
    float cr = cosf(roll  * 0.5f), sr = sinf(roll  * 0.5f);
    float cp = cosf(pitch * 0.5f), sp = sinf(pitch * 0.5f);
    float cy = cosf(yaw   * 0.5f), sy = sinf(yaw   * 0.5f);
    qw = cr*cp*cy + sr*sp*sy;
    qx = sr*cp*cy - cr*sp*sy;
    qy = cr*sp*cy + sr*cp*sy;
    qz = cr*cp*sy - sr*sp*cy;
}

// ─── publish ─────────────────────────────────────────────────────────────
void publish_imu(float ax_ms2, float ay_ms2, float az_ms2,
                 float gx_rads, float gy_rads, float gz_rads) {
    int64_t ms = rmw_uros_epoch_millis();
    imu_msg.header.stamp.sec     = (int32_t)(ms / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);

    float qw, qx, qy, qz;
    eulerToQuaternion(filter.getRollRadians(), filter.getPitchRadians(),
                      filter.getYawRadians(), qw, qx, qy, qz);
    imu_msg.orientation.w = qw;
    imu_msg.orientation.x = qx;
    imu_msg.orientation.y = qy;
    imu_msg.orientation.z = qz;

    imu_msg.angular_velocity.x = gx_rads;
    imu_msg.angular_velocity.y = gy_rads;
    imu_msg.angular_velocity.z = gz_rads;

    imu_msg.linear_acceleration.x = ax_ms2;
    imu_msg.linear_acceleration.y = ay_ms2;
    imu_msg.linear_acceleration.z = az_ms2;

    rcl_ret_t rc = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (rc != RCL_RET_OK) {
        pub_fail_count++;
        if (pub_fail_count >= PUB_FAIL_LIMIT) ESP.restart();   // agent lost -> reboot
    } else {
        pub_fail_count = 0;
    }
}

// ─── setup ─────────────────────────────────────────────────────────────────
void setup() {
    park_gimbal_drivers();
    pinMode(LED_BUILTIN, OUTPUT);

    // shared SPI bus: bring up once, park all CS high
    SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, -1);
    pinMode(ENC1_CS, OUTPUT); digitalWrite(ENC1_CS, HIGH);
    pinMode(ENC2_CS, OUTPUT); digitalWrite(ENC2_CS, HIGH);
    pinMode(IMU_CS,  OUTPUT); digitalWrite(IMU_CS,  HIGH);
    delay(100);

    bool imu_ok = false;
    for (int i = 0; i < 20; i++) {
        if (setup_icm20948()) { imu_ok = true; break; }
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(500);
    }
    if (!imu_ok) {
        while (true) {  // 3 fast blinks = IMU not found
            for (int i = 0; i < 3; i++) {
                digitalWrite(LED_BUILTIN, HIGH); delay(80);
                digitalWrite(LED_BUILTIN, LOW);  delay(80);
            }
            delay(1000);
        }
    }

    filter.begin(IMU_HZ);

#ifdef CALIBRATE_MAG
    calibrate_magnetometer();
#endif

    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);

    // micro-ROS (serial transport over USB)
    set_microros_transports();
    delay(2000);

    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "imu_test_esp32", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/data"));

    imu_msg.header.frame_id.data     = imu_frame_id;
    imu_msg.header.frame_id.size     = strlen(imu_frame_id);
    imu_msg.header.frame_id.capacity = sizeof(imu_frame_id);

    memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.orientation_covariance[4] = 0.01;
    imu_msg.orientation_covariance[8] = 0.01;

    memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
    imu_msg.angular_velocity_covariance[0] = 6.8e-6;
    imu_msg.angular_velocity_covariance[4] = 6.8e-6;
    imu_msg.angular_velocity_covariance[8] = 6.8e-6;

    memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));
    imu_msg.linear_acceleration_covariance[0] = 0.005;
    imu_msg.linear_acceleration_covariance[4] = 0.005;
    imu_msg.linear_acceleration_covariance[8] = 0.005;

    rmw_uros_sync_session(1000);
    digitalWrite(LED_BUILTIN, HIGH);  // solid on = session OK
}

// ─── loop ──────────────────────────────────────────────────────────────────
void loop() {
    unsigned long now = millis();
    if (now - last_pub_ms < IMU_PERIOD_MS) return;
    last_pub_ms = now;

    if (!myICM.dataReady()) return;
    myICM.getAGMT();

    float mx_cal = (myICM.magX() - mag_offset[0]) * mag_scale[0];
    float my_cal = (myICM.magY() - mag_offset[1]) * mag_scale[1];
    float mz_cal = (myICM.magZ() - mag_offset[2]) * mag_scale[2];

    filter.update(myICM.gyrX(), myICM.gyrY(), myICM.gyrZ(),
                  myICM.accX()/1000.0f, myICM.accY()/1000.0f, myICM.accZ()/1000.0f,
                  mx_cal, my_cal, mz_cal);

    float ax_ms2  = myICM.accX() * 9.80665f / 1000.0f;
    float ay_ms2  = myICM.accY() * 9.80665f / 1000.0f;
    float az_ms2  = myICM.accZ() * 9.80665f / 1000.0f;
    float gx_rads = myICM.gyrX() * (PI / 180.0f);
    float gy_rads = myICM.gyrY() * (PI / 180.0f);
    float gz_rads = myICM.gyrZ() * (PI / 180.0f);

    publish_imu(ax_ms2, ay_ms2, az_ms2, gx_rads, gy_rads, gz_rads);
}
