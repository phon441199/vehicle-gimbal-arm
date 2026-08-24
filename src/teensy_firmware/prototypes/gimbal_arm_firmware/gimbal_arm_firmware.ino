/**
 * gimbal_arm_firmware.ino
 * Teensy 4.1 — micro-ROS 5축 로봇 팔 펌웨어
 *
 * 조인트 매핑:
 *   0: base_yaw    — 디지털 서보  (PWM)
 *   1: shoulder    — QDD 모터    (CAN / UART)
 *   2: elbow       — QDD 모터    (CAN / UART)
 *   3: wrist_pitch — 짐벌 모터   (PWM / UART)
 *   4: wrist_roll  — 짐벌 모터   (PWM / UART)
 *
 * micro-ROS 통신: USB Serial (115200 baud)
 *
 * Board 설정: Tools → Board → Teensyduino → Teensy 4.1
 *             Tools → USB Type → Serial
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/joint_state.h>
#include <Servo.h>

// ── 설정 ──────────────────────────────────────────────────────────────
#define NUM_JOINTS           5
#define STATE_PUB_HZ         50
#define STATE_PUB_PERIOD_MS  (1000 / STATE_PUB_HZ)
#define STRING_BUFFER_LEN    20

// 핀 번호
#define SERVO_BASE_YAW_PIN       2
#define GIMBAL_WRIST_PITCH_PIN   3
#define GIMBAL_WRIST_ROLL_PIN    4

// ── micro-ROS 객체 ────────────────────────────────────────────────────
rcl_node_t          node;
rcl_allocator_t     allocator;
rclc_support_t      support;
rclc_executor_t     executor;
rcl_subscription_t  joint_cmd_sub;
rcl_publisher_t     joint_state_pub;

sensor_msgs__msg__JointState joint_cmd_msg;
sensor_msgs__msg__JointState joint_state_msg;

// ── 정적 메모리 버퍼 (micro_ros_arduino은 동적 시퀀스 init 미지원) ───
#define MAKE_STATIC_BUFFERS(prefix)                                   \
    static double         prefix##_pos[NUM_JOINTS];                   \
    static double         prefix##_vel[NUM_JOINTS];                   \
    static double         prefix##_eff[NUM_JOINTS];                   \
    static rosidl_runtime_c__String  prefix##_names[NUM_JOINTS];      \
    static char           prefix##_name_buf[NUM_JOINTS][STRING_BUFFER_LEN];

MAKE_STATIC_BUFFERS(cmd)
MAKE_STATIC_BUFFERS(state)

// ── 서보/짐벌 객체 ────────────────────────────────────────────────────
Servo servo_base_yaw;
Servo gimbal_wrist_pitch;
Servo gimbal_wrist_roll;

double current_positions[NUM_JOINTS]  = {0.0};
double current_velocities[NUM_JOINTS] = {0.0};
double current_efforts[NUM_JOINTS]    = {0.0};

unsigned long last_pub_ms = 0;

const char* joint_names[NUM_JOINTS] = {
    "base_yaw", "shoulder", "elbow", "wrist_pitch", "wrist_roll"
};

// ── 에러 처리 ─────────────────────────────────────────────────────────
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if(rc != RCL_RET_OK){ error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
    while (true) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        delay(100);
    }
}

// ── 메시지 정적 메모리 연결 ───────────────────────────────────────────
void bind_msg_buffers(sensor_msgs__msg__JointState* msg,
                      rosidl_runtime_c__String* names,
                      char name_buf[][STRING_BUFFER_LEN],
                      double* pos, double* vel, double* eff)
{
    for (int i = 0; i < NUM_JOINTS; i++) {
        strncpy(name_buf[i], joint_names[i], STRING_BUFFER_LEN - 1);
        name_buf[i][STRING_BUFFER_LEN - 1] = '\0';
        names[i].data     = name_buf[i];
        names[i].size     = strlen(joint_names[i]);
        names[i].capacity = STRING_BUFFER_LEN;
        pos[i] = vel[i] = eff[i] = 0.0;
    }

    msg->name.data     = names;
    msg->name.size     = NUM_JOINTS;
    msg->name.capacity = NUM_JOINTS;

    msg->position.data     = pos;
    msg->position.size     = NUM_JOINTS;
    msg->position.capacity = NUM_JOINTS;

    msg->velocity.data     = vel;
    msg->velocity.size     = NUM_JOINTS;
    msg->velocity.capacity = NUM_JOINTS;

    msg->effort.data     = eff;
    msg->effort.size     = NUM_JOINTS;
    msg->effort.capacity = NUM_JOINTS;
}

// ── 모터 드라이버 ────────────────────────────────────────────────────

void drive_servo_base_yaw(double rad) {
    // -π ~ +π → 0 ~ 180도
    int deg = (int)((rad + 3.14159265) / (2.0 * 3.14159265) * 180.0);
    servo_base_yaw.write(constrain(deg, 0, 180));
    current_positions[0] = rad;
}

void drive_qdd_shoulder(double rad) {
    // TODO: CAN/UART 프로토콜 구현
    current_positions[1] = rad;
}

void drive_qdd_elbow(double rad) {
    // TODO: CAN/UART 프로토콜 구현
    current_positions[2] = rad;
}

void drive_gimbal_wrist_pitch(double rad) {
    // -π/4 ~ +π/4 → 45 ~ 135도
    int deg = (int)((rad + 0.7854) / (2.0 * 0.7854) * 90.0) + 45;
    gimbal_wrist_pitch.write(constrain(deg, 0, 180));
    current_positions[3] = rad;
}

void drive_gimbal_wrist_roll(double rad) {
    int deg = (int)((rad + 0.7854) / (2.0 * 0.7854) * 90.0) + 45;
    gimbal_wrist_roll.write(constrain(deg, 0, 180));
    current_positions[4] = rad;
}

// ── /joint_commands 콜백 ─────────────────────────────────────────────

void joint_cmd_callback(const void* msgin) {
    const sensor_msgs__msg__JointState* msg =
        (const sensor_msgs__msg__JointState*)msgin;

    for (size_t i = 0; i < msg->name.size && i < (size_t)NUM_JOINTS; i++) {
        const char* name = msg->name.data[i].data;
        double pos = (i < msg->position.size) ? msg->position.data[i] : 0.0;

        if      (strcmp(name, "base_yaw") == 0)     drive_servo_base_yaw(pos);
        else if (strcmp(name, "shoulder") == 0)      drive_qdd_shoulder(pos);
        else if (strcmp(name, "elbow") == 0)         drive_qdd_elbow(pos);
        else if (strcmp(name, "wrist_pitch") == 0)   drive_gimbal_wrist_pitch(pos);
        else if (strcmp(name, "wrist_roll") == 0)    drive_gimbal_wrist_roll(pos);
    }
}

// ── /joint_states 발행 ───────────────────────────────────────────────

void publish_joint_states() {
    for (int i = 0; i < NUM_JOINTS; i++) {
        joint_state_msg.position.data[i] = current_positions[i];
        joint_state_msg.velocity.data[i] = current_velocities[i];
        joint_state_msg.effort.data[i]   = current_efforts[i];
    }
    int64_t ms = rmw_uros_epoch_millis();
    joint_state_msg.header.stamp.sec     = (int32_t)(ms / 1000);
    joint_state_msg.header.stamp.nanosec = (uint32_t)((ms % 1000) * 1000000UL);

    RCSOFTCHECK(rcl_publish(&joint_state_pub, &joint_state_msg, NULL));
}

// ── setup / loop ──────────────────────────────────────────────────────

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);

    servo_base_yaw.attach(SERVO_BASE_YAW_PIN);
    gimbal_wrist_pitch.attach(GIMBAL_WRIST_PITCH_PIN);
    gimbal_wrist_roll.attach(GIMBAL_WRIST_ROLL_PIN);

    set_microros_transports();
    delay(2000);

    allocator = rcl_get_default_allocator();
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    RCCHECK(rcl_init_options_set_domain_id(&init_options, 5));
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
    RCCHECK(rclc_node_init_default(&node, "gimbal_arm_teensy", "", &support));

    // Publisher
    RCCHECK(rclc_publisher_init_default(
        &joint_state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_states"
    ));

    // Subscriber — 수신 버퍼에 정적 메모리 연결
    bind_msg_buffers(&joint_cmd_msg,
                     cmd_names, cmd_name_buf,
                     cmd_pos, cmd_vel, cmd_eff);
    RCCHECK(rclc_subscription_init_default(
        &joint_cmd_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/joint_commands"
    ));

    // 발행 버퍼에 정적 메모리 연결
    bind_msg_buffers(&joint_state_msg,
                     state_names, state_name_buf,
                     state_pos, state_vel, state_eff);

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &joint_cmd_sub, &joint_cmd_msg,
        &joint_cmd_callback, ON_NEW_DATA
    ));

    rmw_uros_sync_session(1000);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {
    RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));

    unsigned long now = millis();
    if (now - last_pub_ms >= STATE_PUB_PERIOD_MS) {
        publish_joint_states();
        last_pub_ms = now;
    }
}
