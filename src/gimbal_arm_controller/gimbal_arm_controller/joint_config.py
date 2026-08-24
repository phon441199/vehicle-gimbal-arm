import math

# 조인트 이름 (Teensy 펌웨어와 순서 동일하게 유지)
JOINT_NAMES = [
    'base_yaw',     # 0: 디지털 서보 - 베이스 yaw 회전
    'shoulder',     # 1: QDD 모터 - 어깨
    'elbow',        # 2: QDD 모터 - 팔꿈치
    'wrist_pitch',  # 3: 짐벌 모터 - 손목 pitch
    'wrist_roll',   # 4: 짐벌 모터 - 손목 roll
]

JOINT_TYPES = {
    'base_yaw':    'servo',
    'shoulder':    'qdd',
    'elbow':       'qdd',
    'wrist_pitch': 'gimbal',
    'wrist_roll':  'gimbal',
}

# 단위: 라디안
JOINT_LIMITS = {
    'base_yaw':    (-math.pi / 2,    math.pi / 2),
    'shoulder':    (-math.pi / 2,    math.pi / 2),
    'elbow':       (-math.pi * 2/3,  math.pi * 2/3),
    'wrist_pitch': (-math.pi / 4,    math.pi / 4),
    'wrist_roll':  (-math.pi / 4,    math.pi / 4),
}

# 조인트별 속도/가속도 한계 (라디안/s, 라디안/s²) -- quintic peak 제한용.
# 보수적 기본값: base_yaw 디지털서보 ~6.5rad/s, AK40 38rad/s지만 안전하게 낮춤,
# 짐벌 손목 느림. HW 검증 후 조정.
JOINT_VEL_LIMITS = {
    'base_yaw':    5.0,
    'shoulder':    5.0,
    'elbow':       5.0,
    'wrist_pitch': 4.0,
    'wrist_roll':  4.0,
}
JOINT_ACC_LIMITS = {
    'base_yaw':    20.0,
    'shoulder':    20.0,
    'elbow':       20.0,
    'wrist_pitch': 15.0,
    'wrist_roll':  15.0,
}

NUM_JOINTS = len(JOINT_NAMES)
