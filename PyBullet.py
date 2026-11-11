import pybullet as p
import pybullet_data
import time
import math
import csv


# ============================================================
# 1. PyBullet 시작
# ============================================================

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# 중력
p.setGravity(0, 0, -9.81)

# 1초 = 60 step
TIME_STEP = 1.0 / 60.0
p.setTimeStep(TIME_STEP)


# ============================================================
# 2. PyBullet 화면 설정
# ============================================================

# 좌우 GUI 패널 숨기기
p.configureDebugVisualizer(
    p.COV_ENABLE_GUI,
    0
)

# 그림자
p.configureDebugVisualizer(
    p.COV_ENABLE_SHADOWS,
    1
)


# ============================================================
# 3. 차량 파라미터
# ============================================================

CAR_LENGTH = 2.0
CAR_WIDTH = 1.0
CAR_HEIGHT = 0.40

# 연료를 제외한 차량 질량
CAR_MASS = 500.0

# 최대 속도
MAX_SPEED = 5.0

# 가속도
ACCELERATION = 1.0

# 감속도
DECELERATION = 1.0


# ============================================================
# 4. 연료 파라미터
# ============================================================

INITIAL_FUEL = 100.0
fuel = INITIAL_FUEL

# 최대속도에서 초당 연료 소비량
MAX_FUEL_RATE = 0.50


# ============================================================
# 5. 시뮬레이션 시간 설정
# ============================================================

# 15초부터 방향 전환
TURN_START_TIME = 15.0

# 20초에 90도 회전 완료
TURN_END_TIME = 20.0

# 30초부터 감속
BRAKE_START_TIME = 30.0

# 37초 종료
SIMULATION_END_TIME = 37.0


# ============================================================
# 6. 바닥 생성
# ============================================================

plane_id = p.loadURDF("plane.urdf")


# ============================================================
# 7. 자동차 충돌 모델
# ============================================================

car_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[
        CAR_LENGTH / 2,
        CAR_WIDTH / 2,
        CAR_HEIGHT / 2
    ]
)


# ============================================================
# 8. 자동차 외형 생성
# ============================================================

wheel_orientation = p.getQuaternionFromEuler(
    [math.pi / 2, 0, 0]
)

car_visual = p.createVisualShapeArray(

    shapeTypes=[
        p.GEOM_BOX,
        p.GEOM_BOX,
        p.GEOM_BOX,

        p.GEOM_CYLINDER,
        p.GEOM_CYLINDER,
        p.GEOM_CYLINDER,
        p.GEOM_CYLINDER
    ],

    halfExtents=[
        [1.0, 0.5, 0.20],
        [0.45, 0.42, 0.22],
        [0.05, 0.38, 0.17],

        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ],

    radii=[
        0,
        0,
        0,

        0.22,
        0.22,
        0.22,
        0.22
    ],

    lengths=[
        0,
        0,
        0,

        0.16,
        0.16,
        0.16,
        0.16
    ],

    visualFramePositions=[
        [0, 0, 0],              # 차체
        [-0.10, 0, 0.38],       # 지붕
        [0.38, 0, 0.38],        # 앞유리

        [0.62, 0.55, -0.10],
        [0.62, -0.55, -0.10],
        [-0.62, 0.55, -0.10],
        [-0.62, -0.55, -0.10]
    ],

    visualFrameOrientations=[
        [0, 0, 0, 1],
        [0, 0, 0, 1],
        [0, 0, 0, 1],

        wheel_orientation,
        wheel_orientation,
        wheel_orientation,
        wheel_orientation
    ],

    rgbaColors=[
        [0.05, 0.25, 0.85, 1],
        [0.10, 0.35, 0.95, 1],
        [0.30, 0.75, 0.95, 1],

        [0.05, 0.05, 0.05, 1],
        [0.05, 0.05, 0.05, 1],
        [0.05, 0.05, 0.05, 1],
        [0.05, 0.05, 0.05, 1]
    ]
)


# ============================================================
# 9. 자동차 생성
# ============================================================

total_mass = CAR_MASS + fuel

car_id = p.createMultiBody(

    baseMass=total_mass,

    baseCollisionShapeIndex=car_collision,

    baseVisualShapeIndex=car_visual,

    basePosition=[
        0,
        0,
        0.42
    ]
)


# ============================================================
# 10. 초기 상태
# ============================================================

simulation_time = 0.0

speed = 0.0
acceleration = 0.0

# 처음 +X 방향
direction_angle = 0.0

# 큰 주행 상태
driving_state = "START"

# 세부 운동 상태
motion_state = "STOP"

# 규칙 상태
fuel_rule_active = True
turn_rule_active = False


# ============================================================
# 11. CSV 로그
# ============================================================

log_file = open(
    "driving_log.csv",
    "w",
    newline="",
    encoding="utf-8-sig"
)

writer = csv.writer(log_file)

writer.writerow([
    "Time(s)",

    "X(m)",
    "Y(m)",
    "Z(m)",

    "Speed(m/s)",

    "Acceleration(m/s^2)",

    "Motion",

    "Fuel",

    "Fuel(%)",

    "Fuel Rate",

    "Total Mass(kg)",

    "Direction(deg)",

    "Driving State",

    "Fuel Rule",

    "Turn Rule"
])


# ============================================================
# 12. HUD ID
# ============================================================

hud_ids = []


# ============================================================
# 13. 카메라 초기 위치
# ============================================================

p.resetDebugVisualizerCamera(

    cameraDistance=14,

    cameraYaw=45,

    cameraPitch=-30,

    cameraTargetPosition=[
        0,
        0,
        0
    ]
)


# ============================================================
# 14. 메인 시뮬레이션
# ============================================================

while simulation_time <= SIMULATION_END_TIME:


    # ========================================================
    # 현재 차량 좌표
    # ========================================================

    position, orientation = (
        p.getBasePositionAndOrientation(car_id)
    )

    x = position[0]
    y = position[1]
    z = position[2]


    # ========================================================
    # 가속 / 정속 / 감속
    # ========================================================

    if simulation_time < BRAKE_START_TIME:

        if fuel > 0:

            # ---------------------------
            # 가속
            # ---------------------------

            if speed < MAX_SPEED:

                acceleration = ACCELERATION

                speed = (
                    speed
                    + acceleration * TIME_STEP
                )

                if speed > MAX_SPEED:
                    speed = MAX_SPEED

                motion_state = "ACCEL"


            # ---------------------------
            # 정속
            # ---------------------------

            else:

                speed = MAX_SPEED

                acceleration = 0.0

                motion_state = "CONSTANT"


        # -------------------------------
        # 연료 소진
        # -------------------------------

        else:

            fuel_rule_active = False

            if speed > 0:

                acceleration = -DECELERATION

                speed = (
                    speed
                    + acceleration * TIME_STEP
                )

                if speed < 0:
                    speed = 0.0

                motion_state = "DECEL"

            else:

                speed = 0.0

                acceleration = 0.0

                motion_state = "STOP"


    # ========================================================
    # 30초 이후 감속
    # ========================================================

    else:

        if speed > 0:

            acceleration = -DECELERATION

            speed = (
                speed
                + acceleration * TIME_STEP
            )

            if speed < 0:
                speed = 0.0

            motion_state = "DECEL"

        else:

            speed = 0.0

            acceleration = 0.0

            motion_state = "STOP"


    # ========================================================
    # 큰 주행 상태 결정
    # START / DRIVING / STOP
    # ========================================================

    if simulation_time < 1.0:

        driving_state = "START"

    elif speed > 0:

        driving_state = "DRIVING"

    else:

        driving_state = "STOP"


    # ========================================================
    # 규칙 1 : 속도에 따른 연료 소비
    # ========================================================

    if speed > 0 and fuel > 0:

        fuel_rule_active = True

        speed_ratio = (
            speed / MAX_SPEED
        )

        current_fuel_rate = (
            MAX_FUEL_RATE
            * speed_ratio
        )

        fuel_used = (
            current_fuel_rate
            * TIME_STEP
        )

        fuel = (
            fuel
            - fuel_used
        )

        if fuel < 0:
            fuel = 0.0

    else:

        current_fuel_rate = 0.0


    # ========================================================
    # 연료 %
    # ========================================================

    fuel_percent = (
        fuel
        / INITIAL_FUEL
        * 100.0
    )


    # ========================================================
    # 연료 감소에 따른 차량 총질량 변화
    # ========================================================

    total_mass = (
        CAR_MASS
        + fuel
    )

    p.changeDynamics(
        car_id,
        -1,
        mass=total_mass
    )


    # ========================================================
    # 규칙 2 : 15~20초 방향 전환
    # ========================================================

    if (
        TURN_START_TIME
        <= simulation_time
        < TURN_END_TIME
    ):

        turn_rule_active = True

        turn_progress = (
            simulation_time
            - TURN_START_TIME
        ) / (
            TURN_END_TIME
            - TURN_START_TIME
        )

        direction_angle = math.radians(
            90.0 * turn_progress
        )


    elif simulation_time >= TURN_END_TIME:

        turn_rule_active = False

        direction_angle = math.radians(90.0)


    else:

        turn_rule_active = False

        direction_angle = 0.0


    # ========================================================
    # 차량 진행 방향
    # ========================================================

    vx = (
        speed
        * math.cos(direction_angle)
    )

    vy = (
        speed
        * math.sin(direction_angle)
    )


    # ========================================================
    # 차량 외형 회전
    # ========================================================

    car_orientation = p.getQuaternionFromEuler(
        [
            0,
            0,
            direction_angle
        ]
    )

    p.resetBasePositionAndOrientation(

        car_id,

        [
            x,
            y,
            z
        ],

        car_orientation
    )


    # ========================================================
    # 차량 속도 적용
    # ========================================================

    p.resetBaseVelocity(

        car_id,

        linearVelocity=[
            vx,
            vy,
            0
        ]
    )


    # ========================================================
    # 물리엔진 실행
    # ========================================================

    p.stepSimulation()


    # ========================================================
    # 연료 게이지
    # ========================================================

    gauge_length = 20

    filled = int(
        fuel_percent
        / 100.0
        * gauge_length
    )

    filled = max(
        0,
        min(gauge_length, filled)
    )

    empty = (
        gauge_length
        - filled
    )

    fuel_gauge = (
        "["
        + "#" * filled
        + "-" * empty
        + "]"
    )


    # ========================================================
    # 규칙 ON/OFF 표시
    # ========================================================

    if fuel_rule_active:
        fuel_rule_text = "ON"
    else:
        fuel_rule_text = "OFF"


    if turn_rule_active:
        turn_rule_text = "ON"
    else:
        turn_rule_text = "OFF"


    # ========================================================
    # HUD
    # ========================================================

    hud_lines = [

        "===== VEHICLE STATUS =====",

        f"Time          : "
        f"{simulation_time:5.1f} / "
        f"{SIMULATION_END_TIME:.0f} s",

        f"Position      : "
        f"({x:6.2f}, {y:6.2f}, {z:5.2f}) m",

        f"Speed         : "
        f"{speed:5.2f} m/s",

        f"Acceleration  : "
        f"{acceleration:+5.2f} m/s^2",

        f"Motion        : "
        f"{motion_state}",

        f"Direction     : "
        f"{math.degrees(direction_angle):5.1f} deg",

        "",

        "===== FUEL STATUS =====",

        f"Fuel          : "
        f"{fuel:6.2f} / "
        f"{INITIAL_FUEL:.0f}",

        f"Fuel Gauge    : "
        f"{fuel_gauge} "
        f"{fuel_percent:5.1f}%",

        f"Fuel Rate     : "
        f"{current_fuel_rate:.3f} /s",

        f"Total Mass    : "
        f"{total_mass:6.2f} kg",

        "",

        "===== RULE STATUS =====",

        f"Fuel Rule     : "
        f"{fuel_rule_text}",

        f"Turn Rule     : "
        f"{turn_rule_text}",

        "",

        "===== CURRENT STATE =====",

        f"State         : "
        f"{driving_state}"
    ]


    # ========================================================
    # HUD ID 최초 생성
    # ========================================================

    if len(hud_ids) == 0:

        hud_ids = (
            [-1] * len(hud_lines)
        )


    # ========================================================
    # HUD 위치
    # ========================================================

    # 차량 기준 왼쪽 위
    hud_x = x - 7.0
    hud_y = y + 5.0
    hud_z = z + 9.0

    # 줄 간격
    line_spacing = 0.72


    # ========================================================
    # HUD 출력
    # ========================================================

    for i, line in enumerate(hud_lines):

        if line == "":
            continue

        hud_ids[i] = p.addUserDebugText(

            line,

            [
                hud_x,
                hud_y,
                hud_z - i * line_spacing
            ],

            textColorRGB=[
                0.02,
                0.02,
                0.02
            ],

            textSize=0.85,

            lifeTime=0,

            replaceItemUniqueId=hud_ids[i]
        )


    # ========================================================
    # CSV 저장
    # ========================================================

    writer.writerow([

        round(
            simulation_time,
            3
        ),

        round(x, 3),

        round(y, 3),

        round(z, 3),

        round(
            speed,
            3
        ),

        round(
            acceleration,
            3
        ),

        motion_state,

        round(
            fuel,
            3
        ),

        round(
            fuel_percent,
            2
        ),

        round(
            current_fuel_rate,
            4
        ),

        round(
            total_mass,
            3
        ),

        round(
            math.degrees(
                direction_angle
            ),
            2
        ),

        driving_state,

        fuel_rule_text,

        turn_rule_text
    ])


    # ========================================================
    # 터미널에 1초마다 출력
    # ========================================================

    step_number = int(
        simulation_time
        / TIME_STEP
    )

    if step_number % 60 == 0:

        print(

            f"Time={simulation_time:5.1f}s | "

            f"Position="
            f"({x:6.2f}, {y:6.2f}) | "

            f"Speed="
            f"{speed:4.2f}m/s | "

            f"Acc="
            f"{acceleration:+4.2f}m/s^2 | "

            f"Motion="
            f"{motion_state} | "

            f"Fuel="
            f"{fuel:6.2f}% | "

            f"TurnRule="
            f"{turn_rule_text} | "

            f"State="
            f"{driving_state}"
        )


    # ========================================================
    # 카메라 차량 추적
    # ========================================================

    p.resetDebugVisualizerCamera(

        cameraDistance=16,

        cameraYaw=45,

        cameraPitch=-28,

        cameraTargetPosition=[
            x,
            y,
            z + 1.0
        ]
    )


    # ========================================================
    # 시간 진행
    # ========================================================

    time.sleep(TIME_STEP)

    simulation_time += TIME_STEP


# ============================================================
# 15. 종료
# ============================================================

log_file.close()


print()
print("====================================")
print("       SIMULATION FINISHED")
print("====================================")

print(
    f"Final Position : "
    f"({x:.2f}, {y:.2f}, {z:.2f}) m"
)

print(
    f"Final Speed : "
    f"{speed:.2f} m/s"
)

print(
    f"Remaining Fuel : "
    f"{fuel:.2f} / "
    f"{INITIAL_FUEL:.0f}"
)

print(
    f"Final Mass : "
    f"{total_mass:.2f} kg"
)

print(
    "Log File : driving_log.csv"
)


# 최종 정지 상태 3초 표시
time.sleep(3)

p.disconnect()
