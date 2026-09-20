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

# 중력 설정
p.setGravity(0, 0, -9.81)

# 시간 간격 : 1초에 60 step
TIME_STEP = 1.0 / 60.0
p.setTimeStep(TIME_STEP)


# ============================================================
# 2. PyBullet 화면 설정
# ============================================================

# 왼쪽 Example Browser / 오른쪽 Parameter 창 숨기기
p.configureDebugVisualizer(
    p.COV_ENABLE_GUI,
    0
)

# 그림자 표시
p.configureDebugVisualizer(
    p.COV_ENABLE_SHADOWS,
    1
)


# ============================================================
# 3. 차량 파라미터
# ============================================================

# 차량 크기
CAR_LENGTH = 2.0       # m
CAR_WIDTH = 1.0        # m
CAR_HEIGHT = 0.40      # m

# 연료를 제외한 차량 자체 질량
CAR_MASS = 500.0       # kg

# 최대 속도
MAX_SPEED = 5.0        # m/s

# 가속도
ACCELERATION = 1.0     # m/s^2

# 감속도
DECELERATION = 1.0     # m/s^2


# ============================================================
# 4. 연료 파라미터
# ============================================================

# 초기 연료
INITIAL_FUEL = 100.0

fuel = INITIAL_FUEL

# 최대속도 5m/s에서 초당 연료 소비량
MAX_FUEL_RATE = 0.50


# ============================================================
# 5. 주행 시간 설정 (35초 완전 정지 타임라인 반영)
# ============================================================

# 15초부터 방향 전환 시작
TURN_START_TIME = 15.0

# 20초에 90도 방향 전환 완료
TURN_END_TIME = 20.0

# 30초부터 감속 시작 (5m/s에서 1m/s^2 감속 시 35초에 속도 0 도달)
BRAKE_START_TIME = 30.0

# 35초에 완전 정지 후 37초까지 정지 상태 유지
SIMULATION_END_TIME = 37.0


# ============================================================
# 6. 평면 생성
# ============================================================

plane_id = p.loadURDF("plane.urdf")


# ============================================================
# 7. 자동차 물리 모델 생성
# ============================================================

# 실제 충돌 계산에 사용하는 차량 영역
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

# 바퀴 방향
wheel_orientation = p.getQuaternionFromEuler(
    [math.pi / 2, 0, 0]
)


# 차체 + 지붕 + 앞유리 + 바퀴 4개
car_visual = p.createVisualShapeArray(

    shapeTypes=[
        p.GEOM_BOX,       # 차체
        p.GEOM_BOX,       # 지붕
        p.GEOM_BOX,       # 앞유리

        p.GEOM_CYLINDER,  # 앞 왼쪽 바퀴
        p.GEOM_CYLINDER,  # 앞 오른쪽 바퀴
        p.GEOM_CYLINDER,  # 뒤 왼쪽 바퀴
        p.GEOM_CYLINDER   # 뒤 오른쪽 바퀴
    ],

    halfExtents=[
        [1.0, 0.5, 0.20],       # 차체
        [0.45, 0.42, 0.22],     # 지붕
        [0.05, 0.38, 0.17],     # 앞유리

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
        # 차체
        [0, 0, 0],

        # 지붕
        [-0.10, 0, 0.38],

        # 앞유리
        [0.38, 0, 0.38],

        # 앞 왼쪽 바퀴
        [0.62, 0.55, -0.10],

        # 앞 오른쪽 바퀴
        [0.62, -0.55, -0.10],

        # 뒤 왼쪽 바퀴
        [-0.62, 0.55, -0.10],

        # 뒤 오른쪽 바퀴
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
        [0.05, 0.25, 0.85, 1],   # 차체
        [0.10, 0.35, 0.95, 1],   # 지붕
        [0.30, 0.75, 0.95, 1],   # 앞유리

        [0.05, 0.05, 0.05, 1],   # 바퀴
        [0.05, 0.05, 0.05, 1],
        [0.05, 0.05, 0.05, 1],
        [0.05, 0.05, 0.05, 1]
    ]
)


# ============================================================
# 9. 자동차 생성
# ============================================================

# 초기 총질량
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

# 출발 시 속도 0
speed = 0.0

# 초기 가속도
acceleration = 0.0

# 처음에는 +X 방향
direction_angle = 0.0

# 현재 상태
state = "START"

# 방향전환 규칙 작동 여부
turn_rule_active = False


# ============================================================
# 11. CSV 로그 생성
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

    "Fuel",

    "Fuel(%)",

    "Fuel Rate",

    "Total Mass(kg)",

    "Direction(deg)",

    "State",

    "Fuel Rule",

    "Turn Rule"
])


# ============================================================
# 12. 실시간 계기판 ID
# ============================================================

hud_ids = []


# ============================================================
# 13. 카메라 초기 설정
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
    # 현재 차량 위치 확인
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

    # 30초 이전
    if simulation_time < BRAKE_START_TIME:


        # 연료가 남아 있는 경우
        if fuel > 0:


            # 최대속도보다 느리면 가속
            if speed < MAX_SPEED:

                acceleration = ACCELERATION

                # v(new) = v(old) + a * dt
                speed = (
                    speed
                    + acceleration * TIME_STEP
                )

                # 최대속도 제한
                if speed > MAX_SPEED:

                    speed = MAX_SPEED

                state = "ACCELERATING"


            # 최대속도에 도달하면 정속주행
            else:

                speed = MAX_SPEED

                acceleration = 0.0

                state = "DRIVING"


        # 연료가 없는 경우
        else:

            acceleration = -DECELERATION

            speed = (
                speed
                + acceleration * TIME_STEP
            )

            if speed < 0:
                speed = 0

            if speed > 0:

                state = "FUEL EMPTY - DECEL"

            else:

                state = "STOP"


    # --------------------------------------------------------
    # 30초 이후 감속 (35초에 속도 0 달성)
    # --------------------------------------------------------

    else:


        if speed > 0:

            acceleration = -DECELERATION

            speed = (
                speed
                + acceleration * TIME_STEP
            )

            if speed < 0:
                speed = 0

            state = "DECELERATING"


        else:

            speed = 0.0

            acceleration = 0.0

            state = "STOP"


    # ========================================================
    # 규칙 1 : 속도에 따른 연료 소비
    # ========================================================

    if speed > 0 and fuel > 0:

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

        fuel = fuel - fuel_used

        if fuel < 0:

            fuel = 0.0

    else:

        current_fuel_rate = 0.0


    # ========================================================
    # 연료 게이지 %
    # ========================================================

    fuel_percent = (
        fuel
        / INITIAL_FUEL
        * 100.0
    )


    # ========================================================
    # 연료 감소에 따른 차량 질량 변화
    # ========================================================

    total_mass = (
        CAR_MASS
        + fuel
    )

    # Bullet에 현재 질량 적용
    p.changeDynamics(
        car_id,
        -1,
        mass=total_mass
    )


    # ========================================================
    # 규칙 2 : 일정 시간 후 방향 전환
    # ========================================================

    # 15초 ~ 20초
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

        state = "TURNING"


    # 20초 이후
    elif simulation_time >= TURN_END_TIME:

        turn_rule_active = False

        direction_angle = math.radians(90.0)


    # 15초 이전
    else:

        turn_rule_active = False

        direction_angle = 0.0


    # ========================================================
    # 차량 진행 방향 계산
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
    # 자동차 외형도 진행 방향으로 회전
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
    # Bullet 물리엔진 1 step
    # ========================================================

    p.stepSimulation()


    # ========================================================
    # 연료 게이지 생성
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
    # 방향전환 규칙 표시
    # ========================================================

    if turn_rule_active:

        turn_rule_text = "ON"

    else:

        turn_rule_text = "OFF"


    # ========================================================
    # 가속 상태 표시
    # ========================================================

    if acceleration > 0:

        acceleration_status = "ACCEL"

    elif acceleration < 0:

        acceleration_status = "DECEL"

    else:

        acceleration_status = "CONSTANT"


    # ========================================================
    # 실시간 계기판 내용
    # ========================================================

    hud_lines = [

        "===== VEHICLE STATUS =====",

        f"Time          : "
        f"{simulation_time:5.1f} / "
        f"{SIMULATION_END_TIME:.0f} s",

        f"Position X    : {x:6.2f} m",

        f"Position Y    : {y:6.2f} m",

        f"Position Z    : {z:5.2f} m",

        f"Speed         : {speed:5.2f} m/s",

        f"Acceleration  : "
        f"{acceleration:+5.2f} m/s^2 "
        f"[{acceleration_status}]",

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

        "===== DRIVING RULE =====",

        "Fuel Rule     : ON",

        f"Turn Rule     : "
        f"{turn_rule_text}",

        f"State         : "
        f"{state}"
    ]


    # ========================================================
    # HUD ID 개수 설정
    # ========================================================

    if len(hud_ids) == 0:

        hud_ids = [-1] * len(hud_lines)


    # ========================================================
    # 계기판 위치 및 출력
    # ========================================================

    hud_x = x - 6.5
    hud_y = y + 4.5
    hud_z = z + 8.0

    line_spacing = 0.70


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
    # CSV 상태 로그 저장
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

        state,

        "ON",

        turn_rule_text
    ])


    # ========================================================
    # 터미널에 1초마다 상태 출력
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

            f"Fuel="
            f"{fuel:6.2f}% | "

            f"FuelRate="
            f"{current_fuel_rate:.3f}/s | "

            f"Mass="
            f"{total_mass:6.2f}kg | "

            f"Direction="
            f"{math.degrees(direction_angle):5.1f}deg | "

            f"State="
            f"{state}"
        )


    # ========================================================
    # 카메라 동적 추적
    # ========================================================

    p.resetDebugVisualizerCamera(

        cameraDistance=15,

        cameraYaw=45,

        cameraPitch=-28,

        cameraTargetPosition=[
            x,
            y,
            z + 1.0
        ]
    )


    # ========================================================
    # 타임 스텝 진행
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


# 최종 정지 상태를 3초간 보여준 뒤 disconnect
time.sleep(3)

p.disconnect()
