import pybullet as p
import pybullet_data
import time
import math
import csv


# =========================================================
# 1. PyBullet 시뮬레이션 시작
# =========================================================

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)

# 1초에 60 step
TIME_STEP = 1.0 / 60.0
p.setTimeStep(TIME_STEP)


# =========================================================
# 2. 차량 파라미터
# =========================================================

CAR_LENGTH = 2.0       # 차량 길이 [m]
CAR_WIDTH = 1.0        # 차량 폭 [m]
CAR_HEIGHT = 0.40      # 차체 높이 [m]

CAR_MASS = 500.0       # 연료 제외 차량 질량 [kg]

MAX_SPEED = 5.0        # 최대 속도 [m/s]
ACCELERATION = 1.0     # 가속도 [m/s^2]
DECELERATION = 1.0     # 감속도 [m/s^2]


# =========================================================
# 3. 연료 파라미터
# =========================================================

INITIAL_FUEL = 20.0
FUEL_CONSUMPTION = 0.02     # kg/m

fuel = INITIAL_FUEL


# =========================================================
# 4. 주행 시간
# =========================================================

TURN_START_TIME = 15.0
TURN_END_TIME = 20.0

BRAKE_START_TIME = 35.0
SIMULATION_END_TIME = 41.0


# =========================================================
# 5. 평면 생성
# =========================================================

plane_id = p.loadURDF("plane.urdf")


# =========================================================
# 6. 자동차 모델 생성
# =========================================================

# ---------------------------------------------------------
# 자동차의 물리적 충돌 영역
# 실제 물리 계산에서는 하나의 차체로 계산
# ---------------------------------------------------------

car_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[
        CAR_LENGTH / 2,
        CAR_WIDTH / 2,
        CAR_HEIGHT / 2
    ]
)


# ---------------------------------------------------------
# 자동차 외형 만들기
#
# ① 차체
# ② 지붕
# ③ 앞유리
# ④ 왼쪽 앞바퀴
# ⑤ 오른쪽 앞바퀴
# ⑥ 왼쪽 뒷바퀴
# ⑦ 오른쪽 뒷바퀴
# ---------------------------------------------------------

wheel_orientation = p.getQuaternionFromEuler(
    [math.pi / 2, 0, 0]
)

car_visual = p.createVisualShapeArray(

    shapeTypes=[
        p.GEOM_BOX,       # 차체
        p.GEOM_BOX,       # 지붕
        p.GEOM_BOX,       # 앞유리
        p.GEOM_CYLINDER,  # 앞바퀴 1
        p.GEOM_CYLINDER,  # 앞바퀴 2
        p.GEOM_CYLINDER,  # 뒷바퀴 1
        p.GEOM_CYLINDER   # 뒷바퀴 2
    ],

    # BOX는 halfExtents
    # CYLINDER는 여기 값이 사용되지 않으므로 [0,0,0]
    halfExtents=[
        [1.0, 0.5, 0.20],       # 차체
        [0.45, 0.42, 0.22],     # 지붕
        [0.05, 0.38, 0.17],     # 앞유리
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
    ],

    # 원통 반지름
    radii=[
        0,
        0,
        0,
        0.22,
        0.22,
        0.22,
        0.22
    ],

    # 원통 폭
    lengths=[
        0,
        0,
        0,
        0.16,
        0.16,
        0.16,
        0.16
    ],

    # 각 부품의 위치
    visualFramePositions=[

        [0, 0, 0],              # 차체

        [-0.10, 0, 0.38],       # 지붕

        [0.38, 0, 0.38],        # 앞유리

        [0.62, 0.55, -0.10],    # 앞 왼쪽 바퀴
        [0.62, -0.55, -0.10],   # 앞 오른쪽 바퀴

        [-0.62, 0.55, -0.10],   # 뒤 왼쪽 바퀴
        [-0.62, -0.55, -0.10]   # 뒤 오른쪽 바퀴
    ],

    # 각 부품 방향
    visualFrameOrientations=[

        [0, 0, 0, 1],
        [0, 0, 0, 1],
        [0, 0, 0, 1],

        wheel_orientation,
        wheel_orientation,
        wheel_orientation,
        wheel_orientation
    ],

    # 색상
    rgbaColors=[

        [0.05, 0.25, 0.85, 1],  # 차체 - 파란색

        [0.10, 0.35, 0.95, 1],  # 지붕

        [0.30, 0.75, 0.95, 1],  # 앞유리

        [0.05, 0.05, 0.05, 1],  # 바퀴
        [0.05, 0.05, 0.05, 1],
        [0.05, 0.05, 0.05, 1],
        [0.05, 0.05, 0.05, 1]
    ]
)


# 초기 총질량
total_mass = CAR_MASS + fuel


# 자동차 생성
car_id = p.createMultiBody(

    baseMass=total_mass,

    baseCollisionShapeIndex=car_collision,

    baseVisualShapeIndex=car_visual,

    # 바퀴가 땅속으로 들어가지 않도록 조금 높게 시작
    basePosition=[0, 0, 0.42]
)


# =========================================================
# 7. 초기값
# =========================================================

simulation_time = 0.0

speed = 0.0
acceleration = 0.0

# 처음에는 +X 방향
direction_angle = 0.0

state = "출발"
rule = "연료 소비"


# =========================================================
# 8. CSV 로그 생성
# =========================================================

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
    "Fuel(kg)",
    "Total Mass(kg)",
    "Direction(deg)",
    "State",
    "Rule"
])


# =========================================================
# 9. 카메라 설정
# =========================================================

p.resetDebugVisualizerCamera(
    cameraDistance=15,
    cameraYaw=45,
    cameraPitch=-35,
    cameraTargetPosition=[0, 0, 0]
)


# =========================================================
# 10. 메인 시뮬레이션
# =========================================================

while simulation_time <= SIMULATION_END_TIME:

    # -----------------------------------------------------
    # 현재 차량 위치
    # -----------------------------------------------------

    position, orientation = p.getBasePositionAndOrientation(car_id)

    x = position[0]
    y = position[1]
    z = position[2]


    # =====================================================
    # 가속 / 정속 / 감속
    # =====================================================

    if simulation_time < BRAKE_START_TIME:

        # 연료가 남아 있을 경우
        if fuel > 0:

            # 최대속도까지 가속
            if speed < MAX_SPEED:

                acceleration = ACCELERATION

                speed += acceleration * TIME_STEP

                if speed > MAX_SPEED:
                    speed = MAX_SPEED

                state = "가속"

            else:

                speed = MAX_SPEED
                acceleration = 0.0

                state = "주행"

        # 연료가 모두 소모된 경우
        else:

            acceleration = -DECELERATION

            speed += acceleration * TIME_STEP

            if speed < 0:
                speed = 0

            state = "연료 부족 감속"


    # -----------------------------------------------------
    # 35초 이후 감속
    # -----------------------------------------------------

    else:

        if speed > 0:

            acceleration = -DECELERATION

            speed += acceleration * TIME_STEP

            if speed < 0:
                speed = 0

            state = "감속"

        else:

            speed = 0
            acceleration = 0

            state = "정지"


    # =====================================================
    # 규칙 1 : 연료 소비
    # =====================================================

    if speed > 0 and fuel > 0:

        # 한 step 동안 이동한 거리
        distance = speed * TIME_STEP

        # 연료 사용량
        fuel_used = FUEL_CONSUMPTION * distance

        fuel -= fuel_used

        if fuel < 0:
            fuel = 0


    # -----------------------------------------------------
    # 연료 감소에 따른 차량 총질량 변화
    # -----------------------------------------------------

    total_mass = CAR_MASS + fuel

    p.changeDynamics(
        car_id,
        -1,
        mass=total_mass
    )


    # =====================================================
    # 규칙 2 : 15초 후 방향 전환
    # =====================================================

    if TURN_START_TIME <= simulation_time < TURN_END_TIME:

        # 15~20초 동안
        # 0도 -> 90도로 점진적으로 방향 전환

        turn_progress = (
            simulation_time - TURN_START_TIME
        ) / (
            TURN_END_TIME - TURN_START_TIME
        )

        direction_angle = math.radians(
            90.0 * turn_progress
        )

        rule = "연료 소비 + 방향 전환"


    elif simulation_time >= TURN_END_TIME:

        # 방향 전환 완료
        direction_angle = math.radians(90)

        rule = "연료 소비"


    else:

        direction_angle = 0.0

        rule = "연료 소비"


    # =====================================================
    # 진행 방향 계산
    # =====================================================

    # Vx = V cos(theta)
    vx = speed * math.cos(direction_angle)

    # Vy = V sin(theta)
    vy = speed * math.sin(direction_angle)


    # =====================================================
    # 자동차 외형 방향 변경
    # =====================================================

    car_orientation = p.getQuaternionFromEuler(
        [0, 0, direction_angle]
    )

    p.resetBasePositionAndOrientation(
        car_id,
        [x, y, z],
        car_orientation
    )


    # =====================================================
    # 차량 속도 적용
    # =====================================================

    p.resetBaseVelocity(
        car_id,
        linearVelocity=[vx, vy, 0]
    )


    # =====================================================
    # Bullet 물리엔진 1 step
    # =====================================================

    p.stepSimulation()


    # =====================================================
    # CSV 저장
    # =====================================================

    writer.writerow([
        round(simulation_time, 3),
        round(x, 3),
        round(y, 3),
        round(z, 3),
        round(speed, 3),
        round(acceleration, 3),
        round(fuel, 4),
        round(total_mass, 4),
        round(math.degrees(direction_angle), 2),
        state,
        rule
    ])


    # =====================================================
    # 터미널 출력
    # =====================================================

    step_number = int(simulation_time / TIME_STEP)

    if step_number % 60 == 0:

        print(
            f"Time: {simulation_time:5.1f}s | "
            f"Position: ({x:6.2f}, {y:6.2f}) | "
            f"Speed: {speed:4.2f}m/s | "
            f"Fuel: {fuel:5.2f}kg | "
            f"Mass: {total_mass:6.2f}kg | "
            f"Direction: {math.degrees(direction_angle):5.1f}deg | "
            f"State: {state} | "
            f"Rule: {rule}"
        )


    # =====================================================
    # 차량 따라 카메라 이동
    # =====================================================

    p.resetDebugVisualizerCamera(
        cameraDistance=10,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=[x, y, z]
    )


    # 실제 시간과 비슷하게 진행
    time.sleep(TIME_STEP)

    simulation_time += TIME_STEP


# =========================================================
# 11. 종료
# =========================================================

log_file.close()

print()
print("================================")
print("시뮬레이션 종료")
print("================================")
print(f"최종 속도 : {speed:.2f} m/s")
print(f"남은 연료 : {fuel:.2f} kg")
print(f"최종 질량 : {total_mass:.2f} kg")
print("로그 파일 : driving_log.csv")

time.sleep(3)

p.disconnect()
