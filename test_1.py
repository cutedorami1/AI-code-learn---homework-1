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

# 중력 설정
p.setGravity(0, 0, -9.81)

# 1초에 60번 계산
TIME_STEP = 1.0 / 60.0
p.setTimeStep(TIME_STEP)


# =========================================================
# 2. 차량 파라미터 설정
# =========================================================

CAR_LENGTH = 2.0       # 차량 길이 [m]
CAR_WIDTH = 1.0        # 차량 폭 [m]
CAR_HEIGHT = 0.5       # 차량 높이 [m]

CAR_MASS = 500.0       # 연료를 제외한 차량 질량 [kg]

MAX_SPEED = 5.0        # 최대 속도 [m/s]

ACCELERATION = 1.0     # 가속도 [m/s^2]
DECELERATION = 1.0     # 감속도 [m/s^2]


# =========================================================
# 3. 연료 파라미터
# =========================================================

INITIAL_FUEL = 20.0        # 초기 연료 [kg]

# 1m 주행할 때 사용하는 연료
FUEL_CONSUMPTION = 0.02    # [kg/m]

fuel = INITIAL_FUEL


# =========================================================
# 4. 주행 시간 설정
# =========================================================

# 0~5초   : 가속
# 5~15초  : 직진
# 15~20초 : 방향 전환
# 20~35초 : 직진
# 35~40초 : 감속
# 40초    : 정지

TURN_START_TIME = 15.0
TURN_END_TIME = 20.0

BRAKE_START_TIME = 35.0
SIMULATION_END_TIME = 41.0


# =========================================================
# 5. 평면 생성
# =========================================================

plane_id = p.loadURDF("plane.urdf")


# =========================================================
# 6. 차량 생성
# =========================================================

# 차량 충돌 형태
car_collision = p.createCollisionShape(
    p.GEOM_BOX,
    halfExtents=[
        CAR_LENGTH / 2,
        CAR_WIDTH / 2,
        CAR_HEIGHT / 2
    ]
)

# 차량의 화면상 모습
car_visual = p.createVisualShape(
    p.GEOM_BOX,
    halfExtents=[
        CAR_LENGTH / 2,
        CAR_WIDTH / 2,
        CAR_HEIGHT / 2
    ],
    rgbaColor=[0.1, 0.3, 1.0, 1.0]
)

# 초기 총질량 = 차량 질량 + 연료 질량
total_mass = CAR_MASS + fuel

car_id = p.createMultiBody(
    baseMass=total_mass,
    baseCollisionShapeIndex=car_collision,
    baseVisualShapeIndex=car_visual,
    basePosition=[0, 0, CAR_HEIGHT / 2]
)


# =========================================================
# 7. 초기값 설정
# =========================================================

simulation_time = 0.0

speed = 0.0
acceleration = 0.0

# 처음에는 +X 방향
direction_angle = 0.0

state = "출발"
rule = "연료 소비"


# =========================================================
# 8. CSV 로그 파일 생성
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
    cameraPitch=-40,
    cameraTargetPosition=[0, 0, 0]
)


# =========================================================
# 10. 메인 시뮬레이션
# =========================================================

while simulation_time <= SIMULATION_END_TIME:

    # -----------------------------------------------------
    # 현재 위치 확인
    # -----------------------------------------------------

    position, orientation = p.getBasePositionAndOrientation(car_id)

    x = position[0]
    y = position[1]
    z = position[2]


    # =====================================================
    # 차량의 가속 / 정속 / 감속 결정
    # =====================================================

    if simulation_time < BRAKE_START_TIME:

        # 연료가 남아 있을 경우
        if fuel > 0:

            # 최대속도까지 가속
            if speed < MAX_SPEED:

                acceleration = ACCELERATION

                speed += acceleration * TIME_STEP

                # 최대속도 초과 방지
                if speed > MAX_SPEED:
                    speed = MAX_SPEED

                state = "가속"

            else:

                speed = MAX_SPEED
                acceleration = 0.0
                state = "주행"

        # 연료가 없으면 감속
        else:

            acceleration = -DECELERATION

            speed += acceleration * TIME_STEP

            if speed < 0:
                speed = 0

            state = "연료 부족 감속"


    # 35초 이후 정상 감속
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

        # 한 step 동안 이동하는 거리
        distance = speed * TIME_STEP

        # 사용한 연료
        fuel_used = FUEL_CONSUMPTION * distance

        # 현재 연료 감소
        fuel -= fuel_used

        if fuel < 0:
            fuel = 0


    # 차량의 현재 총질량 계산
    total_mass = CAR_MASS + fuel


    # Bullet 차량 질량에도 현재 질량 적용
    p.changeDynamics(
        car_id,
        -1,
        mass=total_mass
    )


    # =====================================================
    # 규칙 2 : 일정 시간 후 방향 전환
    # =====================================================

    if TURN_START_TIME <= simulation_time < TURN_END_TIME:

        # 5초 동안 90도 회전
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

        # 회전 완료 후 90도 유지
        direction_angle = math.radians(90)

        rule = "연료 소비"

    else:

        direction_angle = 0.0

        rule = "연료 소비"


    # =====================================================
    # 차량 진행 방향 계산
    # =====================================================

    # Vx = V cos(theta)
    vx = speed * math.cos(direction_angle)

    # Vy = V sin(theta)
    vy = speed * math.sin(direction_angle)


    # =====================================================
    # 차량의 방향(회전)도 화면에 적용
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
    # 차량 속도를 Bullet에 적용
    # =====================================================

    p.resetBaseVelocity(
        car_id,
        linearVelocity=[vx, vy, 0]
    )


    # =====================================================
    # 물리엔진 1 step 실행
    # =====================================================

    p.stepSimulation()


    # =====================================================
    # CSV 로그 저장
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

    # 너무 많이 출력되지 않도록 약 1초마다 출력
    step_number = int(simulation_time / TIME_STEP)

    if step_number % 60 == 0:

        print(
            f"Time: {simulation_time:5.1f}s | "
            f"Position: ({x:6.2f}, {y:6.2f}) | "
            f"Speed: {speed:4.2f}m/s | "
            f"Fuel: {fuel:5.2f}kg | "
            f"Mass: {total_mass:6.2f}kg | "
            f"Direction: {math.degrees(direction_angle):5.1f}deg | "
            f"State: {state}"
        )


    # =====================================================
    # 차량을 따라 카메라 이동
    # =====================================================

    p.resetDebugVisualizerCamera(
        cameraDistance=12,
        cameraYaw=45,
        cameraPitch=-40,
        cameraTargetPosition=[x, y, 0]
    )


    # 실제 시간과 비슷한 속도로 실행
    time.sleep(TIME_STEP)

    # 시간 증가
    simulation_time += TIME_STEP


# =========================================================
# 11. 종료
# =========================================================

log_file.close()

print()
print("==============================")
print("시뮬레이션 종료")
print("==============================")
print(f"최종 속도 : {speed:.2f} m/s")
print(f"남은 연료 : {fuel:.2f} kg")
print(f"최종 질량 : {total_mass:.2f} kg")
print("로그 파일 : driving_log.csv")

# 결과를 잠시 볼 수 있도록 3초 대기
time.sleep(3)

p.disconnect()
