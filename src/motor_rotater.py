# 사람의 크기가 더 크거나 한 것은 어쩔 수 없음.
import cnr
import RPi.GPIO as GPIO  # RPi.GPIO 라이브러리를 GPIO로 사용
from time import sleep  # time 라이브러리의 sleep함수 사용


def track_angle(gridbox):
    now_x = (gridbox[1] + gridbox[3])/2
    now_y = gridbox[2]
    angle = cnr.angle_setter(now_x, now_y)
    if abs(angle) >= 5:
        return angle*2/3
    else:
        return 0


def set_angle(gridbox, setbox):
    # x의 중앙점을 기준으로 생각
    now_x = (gridbox[1] + gridbox[3])/2
    des_x = (setbox[1] + setbox[3])/2
    angle_x = 0
    now_y = gridbox[2]
    des_y = setbox[2]
    angle_y = 0

    if abs(now_x - des_x) <= 100:
        angle_x = 0
    # 왼쪽으로 돌려야하는 경우, cnr로 각도 계산하자
    elif now_x < des_x:
        angle_x = 3
    elif now_x > des_x:
        angle_x = -3
    # y의 경우, 발바닥을 기준으로 생각

    if abs(now_y - des_y) <= 100:
        angle_y = 0
    # 카메라 각도를 조금 올려야하는 경우 (아주 조금씩만 올리는게 나을 것이라고 생각됨)
    elif now_y < des_y:
        angle_y = 3
    elif now_y > des_y:
        angle_y = -3
    return [angle_x, angle_y]


def duty_maker(degree):
    degree = degree + 90
    if degree > 180:
        degree = 180
    if degree < 0:
        degree = 0
    SERVO_MAX_DUTY = 12.5  # 서보의 최대(180도) 위치의 주기s
    SERVO_MIN_DUTY = 2.5    # 서보의 최소(0도) 위치의 주기
    duty = SERVO_MIN_DUTY+(degree*(SERVO_MAX_DUTY-SERVO_MIN_DUTY)/180.0)
    return duty


def camera_motor_control(now_x, rotate_x, now_y, rotate_y):
    # 각도는 180도를 넘을 수 없다.
    servoPin_x = 18   # 서보 핀 x
    GPIO.setmode(GPIO.BCM)        # GPIO 설정nchips
    GPIO.setup(servoPin_x, GPIO.OUT)  # 서보핀 출력으로 설정
    servo_x = GPIO.PWM(servoPin_x, 50)  # 서보핀을 PWM 모드 50Hz로 사용하기 (50Hz > 20ms)
    servo_x.start(0)  # 서보 PWM 시작 duty = 0, duty가 0이면 서보는 동작하지 않s는다.
    servo_x.ChangeDutyCycle(duty_maker(now_x+rotate_x))  # 5~130으로 조절
    sleep(0.2)
    GPIO.cleanup()

    servoPin_y = 23   # 서보 핀 y
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(servoPin_y, GPIO.OUT)  # 서보핀 출력으로 설정
    servo_y = GPIO.PWM(servoPin_y, 50)  # 서보핀을 PWM 모드 50Hz로 사용하기 (50Hz > 20ms)
    servo_y.start(0)  # 서보 PWM 시작 duty = 0, duty가 0이면 서보는 동작하지 않s는다.
    servo_y.ChangeDutyCycle(duty_maker(now_y+rotate_y))  # 5로 다시 조절
    sleep(0.2)
    GPIO.cleanup()
    return


camera_motor_control(0, 0, 0, 0)
