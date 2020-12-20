import lalaca_matching
import math
from similarity import cosine_map
from similarity import cos_similarity
import time
import impact
import datetime
import copy
import collections as col
import numpy as np
WINDOW_SIZE = 6  # (20fps 기준 약 1.5초)


def qr_householder(A):
    m, n = A.shape
    Q = np.eye(m)  # Orthogonal transform so far
    R = A.copy()  # Transformed matrix so far

    for j in range(n):
        # Find H = I - beta*u*u' to put zeros below R[j,j]
        x = R[j:, j]
        normx = np.linalg.norm(x)
        rho = -np.sign(x[0])
        u1 = x[0] - rho * normx
        u = x / u1
        u[0] = 1
        beta = -rho * u1 / normx

        R[j:, :] = R[j:, :] - beta * np.outer(u, u).dot(R[j:, :])
        Q[:, j:] = Q[:, j:] - beta * Q[:, j:].dot(np.outer(u, u))

    return Q, R


def new_sol(data):
    m, n = data.shape
    A = np.array([data[:, 0], np.ones(m)]).T
    b = data[:, 1]

    Q, R = qr_householder(A)
    b_hat = Q.T.dot(b)

    R_upper = R[:n, :]
    b_upper = b_hat[:n]

    x = np.linalg.solve(R_upper, b_upper)
    slope, intercept = x
    return slope, intercept


class detect_object:
    def __init__(self, label, number, value):
        # 검출 객체 종류
        self.label = label
        self.number = number
        # 상태값 초기화
        self.point = value['point']  # 현재 좌표값
        self.point_time = time.time()
        self.color = value['color']  # 색
        self.gridbox = value['gridbox']
        self.timer = time.time()  # 감지된 시간
        self.detect = True
        self.path_x = col.deque(maxlen=WINDOW_SIZE)
        self.path_y = col.deque(maxlen=WINDOW_SIZE)
        self.path_t = col.deque(maxlen=WINDOW_SIZE)
        self.path_x.append(self.point[0])
        self.path_y.append(self.point[1])
        self.path_t.append(self.timer)
        self.midpoint = [0, 0]
        self.vector = [0, 0]
        self.crush_level = 0  # 최고 레벨 3. 3일 경우 제동이 들어가야함
        self.crush_time = 999999
        self.crush_dir = 'Middle'
        self.cnt = 0

        self.move_time = time.time()
        self.dont_move_time = 0

    def update(self, value):
        self.timer = time.time()
        self.path_x.append(value['point'][0])
        self.path_y.append(value['point'][1])
        self.path_t.append(self.timer)
        self.color = value['color']
        self.gridbox = value['gridbox']
        self.detect = True
        self.cnt = self.cnt+1

        if self.cnt == WINDOW_SIZE:
            self.cnt = 0
            interval = self.timer - self.point_time
            x_point = sum(self.path_x) / WINDOW_SIZE
            y_point = sum(self.path_y) / WINDOW_SIZE
            self.vector = [(x_point - self.point[0])/interval,
                           (y_point - self.point[1])/interval]
            self.point = [x_point, y_point]
            self.point_time = self.timer

        if abs(self.vector[0]) <= 0.2 and abs(self.vector[1]) <= 0.2:
            self.dont_move_time = self.timer - self.move_time
        else:
            self.move_time = self.timer
            self.dont_move_time = 0

        return


def char_changer(map):
    n = len(map)
    p_map = ""
    for i in range(n):
        for j in range(len(map[i])):
            p_map = p_map + str(round(map[i][j], 1)) + " "
        p_map = p_map + '\n'
    return p_map


def track(exists, values, detect_list):
    #f = open("debuger.txt", 'a')
    end_flag = False
    end_box = []
    min_crush_time = 999999
    c_dir = 'Middle'
    for object_label in detect_list:
        exist_list = exists[object_label]  # 얘는 class
        value_list = values[object_label]  # 얘는 딕셔너리 형태
        LEN_EXIST = len(exist_list)
        LEN_NEW = len(value_list)
        # 번호 지정
        deq = col.deque()
        numberlist = [False for i in range(11)]
        for i in range(LEN_EXIST):
            numberlist[exist_list[i].number] = True
        for i in range(1, 11):
            if numberlist[i] == False:
                deq.append(i)

        # color 맵 구성
        exist_color = []
        for i in range(LEN_EXIST):
            exist_color.append(exist_list[i].color)
        new_color = []
        for i in range(LEN_NEW):
            new_color.append(value_list[i]['color'])
        color_map = cosine_map(exist_color, new_color)
        # color 에 의한 헝가리안 매칭, 거리가 일정 수준 이하인 매칭의 경우 따로 빼줄 수 있도록 한다.

        # xy coordinate 맵 구성
        exist_point = []
        for i in range(LEN_EXIST):
            exist_point.append(exist_list[i].point)  # 일단 현재 지점만 가지고 해보자

        # 측정지점의 경우 그대로 사용한다
        new_point = []
        for i in range(LEN_NEW):
            new_point.append(value_list[i]['point'])

        xy_map = cosine_map(exist_point, new_point)

        # 매칭을 실행한다
        update_exist = []
        match = adas_matching.matching(color_map, xy_map)

        for past, now in match:
            if (past == -1 or past >= LEN_EXIST) and now < LEN_NEW and now != -1:
                # 새로운 객체를 생성한다
                exist_list.append(detect_object(
                    object_label, deq.popleft(), value_list[now]))
                update_exist.append(len(exist_list)-1)

                # 벡터 생성 최대한 멋지게

            elif now < LEN_NEW and now != -1:
                # 존재하던 객체를 갱신한다
                exist_list[past].update(value_list[now])
                exist_list[past].crush_dir, exist_list[past].crush_time = impact.impact(
                    exist_list[past].point, exist_list[past].vector)
                update_exist.append(past)

                # 벡터가 일정 기준 이상 모였을 경우 -> 벡터를 포함해서 판단한다.

                # 현재 검출되지 않은 객체의 경우 실존하지 않음은 알려줘야한다 (왜냐하면 그걸로 멈출순 없으니까)
        temp_exist_list = []
        for i in range(len(exist_list)):
            if i not in update_exist:
                pass
            else:
                temp_exist_list.append(exist_list[i])
        exist_list = copy.deepcopy(temp_exist_list)

        for i in range(len(exist_list)):
            if exist_list[i].detect == True:
                if exist_list[i].dont_move_time >= 5:
                    end_flag = True
                end_box = exist_list[i].gridbox

    return exists, end_flag, end_box
