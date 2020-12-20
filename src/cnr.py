# Calibration & Roi's Color
import math
# pixel 좌표계를 현실의 xy 좌표계로 변환해줍니다.  인자 : left- pixel x , left- pixel y ~~~


def angle_setter(px, py):
    # fx = 1101.541994688683  # focal length x
    # fy = 1121.5650528931951  # focal length y 1010.1384036287627
    # cx = 790.94897802980176  # Principal point x 633.72118964017716
    # cy = 489.8524199469291  # Principal point y 374.807222496245
    # angle = 0
    # h = 0.5  # 카메라 높이 h
    # u1 = (px - cx) / fx
    # v1 = (py - cy) / fy
    # m_ry = h*math.tan(math.pi/2 - math.radians(angle) - math.atan(v1))
    # m_rx = u1*m_ry
    # if m_rx < 0:
    #     return (math.atan((m_rx*-1)/m_ry)*180/math.pi)
    # else:
    #     return (math.atan(m_rx/m_ry)*180/math.pi)*-1

    if px < 500:
        return 10
    elif px > 700:
        return -10
    else:
        return 0


def real_coordinate(p1, p2):
    l_px, l_py = p1
    r_px, r_py = p2
    px = (l_px + r_px)/2
    py = (l_py + r_py)/2

    fx = 1101.541994688683  # focal length x
    fy = 1121.5650528931951  # focal length y 1010.1384036287627
    cx = 790.94897802980176  # Principal point x 633.72118964017716
    cy = 489.8524199469291  # Principal point y 374.807222496245

    # # 나이트 비전 캠
    # fx = 1007.6918979434037
    # fy = 1010.1384036287627
    # cx = 633.72118964017716
    # cy = 374.807222496245
    angle = 0
    h = 0.88  # 카메라 높이 h

    # # 각도를 안주었을때
    # # 왼쪽 아래점 변환
    # u1 = (l_px - cx) / fx
    # v1 = (l_py - cy) / fy
    # print("v1 : ", v1)
    # l_ry = h/v1  # left- real y
    # l_rx = u1 * l_ry

    # # 오른쪽 아래점 변환
    # u1 = (r_px - cx) / fx
    # v1 = (r_py - cy) / fy
    # print("v1 : ", v1)
    # r_ry = h/v1
    # r_rx = u1 * r_ry

    # # 중앙점 반환
    # m_rx = (r_rx + l_rx)/2
    # m_ry = ((r_ry + l_ry)/2)*-1

    # 각도를 주었을때
    u1 = (px - cx) / fx
    v1 = (py - cy) / fy

    m_ry = h*math.tan(math.pi/2 - math.radians(angle) - math.atan(v1))
    m_rx = u1*m_ry

    return [m_rx, m_ry]


# ROI 영역의 색을 구해줍니다. 현재는 평균값 사용
def determine_color(roi_color):
    color = [0, 0, 0]
    n = len(roi_color)
    for r, g, b in roi_color:
        color = [color[0]+r, color[1]+g, color[2]+b]
    color = [color[0]//n, color[1]//n, color[2]//n]
    return color
