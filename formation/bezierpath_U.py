import matplotlib.pyplot as plt
import numpy as np
import scipy.special
import math
d=10
ang = 30*math.pi/180 # 等腰三角形(60) # 正三角形 (30)
ang2 = 90*math.pi/180
show_animation = True
def calc_4points_bezier_path(sx, sy, syaw, ex, ey, eyaw, offset): #給定控制點輸入 控制點起點(現在的位置) 終點(下一秒的位置) 起始角度(自己) 結速角度(長機)
    """
    Compute control points and path given start and end position.

    :param sx: (float) x-coordinate of the starting point
    :param sy: (float) y-coordinate of the starting point
    :param syaw: (float) yaw angle at start
    :param ex: (float) x-coordinate of the ending point
    :param ey: (float) y-coordinate of the ending point
    :param eyaw: (float) yaw angle at the end
    :param offset: (float)
    :return: (numpy array, numpy array)
    """
    dist = np.hypot(sx - ex, sy - ey) * offset
    control_points = np.array(
         [[sx, sy],
         [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
         [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
         [ex, ey]])

    path = calc_bezier_path(control_points, n_points=5) # path存放了[0,1,sample = 5 ]每一個位置點 = traj[ x,y
                                                                                                     #x1,y1 ..]

    return path, control_points

def calc_bezier_path(control_points, n_points):  #將所有位置點儲存起來變成上面的 path,在下面的if迴圈裡面開始畫每一點的位置將他連起來變成軌跡
    """
    Compute bezier path (trajectory) given control points.

    :param control_points: (numpy array)
    :param n_points: (int) number of points in the trajectory
    :return: (numpy array)
    """
    traj = []
    for t in np.linspace(0, 1, n_points): #(Start,End,Samples to generate)
        traj.append(bezier(t, control_points)) #每個點的計算結果

    return np.array(traj)

def bernstein_poly(n, i, t): #透過博多恩多項式計算出要的函數項次(幾階)
    """
    Bernstein polynom.

    :param n: (int) polynom degree
    :param i: (int)
    :param t: (float)
    :return: (float)
    """
    return scipy.special.comb(n, i) * t ** i * (1 - t) ** (n - i)


def bezier(t, control_points): # 返回貝賽爾曲線上的每一點 t帶入計算出每一點介於[0,1] 輸出位置點[x,y]
    """
    Return one point on the bezier curve.

    :param t: (float) number in [0, 1]
    :param control_points: (numpy array)
    :return: (numpy array) Coordinates of the point
    """
    n = len(control_points) - 1
    return np.sum([bernstein_poly(n, i, t) * control_points[i] for i in range(n + 1)], axis=0)
    # ^透過博多恩產生幾次方的函數再乘上係數(control)最後將每一列[x,y]的元素相加,將矩陣壓縮成一行 出來變成 = array([x,y]) 當t=0; S=P(t)= [sx, sy] ; t=1;S=[ex, ey]


def plot_arrow(x, y, yaw, length=0.1, width=3, fc="r", ec="k"):  # pragma: no cover
    """Plot arrow."""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * np.cos(yaw), length * np.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def main2(wp1x,wp1y,wp2x,wp2y,wp3x,wp3y,wp4x,wp4y,R): #U字形初始軌跡
    """ Plot an example bezier curve.Leader (line1)"""
    start_x2 = wp1x  # [m]
    start_y2 = wp1y  # [m]
    start_yaw2 = np.radians(90.0)  # [rad]

    end_x2 = wp1x  # [m]
    end_y2 = wp2y-R  # [m]
    end_yaw2 = np.radians(90.0)  # [rad]
    offset2 = 0.39

    path2, control_points2 = calc_4points_bezier_path(
        start_x2, start_y2, start_yaw2, end_x2, end_y2, end_yaw2, offset2)

    """Plot an example bezier curve.Leader (line2) """
    start_x = wp1x  # [m]
    start_y = wp2y-R  # [m]
    start_yaw = np.radians(90.0)  # [rad]

    end_x = wp2x+R  # [m]
    end_y = wp2y  # [m]
    end_yaw= np.radians(0.0)  # [rad]
    offset = 0.39

    path, control_points = calc_4points_bezier_path(
        start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)

    """ Plot an example bezier curve.Leader (line3)"""
    start_x3 = wp2x+R  # [m]
    start_y3 = wp2y  # [m]
    start_yaw3 = np.radians(0.0)  # [rad]

    end_x3 = wp3x-R  # [m]
    end_y3 = wp3y  # [m]
    end_yaw3 = np.radians(0.0)  # [rad]
    offset3 = 0.39

    path3, control_points3 = calc_4points_bezier_path(
        start_x3, start_y3, start_yaw3, end_x3, end_y3, end_yaw3, offset3)

    """ Plot an example bezier curve.Leader (line4)"""
    start_x4 = wp3x-R  # [m]
    start_y4 = wp3y  # [m]
    start_yaw4 = np.radians(0.0)  # [rad]

    end_x4 = wp3x  # [m]
    end_y4 = wp3y-R  # [m]
    end_yaw4 = np.radians(-90.0)  # [rad]
    offset4 = 0.39

    path4, control_points4 = calc_4points_bezier_path(
        start_x4, start_y4, start_yaw4, end_x4, end_y4, end_yaw4, offset4)

    """ Plot an example bezier curve.Leader (line5)"""
    start_x5 = wp3x  # [m]
    start_y5 = wp3y-R  # [m]
    start_yaw5 = np.radians(-90.0)  # [rad]

    end_x5 = wp4x  # [m]
    end_y5 = wp4y  # [m]
    end_yaw5 = np.radians(-90.0)  # [rad]
    offset5= 0.39

    path5, control_points5 = calc_4points_bezier_path(
        start_x5, start_y5, start_yaw5, end_x5, end_y5, end_yaw5, offset5)
#--------------------- Follower1 (out) ----------------------------------------
    """ Plot an example bezier curve.Leader (line1)"""

    start_x2F = wp1x + d * math.cos(math.pi - ang + start_yaw2)  # [m]
    start_y2F = wp1y + d * math.sin(math.pi - ang + start_yaw2)  # [m]
    start_yaw2F = np.radians(90.0)  # [rad]
    end_x2F = wp1x + d * math.cos(math.pi - ang + end_yaw2)  # [m]
    end_y2F = (wp2y - R) + d * math.sin(math.pi - ang + end_yaw2)  # [m]
    end_yaw2F = np.radians(90.0)  # [rad]
    offset2F = 0.39

    path2F, control_points2F = calc_4points_bezier_path(
        start_x2F, start_y2F, start_yaw2F, end_x2F, end_y2F, end_yaw2F, offset2F)

    """Plot an example bezier curve.Leader (line2) """
    start_xF = wp1x + d * math.cos(math.pi - ang + start_yaw)  # [m]
    start_yF = wp2y - R+ d * math.sin(math.pi - ang + start_yaw)  # [m]
    start_yawF = np.radians(90.0)  # [rad]
    end_xF = wp2x + R + d * math.cos(math.pi - ang + end_yaw)  # [m]
    end_yF = wp2y + d * math.sin(math.pi - ang + end_yaw)  # [m]
    end_yawF = np.radians(0.0)  # [rad]
    offsetF = 0.39

    pathF, control_pointsF = calc_4points_bezier_path(
        start_xF, start_yF, start_yawF, end_xF, end_yF, end_yawF, offsetF)

    """ Plot an example bezier curve.Leader (line3)"""
    start_x3F = (wp2x + R) + d * math.cos(math.pi - ang + start_yaw3)  # [m]
    start_y3F = (wp2y) + wp1x + d * math.sin(math.pi - ang + start_yaw3)  # [m]
    start_yaw3F = np.radians(0.0)  # [rad]

    end_x3F = (wp3x - R) + d * math.cos(math.pi - ang + end_yaw3)  # [m]
    end_y3F = wp3y + d * math.sin(math.pi - ang + end_yaw3)  # [m]
    end_yaw3F = np.radians(0.0)  # [rad]
    offset3F = 0.39

    path3F, control_points3F = calc_4points_bezier_path(
        start_x3F, start_y3F, start_yaw3F, end_x3F, end_y3F, end_yaw3F, offset3F)

    """ Plot an example bezier curve.Leader (line4)"""
    start_x4F = (wp3x - R) + d * math.cos(math.pi - ang + start_yaw4)  # [m]
    start_y4F = wp3y + d * math.sin(math.pi - ang + start_yaw4)  # [m]
    start_yaw4F = np.radians(0.0)  # [rad]

    end_x4F = wp3x + d * math.cos(math.pi - ang + end_yaw4)  # [m]
    end_y4F = (wp3y - R) + d * math.sin(math.pi - ang + end_yaw4)  # [m]
    end_yaw4F = np.radians(-90.0)  # [rad]
    offset4F = 0.39

    path4F, control_points4F = calc_4points_bezier_path(
        start_x4F, start_y4F, start_yaw4F, end_x4F, end_y4F, end_yaw4F, offset4F)

    """ Plot an example bezier curve.Leader (line5)"""
    start_x5F = wp3x + d * math.cos(math.pi - ang + start_yaw5)  # [m]
    start_y5F = (wp3y - R) + d * math.sin(math.pi - ang + start_yaw5)  # [m]
    start_yaw5F = np.radians(-90.0)  # [rad]

    end_x5F = wp4x + d * math.cos(math.pi - ang + end_yaw5)  # [m]
    end_y5F = wp4y + d * math.sin(math.pi - ang + end_yaw5)  # [m]
    end_yaw5F = np.radians(-90.0)  # [rad]
    offset5F = 0.39

    path5F, control_points5F = calc_4points_bezier_path(
        start_x5F, start_y5F, start_yaw5F, end_x5F, end_y5F, end_yaw5F, offset5F)
#-----------------------------  Follower2  ---------------------------------------------------------
    """ Plot an example bezier curve.Leader (line1)"""

    start_x2F2 = wp1x + d * math.cos(math.pi + ang + start_yaw2)  # [m]
    start_y2F2 = wp1y + d * math.sin(math.pi + ang + start_yaw2)  # [m]
    start_yaw2F2 = np.radians(90.0)  # [rad]
    end_x2F2 = wp1x + d * math.cos(math.pi + ang + end_yaw2)  # [m]
    end_y2F2 = (wp2y - R) + d * math.sin(math.pi + ang + end_yaw2)  # [m]
    end_yaw2F2 = np.radians(90.0)  # [rad]
    offset2F2 = 0.39

    path2F2, control_points2F2 = calc_4points_bezier_path(
        start_x2F2, start_y2F2, start_yaw2F2, end_x2F2, end_y2F2, end_yaw2F2, offset2F2)

    """Plot an example bezier curve.Leader (line2) """
    start_xF2 = wp1x + d * math.cos(math.pi + ang + start_yaw)  # [m]
    start_yF2 = (wp2y - R) + d * math.sin(math.pi + ang + start_yaw)  # [m]
    start_yawF2 = np.radians(90.0)  # [rad]
    end_xF2 = (wp2x + R) + d * math.cos(math.pi + ang + end_yaw)  # [m]
    end_yF2 = wp2y + d * math.sin(math.pi + ang + end_yaw)  # [m]
    end_yawF2 = np.radians(0.0)  # [rad]
    offsetF2 = 0.39

    pathF2, control_pointsF2 = calc_4points_bezier_path(
        start_xF2, start_yF2, start_yawF2, end_xF2, end_yF2, end_yawF2, offsetF2)

    """ Plot an example bezier curve.Leader (line3)"""
    start_x3F2 = (wp2x + R) + d * math.cos(math.pi + ang + start_yaw3)  # [m]
    start_y3F2 = (wp2y) + wp1x + d * math.sin(math.pi + ang + start_yaw3)  # [m]
    start_yaw3F2 = np.radians(0.0)  # [rad]

    end_x3F2 = (wp3x - R) + d * math.cos(math.pi + ang + end_yaw3)  # [m]
    end_y3F2 = wp3y + d * math.sin(math.pi + ang + end_yaw3)  # [m]
    end_yaw3F2 = np.radians(0.0)  # [rad]
    offset3F2 = 0.39

    path3F2, control_points3F2 = calc_4points_bezier_path(
        start_x3F2, start_y3F2, start_yaw3F2, end_x3F2, end_y3F2, end_yaw3F2, offset3F2)

    """ Plot an example bezier curve.Leader (line4)"""
    start_x4F2 = (wp3x - R) + d * math.cos(math.pi + ang + start_yaw4)  # [m]
    start_y4F2 = wp3y + d * math.sin(math.pi + ang + start_yaw4)  # [m]
    start_yaw4F2 = np.radians(0.0)  # [rad]

    end_x4F2 = wp3x + d * math.cos(math.pi + ang + end_yaw4)  # [m]
    end_y4F2 = (wp3y - R) + d * math.sin(math.pi + ang + end_yaw4)  # [m]
    end_yaw4F2 = np.radians(-90.0)  # [rad]
    offset4F2 = 0.39

    path4F2, control_points4F2 = calc_4points_bezier_path(
        start_x4F2, start_y4F2, start_yaw4F2, end_x4F2, end_y4F2, end_yaw4F2, offset4F2)

    """ Plot an example bezier curve.Leader (line5)"""
    start_x5F2 = wp3x + d * math.cos(math.pi + ang + start_yaw5)  # [m]
    start_y5F2 = (wp3y - R) + d * math.sin(math.pi + ang + start_yaw5)  # [m]
    start_yaw5F2 = np.radians(-90.0)  # [rad]

    end_x5F2 = wp4x + d * math.cos(math.pi + ang + end_yaw5)  # [m]
    end_y5F2 = wp4y + d * math.sin(math.pi + ang + end_yaw5)  # [m]
    end_yaw5F2 = np.radians(-90.0)  # [rad]
    offset5F2 = 0.39

    path5F2, control_points5F2 = calc_4points_bezier_path(
        start_x5F2, start_y5F2, start_yaw5F2, end_x5F2, end_y5F2, end_yaw5F2, offset5F2)


#---------------------------------------- 動態軌跡 -------------------------------------------------

    def main3(wp1x, wp1y, wp2x, wp2y, wp3x, wp3y, wp4x, wp4y, R):  # U字形動態軌跡
        """ Plot an example bezier curve.Leader (line1)"""
        start_x2 = wp1x  # [m]
        start_y2 = wp1y  # [m]
        start_yaw2 = np.radians(90.0)  # [rad]

        end_x2 = wp1x  # [m]
        end_y2 = wp2y - R  # [m]
        end_yaw2 = np.radians(90.0)  # [rad]
        offset2 = 3.0

        path2, control_points2 = calc_4points_bezier_path(
            start_x2, start_y2, start_yaw2, end_x2, end_y2, end_yaw2, offset2)

        """Plot an example bezier curve.Leader (line2) """
        start_x = wp1x  # [m]
        start_y = wp2y - R  # [m]
        start_yaw = np.radians(90.0)  # [rad]

        end_x = wp2x + R  # [m]
        end_y = wp2y  # [m]
        end_yaw = np.radians(0.0)  # [rad]
        offset = 3.0

        path, control_points = calc_4points_bezier_path(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, offset)

        """ Plot an example bezier curve.Leader (line3)"""
        start_x3 = wp2x + R  # [m]
        start_y3 = wp2y  # [m]
        start_yaw3 = np.radians(0.0)  # [rad]

        end_x3 = wp3x - R  # [m]
        end_y3 = wp3y  # [m]
        end_yaw3 = np.radians(0.0)  # [rad]
        offset3 = 3.0

        path3, control_points3 = calc_4points_bezier_path(
            start_x3, start_y3, start_yaw3, end_x3, end_y3, end_yaw3, offset3)

        """ Plot an example bezier curve.Leader (line4)"""
        start_x4 = wp3x - R  # [m]
        start_y4 = wp3y  # [m]
        start_yaw4 = np.radians(0.0)  # [rad]

        end_x4 = wp3x  # [m]
        end_y4 = wp3y - R  # [m]
        end_yaw4 = np.radians(-90.0)  # [rad]
        offset4 = 3.0

        path4, control_points4 = calc_4points_bezier_path(
            start_x4, start_y4, start_yaw4, end_x4, end_y4, end_yaw4, offset4)

        """ Plot an example bezier curve.Leader (line5)"""
        start_x5 = wp3x  # [m]
        start_y5 = wp3y - R  # [m]
        start_yaw5 = np.radians(-90.0)  # [rad]

        end_x5 = wp4x  # [m]
        end_y5 = wp4y  # [m]
        end_yaw5 = np.radians(-90.0)  # [rad]
        offset5 = 3.0

        path5, control_points5 = calc_4points_bezier_path(
            start_x5, start_y5, start_yaw5, end_x5, end_y5, end_yaw5, offset5)
    def main4( start_x2,start_y2,end_x2,end_y2):
        # --------------------- Follower1 (out) ----------------------------------------
        """ Plot an example bezier curve.Leader (line1)"""

        start_xF1 = start_x2  # [m]
        start_yF1 = start_y2  # [m]
        start_yawF1 = np.radians(90.0)  # [rad]
        end_xF1 = end_x2  # [m]
        end_yF1 = end_y2  # [m]
        end_yawF1 = np.radians(90.0)  # [rad]
        offsetF1 = 3.0
        pathF1, control_pointsF1 = calc_4points_bezier_path(
            start_xF1, start_yF1, start_yawF1, end_xF1, end_yF1, end_yawF1, offsetF1)
        """Plot an example bezier curve.Leader (line2) """
        start_xF1.append(start_x2)  # [m]
        start_yF1.append(start_y2)  # [m]
        start_yawF1.append(np.radians(90.0))  # [rad]
        end_xF1.append(end_x2)   # [m]
        end_yF1.append(end_x2)   # [m]
        end_yawF1 = np.radians(0.0)  # [rad]
        offsetF1 = 3.0
        pathF1, control_pointsF1.append(calc_4points_bezier_path(
            start_xF1, start_yF1, start_yawF1, end_xF1, end_yF1, end_yawF1, offsetF1))

        """ Plot an example bezier curve.Leader (line3)"""
        start_x3F = (wp2x + R) + d * math.cos(math.pi - ang + start_yaw3)  # [m]
        start_y3F = (wp2y) + wp1x + d * math.sin(math.pi - ang + start_yaw3)  # [m]
        start_yaw3F = np.radians(0.0)  # [rad]

        end_x3F = (wp3x - R) + d * math.cos(math.pi - ang + end_yaw3)  # [m]
        end_y3F = wp3y + d * math.sin(math.pi - ang + end_yaw3)  # [m]
        end_yaw3F = np.radians(0.0)  # [rad]
        offset3F = 3.0

        path3F, control_points3F = calc_4points_bezier_path(
            start_x3F, start_y3F, start_yaw3F, end_x3F, end_y3F, end_yaw3F, offset3F)

        """ Plot an example bezier curve.Leader (line4)"""
        start_x4F = (wp3x - R) + d * math.cos(math.pi - ang + start_yaw4)  # [m]
        start_y4F = wp3y + d * math.sin(math.pi - ang + start_yaw4)  # [m]
        start_yaw4F = np.radians(0.0)  # [rad]

        end_x4F = wp3x + d * math.cos(math.pi - ang + end_yaw4)  # [m]
        end_y4F = (wp3y - R) + d * math.sin(math.pi - ang + end_yaw4)  # [m]
        end_yaw4F = np.radians(-90.0)  # [rad]
        offset4F = 3.0

        path4F, control_points4F = calc_4points_bezier_path(
            start_x4F, start_y4F, start_yaw4F, end_x4F, end_y4F, end_yaw4F, offset4F)

        """ Plot an example bezier curve.Leader (line5)"""
        start_x5F = wp3x + d * math.cos(math.pi - ang + start_yaw5)  # [m]
        start_y5F = (wp3y - R) + d * math.sin(math.pi - ang + start_yaw5)  # [m]
        start_yaw5F = np.radians(-90.0)  # [rad]

        end_x5F = wp4x + d * math.cos(math.pi - ang + end_yaw5)  # [m]
        end_y5F = wp4y + d * math.sin(math.pi - ang + end_yaw5)  # [m]
        end_yaw5F = np.radians(-90.0)  # [rad]
        offset5F = 3.0

        path5F, control_points5F = calc_4points_bezier_path(
            start_x5F, start_y5F, start_yaw5F, end_x5F, end_y5F, end_yaw5F, offset5F)


        # -----------------------------  Follower2  ---------------------------------------------------------
        """ Plot an example bezier curve.Leader (line1)"""

        start_x2F2 = wp1x + d * math.cos(math.pi + ang + start_yaw2)  # [m]
        start_y2F2 = wp1y + d * math.sin(math.pi + ang + start_yaw2)  # [m]
        start_yaw2F2 = np.radians(90.0)  # [rad]
        end_x2F2 = wp1x + d * math.cos(math.pi + ang + end_yaw2)  # [m]
        end_y2F2 = (wp2y - R) + d * math.sin(math.pi + ang + end_yaw2)  # [m]
        end_yaw2F2 = np.radians(90.0)  # [rad]
        offset2F2 = 3.0

        path2F2, control_points2F2 = calc_4points_bezier_path(
            start_x2F2, start_y2F2, start_yaw2F2, end_x2F2, end_y2F2, end_yaw2F2, offset2F2)

        """Plot an example bezier curve.Leader (line2) """
        start_xF2 = wp1x + d * math.cos(math.pi + ang + start_yaw)  # [m]
        start_yF2 = (wp2y - R) + d * math.sin(math.pi + ang + start_yaw)  # [m]
        start_yawF2 = np.radians(90.0)  # [rad]
        end_xF2 = (wp2x + R) + d * math.cos(math.pi + ang + end_yaw)  # [m]
        end_yF2 = wp2y + d * math.sin(math.pi + ang + end_yaw)  # [m]
        end_yawF2 = np.radians(0.0)  # [rad]
        offsetF2 = 3.0

        pathF2, control_pointsF2 = calc_4points_bezier_path(
            start_xF2, start_yF2, start_yawF2, end_xF2, end_yF2, end_yawF2, offsetF2)

        """ Plot an example bezier curve.Leader (line3)"""
        start_x3F2 = (wp2x + R) + d * math.cos(math.pi + ang + start_yaw3)  # [m]
        start_y3F2 = (wp2y) + wp1x + d * math.sin(math.pi + ang + start_yaw3)  # [m]
        start_yaw3F2 = np.radians(0.0)  # [rad]

        end_x3F2 = (wp3x - R) + d * math.cos(math.pi + ang + end_yaw3)  # [m]
        end_y3F2 = wp3y + d * math.sin(math.pi + ang + end_yaw3)  # [m]
        end_yaw3F2 = np.radians(0.0)  # [rad]
        offset3F2 = 3.0

        path3F2, control_points3F2 = calc_4points_bezier_path(
            start_x3F2, start_y3F2, start_yaw3F2, end_x3F2, end_y3F2, end_yaw3F2, offset3F2)

        """ Plot an example bezier curve.Leader (line4)"""
        start_x4F2 = (wp3x - R) + d * math.cos(math.pi + ang + start_yaw4)  # [m]
        start_y4F2 = wp3y + d * math.sin(math.pi + ang + start_yaw4)  # [m]
        start_yaw4F2 = np.radians(0.0)  # [rad]

        end_x4F2 = wp3x + d * math.cos(math.pi + ang + end_yaw4)  # [m]
        end_y4F2 = (wp3y - R) + d * math.sin(math.pi + ang + end_yaw4)  # [m]
        end_yaw4F2 = np.radians(-90.0)  # [rad]
        offset4F2 = 3.0

        path4F2, control_points4F2 = calc_4points_bezier_path(
            start_x4F2, start_y4F2, start_yaw4F2, end_x4F2, end_y4F2, end_yaw4F2, offset4F2)

        """ Plot an example bezier curve.Leader (line5)"""
        start_x5F2 = wp3x + d * math.cos(math.pi + ang + start_yaw5)  # [m]
        start_y5F2 = (wp3y - R) + d * math.sin(math.pi + ang + start_yaw5)  # [m]
        start_yaw5F2 = np.radians(-90.0)  # [rad]

        end_x5F2 = wp4x + d * math.cos(math.pi + ang + end_yaw5)  # [m]
        end_y5F2 = wp4y + d * math.sin(math.pi + ang + end_yaw5)  # [m]
        end_yaw5F2 = np.radians(-90.0)  # [rad]
        offset5F2 = 3.0

        path5F2, control_points5F2 = calc_4points_bezier_path(
            start_x5F2, start_y5F2, start_yaw5F2, end_x5F2, end_y5F2, end_yaw5F2, offset5F2)

    if show_animation:  # pragma: no cover
#------------------------------------------- Leader ------------------------------------------------------------------
        fig,ax = plt.subplots()
        ax.plot(path.T[0], path.T[1],'k',label="Bezier Path",linewidth=3)
        plot_arrow(start_x, start_y, start_yaw)
        plot_arrow(end_x, end_y, end_yaw)
        ax.plot(path2.T[0], path2.T[1],'k', label="Bezier Path",linewidth=3)
        plot_arrow(start_x2, start_y2, start_yaw2)
        plot_arrow(end_x2, end_y2, end_yaw2)
        ax.plot(path3.T[0], path3.T[1],'k', label="Bezier Path",linewidth=3)
        plot_arrow(start_x3, start_y3, start_yaw3)
        plot_arrow(end_x3, end_y3, end_yaw3)
        ax.plot(path4.T[0], path4.T[1],'k', label="Bezier Path",linewidth=3)
        plot_arrow(start_x4, start_y4, start_yaw4)
        plot_arrow(end_x4, end_y4, end_yaw4)
        ax.plot(path5.T[0], path5.T[1], 'k',label="Bezier Path",linewidth=3)
        plot_arrow(start_x5, start_y5, start_yaw5)
        plot_arrow(end_x5, end_y5, end_yaw5)
        # ax.plot(control_points.T[0], control_points.T[1],'--o', label="Control Points")
        # ax.plot(control_points2.T[0], control_points2.T[1], '--o', label="Control Points")
        # ax.plot(control_points3.T[0], control_points3.T[1], '--o', label="Control Points")
        # ax.plot(control_points4.T[0], control_points4.T[1], '--o', label="Control Points")
        # ax.plot(control_points5.T[0], control_points5.T[1], '--o', label="Control Points")
#------------------------------------------ Follower1 ---------------------------------------------------------------
        ax.plot(pathF.T[0], pathF.T[1], 'g' ,label="Bezier Path",linewidth=3)
        plot_arrow(start_xF, start_yF, start_yawF)
        plot_arrow(end_xF, end_yF, end_yawF)
        ax.plot(path2F.T[0], path2F.T[1],'g' , label="Bezier Path",linewidth=3)
        plot_arrow(start_x2F, start_y2F, start_yaw2F)
        plot_arrow(end_x2F, end_y2F, end_yaw2F)
        ax.plot(path3F.T[0], path3F.T[1], 'g' ,label="Bezier Path",linewidth=3)
        plot_arrow(start_x3F, start_y3F, start_yaw3F)
        plot_arrow(end_x3F, end_y3F, end_yaw3F)
        ax.plot(path4F.T[0], path4F.T[1],'g' , label="Bezier Path",linewidth=3)
        plot_arrow(start_x4F, start_y4F, start_yaw4F)
        plot_arrow(end_x4F, end_y4F, end_yaw4F)
        ax.plot(path5F.T[0], path5F.T[1],'g', label="Bezier Path",linewidth=3)
        plot_arrow(start_x5F, start_y5F, start_yaw5F)
        plot_arrow(end_x5F, end_y5F, end_yaw5F)
        # ax.plot(control_pointsF.T[0], control_pointsF.T[1], '--o', label="Control Points")
        # ax.plot(control_points2F.T[0], control_points2F.T[1], '--o', label="Control Points")
        # ax.plot(control_points3F.T[0], control_points3F.T[1], '--o', label="Control Points")
        # ax.plot(control_points4F.T[0], control_points4F.T[1], '--o', label="Control Points")
        # ax.plot(control_points5F.T[0], control_points5F.T[1], '--o', label="Control Points")
#-------------------------------------- Follower2 ----------------------------------------------------------------------
        ax.plot(pathF2.T[0], pathF2.T[1],'gold' ,label="Bezier Path",linewidth=3)
        plot_arrow(start_xF2, start_yF2, start_yawF2)
        plot_arrow(end_xF2, end_yF2, end_yawF2)
        ax.plot(path2F2.T[0], path2F2.T[1], 'gold' ,label="Bezier Path",linewidth=3)
        plot_arrow(start_x2F2, start_y2F2, start_yaw2F2)
        plot_arrow(end_x2F2, end_y2F2, end_yaw2F2)
        ax.plot(path3F2.T[0], path3F2.T[1], 'gold' ,label="Bezier Path",linewidth=3)
        plot_arrow(start_x3F2, start_y3F, start_yaw3F2)
        plot_arrow(end_x3F2, end_y3F2, end_yaw3F2)
        ax.plot(path4F2.T[0], path4F2.T[1], 'gold' ,label="Bezier Path",linewidth=3)
        plot_arrow(start_x4F2, start_y4F2, start_yaw4F2)
        plot_arrow(end_x4F2, end_y4F2, end_yaw4F2)
        ax.plot(path5F2.T[0], path5F2.T[1], 'gold' ,label="Bezier Path",linewidth=3)
        plot_arrow(start_x5F2, start_y5F2, start_yaw5F2)
        plot_arrow(end_x5F2, end_y5F2, end_yaw5F2)
        # ax.plot(control_pointsF2.T[0], control_pointsF2.T[1], '--o', label="Control Points")
        # ax.plot(control_points2F2.T[0], control_points2F2.T[1], '--o', label="Control Points")
        # ax.plot(control_points3F2.T[0], control_points3F2.T[1], '--o', label="Control Points")
        # ax.plot(control_points4F2.T[0], control_points4F2.T[1], '--o', label="Control Points")
        # ax.plot(control_points5F2.T[0], control_points5F2.T[1], '--o', label="Control Points")
#----------------------------------------------------------------------------------------------------------------------------------------------------------------
        ax.plot([path.T[0],pathF.T[0]],[path.T[1],pathF.T[1]],'b--')
        ax.plot([path.T[0],pathF2.T[0]],[path.T[1],pathF2.T[1]],'b--')
        ax.plot([pathF.T[0],pathF2.T[0]],[pathF.T[1],pathF2.T[1]],'b--')
        ax.plot([path4.T[0], path4F.T[0]], [path4.T[1], path4F.T[1]], 'b--')
        ax.plot([path4.T[0], path4F2.T[0]], [path4.T[1], path4F2.T[1]], 'b--')
        ax.plot([path4F.T[0], path4F2.T[0]], [path4F.T[1], path4F2.T[1]], 'b--')
        ax.plot([0.0,0.0],[-45.0,55.0],'k',linewidth=1.5)
        ax.plot([0.0,100.0],[55.0,55.0],'k',linewidth=1.5)
        ax.plot([100.0,100.0],[55.0,-45.0],'k',linewidth=1.5)
        ax.plot([100.0,0.0],[-45.0,-45.0],'k',linewidth=1.5)
        ax.legend()
        ax.axis("equal")
        ax.grid(True)
        print(path)
        plt.show()
        # des_ps[2:4] = [(uavs_p[0]+d*math.cos(math.pi-ang+heading)), (uavs_p[1]+d*math.sin(math.pi-ang+heading))]
        # des_ps[4:] =  [(uavs_p[0]+d*math.cos(math.pi+ang+heading)), (uavs_p[1]+d*math.sin(math.pi+ang+heading))]

if __name__ == '__main__':
    main2(0.0,-45.0, 0.0,55.0, 100.0,55.0, 100.0,-45.0 ,30.0) #轉彎半徑限制 1. 2R  <= 路徑的邊長 L  ex: L = 140  => R最大只能70  2. R >= 3d  ***** if d =10 , R>=3d=30 , L >= 2R = 60 **** ;