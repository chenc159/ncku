import math
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm
from bezierpath_U import calc_4points_bezier_path
from sympy import symbols, Eq, solve
# import pymap3d as pm


def triangle_pi(Desired_dist, Radius, Angle, Waypt_num, x, y):
    
    des_hdg = []
    for i in range(len(x)-1):
        hdg = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        des_hdg.extend([hdg,hdg])

    ang = Angle*math.pi/180
    wp_x1, wp_y1 = [x[0]+Desired_dist*math.cos(math.pi+des_hdg[0]+ang)], [y[0]+Desired_dist*math.sin(math.pi+des_hdg[0]+ang)]
    ang *= -1
    wp_x2, wp_y2 = [x[0]+Desired_dist*math.cos(math.pi+des_hdg[0]+ang)], [y[0]+Desired_dist*math.sin(math.pi+des_hdg[0]+ang)]
    ang = Angle*math.pi/180
    for i in range(1, len(x)-1):
        xs, ys = symbols('x y')
        # y = mx + b 
        m1, m2 = (y[i] - y[i-1])/(x[i] - x[i-1]), (y[i+1] - y[i])/(x[i+1] - x[i])
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        # hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        hdg2 = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))

        ang = Angle*math.pi/180
        x11, y11 = x[i]+Desired_dist*math.cos(math.pi+hdg1+ang), y[i]+Desired_dist*math.sin(math.pi+hdg1+ang)
        x12, y12 = x[i]+Desired_dist*math.cos(math.pi+hdg2+ang), y[i]+Desired_dist*math.sin(math.pi+hdg2+ang)
        b1 = y11 - m1*x11
        b2 = y12 - m2*x12
        eq1 = Eq(ys-m1*xs-b1)
        eq2 = Eq(ys-m2*xs-b2)
        sol = solve((eq1,eq2), (xs, ys))
        # print(sol)
        wp_x1.append(sol[xs])
        wp_y1.append(sol[ys])

        ang *= -1
        x21, y21 = x[i]+Desired_dist*math.cos(math.pi+hdg1+ang), y[i]+Desired_dist*math.sin(math.pi+hdg1+ang)
        x22, y22 = x[i]+Desired_dist*math.cos(math.pi+hdg2+ang), y[i]+Desired_dist*math.sin(math.pi+hdg2+ang)
        b1 = y21 - m1*x21
        b2 = y22 - m2*x22
        eq1 = Eq(ys-m1*xs-b1)
        eq2 = Eq(ys-m2*xs-b2)
        sol = solve((eq1,eq2), (xs, ys))
        # print(sol)
        wp_x2.append(sol[xs])
        wp_y2.append(sol[ys])

    ang = Angle*math.pi/180
    wp_x1.append(x[-1]+Desired_dist*math.cos(math.pi+des_hdg[-1]+ang))
    wp_y1.append(y[-1]+Desired_dist*math.sin(math.pi+des_hdg[-1]+ang))
    ang *= -1
    wp_x2.append(x[-1]+Desired_dist*math.cos(math.pi+des_hdg[-1]+ang))
    wp_y2.append(y[-1]+Desired_dist*math.sin(math.pi+des_hdg[-1]+ang))


    wp = [[x[0],y[0]]]
    wp1 = [[wp_x1[0], wp_y1[0]]]
    wp2 = [[wp_x2[0], wp_y2[0]]]
    for i in range(1, len(x)-1):
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        # hdg2 = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        wp.append([round(x[i]-Radius*math.cos(hdg1),2), round(y[i]-Radius*math.sin(hdg1),2)])
        wp.append([round(x[i]-Radius*math.cos(hdg2),2), round(y[i]-Radius*math.sin(hdg2),2)])
        wp1.append([round(wp_x1[i]-Radius*math.cos(hdg1),2), round(wp_y1[i]-Radius*math.sin(hdg1),2)])
        wp1.append([round(wp_x1[i]-Radius*math.cos(hdg2),2), round(wp_y1[i]-Radius*math.sin(hdg2),2)])
        wp2.append([round(wp_x2[i]-Radius*math.cos(hdg1),2), round(wp_y2[i]-Radius*math.sin(hdg1),2)])
        wp2.append([round(wp_x2[i]-Radius*math.cos(hdg2),2), round(wp_y2[i]-Radius*math.sin(hdg2),2)])

    wp.append([x[-1], y[-1]])
    wp1.append([wp_x1[-1], wp_y1[-1]])
    wp2.append([wp_x2[-1], wp_y2[-1]])


    temp, temp1, temp2 = wp, wp1, wp2
    wp_x, wp_y = [temp[0][0]], [temp[0][1]]
    wp_x1, wp_y1 = [temp1[0][0]], [temp1[0][1]]
    wp_x2, wp_y2 = [temp2[0][0]], [temp2[0][1]]
    for i in range(len(temp)-1):
        path, control_pt = calc_4points_bezier_path(temp[i][0], temp[i][1], des_hdg[i], temp[i+1][0], temp[i+1][1], des_hdg[i+1], 0.39, Waypt_num)
        wp_x.extend([round(a,2) for a,b in list(path[1:])])
        wp_y.extend([round(b,2) for a,b in list(path[1:])])
        path, control_pt = calc_4points_bezier_path(temp1[i][0], temp1[i][1], des_hdg[i], temp1[i+1][0], temp1[i+1][1], des_hdg[i+1], 0.39, Waypt_num)
        wp_x1.extend([round(a,2) for a,b in list(path[1:])])
        wp_y1.extend([round(b,2) for a,b in list(path[1:])])
        path, control_pt = calc_4points_bezier_path(temp2[i][0], temp2[i][1], des_hdg[i], temp2[i+1][0], temp2[i+1][1], des_hdg[i+1], 0.39, Waypt_num)
        wp_x2.extend([round(a,2) for a,b in list(path[1:])])
        wp_y2.extend([round(b,2) for a,b in list(path[1:])])

    
    fig = plt.figure(1)
    norm = colors.Normalize(vmin=0, vmax=6)
    plt.plot(np.asarray(x), np.asarray(y), color = cm.hsv(norm(4)))
    # plt.plot(np.asarray(wp_x), np.asarray(wp_y), color = cm.hsv(norm(1)))
    # plt.plot(np.asarray(wp_x1), np.asarray(wp_y1), color = cm.hsv(norm(2)))
    # plt.plot(np.asarray(wp_x2), np.asarray(wp_y2), color = cm.hsv(norm(3)))
    plt.scatter(np.asarray(wp_x), np.asarray(wp_y), color = cm.hsv(norm(1)))
    plt.scatter(np.asarray(wp_x1), np.asarray(wp_y1), color = cm.hsv(norm(2)))
    plt.scatter(np.asarray(wp_x2), np.asarray(wp_y2), color = cm.hsv(norm(3)))
    plt.grid()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.legend(['wpt','uav1', 'uav2', 'uav3'])
    plt.show()

triangle_pi(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 0])



