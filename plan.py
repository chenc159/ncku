import math
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm
from bezierpath_U import calc_4points_bezier_path
from sympy import symbols, Eq, solve
# import pymap3d as pm

def triangle_straight(ts, id, Desired_dist, Radius, Angle, Waypt_num, x, y):
    des_hdg = []
    for i in range(len(x)-1):
        hdg = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        des_hdg.extend([hdg,hdg])
    wp = [[x[0],y[0]]]
    for i in range(1, len(x)-1):
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        wp.append([round(x[i]-Radius*math.cos(hdg1),2), round(y[i]-Radius*math.sin(hdg1),2)])
        wp.append([round(x[i]-Radius*math.cos(hdg2),2), round(y[i]-Radius*math.sin(hdg2),2)])
    wp.append([x[-1],y[-1]])

    temp = wp
    wp_x, wp_y = [temp[0][0]], [temp[0][1]]
    for i in range(len(temp)-1):
        path, control_pt = calc_4points_bezier_path(temp[i][0], temp[i][1], des_hdg[i], temp[i+1][0], temp[i+1][1], des_hdg[i+1], 0.39, Waypt_num)
        wp_x.extend([round(a,2) for a,b in list(path[1:])])
        wp_y.extend([round(b,2) for a,b in list(path[1:])])
    
    if id == 0: # leader
        return wp_x, wp_y
    else: # followers
        if ts == 1: # triangle formation
            ang = Angle*math.pi/180
            if id == 2: ang *= -1
            wp_x12, wp_y12 = [], []
            # wp_x122, wp_y122 = [], []
            for i in range(len(wp_x)):
                if i != len(wp_x)-1:
                    heading = math.atan2((wp_y[i+1] - wp_y[i]),(wp_x[i+1] - wp_x[i]))
                wp_x12.extend([wp_x[i]+Desired_dist*math.cos(math.pi+heading+ang)])
                wp_y12.extend([wp_y[i]+Desired_dist*math.sin(math.pi+heading+ang)])
                # wp_x122.extend([wp_x[i]+Desired_dist*math.cos(math.pi+heading-ang)])
                # wp_y122.extend([wp_y[i]+Desired_dist*math.sin(math.pi+heading-ang)])
            return wp_x12, wp_y12
        elif ts == 2: # straight line formation
            wp_x12, wp_y12 = [], []
            # wp_x122, wp_y122 = [], []
            for i in range(len(wp_x)):
                if i != len(wp_x)-1:
                    heading = math.atan2((wp_y[i+1] - wp_y[i]),(wp_x[i+1] - wp_x[i]))
                wp_x12.extend([wp_x[i]+id*Desired_dist*math.cos(math.pi+heading)])
                wp_y12.extend([wp_y[i]+id*Desired_dist*math.sin(math.pi+heading)])
                # wp_x122.extend([wp_x[i]+2*Desired_dist*math.cos(math.pi+heading)])
                # wp_y122.extend([wp_y[i]+2*Desired_dist*math.sin(math.pi+heading)])
            return wp_x12, wp_y12

def triangle_p(Desired_dist, Radius, Angle, Waypt_num, x, y):
    des_hdg = []
    for i in range(len(x)-1):
        hdg = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        des_hdg.extend([hdg,hdg])
    wp = [[x[0],y[0]]]
    for i in range(1, len(x)-1):
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        wp.append([round(x[i]-Radius*math.cos(hdg1),2), round(y[i]-Radius*math.sin(hdg1),2)])
        wp.append([round(x[i]-Radius*math.cos(hdg2),2), round(y[i]-Radius*math.sin(hdg2),2)])
    wp.append([x[-1],y[-1]])

    temp = wp
    wp_x, wp_y = [temp[0][0]], [temp[0][1]]
    for i in range(len(temp)-1):
        path, control_pt = calc_4points_bezier_path(temp[i][0], temp[i][1], des_hdg[i], temp[i+1][0], temp[i+1][1], des_hdg[i+1], 0.39, Waypt_num)
        wp_x.extend([round(a,2) for a,b in list(path[1:])])
        wp_y.extend([round(b,2) for a,b in list(path[1:])])
    
    ang = Angle*math.pi/180
    wp_x1, wp_y1 = [], []
    for i in range(len(wp_x)):
        if i != len(wp_x)-1:
            heading = math.atan2((wp_y[i+1] - wp_y[i]),(wp_x[i+1] - wp_x[i]))
        wp_x1.extend([wp_x[i]+Desired_dist*math.cos(math.pi+heading+ang)])
        wp_y1.extend([wp_y[i]+Desired_dist*math.sin(math.pi+heading+ang)])
    ang *= -1
    wp_x2, wp_y2 = [], []
    for i in range(len(wp_x)):
        if i != len(wp_x)-1:
            heading = math.atan2((wp_y[i+1] - wp_y[i]),(wp_x[i+1] - wp_x[i]))
        wp_x2.extend([wp_x[i]+Desired_dist*math.cos(math.pi+heading+ang)])
        wp_y2.extend([wp_y[i]+Desired_dist*math.sin(math.pi+heading+ang)])
    fig = plt.figure(1)
    norm = colors.Normalize(vmin=0, vmax=4)
    plt.plot(np.asarray(wp_x), np.asarray(wp_y), color = cm.hsv(norm(1)))
    plt.plot(np.asarray(wp_x1), np.asarray(wp_y1), color = cm.hsv(norm(2)))
    plt.plot(np.asarray(wp_x2), np.asarray(wp_y2), color = cm.hsv(norm(3)))
    # plt.scatter(d1x, d1y, color = cm.hsv(norm(1)))
    # plt.scatter(d2x, d2y, color = cm.hsv(norm(2)))
    # plt.scatter(d3x, d3y, color = cm.hsv(norm(3)))
    plt.grid()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.legend(['uav1', 'uav2', 'uav3'])
    plt.show()

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

def points_L2F(ts, id, Desired_dist, Angle, Lx, Ly, heading):
    if ts == 1: # triangle formation
        ang = Angle*math.pi/180
        if id == 2: ang *= -1
        Fx = Lx + Desired_dist*math.cos(math.pi+heading+ang)
        Fy = Ly + Desired_dist*math.sin(math.pi+heading+ang)
        return Fx, Fy
    elif ts == 2: # straight line formation
        Fx = Lx+id*Desired_dist*math.cos(math.pi+heading)
        Fy = Ly+id*Desired_dist*math.sin(math.pi+heading)
        return Fx, Fy


# triangle_p(10, 5, 60, 30, [0, 0, 51, 45], [0, 50, 45, 0])
# triangle_pi(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 45, 0])

# # Test points_L2F
# x0, y0 = 2, 5
# x1, y1 = points_L2F(1,1,5,60,x0,y0,0)
# x2, y2 = points_L2F(1,2,5,60,x0,y0,0)
# fig = plt.figure(1)
# norm = colors.Normalize(vmin=0, vmax=4)
# plt.scatter(x0, y0, color = cm.hsv(norm(1)))
# plt.scatter(x1, y1, color = cm.hsv(norm(2)))
# plt.scatter(x2, y2, color = cm.hsv(norm(3)))
# plt.grid()
# plt.xlabel('X')
# plt.ylabel('Y')
# plt.axis('equal')
# plt.legend(['uav1', 'uav2', 'uav3'])
# plt.show()




