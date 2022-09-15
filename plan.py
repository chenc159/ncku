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










def dec_speed(v0, a):
    t = (0-v0)/a
    s = v0*t + 0.5*a*(t**2)
    print('dec_speed', round(s, 3))
    return round(s, 3)

def three_stage_pos(Desired_dist, Angle, x, y, v0, a):
    '''Three stages: go straight with constant speed, go straight decelerating, and turning'''

    # dist to wpt that shall slow down
    dec_v_dist = dec_speed(v0, a)
    
    # pivot point (add start and final just to balance length)
    pivot_id, pivot_x, pivot_y = [0], [x[0]], [y[0]]
    # total turning angle around pivot
    Ltang = [0]
    
    # Calculate angles relative to leader's position
    ang = {0: 0, 1: Angle*math.pi/180, 2: -Angle*math.pi/180}

    # add leader, follow1, follow2's first coordinate (relative to leader)
    des_hdg0 = math.atan2((y[1] - y[0]),(x[1] - x[0]))
    wp_x = {0: [x[0]], 1: [x[0]+Desired_dist*math.cos(math.pi+des_hdg0+ang[1])], 2: [x[0]+Desired_dist*math.cos(math.pi+des_hdg0+ang[2])]}
    wp_y = {0: [y[0]], 1: [y[0]+Desired_dist*math.sin(math.pi+des_hdg0+ang[1])], 2: [y[0]+Desired_dist*math.sin(math.pi+des_hdg0+ang[2])]}

    # calculate vehicles' coordinates
    for i in range(1, len(x)-1): # omit first and last point cuz no turning at these points
        xs, ys = symbols('xs ys')
        # y = mx + b 
        m1, m2 = (y[i] - y[i-1])/(x[i] - x[i-1]), (y[i+1] - y[i])/(x[i+1] - x[i]) # calculate slope
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        # hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        hdg2 = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))

        # Calculate follower1's wpts' path intersection
        x11, y11 = x[i] + Desired_dist*math.cos(math.pi+hdg1+ang[1]), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang[1])
        x12, y12 = x[i] + Desired_dist*math.cos(math.pi+hdg2+ang[1]), y[i] + Desired_dist*math.sin(math.pi+hdg2+ang[1])
        b1 = y11 - m1*x11
        b2 = y12 - m2*x12
        eq1 = Eq(ys-m1*xs-b1)
        eq2 = Eq(ys-m2*xs-b2)
        sol1 = solve((eq1,eq2), (xs, ys))

        # calculate follower2's wpts' path intersection
        x21, y21 = x[i] + Desired_dist*math.cos(math.pi+hdg1+ang[2]), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang[2])
        x22, y22 = x[i] + Desired_dist*math.cos(math.pi+hdg2+ang[2]), y[i] + Desired_dist*math.sin(math.pi+hdg2+ang[2])
        b1 = y21 - m1*x21
        b2 = y22 - m2*x22
        eq1 = Eq(ys-m1*xs-b1)
        eq2 = Eq(ys-m2*xs-b2)
        sol2 = solve((eq1,eq2), (xs, ys))

        # Follower with shorter path-to-travel is the pivot
        if (sol1[xs]-x11)**2+(sol1[ys]-y11)**2 < (sol2[xs]-x21)**2+(sol2[ys]-y21)**2: # turn right, F1 as pivot
            pivot_id.append(1)
            pivot_x.append(sol1[xs])
            pivot_y.append(sol1[ys])
        else: # turn left, F2 as pivot
            pivot_id.append(2)
            pivot_x.append(sol2[xs])
            pivot_y.append(sol2[ys])

        # leader's position before and after turning
        Lpx_bt = pivot_x[i] - Desired_dist*math.cos(math.pi+hdg1+ang[pivot_id[i]])
        Lpy_bt = pivot_y[i] - Desired_dist*math.sin(math.pi+hdg1+ang[pivot_id[i]])
        Lpx_at = pivot_x[i] - Desired_dist*math.cos(math.pi+hdg2+ang[pivot_id[i]])
        Lpy_at = pivot_y[i] - Desired_dist*math.sin(math.pi+hdg2+ang[pivot_id[i]])

        # Leader's (= Follower's) turning angle wrt pivot (1 as start, 2 as end, ang as totle ang)
        Lt1 = math.atan2((pivot_y[i] - Lpy_bt),(pivot_x[i] - Lpx_bt)) + math.pi
        Lt2 = math.atan2((pivot_y[i] - Lpy_at),(pivot_x[i] - Lpx_at)) + math.pi
        Ltang.append(math.copysign(1, pivot_id[i]-1.5)*min(abs(Lt2 - Lt1), abs(2*math.pi - abs(Lt2 - Lt1))))
        
        # add the leader and followers' locations of starting to slow down
        wp_x[0].append(round(Lpx_bt-dec_v_dist*math.cos(hdg1),2))
        wp_y[0].append(round(Lpy_bt-dec_v_dist*math.sin(hdg1),2))
        wp_x[1].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg1+ang[1]),2))
        wp_y[1].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg1+ang[1]),2))
        wp_x[2].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg1+ang[2]),2))
        wp_y[2].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg1+ang[2]),2))

        # add the leader and followers' locations before turning (shall be v = 0)
        wp_x[0].append(round(Lpx_bt,2))
        wp_y[0].append(round(Lpy_bt,2))
        wp_x[1].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg1+ang[1]),2))
        wp_y[1].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg1+ang[1]),2))
        wp_x[2].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg1+ang[2]),2))
        wp_y[2].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg1+ang[2]),2))

        # add the leader and followers' locations after turning (shall be v = 0)
        wp_x[0].append(round(Lpx_at,2))
        wp_y[0].append(round(Lpy_at,2))
        wp_x[1].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg2+ang[1]),2))
        wp_y[1].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg2+ang[1]),2))
        wp_x[2].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg2+ang[2]),2))
        wp_y[2].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg2+ang[2]),2))
    
    # add the decelerating position before ending mission
    wp_x[0].append(round(x[-1]-dec_v_dist*math.cos(hdg2),2))
    wp_y[0].append(round(y[-1]-dec_v_dist*math.sin(hdg2),2))
    wp_x[1].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg2+ang[1]),2))
    wp_y[1].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg2+ang[1]),2))
    wp_x[2].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg2+ang[2]),2))
    wp_y[2].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg2+ang[2]),2))

    # add on the last point
    wp_x[0].append(x[-1])
    wp_y[0].append(y[-1])
    wp_x[1].append(x[-1]+Desired_dist*math.cos(math.pi+hdg2+ang[1]))
    wp_y[1].append(y[-1]+Desired_dist*math.sin(math.pi+hdg2+ang[1]))
    wp_x[2].append(x[-1]+Desired_dist*math.cos(math.pi+hdg2+ang[2]))
    wp_y[2].append(y[-1]+Desired_dist*math.sin(math.pi+hdg2+ang[2]))

    return wp_x, wp_y, Ltang


def speed_time(Desired_dist, Angle, x, y, v0, a, w0):
    '''
    Desired_dist: disired distance between leader and follower (m)
    Angle: desired angle between leader and follower (deg)
    x: x coordinates of waypoints (m)
    y: y coordinates of waypoints (m)
    v0: preferred speed while going straight with constant speed (m/s)
    a: preferred acceleration while going straight and decelerating (m^2/s)
    w0: preferred angular speed while turning (deg/s)
    '''

    # convert unit (deg2rad)
    w0 *= 0.01745
    
    # desired follower to follower distance
    Desired_dist_f2f = 2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist
    
    # get the uavs positions at three stages (straight line with constant speed, straight line decelerating, and turning)
    # and the turing angle
    wp_x, wp_y, Ltang = three_stage_pos(Desired_dist, Angle, x, y, v0, a)
    
    # Calculate the time at each wpt each stage 
    time = [0] 
    for i in range(1, len(x)-1):
        # straight line with constant speed
        time.append(((wp_x[0][(i-1)*3+1] - wp_x[0][(i-1)*3])**2+(wp_y[0][(i-1)*3+1] - wp_y[0][(i-1)*3])**2)**0.5/v0 + time[-1])
        # straight line decelerating
        time.append((0-v0)/a + time[-1])
        # turning
        time.append(abs(Ltang[i])/w0 + time[-1])
    # Add last wpt's 
    time.append(((wp_x[0][-2] - wp_x[0][-3])**2+(wp_y[0][-2] - wp_y[0][-3])**2)**0.5/v0 + time[-1])
    time.append((0-v0)/a + time[-1])
    # # total time len = len(wp_x[0]) = 1st pt + 3 stages of intermediate wpt + slow down and last pt = 1 + 3*(wpt-2) + 2
    # print(len(x), len(wp_x[0]), len(time), time)

    time_no, i = 0, 0 # time no/idex, wpt no/idex
    norm_x = (x[i+1] - x[i])/((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)**0.5
    norm_y = (y[i+1] - y[i])/((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)**0.5
    
    dt = 0.25
    t_lin = np.linspace(time[0], time[-1], int((time[-1]-time[0])/dt))
    
    vx, vy = {0:[], 1:[], 2:[]}, {0:[], 1:[], 2:[]}
    vx, vy = {0:[v0*norm_x], 1:[v0*norm_x], 2:[v0*norm_x]}, {0:[v0*norm_y], 1:[v0*norm_y], 2:[v0*norm_y]}
    px, py = {0:[wp_x[0][0]], 1:[wp_x[1][0]], 2:[wp_x[2][0]]}, {0:[wp_y[0][0]], 1:[wp_y[1][0]], 2:[wp_y[2][0]]}
    theta0 = {0:0, 1:0, 2:0}


    for t in t_lin:
        if t > time[time_no+1]:
            time_no += 1
            i = int(time_no/3)
            if (time_no - i*3 + 1)%3 == 0: # turning
                # Get the pivot follower id (compare distance)
                dx1, dy1 = (wp_x[1][time_no]-wp_x[1][time_no+1]), (wp_y[1][time_no]-wp_y[1][time_no+1])
                dx2, dy2 = (wp_x[2][time_no]-wp_x[2][time_no+1]), (wp_y[2][time_no]-wp_y[2][time_no+1])
                if dx1**2 + dy1**2 < dx2**2 + dy2**2:
                    pivot_id = 1
                    theta0[2] = math.atan2((wp_y[pivot_id][time_no] - wp_y[2][time_no]),(wp_x[pivot_id][time_no] - wp_x[2][time_no])) + math.pi
                else: 
                    pivot_id = 2
                    theta0[1] = math.atan2((wp_y[pivot_id][time_no] - wp_y[1][time_no]),(wp_x[pivot_id][time_no] - wp_x[1][time_no])) + math.pi
                theta0[0] = math.atan2((wp_y[pivot_id][time_no] - wp_y[0][time_no]),(wp_x[pivot_id][time_no] - wp_x[0][time_no])) + math.pi
                dist = {0: Desired_dist, 1: Desired_dist_f2f, 2: Desired_dist_f2f, pivot_id: 0}            
            else:
                norm_x = (x[i+1] - x[i])/((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)**0.5
                norm_y = (y[i+1] - y[i])/((x[i+1] - x[i])**2 + (y[i+1] - y[i])**2)**0.5

        if (time_no - i*3 + 1)%3 == 0: # turning
            for v in range(3):
                theta = theta0[v] + (w0*math.copysign(1,Ltang[i+1]))*(t-time[time_no])
                vx[v].append(-dist[v]*(w0*math.copysign(1,Ltang[i+1]))*math.sin(theta))
                vy[v].append(dist[v]*(w0*math.copysign(1,Ltang[i+1]))*math.cos(theta))
                px[v].append(dist[v]*math.cos(theta) + wp_x[pivot_id][time_no])
                py[v].append(dist[v]*math.sin(theta) + wp_y[pivot_id][time_no])
        else:
            if (time_no - i*3 + 1)%2 == 0: # decelerating
                acc = a
            else: acc = 0 # constant speed
            for v in range(3):
                vx[v].append((v0 + acc*(t-time[time_no]))*norm_x)
                vy[v].append((v0 + acc*(t-time[time_no]))*norm_y)
                px[v].append(wp_x[v][time_no]+(v0*(t-time[time_no])+0.5*acc*(t-time[time_no])**2)*norm_x)
                py[v].append(wp_y[v][time_no]+(v0*(t-time[time_no])+0.5*acc*(t-time[time_no])**2)*norm_y)





class formation_plan():
    def __init__(self, Desired_dist, Angle, x, y, v0, a, w0):
        self.Desired_dist, self.Angle, self.x, self.y, self.v0, self.a, self.w0 = Desired_dist, Angle, x, y, v0, a, w0
        
        # convert unit (deg2rad)
        self.w0 *= 0.01745
        
        # desired follower to follower distance
        self.Desired_dist_f2f = 2*math.sin(self.Angle*0.01745)/math.sin(90*0.01745)*self.Desired_dist
        
        # get the uavs positions at three stages (straight line with constant speed, straight line decelerating, and turning)
        # and the turing angle
        self.wp_x, self.wp_y, self.Ltang = three_stage_pos(self.Desired_dist, self.Angle, self.x, self.y, self.v0, self.a)
        
        # Calculate the time at each wpt each stage 
        self.time = [0.0] 
        for i in range(1, len(self.x)-1):
            # straight line with constant speed
            self.time.append(((self.wp_x[0][(i-1)*3+1] - self.wp_x[0][(i-1)*3])**2+(self.wp_y[0][(i-1)*3+1] - self.wp_y[0][(i-1)*3])**2)**0.5/self.v0 + self.time[-1])
            # straight line decelerating
            self.time.append((0-self.v0)/self.a + self.time[-1])
            # turning
            self.time.append(abs(self.Ltang[i])/self.w0 + self.time[-1])
        # Add last wpt's 
        self.time.append(((self.wp_x[0][-2] - self.wp_x[0][-3])**2+(self.wp_y[0][-2] - self.wp_y[0][-3])**2)**0.5/self.v0 + self.time[-1])
        self.time.append((0-v0)/a + self.time[-1])
        # print('Stages Time: ', self.time)
        # # total time len = len(wp_x[0]) = 1st pt + 3 stages of intermediate wpt + slow down and last pt = 1 + 3*(wpt-2) + 2
        # print(len(x), len(wp_x[0]), len(time), time)
    
    def get_vel_pos(self, id, tc):
        if tc >= self.time[-1]: # if time is larger than the end of mission, return last pos and zero vel
            theta = math.atan2((self.wp_y[id][-1] - self.wp_y[id][-2]),(self.wp_x[id][-1] - self.wp_x[id][-2]))
            return [self.wp_x[id][-1], self.wp_y[id][-1]], [0, 0], theta
        for j in range(len(self.time)):
            if tc < self.time[j]:
                time_no = j - 1
                i = int(time_no/3)
                break
        theta0 = {0:0, 1:0, 2:0}
        if (time_no - i*3 + 1)%3 == 0: # turning
            # Get the pivot follower id (compare distance)
            dx1, dy1 = (self.wp_x[1][time_no]-self.wp_x[1][time_no+1]), (self.wp_y[1][time_no]-self.wp_y[1][time_no+1])
            dx2, dy2 = (self.wp_x[2][time_no]-self.wp_x[2][time_no+1]), (self.wp_y[2][time_no]-self.wp_y[2][time_no+1])
            if dx1**2 + dy1**2 < dx2**2 + dy2**2:
                pivot_id = 1
                theta0[2] = math.atan2((self.wp_y[pivot_id][time_no] - self.wp_y[2][time_no]),(self.wp_x[pivot_id][time_no] - self.wp_x[2][time_no])) + math.pi
            else: 
                pivot_id = 2
                theta0[1] = math.atan2((self.wp_y[pivot_id][time_no] - self.wp_y[1][time_no]),(self.wp_x[pivot_id][time_no] - self.wp_x[1][time_no])) + math.pi
            theta0[0] = math.atan2((self.wp_y[pivot_id][time_no] - self.wp_y[0][time_no]),(self.wp_x[pivot_id][time_no] - self.wp_x[0][time_no])) + math.pi
            dist = {0: self.Desired_dist, 1: self.Desired_dist_f2f, 2: self.Desired_dist_f2f, pivot_id: 0}            
        else:
            norm_x = (self.x[i+1] - self.x[i])/((self.x[i+1] - self.x[i])**2 + (self.y[i+1] - self.y[i])**2)**0.5
            norm_y = (self.y[i+1] - self.y[i])/((self.x[i+1] - self.x[i])**2 + (self.y[i+1] - self.y[i])**2)**0.5
        if (time_no - i*3 + 1)%3 == 0: # turning
            theta = theta0[id] + (self.w0*math.copysign(1,self.Ltang[i+1]))*(tc-self.time[time_no])
            vx = -dist[id]*(self.w0*math.copysign(1,self.Ltang[i+1]))*math.sin(theta)
            vy = dist[id]*(self.w0*math.copysign(1,self.Ltang[i+1]))*math.cos(theta)
            px = dist[id]*math.cos(theta) + self.wp_x[pivot_id][time_no]
            py = dist[id]*math.sin(theta) + self.wp_y[pivot_id][time_no]
            ratio = self.Desired_dist_f2f/self.Desired_dist
        else:
            if (time_no - i*3 + 1)%2 == 0: # decelerating
                acc = self.a
            else: acc = 0 # constant speed
            theta = math.atan2((self.wp_y[id][time_no+1] - self.wp_y[id][time_no]),(self.wp_x[id][time_no+1] - self.wp_x[id][time_no]))
            vx = (self.v0 + acc*(tc-self.time[time_no]))*norm_x
            vy = (self.v0 + acc*(tc-self.time[time_no]))*norm_y
            px = self.wp_x[id][time_no]+(self.v0*(tc-self.time[time_no])+0.5*acc*(tc-self.time[time_no])**2)*norm_x
            py = self.wp_y[id][time_no]+(self.v0*(tc-self.time[time_no])+0.5*acc*(tc-self.time[time_no])**2)*norm_y
            ratio = 1
        return [float(round(px, 3)), float(round(py, 3))], [float(round(vx, 3)), float(round(vy, 3))], float(round(theta, 3)), float(round(ratio, 3))

    def dec_speed(self, v0, a):
        t = (0-v0)/a
        s = v0*t + 0.5*a*(t**2)
        print('dec_speed', round(s, 3))
        return round(s, 3)

    def three_stage_pos(self, Desired_dist, Angle, x, y, v0, a):
        '''Three stages: go straight with constant speed, go straight decelerating, and turning'''

        # dist to wpt that shall slow down
        dec_v_dist = self.dec_speed(v0, a)
        
        # pivot point (add start and final just to balance length)
        pivot_id, pivot_x, pivot_y = [0], [x[0]], [y[0]]
        # total turning angle around pivot
        Ltang = [0]
        
        # Calculate angles relative to leader's position
        ang = {0: 0, 1: Angle*math.pi/180, 2: -Angle*math.pi/180}

        # add leader, follow1, follow2's first coordinate (relative to leader)
        des_hdg0 = math.atan2((y[1] - y[0]),(x[1] - x[0]))
        wp_x = {0: [x[0]], 1: [x[0]+Desired_dist*math.cos(math.pi+des_hdg0+ang[1])], 2: [x[0]+Desired_dist*math.cos(math.pi+des_hdg0+ang[2])]}
        wp_y = {0: [y[0]], 1: [y[0]+Desired_dist*math.sin(math.pi+des_hdg0+ang[1])], 2: [y[0]+Desired_dist*math.sin(math.pi+des_hdg0+ang[2])]}

        # calculate vehicles' coordinates
        for i in range(1, len(x)-1): # omit first and last point cuz no turning at these points
            xs, ys = symbols('xs ys')
            # y = mx + b 
            m1, m2 = (y[i] - y[i-1])/(x[i] - x[i-1]), (y[i+1] - y[i])/(x[i+1] - x[i]) # calculate slope
            hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
            # hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
            hdg2 = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))

            # Calculate follower1's wpts' path intersection
            x11, y11 = x[i] + Desired_dist*math.cos(math.pi+hdg1+ang[1]), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang[1])
            x12, y12 = x[i] + Desired_dist*math.cos(math.pi+hdg2+ang[1]), y[i] + Desired_dist*math.sin(math.pi+hdg2+ang[1])
            b1 = y11 - m1*x11
            b2 = y12 - m2*x12
            eq1 = Eq(ys-m1*xs-b1)
            eq2 = Eq(ys-m2*xs-b2)
            sol1 = solve((eq1,eq2), (xs, ys))

            # calculate follower2's wpts' path intersection
            x21, y21 = x[i] + Desired_dist*math.cos(math.pi+hdg1+ang[2]), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang[2])
            x22, y22 = x[i] + Desired_dist*math.cos(math.pi+hdg2+ang[2]), y[i] + Desired_dist*math.sin(math.pi+hdg2+ang[2])
            b1 = y21 - m1*x21
            b2 = y22 - m2*x22
            eq1 = Eq(ys-m1*xs-b1)
            eq2 = Eq(ys-m2*xs-b2)
            sol2 = solve((eq1,eq2), (xs, ys))

            # Follower with shorter path-to-travel is the pivot
            if (sol1[xs]-x11)**2+(sol1[ys]-y11)**2 < (sol2[xs]-x21)**2+(sol2[ys]-y21)**2: # turn right, F1 as pivot
                pivot_id.append(1)
                pivot_x.append(sol1[xs])
                pivot_y.append(sol1[ys])
            else: # turn left, F2 as pivot
                pivot_id.append(2)
                pivot_x.append(sol2[xs])
                pivot_y.append(sol2[ys])

            # leader's position before and after turning
            Lpx_bt = pivot_x[i] - Desired_dist*math.cos(math.pi+hdg1+ang[pivot_id[i]])
            Lpy_bt = pivot_y[i] - Desired_dist*math.sin(math.pi+hdg1+ang[pivot_id[i]])
            Lpx_at = pivot_x[i] - Desired_dist*math.cos(math.pi+hdg2+ang[pivot_id[i]])
            Lpy_at = pivot_y[i] - Desired_dist*math.sin(math.pi+hdg2+ang[pivot_id[i]])

            # Leader's (= Follower's) turning angle wrt pivot (1 as start, 2 as end, ang as totle ang)
            Lt1 = math.atan2((pivot_y[i] - Lpy_bt),(pivot_x[i] - Lpx_bt)) + math.pi
            Lt2 = math.atan2((pivot_y[i] - Lpy_at),(pivot_x[i] - Lpx_at)) + math.pi
            Ltang.append(math.copysign(1, pivot_id[i]-1.5)*min(abs(Lt2 - Lt1), abs(2*math.pi - abs(Lt2 - Lt1))))
            
            # add the leader and followers' locations of starting to slow down
            wp_x[0].append(round(Lpx_bt-dec_v_dist*math.cos(hdg1),2))
            wp_y[0].append(round(Lpy_bt-dec_v_dist*math.sin(hdg1),2))
            wp_x[1].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg1+ang[1]),2))
            wp_y[1].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg1+ang[1]),2))
            wp_x[2].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg1+ang[2]),2))
            wp_y[2].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg1+ang[2]),2))

            # add the leader and followers' locations before turning (shall be v = 0)
            wp_x[0].append(round(Lpx_bt,2))
            wp_y[0].append(round(Lpy_bt,2))
            wp_x[1].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg1+ang[1]),2))
            wp_y[1].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg1+ang[1]),2))
            wp_x[2].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg1+ang[2]),2))
            wp_y[2].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg1+ang[2]),2))

            # add the leader and followers' locations after turning (shall be v = 0)
            wp_x[0].append(round(Lpx_at,2))
            wp_y[0].append(round(Lpy_at,2))
            wp_x[1].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg2+ang[1]),2))
            wp_y[1].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg2+ang[1]),2))
            wp_x[2].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg2+ang[2]),2))
            wp_y[2].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg2+ang[2]),2))
        
        # add the decelerating position before ending mission
        wp_x[0].append(round(x[-1]-dec_v_dist*math.cos(hdg2),2))
        wp_y[0].append(round(y[-1]-dec_v_dist*math.sin(hdg2),2))
        wp_x[1].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg2+ang[1]),2))
        wp_y[1].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg2+ang[1]),2))
        wp_x[2].append(round(wp_x[0][-1]+Desired_dist*math.cos(math.pi+hdg2+ang[2]),2))
        wp_y[2].append(round(wp_y[0][-1]+Desired_dist*math.sin(math.pi+hdg2+ang[2]),2))

        # add on the last point
        wp_x[0].append(x[-1])
        wp_y[0].append(y[-1])
        wp_x[1].append(x[-1]+Desired_dist*math.cos(math.pi+hdg2+ang[1]))
        wp_y[1].append(y[-1]+Desired_dist*math.sin(math.pi+hdg2+ang[1]))
        wp_x[2].append(x[-1]+Desired_dist*math.cos(math.pi+hdg2+ang[2]))
        wp_y[2].append(y[-1]+Desired_dist*math.sin(math.pi+hdg2+ang[2]))

        return wp_x, wp_y, Ltang

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

# c = formation_plan(10, 60, [0, 1, 51, 50], [0, 50, 52, 2], 3, -1.5, 60)
# print(c.time)
# dt = 0.25
# t_lin = np.linspace(0, c.time[-1]-0.01, int((c.time[-1]-0.01)/dt))
# for t in t_lin:
#     pos, vel, theta, ratio = c.get_vel_pos(0, t)
#     print(type(pos[0]))

    # print(t, pos, vel, theta, ratio, type(pos[0]))

# tt = time.time()
# print(c.get_vel_pos(0, 29.95))
# print(time.time()-tt)
# pos, vel, theta, ratio = c.get_vel_pos(0, 29.95)
# print(pos, vel, theta, ratio)


