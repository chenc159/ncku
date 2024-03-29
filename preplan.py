from ast import Lt
from cmath import sqrt
import math
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm
from bezierpath_U import calc_4points_bezier_path
from sympy import symbols, Eq, solve
# import pymap3d as pm


# leader and followers turn with the same turning radius
def triangle_pi(Desired_dist, Radius, Angle, Waypt_num, x, y):
    
    # Calculate turning angles
    des_hdg = []
    for i in range(len(x)-1):
        hdg = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        des_hdg.extend([hdg,hdg])

    # add follow1 (wp_x1), follow2 (wp_x2) first coordinate (relative to leader)
    ang = Angle*math.pi/180
    wp_x1, wp_y1 = [x[0]+Desired_dist*math.cos(math.pi+des_hdg[0]+ang)], [y[0]+Desired_dist*math.sin(math.pi+des_hdg[0]+ang)]
    ang *= -1
    wp_x2, wp_y2 = [x[0]+Desired_dist*math.cos(math.pi+des_hdg[0]+ang)], [y[0]+Desired_dist*math.sin(math.pi+des_hdg[0]+ang)]
    
    # calculate follows' turning coordinate
    for i in range(1, len(x)-1): # omit first and last point cuz no turning at these points
        xs, ys = symbols('xs ys')
        # y = mx + b 
        m1, m2 = (y[i] - y[i-1])/(x[i] - x[i-1]), (y[i+1] - y[i])/(x[i+1] - x[i]) # calculate slope
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        # hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        hdg2 = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))

        # calculate follower 1's turning coordinate
        ang = Angle*math.pi/180
        x11, y11 = x[i] + Desired_dist*math.cos(math.pi+hdg1+ang), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang)
        x12, y12 = x[i] + Desired_dist*math.cos(math.pi+hdg2+ang), y[i] + Desired_dist*math.sin(math.pi+hdg2+ang)
        b1 = y11 - m1*x11
        b2 = y12 - m2*x12
        eq1 = Eq(ys-m1*xs-b1)
        eq2 = Eq(ys-m2*xs-b2)
        sol = solve((eq1,eq2), (xs, ys))
        wp_x1.append(sol[xs])
        wp_y1.append(sol[ys])

        # calculate follower 2's turning coordinate
        ang *= -1
        x21, y21 = x[i] + Desired_dist*math.cos(math.pi+hdg1+ang), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang)
        x22, y22 = x[i] + Desired_dist*math.cos(math.pi+hdg2+ang), y[i] + Desired_dist*math.sin(math.pi+hdg2+ang)
        b1 = y21 - m1*x21
        b2 = y22 - m2*x22
        eq1 = Eq(ys-m1*xs-b1)
        eq2 = Eq(ys-m2*xs-b2)
        sol = solve((eq1,eq2), (xs, ys))
        wp_x2.append(sol[xs])
        wp_y2.append(sol[ys])

    # add on the last point
    ang = Angle*math.pi/180
    wp_x1.append(x[-1]+Desired_dist*math.cos(math.pi+des_hdg[-1]+ang))
    wp_y1.append(y[-1]+Desired_dist*math.sin(math.pi+des_hdg[-1]+ang))
    ang *= -1
    wp_x2.append(x[-1]+Desired_dist*math.cos(math.pi+des_hdg[-1]+ang))
    wp_y2.append(y[-1]+Desired_dist*math.sin(math.pi+des_hdg[-1]+ang))

    # first point
    wp = [[x[0],y[0]]] # leader
    wp1 = [[wp_x1[0], wp_y1[0]]] # follower 1
    wp2 = [[wp_x2[0], wp_y2[0]]] # follower 2
    # get the turning radius coordinate
    for i in range(1, len(x)-1):
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        wp.append([round(x[i]-Radius*math.cos(hdg1),2), round(y[i]-Radius*math.sin(hdg1),2)])
        wp.append([round(x[i]-Radius*math.cos(hdg2),2), round(y[i]-Radius*math.sin(hdg2),2)])
        wp1.append([round(wp_x1[i]-Radius*math.cos(hdg1),2), round(wp_y1[i]-Radius*math.sin(hdg1),2)])
        wp1.append([round(wp_x1[i]-Radius*math.cos(hdg2),2), round(wp_y1[i]-Radius*math.sin(hdg2),2)])
        wp2.append([round(wp_x2[i]-Radius*math.cos(hdg1),2), round(wp_y2[i]-Radius*math.sin(hdg1),2)])
        wp2.append([round(wp_x2[i]-Radius*math.cos(hdg2),2), round(wp_y2[i]-Radius*math.sin(hdg2),2)])
    # add on the last point
    wp.append([x[-1], y[-1]])
    wp1.append([wp_x1[-1], wp_y1[-1]])
    wp2.append([wp_x2[-1], wp_y2[-1]])

    # Get the intermediate points via bezier method
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

    # dist = {0: [], 1: [], 2: []}
    # for i in range(len(x)-1 + (len(x)-2)):
    #     dis, dis1, dis2 = 0, 0, 0
    #     for j in range(8):
    #         dis += ((wp_x[i*9+j]-wp_x[i*9+j+1])**2 + (wp_y[i*9+j]-wp_y[i*9+j+1])**2)**0.5
    #         dis1 += ((wp_x1[i*9+j]-wp_x1[i*9+j+1])**2 + (wp_y1[i*9+j]-wp_y1[i*9+j+1])**2)**0.5
    #         dis2 += ((wp_x2[i*9+j]-wp_x2[i*9+j+1])**2 + (wp_y2[i*9+j]-wp_y2[i*9+j+1])**2)**0.5
    #     dist[0].append(dis)
    #     dist[1].append(dis1)    
    #     dist[2].append(dis2)
    # print(dist)

    
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


# leader and followers turn simultaneously
def triangle_s(Desired_dist, Radius, Angle, Waypt_num, x, y):
    
    # Calculate turning angles
    des_hdg = []
    for i in range(len(x)-1):
        hdg = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        des_hdg.extend([hdg,hdg])

    # add first coordinate (leader: wp_x, follow1: wp_x1, follow2: wp_x2)
    wp_x, wp_y = [x[0]], [y[0]] # leader
    ang = Angle*math.pi/180
    wp_x1, wp_y1 = [x[0]+Desired_dist*math.cos(math.pi+des_hdg[0]+ang)], [y[0]+Desired_dist*math.sin(math.pi+des_hdg[0]+ang)]
    ang *= -1
    wp_x2, wp_y2 = [x[0]+Desired_dist*math.cos(math.pi+des_hdg[0]+ang)], [y[0]+Desired_dist*math.sin(math.pi+des_hdg[0]+ang)]
    
    # get the turning radius coordinate
    for i in range(1, len(x)-1):
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        # leader
        wp_x.extend([round(x[i]-Radius*math.cos(hdg1),2), round(x[i]-Radius*math.cos(hdg2),2)])
        wp_y.extend([round(y[i]-Radius*math.sin(hdg1),2), round(y[i]-Radius*math.sin(hdg2),2)])
        
        hdg2 = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        # follower 1
        ang = Angle*math.pi/180
        wp_x1.extend([round(wp_x[-2]+Desired_dist*math.cos(math.pi+hdg1+ang)), round(wp_x[-1]+Desired_dist*math.cos(math.pi+hdg2+ang))])
        wp_y1.extend([round(wp_y[-2]+Desired_dist*math.sin(math.pi+hdg1+ang)), round(wp_y[-1]+Desired_dist*math.sin(math.pi+hdg2+ang))])
    
        # follower 2
        ang *= -1
        wp_x2.extend([wp_x[-2]+Desired_dist*math.cos(math.pi+hdg1+ang), wp_x[-1]+Desired_dist*math.cos(math.pi+hdg2+ang)])
        wp_y2.extend([wp_y[-2]+Desired_dist*math.sin(math.pi+hdg1+ang), wp_y[-1]+Desired_dist*math.sin(math.pi+hdg2+ang)])

    # add the last point
    wp_x.append(x[-1])
    wp_y.append(y[-1])
    ang = Angle*math.pi/180
    wp_x1.append(x[-1]+Desired_dist*math.cos(math.pi+des_hdg[-1]+ang))
    wp_y1.append(y[-1]+Desired_dist*math.sin(math.pi+des_hdg[-1]+ang))
    ang *= -1
    wp_x2.append(x[-1]+Desired_dist*math.cos(math.pi+des_hdg[-1]+ang))
    wp_y2.append(y[-1]+Desired_dist*math.sin(math.pi+des_hdg[-1]+ang))

    # Get the intermediate points via bezier method
    tempx, temp1x, temp2x = wp_x, wp_x1, wp_x2
    tempy, temp1y, temp2y = wp_y, wp_y1, wp_y2
    wp_x, wp_y = [wp_x[0]], [wp_y[0]]
    wp_x1, wp_y1 = [wp_x1[0]], [wp_y1[0]]
    wp_x2, wp_y2 = [wp_x2[0]], [wp_y2[0]]
    for i in range(len(tempx)-1):
        path, control_pt = calc_4points_bezier_path(tempx[i], tempy[i], des_hdg[i], tempx[i+1], tempy[i+1], des_hdg[i+1], 0.39, Waypt_num)
        wp_x.extend([round(a,2) for a,b in list(path[1:])])
        wp_y.extend([round(b,2) for a,b in list(path[1:])])
        path, control_pt = calc_4points_bezier_path(temp1x[i], temp1y[i], des_hdg[i], temp1x[i+1], temp1y[i+1], des_hdg[i+1], 0.39, Waypt_num)
        wp_x1.extend([round(a,2) for a,b in list(path[1:])])
        wp_y1.extend([round(b,2) for a,b in list(path[1:])])
        path, control_pt = calc_4points_bezier_path(temp2x[i], temp2y[i], des_hdg[i], temp2x[i+1], temp2y[i+1], des_hdg[i+1], 0.39, Waypt_num)
        wp_x2.extend([round(a,2) for a,b in list(path[1:])])
        wp_y2.extend([round(b,2) for a,b in list(path[1:])])
    
    # dist = {0: [], 1: [], 2: []}
    # for i in range(len(x)-1 + (len(x)-2)):
    #     dis, dis1, dis2 = 0, 0, 0
    #     for j in range(8):
    #         dis += ((wp_x[i*9+j]-wp_x[i*9+j+1])**2 + (wp_y[i*9+j]-wp_y[i*9+j+1])**2)**0.5
    #         dis1 += ((wp_x1[i*9+j]-wp_x1[i*9+j+1])**2 + (wp_y1[i*9+j]-wp_y1[i*9+j+1])**2)**0.5
    #         dis2 += ((wp_x2[i*9+j]-wp_x2[i*9+j+1])**2 + (wp_y2[i*9+j]-wp_y2[i*9+j+1])**2)**0.5
    #     dist[0].append(dis)
    #     dist[1].append(dis1)    
    #     dist[2].append(dis2)
    # print(dist)
    
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



# leader and followers turn with the inner follower as pivot
def triangle_c(Desired_dist, Radius, Angle, Waypt_num, x, y):

    # desired follower to follower distance
    Desired_dist_f2f = 2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist
    
    # pivot point (add start and final just to balance length)
    pivot_id, pivot_x, pivot_y = [0], [x[0]], [y[0]]
    
    # Calculate turning angles
    des_hdg = []
    for i in range(len(x)-1):
        hdg = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        des_hdg.extend([hdg,hdg])

    ang1 = Angle*math.pi/180
    ang2 = -Angle*math.pi/180

    # add follow1 (wp_x1), follow2 (wp_x2) first coordinate (relative to leader)
    wp_x1, wp_y1 = [x[0]+Desired_dist*math.cos(math.pi+des_hdg[0]+ang1)], [y[0]+Desired_dist*math.sin(math.pi+des_hdg[0]+ang1)]
    wp_x2, wp_y2 = [x[0]+Desired_dist*math.cos(math.pi+des_hdg[0]+ang2)], [y[0]+Desired_dist*math.sin(math.pi+des_hdg[0]+ang2)]
    
    # calculate follows' turning coordinate
    for i in range(1, len(x)-1): # omit first and last point cuz no turning at these points
        xs, ys = symbols('xs ys')
        # y = mx + b 
        m1, m2 = (y[i] - y[i-1])/(x[i] - x[i-1]), (y[i+1] - y[i])/(x[i+1] - x[i]) # calculate slope
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        # hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        hdg2 = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))

        # calculate follower 1's turning coordinate
        x11, y11 = x[i] + Desired_dist*math.cos(math.pi+hdg1+ang1), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang1)
        x12, y12 = x[i] + Desired_dist*math.cos(math.pi+hdg2+ang1), y[i] + Desired_dist*math.sin(math.pi+hdg2+ang1)
        b1 = y11 - m1*x11
        b2 = y12 - m2*x12
        eq1 = Eq(ys-m1*xs-b1)
        eq2 = Eq(ys-m2*xs-b2)
        sol = solve((eq1,eq2), (xs, ys))
        wp_x1.append(sol[xs])
        wp_y1.append(sol[ys])

        # calculate follower 2's turning coordinate
        x21, y21 = x[i] + Desired_dist*math.cos(math.pi+hdg1+ang2), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang2)
        x22, y22 = x[i] + Desired_dist*math.cos(math.pi+hdg2+ang2), y[i] + Desired_dist*math.sin(math.pi+hdg2+ang2)
        b1 = y21 - m1*x21
        b2 = y22 - m2*x22
        eq1 = Eq(ys-m1*xs-b1)
        eq2 = Eq(ys-m2*xs-b2)
        sol = solve((eq1,eq2), (xs, ys))
        wp_x2.append(sol[xs])
        wp_y2.append(sol[ys])

        # Follower with shorter path-to-travel is the pivot
        if (wp_x1[-1]-x11)**2+(wp_y1[-1]-y11)**2 < (wp_x2[-1]-x21)**2+(wp_y2[-1]-y21)**2: # turn right, F1 as pivot
            pivot_id.append(1)
            pivot_x.append(wp_x1[-1])
            pivot_y.append(wp_y1[-1])
        else: # turn left, F2 as pivot
            pivot_id.append(2)
            pivot_x.append(wp_x2[-1])
            pivot_y.append(wp_y2[-1])


    # add on the last point
    wp_x1.append(x[-1]+Desired_dist*math.cos(math.pi+des_hdg[-1]+ang1))
    wp_y1.append(y[-1]+Desired_dist*math.sin(math.pi+des_hdg[-1]+ang1))
    wp_x2.append(x[-1]+Desired_dist*math.cos(math.pi+des_hdg[-1]+ang2))
    wp_y2.append(y[-1]+Desired_dist*math.sin(math.pi+des_hdg[-1]+ang2))

    pivot_id.append(0)
    pivot_x.append(x[-1])
    pivot_y.append(y[-1])
    
    fig = plt.figure(1)
    norm = colors.Normalize(vmin=0, vmax=6)
    plt.plot(np.asarray(x), np.asarray(y), color = cm.hsv(norm(4)))
    # plt.plot(np.asarray(wp_x), np.asarray(wp_y), color = cm.hsv(norm(1)))
    plt.plot(np.asarray(wp_x1), np.asarray(wp_y1), color = cm.hsv(norm(2)))
    plt.plot(np.asarray(wp_x2), np.asarray(wp_y2), color = cm.hsv(norm(3)))
    # plt.scatter(np.asarray(wp_x), np.asarray(wp_y), color = cm.hsv(norm(1)))
    plt.scatter(np.asarray(wp_x1), np.asarray(wp_y1), color = cm.hsv(norm(2)))
    plt.scatter(np.asarray(wp_x2), np.asarray(wp_y2), color = cm.hsv(norm(3)))

    '''Draw the formation shape at waypoints'''
    # for i in range(1, len(x)):
    #     hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
    #     tri_x = np.asarray([x[i], x[i] + Desired_dist*math.cos(math.pi+hdg1+ang1), x[i] + Desired_dist*math.cos(math.pi+hdg1+ang2), x[i]])
    #     tri_y = np.asarray([y[i], y[i] + Desired_dist*math.sin(math.pi+hdg1+ang1), y[i] + Desired_dist*math.sin(math.pi+hdg1+ang2), y[i]])
    #     plt.plot(tri_x, tri_y, color = 'k')

    for i in range(1, len(x)-1):
        xs, ys = symbols('xs ys')
        m1, m2 = (y[i] - y[i-1])/(x[i] - x[i-1]), (y[i+1] - y[i])/(x[i+1] - x[i]) # calculate slope
        b1, b2 = y[i] - m1*x[i], y[i] - m2*x[i]
        hdg1 = math.atan2((y[i] - y[i-1]),(x[i] - x[i-1]))
        hdg2 = math.atan2((y[i] - y[i+1]),(x[i] - x[i+1]))
        hdg2f = math.atan2((y[i+1] - y[i]),(x[i+1] - x[i]))
        if pivot_id[i] == 1:  ang_t = ang1
        else: ang_t = ang2

        '''Draw the circle/turning path of leader and outer follower with inner follower as pivot'''
        deg = 0
        cir_step = 0.1
        cir_lx, cir_ly, cir_ofx, cir_ofy = [], [], [], []
        while deg <= 2*math.pi:
            cir_lx.append(Desired_dist*math.cos(deg) + pivot_x[i])
            cir_ly.append(Desired_dist*math.sin(deg) + pivot_y[i])
            cir_ofx.append(Desired_dist_f2f*math.cos(deg) + pivot_x[i])
            cir_ofy.append(Desired_dist_f2f*math.sin(deg) + pivot_y[i])
            deg += cir_step
        cir_lx.append(Desired_dist*math.cos(0) + pivot_x[i])
        cir_ly.append(Desired_dist*math.sin(0) + pivot_y[i])
        cir_ofx.append(Desired_dist_f2f*math.cos(0) + pivot_x[i])
        cir_ofy.append(Desired_dist_f2f*math.sin(0) + pivot_y[i])
        plt.plot(np.asarray(cir_lx), np.asarray(cir_ly), color = 'y')
        plt.plot(np.asarray(cir_ofx), np.asarray(cir_ofy), color = 'y')

        '''Draw the intersection points of the leader straight path and the circle with inner follower as pivot'''
        # eq1 = Eq((xs-pivot_x[i])**2+(ys-pivot_y[i])**2-Desired_dist**2)
        # eq2 = Eq(ys-m1*xs-b1)
        # sol = solve((eq1,eq2), (xs, ys))
        # intx = [sol[0][0], sol[1][0]]
        # inty = [sol[0][1], sol[1][1]]
        # plt.plot(np.asarray(intx), np.asarray(inty), color = cm.hsv(norm(2)))

        # eq2 = Eq(ys-m2*xs-b2)
        # sol = solve((eq1,eq2), (xs, ys))
        # intx = [sol[0][0], sol[1][0]]
        # inty = [sol[0][1], sol[1][1]]
        # plt.plot(np.asarray(intx), np.asarray(inty), color = cm.hsv(norm(2)))

        '''Based on the inner follower's position (pivot), draw the formation shape before and after turning'''
        Lpx_bt = pivot_x[i] - Desired_dist*math.cos(math.pi+hdg1+ang_t)
        Lpy_bt = pivot_y[i] - Desired_dist*math.sin(math.pi+hdg1+ang_t)
        tri_x = np.asarray([Lpx_bt, Lpx_bt + Desired_dist*math.cos(math.pi+hdg1+ang1), Lpx_bt + Desired_dist*math.cos(math.pi+hdg1+ang2), Lpx_bt])
        tri_y = np.asarray([Lpy_bt, Lpy_bt + Desired_dist*math.sin(math.pi+hdg1+ang1), Lpy_bt + Desired_dist*math.sin(math.pi+hdg1+ang2), Lpy_bt])
        plt.plot(tri_x, tri_y, color = 'k')

        Lpx_at = pivot_x[i] - Desired_dist*math.cos(math.pi+hdg2f+ang_t)
        Lpy_at = pivot_y[i] - Desired_dist*math.sin(math.pi+hdg2f+ang_t)
        tri_x = np.asarray([Lpx_at, Lpx_at + Desired_dist*math.cos(math.pi+hdg2f+ang1), Lpx_at + Desired_dist*math.cos(math.pi+hdg2f+ang2), Lpx_at])
        tri_y = np.asarray([Lpy_at, Lpy_at + Desired_dist*math.sin(math.pi+hdg2f+ang1), Lpy_at + Desired_dist*math.sin(math.pi+hdg2f+ang2), Lpy_at])
        plt.plot(tri_x, tri_y, color = 'k')

        '''Draw the leader and outer follower's turning path'''
        wpt_num = 4
        # Leader's turning angle wrt pivot (1 as start, 2 as end)
        Lt1 = math.atan2((pivot_y[i] - Lpy_bt),(pivot_x[i] - Lpx_bt)) + math.pi
        Lt2 = math.atan2((pivot_y[i] - Lpy_at),(pivot_x[i] - Lpx_at)) + math.pi
        # plt.scatter(Desired_dist*math.cos(Lt1) + pivot_x[i], Desired_dist*math.sin(Lt1) + pivot_y[i], color = 'g')
        # plt.scatter(Desired_dist*math.cos(Lt2) + pivot_x[i], Desired_dist*math.sin(Lt2) + pivot_y[i], color = 'm')

        # Outer follower's turning angle wrt pivot (1 as start, 2 as end)
        Ft1 = math.atan2((pivot_y[i] - (Lpy_bt+Desired_dist*math.sin(math.pi+hdg1-ang_t))),(pivot_x[i] - (Lpx_bt+Desired_dist*math.cos(math.pi+hdg1-ang_t)))) + math.pi
        Ft2 = math.atan2((pivot_y[i] - (Lpy_at+Desired_dist*math.sin(math.pi+hdg2f-ang_t))),(pivot_x[i] - (Lpx_at+Desired_dist*math.cos(math.pi+hdg2f-ang_t)))) + math.pi
        # plt.scatter(Desired_dist_f2f*math.cos(Ft1) + pivot_x[i], Desired_dist_f2f*math.sin(Ft1) + pivot_y[i], color = 'g')
        # plt.scatter(Desired_dist_f2f*math.cos(Ft2) + pivot_x[i], Desired_dist_f2f*math.sin(Ft2) + pivot_y[i], color = 'm')

        # At each waypt, leader and follower's angle (rad)
        Lang, Fang = Lt1, Ft1
        # Change of angle, neg if pivot_id=1, pos if pivot_id[i]=2 (dLang == dFang)
        dLang = math.copysign(1, pivot_id[i]-1.5)*min(abs(Lt2 - Lt1), abs(2*math.pi - abs(Lt2 - Lt1)))/wpt_num 
        dFang = math.copysign(1, pivot_id[i]-1.5)*min(abs(Ft2 - Ft1), abs(2*math.pi - abs(Ft2 - Ft1)))/wpt_num
        cir_lx, cir_ly, cir_ofx, cir_ofy = [], [], [], []
        for j in range(wpt_num):
            cir_lx.append(Desired_dist*math.cos(Lang) + pivot_x[i])
            cir_ly.append(Desired_dist*math.sin(Lang) + pivot_y[i])
            Lang += dLang

            cir_ofx.append(Desired_dist_f2f*math.cos(Fang) + pivot_x[i])
            cir_ofy.append(Desired_dist_f2f*math.sin(Fang) + pivot_y[i])
            Fang += dFang

        cir_lx.append(Desired_dist*math.cos(Lt2) + pivot_x[i])
        cir_ly.append(Desired_dist*math.sin(Lt2) + pivot_y[i])
        cir_ofx.append(Desired_dist_f2f*math.cos(Ft2) + pivot_x[i])
        cir_ofy.append(Desired_dist_f2f*math.sin(Ft2) + pivot_y[i])
        plt.plot(np.asarray(cir_lx), np.asarray(cir_ly), color = cm.hsv(norm(4)))
        plt.plot(np.asarray(cir_ofx), np.asarray(cir_ofy), color = cm.hsv(norm(4-pivot_id[i])))
        plt.scatter(np.asarray(cir_lx), np.asarray(cir_ly), color = cm.hsv(norm(4)))
        plt.scatter(np.asarray(cir_ofx), np.asarray(cir_ofy), color = cm.hsv(norm(4-pivot_id[i])))
        
        '''Draw the formation shape of every turning wpt'''
        # for j in range(1, len(cir_lx)-1):
        #     tri_x = np.asarray([cir_lx[j], pivot_x[i], cir_ofx[j], cir_lx[j]])
        #     tri_y = np.asarray([cir_ly[j], pivot_y[i], cir_ofy[j], cir_ly[j]])
        #     plt.plot(tri_x, tri_y, color = 'k')

        

    plt.grid()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    # plt.legend(['wpt','uav1', 'uav2', 'uav3'])
    plt.show()

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
    
    fig = plt.figure()
    norm = colors.Normalize(vmin=0, vmax=6)
    plt.plot(np.asarray(x), np.asarray(y), color = cm.hsv(norm(4)))
    # plt.plot(np.asarray(wp_x[0]), np.asarray(wp_y[0]), color = cm.hsv(norm(1)))
    # plt.plot(np.asarray(wp_x[1]), np.asarray(wp_y[1]), color = cm.hsv(norm(2)))
    # plt.plot(np.asarray(wp_x[2]), np.asarray(wp_y[2]), color = cm.hsv(norm(3)))
    plt.scatter(np.asarray(wp_x[0]), np.asarray(wp_y[0]), color = cm.hsv(norm(1)))
    plt.scatter(np.asarray(wp_x[1]), np.asarray(wp_y[1]), color = cm.hsv(norm(2)))
    plt.scatter(np.asarray(wp_x[2]), np.asarray(wp_y[2]), color = cm.hsv(norm(3)))
    plt.grid()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.show()

def t2pv():
    pass

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
    

    fig = plt.figure()
    norm = colors.Normalize(vmin=0, vmax=6)
    plt.plot(np.asarray(x), np.asarray(y), color = cm.hsv(norm(4)))
    plt.plot(np.asarray(px[0]), np.asarray(py[0]), color = cm.hsv(norm(1)))
    plt.plot(np.asarray(px[1]), np.asarray(py[1]), color = cm.hsv(norm(2)))
    plt.plot(np.asarray(px[2]), np.asarray(py[2]), color = cm.hsv(norm(3)))
    # plt.scatter(np.asarray(px[0]), np.asarray(py[0]), color = 'r') #cm.hsv(norm(1))
    # plt.scatter(np.asarray(px[1]), np.asarray(py[1]), color = cm.hsv(norm(2)))
    # plt.scatter(np.asarray(px[2]), np.asarray(py[2]), color = cm.hsv(norm(3)))
    # plt.plot(np.asarray(wp_x[0]), np.asarray(wp_y[0]), color = cm.hsv(norm(1)))
    # plt.plot(np.asarray(wp_x[1]), np.asarray(wp_y[1]), color = cm.hsv(norm(2)))
    # plt.plot(np.asarray(wp_x[2]), np.asarray(wp_y[2]), color = cm.hsv(norm(3)))
    plt.scatter(np.asarray(wp_x[0]), np.asarray(wp_y[0]), color = cm.hsv(norm(1)))
    plt.scatter(np.asarray(wp_x[1]), np.asarray(wp_y[1]), color = cm.hsv(norm(2)))
    plt.scatter(np.asarray(wp_x[2]), np.asarray(wp_y[2]), color = cm.hsv(norm(3)))
    plt.grid()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.show()

# triangle_pi(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 0])
# triangle_s(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 0])
# triangle_c(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 0])
# triangle_c(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 100])
# triangle_c(10, 10, 60, 10, [0, 1, 51, 45, -30, -5, 60], [20, 70, 60, 0, 3, -50, -40])
# triangle_c(10, 10, 60, 10, [0, 1, 51, 40, 60, 100, 80, 120], [0, 50, 52, 2, -50, -45, 40, 35])
# dec_speed(3, -1.8) # 2.5 m
# three_stage_pos(10, 60, [0, 1, 51, 40, 60, 100, 80, 120], [0, 50, 52, 2, -50, -45, 40, 35], 3, -1.5)
speed_time(10, 60, [0, 1, 51, 40, 60, 100, 80, 120], [0, 50, 52, 2, -50, -45, 40, 35], 3, -1.5, 30)
# speed_time(10, 60, [0, 1, 51, 40], [0, 50, 52, 2], 3, -1.5, 60)
