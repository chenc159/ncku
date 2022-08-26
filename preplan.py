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

        if hdg2 < hdg1: # turn right, F1 as pivot
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
            cir_ofx.append(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.cos(deg) + pivot_x[i])
            cir_ofy.append(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.sin(deg) + pivot_y[i])
            deg += cir_step
        cir_lx.append(Desired_dist*math.cos(0) + pivot_x[i])
        cir_ly.append(Desired_dist*math.sin(0) + pivot_y[i])
        cir_ofx.append(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.cos(0) + pivot_x[i])
        cir_ofy.append(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.sin(0) + pivot_y[i])
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
        wpt_num = 8
        # Leader's turning angle wrt pivot (1 as start, 2 as end)
        Lt1 = math.atan2((pivot_y[i] - Lpy_bt),(pivot_x[i] - Lpx_bt)) + math.pi
        Lt2 = math.atan2((pivot_y[i] - Lpy_at),(pivot_x[i] - Lpx_at)) + math.pi
        if Lt1 > math.pi:
            Lt1 -= 2*math.pi
        elif Lt1 < -math.pi:
            Lt1 += 2*math.pi
        if Lt2 > math.pi:
            Lt2 -= 2*math.pi
        elif Lt2 < -math.pi:
            Lt2 += 2*math.pi
        # plt.scatter(Desired_dist*math.cos(Lt1) + pivot_x[i], Desired_dist*math.sin(Lt1) + pivot_y[i], color = 'g')
        # plt.scatter(Desired_dist*math.cos(Lt2) + pivot_x[i], Desired_dist*math.sin(Lt2) + pivot_y[i], color = 'm')

        # Outer follower's turning angle wrt pivot (1 as start, 2 as end)
        Ft1 = math.atan2((pivot_y[i] - (Lpy_bt+Desired_dist*math.sin(math.pi+hdg1-ang_t))),(pivot_x[i] - (Lpx_bt+Desired_dist*math.cos(math.pi+hdg1-ang_t)))) + math.pi
        Ft2 = math.atan2((pivot_y[i] - (Lpy_at+Desired_dist*math.sin(math.pi+hdg2f-ang_t))),(pivot_x[i] - (Lpx_at+Desired_dist*math.cos(math.pi+hdg2f-ang_t)))) + math.pi
        if Ft1 > math.pi:
            Ft1 -= 2*math.pi
        elif Ft1 < -math.pi:
            Ft1 += 2*math.pi
        if Ft2 > math.pi:
            Ft2 -= 2*math.pi
        elif Ft2 < -math.pi:
            Ft2 += 2*math.pi
        # plt.scatter(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.cos(Ft1) + pivot_x[i], 2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.sin(Ft1) + pivot_y[i], color = 'g')
        # plt.scatter(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.cos(Ft2) + pivot_x[i], 2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.sin(Ft2) + pivot_y[i], color = 'm')

        Ldeg, Fdeg = Lt1, Ft1
        cir_lx, cir_ly, cir_ofx, cir_ofy = [], [], [], []
        for j in range(wpt_num):

            cir_lx.append(Desired_dist*math.cos(Ldeg) + pivot_x[i])
            cir_ly.append(Desired_dist*math.sin(Ldeg) + pivot_y[i])
            Ldeg += (Lt2 - Lt1)/wpt_num

            cir_ofx.append(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.cos(Fdeg) + pivot_x[i])
            cir_ofy.append(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.sin(Fdeg) + pivot_y[i])
            Fdeg += (Ft2 - Ft1)/wpt_num

        cir_lx.append(Desired_dist*math.cos(Lt2) + pivot_x[i])
        cir_ly.append(Desired_dist*math.sin(Lt2) + pivot_y[i])
        cir_ofx.append(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.cos(Ft2) + pivot_x[i])
        cir_ofy.append(2*math.sin(Angle*0.01745)/math.sin(90*0.01745)*Desired_dist*math.sin(Ft2) + pivot_y[i])
        plt.plot(np.asarray(cir_lx), np.asarray(cir_ly), color = cm.hsv(norm(4)))
        plt.plot(np.asarray(cir_ofx), np.asarray(cir_ofy), color = cm.hsv(norm(4-pivot_id[i])))
        plt.scatter(np.asarray(cir_lx), np.asarray(cir_ly), color = cm.hsv(norm(4)))
        plt.scatter(np.asarray(cir_ofx), np.asarray(cir_ofy), color = cm.hsv(norm(4-pivot_id[i])))

        

    plt.grid()
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    # plt.legend(['wpt','uav1', 'uav2', 'uav3'])
    plt.show()

# triangle_pi(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 0])
# triangle_s(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 0])
# triangle_c(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 0])
# triangle_c(10, 10, 60, 10, [0, 1, 51, 45, -40], [0, 50, 60, 100, 190])
triangle_c(10, 10, 60, 10, [0, 1, 51, 45], [0, 50, 60, 100])



