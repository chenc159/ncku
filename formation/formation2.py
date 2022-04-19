
import math
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm

r, p = 0, 0
F, m, d = 6, 2, 5
dt, tm = 0.1, 120
kp_p = [0.2, 1.4, 1.4]
kp_r = [8, 8, 8]
ka, kv = 10, 1
theta = 0
y = [0,0,0]

# Initizlize uav initial position and velocity
n = 3 # total of 3 uavs
wp = [[0, 0], [0, 40], [40, 40], [40, 0]] # waypoints (desired location for uav1/leader)
wp_id = 0 # wp id the uav1/leader is heading to 
uavs_p = [-1, -1, -5, 0, -4, 0] # initial uav1_px, uav1_py, uav2_px, uav2_py, ...
uavs_v = [0, 0, 0, 0, 0, 0] # initial uav1_vx, uav1_vy, uav2_vx, uav2_vy, ...
uavs_vmax = [2, 4, 4] # maximum allowable velocity of uavs
uavs_amax = [5, 5, 5] # maximum allowable acceleration of uavs
rs = np.zeros((3)) # roll of uavs
ps = np.zeros((3)) # pitch of uavs
uavs_a = np.zeros((6)) # accel of uavs

heading = math.atan2((wp[wp_id][1] - uavs_p[1]),(wp[wp_id][0] - uavs_p[0])) # angle of where the leader will head
ang = 30*math.pi/180 # angles of the points of the equilateral triangle

# desired position of uavs
des_ps = [uavs_p[0], uavs_p[1]] # wait of followers to be in their ready position
des_ps.extend([(uavs_p[0]+d*math.cos(math.pi-ang+heading)), (uavs_p[1]+d*math.sin(math.pi-ang+heading))]) # desired pposition for uav2/follower1
des_ps.extend([(uavs_p[0]+d*math.cos(math.pi+ang+heading)), (uavs_p[1]+d*math.sin(math.pi+ang+heading))]) # desired pposition for uav3/follower2

got_init = False # if all uavs have gotten to their desired initial location


p1x, p2x, p3x = [uavs_p[0]], [uavs_p[2]], [uavs_p[4]]
p1y, p2y, p3y = [uavs_p[1]], [uavs_p[3]], [uavs_p[5]]
d1x, d2x, d3x = [des_ps[0]], [des_ps[2]], [des_ps[4]]
d1y, d2y, d3y = [des_ps[1]], [des_ps[3]], [des_ps[5]]

t, t_list = 0, [0]
start_time = time.time()
while t<tm:
    if not got_init:
        got_init = True
        for i in range(3):
            if ((uavs_p[2*i]-des_ps[2*i])**2 + (uavs_p[2*i+1]-des_ps[2*i+1])**2 > 0.05**2):
                got_init = False
                ka = 0.1
    else:
        # print('here')
        # break
        if ((wp[wp_id][1] - uavs_p[1])**2 + (wp[wp_id][0] - uavs_p[0])**2) <= 0.01:
            wp_id += 1
            if wp_id == len(wp):
                break
        ka = 10
        heading = math.atan2((wp[wp_id][1] - uavs_p[1]),(wp[wp_id][0] - uavs_p[0])) # angle of where the leader will head
        des_ps[:2] = wp[wp_id]
        des_ps[2:4] = [(uavs_p[0]+d*math.cos(math.pi-ang+heading)), (uavs_p[1]+d*math.sin(math.pi-ang+heading))]
        des_ps[4:] =  [(uavs_p[0]+d*math.cos(math.pi+ang+heading)), (uavs_p[1]+d*math.sin(math.pi+ang+heading))]
        
        d1x.append(des_ps[0])
        d1y.append(des_ps[1])
        d2x.append(des_ps[2])
        d2y.append(des_ps[3])
        d3x.append(des_ps[4])
        d3y.append(des_ps[5])
    for i in range(3):
        ''' Shall add path planning collision avoidance .... '''
        # rs[i] = 0.01745*(-kp_r[i]*(des_ps[2*i]-uavs_p[2*i]))
        # ps[i] = 0.01745*(-kp_p[i]*(des_ps[2*i+1]-uavs_p[2*i+1]))
        uavs_a[2*i] = ka*float(des_ps[2*i]-uavs_p[2*i]) - kv*uavs_v[2*i] #float(-F/m*(math.cos(rs[i])*math.sin(ps[i])*math.sin(y[i])+math.sin(rs[i])*math.cos(y[i])))
        uavs_a[2*i+1] = ka*float(des_ps[2*i+1]-uavs_p[2*i+1]) - kv*uavs_v[2*i+1] #float(-F/m*(math.cos(rs[i])*math.sin(ps[i])*math.cos(y[i])+math.sin(rs[i])*math.sin(y[i])))
        uavs_v[2*i] += float(uavs_a[2*i]*dt)
        uavs_v[2*i+1] += float(uavs_a[2*i+1]*dt)

        if uavs_v[2*i]**2 + uavs_v[2*i+1]**2 > uavs_vmax[i]**2:
            temp = (uavs_v[2*i]**2 + uavs_v[2*i+1]**2)**0.5
            ovx, ovy = uavs_v[2*i], uavs_v[2*i+1]
            uavs_v[2*i] *= uavs_vmax[i]/temp
            uavs_v[2*i+1] *= uavs_vmax[i]/temp

        if got_init and (i!=0) and ((uavs_p[2*i]-des_ps[2*i])**2 + (uavs_p[2*i+1]-des_ps[2*i+1])**2 < 0.05**2):
            uavs_v[2*i], uavs_v[2*i+1] = uavs_v[0], uavs_v[1]
        uavs_p[2*i] += float(uavs_v[2*i]*dt + 0.5*uavs_a[2*i]*dt*dt)
        uavs_p[2*i+1] += float(uavs_v[2*i+1]*dt + 0.5*uavs_a[2*i+1]*dt*dt)    

    t += dt
    t = round(t,2)

    p1x.append(uavs_p[0])
    p1y.append(uavs_p[1])
    p2x.append(uavs_p[2])
    p2y.append(uavs_p[3])
    p3x.append(uavs_p[4])
    p3y.append(uavs_p[5])
    t_list.append(t)


print('Totoal time: ', t)
print('Excution time: ', time.time()-start_time)

# Plotting
fig = plt.figure(1)
norm = colors.Normalize(vmin=0, vmax=4)
plt.plot(p1x, p1y, color = cm.hsv(norm(1)))
plt.plot(p2x, p2y, color = cm.hsv(norm(2)))
plt.plot(p3x, p3y, color = cm.hsv(norm(3)))
plt.scatter(d1x, d1y, color = cm.hsv(norm(1)))
plt.scatter(d2x, d2y, color = cm.hsv(norm(2)))
plt.scatter(d3x, d3y, color = cm.hsv(norm(3)))
plt.grid()
plt.xlabel('X')
plt.ylabel('Y')
plt.legend(['uav1', 'uav2', 'uav3'])
plt.show()

fig = plt.figure(2)
norm = colors.Normalize(vmin=0, vmax=4)
plt.subplot(1,2,1)
plt.plot(t_list, p1x, color = cm.hsv(norm(1)))
plt.plot(t_list, p2x, color = cm.hsv(norm(2)))
plt.plot(t_list, p3x, color = cm.hsv(norm(3)))
plt.grid()
plt.xlabel('T')
plt.ylabel('X')
plt.legend(['uav1', 'uav2', 'uav3'])
plt.subplot(1,2,2)
plt.plot(t_list, p1y, color = cm.hsv(norm(1)))
plt.plot(t_list, p2y, color = cm.hsv(norm(2)))
plt.plot(t_list, p3y, color = cm.hsv(norm(3)))
plt.grid()
plt.xlabel('T')
plt.ylabel('Y')
plt.legend(['uav1', 'uav2', 'uav3'])
plt.show()