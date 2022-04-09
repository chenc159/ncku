import math
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm

r, p = 0, 0
F, m, d = 6, 2, 5
dt, tm = 0.1, 20
kp1p, kp1r =0.2, 8
kp2p, kp2r, kp2dr, kp2dp = 1.4, 8, 5, 7
kp3p, kp3r, kp3dr, kp3dp = 1.4, 8, 10.5, 5.5
theta = 0
y1, y2, y3 = 0, 0, 0

# Initizlize uav initial position and velocity
uav1Lp = [0, 0]
uav1Lv = [0, 0]
uav2Fp = [2.5 , (d*math.cos(150*math.pi/180)-5) ] 
uav3Fp = [-2.5 , (d*math.cos(150*math.pi/180)-5) ]  
uav2Fv = [0, 0]
uav3Fv = [0, 0]
uav2dpv = [0, 2]
uav3dpv = [0, 2]
# # waypoints
# wp = [[0, 0], [0, 40], [40, 40], [40, 0]]

# # Sizing
# s1 = np.zeros((round(tm/dt),1))
# s2 = np.zeros((round(tm/dt),2))
# r1t, p1t, y1t = s1, s1, s1
# r2t, p2t, y2t = s1, s1, s1
# r3t, p3t, y3t = s1, s1, s1
# a1Lt, a2Ft, a3Ft = s2, s2, s2
# uav1Lvt, uav1Lpt = s2, s2
# uav2dpt, uav2Fvt, uav2Fpt = s2, s2, s2 
# uav3dpt, uav3Fvt, uav3Fpt = s2, s2, s2

p1x, p2x, p3x = [uav1Lp[0]], [uav2Fp[0]], [uav3Fp[0]]
p1y, p2y, p3y = [uav1Lp[1]], [uav2Fp[1]], [uav3Fp[1]]

t, t_list = 0, [0]
start_time = time.time()
while t<tm:
    if uav1Lp[0]==0 and uav1Lp[1]>=0 and uav1Lp[1]<30:
        # 位置誤差判斷長機與目標點
        r1 = 0.01745*(-kp1r*(0-uav1Lp[0]))
        p1= 0.01745*(-kp1p*(30-uav1Lp[1]))
        a1L= [  -F/m*(math.cos(r1)*math.sin(p1)*math.sin(y1)+math.sin(r1)*math.cos(y1)),
                -F/m*(math.cos(r1)*math.sin(p1)*math.cos(y1)+math.sin(r1)*math.sin(y1))]
        uav1Lv[0] += a1L[0]*dt
        uav1Lv[1] += a1L[1]*dt
        uav1Lp[0] += uav1Lv[0]*dt + 0.5*a1L[0]*dt*dt
        uav1Lp[1] += uav1Lv[1]*dt + 0.5*a1L[1]*dt*dt
        uav2dp = [uav1Lp[0]+d*math.sin(0.01745*(150+theta)), uav1Lp[1]+d*math.cos(0.01745*(150+theta))]
        uav3dp = [uav1Lp[0]+d*math.sin(0.01745*(210+theta)), uav1Lp[1]+d*math.cos(0.01745*(210+theta))]
        if uav1Lv[1] > 2:
            uav1Lv[1] = 2
            # a1L = [0,0]
        
        # 位置誤差判斷黃點與僚機
        r2 = 0.01745*(-kp2r*(uav2dp[0]-uav2Fp[0]))
        p2 = 0.01745*(-kp2p*(uav2dp[1]-uav2Fp[1]))
        a2F= [  -F/m*(math.cos(r2)*math.sin(p2)*math.sin(y2)+math.sin(r2)*math.cos(y2)),
                -F/m*(math.cos(r2)*math.sin(p2)*math.cos(y2)+math.sin(r2)*math.sin(y2))]
        uav2Fv[0] += a2F[0]*dt
        uav2Fv[1] += a2F[1]*dt
        uav2Fp[0] += uav2Fv[0]*dt + 0.5*a2F[0]*dt*dt
        uav2Fp[1] += uav2Fv[1]*dt + 0.5*a2F[1]*dt*dt
        
        r3 = 0.01745*(-kp3r*(uav3dp[0]-uav3Fp[0]))
        p3 = 0.01745*(-kp3p*(uav3dp[1]-uav3Fp[1]))
        a3F= [  -F/m*(math.cos(r3)*math.sin(p3)*math.sin(y3)+math.sin(r3)*math.cos(y3)),
                -F/m*(math.cos(r3)*math.sin(p3)*math.cos(y3)+math.sin(r3)*math.sin(y3))]
        uav3Fv[0] += a3F[0]*dt
        uav3Fv[1] += a3F[1]*dt
        uav3Fp[0] += uav3Fv[0]*dt + 0.5*a3F[0]*dt*dt
        uav3Fp[1] += uav3Fv[1]*dt + 0.5*a3F[1]*dt*dt

        if uav2Fv[1] > 4:
            uav2Fv[1] = 4
            # a2F = [0,0]
        if uav3Fv[1] > 4:
            uav3Fv[1] = 4
            # a3F = [0,0]
        if uav2dp[1]-uav2Fp[1] < 0.05 or uav3dp[1]-uav3Fp[1] < 0.05:
            uav2Fv[1] = uav1Lv[1]
            uav3Fv[1] = uav1Lv[1]
        
        t += dt

        p1x.append(uav1Lp[0])
        p2x.append(uav2Fp[0])
        p3x.append(uav3Fp[0])
        p1y.append(uav1Lp[1])
        p2y.append(uav2Fp[1])
        p3y.append(uav3Fp[1])
        t_list.append(t)
    else:
        break

print('Totoal time: ', t)
print('Excution time: ', time.time()-start_time)

# Plotting
fig = plt.figure(1)
norm = colors.Normalize(vmin=0, vmax=4)
plt.plot(p1x, p1y, color = cm.hsv(norm(1)))
plt.plot(p2x, p2y, color = cm.hsv(norm(2)))
plt.plot(p3x, p3y, color = cm.hsv(norm(3)))
plt.grid()
plt.xlabel('X')
plt.ylabel('Y')
plt.legend(['uav1', 'uav2', 'uav3'])
plt.show()

fig = plt.figure(2)
norm = colors.Normalize(vmin=0, vmax=4)
plt.plot(t_list, p1y, color = cm.hsv(norm(1)))
plt.plot(t_list, p2y, color = cm.hsv(norm(2)))
plt.plot(t_list, p3y, color = cm.hsv(norm(3)))
plt.grid()
plt.xlabel('T')
plt.ylabel('Y')
plt.legend(['uav1', 'uav2', 'uav3'])
plt.show()