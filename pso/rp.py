import math
# import random
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm
import copy

def plots():
    # Plotting
    fig = plt.figure()
    norm = colors.Normalize(vmin=0, vmax=n)
    led =[]
    for j in range(n):
        plt.scatter(uavs[j].des_pos[0], uavs[j].des_pos[1], s=400, color = cm.hsv(norm(j)), marker="*")
        plt.plot(uavs[j].x_list, uavs[j].y_list, color = cm.hsv(norm(j)))
        plt.scatter(uavs[j].x_list, uavs[j].y_list, s=40, color = cm.hsv(norm(j)), marker=".")
        led.append('uav'+str(j))
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.legend(led)
    plt.grid()
    plt.show()


class uav():
    def __init__(self, init, des, theta, v0, r):
        self.ini_pos = copy.deepcopy(init)
        self.cur_pos = copy.deepcopy(init)
        self.des_pos = des
        self.nxt_pos = np.asarray([0,0])
        self.cur_vel = np.asarray([v0, v0])
        self.nxt_vel = np.asarray([v0, v0])
        self.cur_acc = np.asarray([0.0, 0.0])
        self.nxt_acc = np.asarray([0.0, 0.0])
        self.r = r
        self.x_list = [init[0]]
        self.y_list = [init[1]]

    def update_nxt(self, ang):
        # print(ang)
        self.nxt_acc[0] =  9.81*ang[0]
        self.nxt_acc[1] =  -9.81*ang[1]
        self.nxt_vel[0] = self.cur_vel[0] + self.nxt_acc[0]
        self.nxt_vel[1] = self.cur_vel[1] + self.nxt_acc[1]
        if (self.nxt_vel[0]**2 + self.nxt_vel[1]**2)**0.5 > MaxVel:
            self.nxt_vel[0] = MaxVel*self.nxt_vel[0]/(self.nxt_vel[0]**2+self.nxt_vel[1]**2)**0.5
            self.nxt_vel[1] = MaxVel*self.nxt_vel[1]/(self.nxt_vel[0]**2+self.nxt_vel[1]**2)**0.5
        self.nxt_pos = self.cur_pos + self.nxt_vel
        
    def update_cur(self, ang):
        self.cur_acc[0] = 9.81*ang[0]
        self.cur_acc[1] = -9.81*ang[1]
        self.cur_vel[0] = self.cur_vel[0] + self.cur_acc[0]
        self.cur_vel[1] = self.cur_vel[1] + self.cur_acc[1]
        if (self.cur_vel[0]**2+self.cur_vel[1]**2)**0.5 > MaxVel:
            self.cur_vel[0] = MaxVel*self.cur_vel[0]/(self.cur_vel[0]**2+self.cur_vel[1]**2)**0.5
            self.cur_vel[1] = MaxVel*self.cur_vel[1]/(self.cur_vel[0]**2+self.cur_vel[1]**2)**0.5
        self.cur_pos += self.cur_vel
        self.x_list.extend([self.cur_pos[0]])
        self.y_list.extend([self.cur_pos[1]])

def finished():
    for i in range(n):
        x,y = uavs[i].cur_pos[0], uavs[i].cur_pos[1]
        dx,dy = uavs[i].des_pos[0], uavs[i].des_pos[1]
        if ((x-dx)**2+(y-dy)**2 > 5**2):
            return False
    return True

class particle():
    def __init__(self):
        # # self.position = np.random.uniform(VarMin, VarMax, (nPop, nVar, nCom))
        self.position = [] 
        for i in range(nPop):
            self.position.extend([np.random.uniform(VarMin, VarMax, (nVar, nCom))])
        self.position = np.asarray(self.position)
        self.velocity = np.zeros((nPop, nVar, nCom))
        self.cost = np.ones((nPop))
        self.best_position = copy.deepcopy(self.position)
        self.best_cost = np.ones((nPop))
        self.globalbest_cost = math.inf
        self.globalbest_pos = np.zeros((nVar, nCom))
    
def ObjectiveFunction():

    F1, F2, F3, F4 = 0, 0, 0, 0
    for i in range(n):
        xi, yi = uavs[i].ini_pos[0], uavs[i].ini_pos[1]
        xc, yc = uavs[i].cur_pos[0], uavs[i].cur_pos[1]
        xg, yg = uavs[i].des_pos[0], uavs[i].des_pos[1]
        xn, yn = uavs[i].nxt_pos[0], uavs[i].nxt_pos[1]
        # F1 += ((xc-xn)**2 + (yc-yn)**2) + ((xn-xg)**2 + (yn-yg)**2)
        # F1 += ((xc-xn)**2 + (yc-yn)**2)**(0.5) + ((xn-xg)**2 + (yn-yg)**2)**(0.5)
        F1 += ((xn-xg)**2 + (yn-yg)**2)**(0.5)
        # print(F1)
        
        xnc, ync, xng, yng, xcg, ycg = xn-xc, yn-yc, xn-xg, yn-yg, xc-xg, yc-yg
        pnc, png, pcg = (xnc**2+ync**2), (xng**2+yng**2), (xcg**2+ycg**2)
        # if pnc*png!=0.0:
        #     if (pnc+png-pcg)/(2*pnc**0.5*png**0.5) <= 1 and (pnc+png-pcg)/(2*pnc**0.5*png**0.5) >= -1:
        #         F3 += math.pi - math.acos((pnc+png-pcg)/(2*(pnc**0.5)*(png**0.5)))
        if ((xc-xi)**2+(yc-yi)**2) >= ((xn-xi)**2+(yn-yi)**2): # going backward
            F4 += 1
            # print(F4)
        f3_bool = False
        for j in range(n):
            if i!=j:
                xn2, yn2 = uavs[j].nxt_pos[0], uavs[j].nxt_pos[1]
                if ((xn-xn2)**2 + (yn-yn2)**2)**(0.5) <= 1.0*uavs[i].r:
                    F2 += 1e5
                else:
                    F2 += 1.0/abs(((xn-xn2)**2 + (yn-yn2)**2)**(0.5) - 1.0*uavs[i].r)
                if ((xn-xn2)**2 + (yn-yn2)**2)**(0.5) <= 3.0*uavs[i].r:
                    f3_bool = True
        if f3_bool and (pnc+png-pcg)/(2*pnc**0.5*png**0.5) <= 1 and (pnc+png-pcg)/(2*pnc**0.5*png**0.5) >= -1:
            F3 += math.pi - math.acos((pnc+png-pcg)/(2*(pnc**0.5)*(png**0.5))) # cos law for smoothness detection

    return 1.0*F1 + 1.0*F2 + 0.0*F3 + 150.0*F4


def pso(ptcle):
    w = 1
    for j in range(n):
        a = uavs[j].des_pos[0] - uavs[j].cur_pos[0]
        b = uavs[j].des_pos[1] - uavs[j].cur_pos[1]
        ptcle.position[0,j,:] = [a, -b]
        ptcle.best_position[0,j,:] = [a, -b]
        # print(ptcle.position[0,j,:])
    for i in range(nPop):
        for j in range(n):
            uavs[j].update_nxt(ptcle.position[i,j,:])
        ptcle.cost[i] = ObjectiveFunction()
        ptcle.best_cost[i] = ptcle.cost[i]

        if ptcle.best_cost[i] < ptcle.globalbest_cost:
            ptcle.globalbest_cost = ptcle.best_cost[i]
            ptcle.globalbest_pos = ptcle.best_position[i,:,:]
            # print(ptcle.globalbest_cost)
    
    # PSO main loop
    for it in range(MaxIt):
        for i in range(nPop):
            for j in range(n):
                for k in range(nCom):
                    # update velocity
                    ptcle.velocity[i,j,k] = (w*ptcle.velocity[i,j,k] +
                                        c1*np.random.random()*(ptcle.best_position[i,j,k] - ptcle.position[i,j,k]) +
                                        c2*np.random.random()*(ptcle.globalbest_pos[j,k] - ptcle.position[i,j,k]))
                    # apply velocity limits
                    ptcle.velocity[i,j,k] = min(max(ptcle.velocity[i,j,k], VelMin), VelMax)
                    # update position
                    ptcle.position[i,j,k] += ptcle.velocity[i,j,k]
                    # velocity mirror effect
                    if ptcle.position[i,j,k] < VarMin or ptcle.position[i,j,k] > VarMax:
                        ptcle.velocity[i,j,k] *= -1
                    # apply position limits
                    ptcle.position[i,j] = min(max(ptcle.position[i,j,k], VarMin), VarMax)
                uavs[j].update_nxt(ptcle.position[i,j,:])
            ptcle.cost[i] = ObjectiveFunction()
            # update personal best
            if ptcle.cost[i] < ptcle.best_cost[i]:
                ptcle.best_position[i,:,:] = ptcle.position[i,:,:]
                ptcle.best_cost[i] = ptcle.cost[i]
                # update global best
                if ptcle.best_cost[i] < ptcle.globalbest_cost:
                    ptcle.globalbest_cost = ptcle.best_cost[i]
                    ptcle.globalbest_pos = ptcle.best_position[i,:,:]
                    # print('a',ptcle.globalbest_cost) #, ptcle.best_position[i,:,:])

        w *= wdamp
                    


if __name__ == '__main__':

    # random.seed(3)
    np.random.seed(3)

    # initialize
    rng = 30.0
    init_x = [0.0, 0.0]
    init_y = [-rng, rng]
    des_x = [0.0, 0.0]
    des_y = [rng, -rng]
    # init_x = [0, 30, 30, 0]
    # init_y = [0, 0, 30, 30]
    # des_x = [30, 0, 0, 30]
    # des_y = [30, 30, 0, 0]
    n = len(init_x) # number of uav
    MaxAcc = 8.0
    MaxVel = 5.0

    # Problem Definition
    nVar = n                # Number of Decision Variables
    nCom = 2                # Number of Components (roll, pitch)
    # VarSize = [1 nVar]      # Size of Decision Variables Matrix
    # VarMax = 1.0*math.pi      # Upper Bound of Variables, 2*math.pi
    # VarMin = -VarMax              # Lower Bound of Variables, 0
    VarMax = math.pi/6.0      
    VarMin = -VarMax     

    # PSO parameters
    MaxIt = 100       # Maximum Number of Iterations
    nPop = 100        # Population Size (Swarm Size)
    w = 1             # Inertia Weight
    wdamp = 0.99      # Inertia Weight Damping Ratio
    c1 = 1.5          # Personal Learning Coefficient
    c2 = 2.0          # Global Learning Coefficient

    # Velocity Limits
    VelMax = 0.5*(VarMax-VarMin)
    VelMin = -VelMax

    # 速度 & 安全距離設定 velocity and safty range
    v0, r = 0.0, 5.0

    uavs = {}
    for i in range(n):
        # v0 = [des_x[i]-init_x[i], des_y[i]-init_x[i]]
        # v0 = [0,0]
        uavs[i] = uav([init_x[i], init_y[i]], [des_x[i], des_y[i]], 0.0, v0, r)

    start_time = time.time()
    for i in range(1000):
        try: 
            print('iteration: ', i)
            ptcle = particle()
            pso(ptcle)
            for j in range(n):
                uavs[j].update_cur(ptcle.globalbest_pos[j])
            
            if finished():
                print('done')
                break
        except KeyboardInterrupt:
            plots()
            print('KeyboardInterrupt')
            num = input('1 to continue, others to stop')
            if int(num) != 1:
                break

    print('Excution time: ', time.time()-start_time)
    plots()




