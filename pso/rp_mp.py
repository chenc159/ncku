import math
# import random
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm
import copy
import multiprocessing as mp

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

def plots2():
    # Plotting
    fig = plt.figure()
    norm = colors.Normalize(vmin=0, vmax=n)
    led =[]
    for j in range(n):
        plt.scatter(des_x[j], des_y[j], s=400, color = cm.hsv(norm(j)), marker="*")
        plt.plot(uav_res[j][0], uav_res[j][1], color = cm.hsv(norm(j)))
        plt.scatter(uav_res[j][0], uav_res[j][1], s=40, color = cm.hsv(norm(j)), marker=".")
        led.append('uav'+str(j))
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.legend(led)
    plt.grid()
    plt.show()


class uav():

    def __init__(self, id, init, des, theta, v0, r):
        self.id = id
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
        self.isfinished = False
        self.neighbor = {}

    def update_nxt(self, ang):
        # print(ang)
        self.nxt_acc[0] =  9.81*ang[0]
        self.nxt_acc[1] =  -9.81*ang[1]
        temp_a, temp_b = self.nxt_acc[0], self.nxt_acc[1]
        if (self.nxt_acc[0]**2 + self.nxt_acc[1]**2)**0.5 > MaxAcc:
            self.nxt_acc[0] = MaxAcc*temp_a/(temp_a**2+temp_b**2)**0.5
            self.nxt_acc[1] = MaxAcc*temp_b/(temp_a**2+temp_b**2)**0.5
        self.nxt_vel[0] = self.cur_vel[0] + self.nxt_acc[0]*dt
        self.nxt_vel[1] = self.cur_vel[1] + self.nxt_acc[1]*dt
        temp_a, temp_b = self.nxt_vel[0], self.nxt_vel[1]
        if (self.nxt_vel[0]**2 + self.nxt_vel[1]**2)**0.5 > MaxVel:
            self.nxt_vel[0] = MaxVel*temp_a/(temp_a**2+temp_b**2)**0.5
            self.nxt_vel[1] = MaxVel*temp_b/(temp_a**2+temp_b**2)**0.5
        self.nxt_pos = self.cur_pos + self.nxt_vel*dt
        
    def update_cur(self, ang, uav_res):
        self.cur_acc[0] = 9.81*ang[0]
        self.cur_acc[1] = -9.81*ang[1]
        temp_a, temp_b = self.cur_acc[0], self.cur_acc[1]
        if (self.cur_acc[0]**2+self.cur_acc[1]**2)**0.5 > MaxAcc:
            self.cur_acc[0] = MaxAcc*temp_a/(temp_a**2+temp_b**2)**0.5
            self.cur_acc[1] = MaxAcc*temp_b/(temp_a**2+temp_b**2)**0.5
        self.cur_vel[0] = self.cur_vel[0] + self.cur_acc[0]*dt
        self.cur_vel[1] = self.cur_vel[1] + self.cur_acc[1]*dt
        temp_a, temp_b = self.cur_vel[0], self.cur_vel[1]
        if (self.cur_vel[0]**2+self.cur_vel[1]**2)**0.5 > MaxVel:
            self.cur_vel[0] = MaxVel*temp_a/(temp_a**2+temp_b**2)**0.5
            self.cur_vel[1] = MaxVel*temp_b/(temp_a**2+temp_b**2)**0.5
        self.cur_pos += self.cur_vel*dt
        self.x_list.extend([self.cur_pos[0]])
        self.y_list.extend([self.cur_pos[1]])
        uav_res[self.id][0] = self.x_list
        uav_res[self.id][1] = self.y_list

    def finished(self):
        if ((self.cur_pos[0]-self.des_pos[0])**2+(self.cur_pos[1]-self.des_pos[1])**2 > 5**2):
            return False
        return True
    
    def pathplan(self, uav_pos, uav_res):
        # if no neightbor: go straight
        # else: pso
        # print('here')
        for i in range(1000):
            print('UAV id: ', self.id, ', Iteration: ', i)
            time.sleep(0.5)
            for other_id in uav_pos.keys():
                if self.id != other_id:
                    self.neighbor[other_id] = [uav_pos[other_id][0], uav_pos[other_id][1]]
            self.ptcle = particle()
            self.pso()
            self.update_cur(self.ptcle.globalbest_pos, uav_res)
            uav_pos[self.id][0], uav_pos[self.id][1] = self.cur_pos[0], self.cur_pos[1]
            if self.finished():
                print('UAV id: ', self.id, ' Done!')
                break            

    
    def ObjectiveFunction(self): #, uav_pos):
        F1, F2, F3, F4 = 0, 0, 0, 0
        w1, w2, w3, w4 = 0.1, 0.9, 0, 0
        xi, yi = self.ini_pos[0], self.ini_pos[1]
        xc, yc = self.cur_pos[0], self.cur_pos[1]
        xg, yg = self.des_pos[0], self.des_pos[1]
        xn, yn = self.nxt_pos[0], self.nxt_pos[1]
        xnc, ync, xng, yng, xcg, ycg = xn-xc, yn-yc, xn-xg, yn-yg, xc-xg, yc-yg
        F1 += (xng**2 + yng**2)**(0.5)

        for j in self.neighbor.keys():
            x2, y2 = self.neighbor[j][0], self.neighbor[j][1]
            d12 = ((xn-x2)**2 + (yn-y2)**2)**(0.5)
            if d12 <= 1.0*self.r:
                F2 += 1
            elif d12 > 1.0*self.r and d12 <= 3.0*self.r:
                F2 += self.r/d12

        return w1*F1 + w2*F2 + w3*F3 + w4*F4

    def pso(self):
        w = 1
        self.ptcle.position[0,:] = [self.des_pos[0] - self.cur_pos[0], -(self.des_pos[1] - self.cur_pos[1])]
        # print(self.id, self.ptcle.position[0,:])
        self.ptcle.best_position[0,:] = [self.des_pos[0] - self.cur_pos[0], -(self.des_pos[1] - self.cur_pos[1])]
        for i in range(nPop):
            self.update_nxt(self.ptcle.position[i,:])
            self.ptcle.cost[i] = self.ObjectiveFunction()
            self.ptcle.best_cost[i] = self.ptcle.cost[i]

            if self.ptcle.best_cost[i] < self.ptcle.globalbest_cost:
                self.ptcle.globalbest_cost = self.ptcle.best_cost[i]
                self.ptcle.globalbest_pos = self.ptcle.best_position[i,:]
                # print(ptcle.globalbest_cost)
        
        # PSO main loop
        for it in range(MaxIt):
            # print(it)
            for i in range(nPop):
                for k in range(nCom):
                    # update velocity
                    self.ptcle.velocity[i,k] = (w*self.ptcle.velocity[i,k] +
                                        c1*np.random.random()*(self.ptcle.best_position[i,k] - self.ptcle.position[i,k]) +
                                        c2*np.random.random()*(self.ptcle.globalbest_pos[k] - self.ptcle.position[i,k]))
                    # apply velocity limits
                    self.ptcle.velocity[i,k] = min(max(self.ptcle.velocity[i,k], VelMin), VelMax)
                    # update position
                    self.ptcle.position[i,k] += self.ptcle.velocity[i,k]
                    # velocity mirror effect
                    if self.ptcle.position[i,k] < VarMin or self.ptcle.position[i,k] > VarMax:
                        self.ptcle.velocity[i,k] *= -1
                    # apply position limits
                    self.ptcle.position[i,k] = min(max(self.ptcle.position[i,k], VarMin), VarMax)
                self.update_nxt(self.ptcle.position[i,:])
                self.ptcle.cost[i] = self.ObjectiveFunction()
                # update personal best
                if self.ptcle.cost[i] < self.ptcle.best_cost[i]:
                    self.ptcle.best_position[i,:] = self.ptcle.position[i,:]
                    self.ptcle.best_cost[i] = self.ptcle.cost[i]
                    # update global best
                    if self.ptcle.best_cost[i] < self.ptcle.globalbest_cost:
                        self.ptcle.globalbest_cost = self.ptcle.best_cost[i]
                        self.ptcle.globalbest_pos = self.ptcle.best_position[i,:]

            w *= wdamp

class particle():
    def __init__(self):
        self.position = [] 
        for i in range(nPop):
            self.position.extend([np.random.uniform(VarMin, VarMax, nCom)])
        self.position = np.asarray(self.position)
        self.velocity = np.zeros((nPop, nCom))
        self.cost = np.ones((nPop))
        self.best_position = copy.deepcopy(self.position)
        self.best_cost = np.ones((nPop))
        self.globalbest_cost = math.inf
        self.globalbest_pos = np.zeros((nCom))
    

if __name__ == '__main__':

    start_time = time.time()

    np.random.seed(4)
    cpu_count = mp.cpu_count()

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
    # Define max acc, max vel, initial velocity, safty range, time interval
    MaxAcc = 8.0
    MaxVel = 5.0
    v0, r, dt = 0.0, 5.0, 1.0

    # Problem Definition
    nVar = n                # Number of Decision Variables
    nCom = 2                # Number of Components (roll, pitch)
    # Upper/Lower Bound of Variables
    VarMax = math.pi/6.0      
    VarMin = -VarMax     
    # Velocity Limits
    VelMax = 0.5*(VarMax-VarMin)
    VelMin = -VelMax

    # PSO parameters
    MaxIt = 100       # Maximum Number of Iterations
    nPop = 100        # Population Size (Swarm Size)
    w = 1             # Inertia Weight
    wdamp = 0.99      # Inertia Weight Damping Ratio
    c1 = 1.5          # Personal Learning Coefficient
    c2 = 2.0          # Global Learning Coefficient


    uavs = {} # initialize UAVs and put them in dictionary
    for i in range(n):
        uavs[i] = uav(i, [init_x[i], init_y[i]], [des_x[i], des_y[i]], 0.0, v0, r)

    mgr = mp.Manager()
    uav_pos, uav_res = mgr.dict(), mgr.dict()
    for i in range(n):
        # for v2v communication
        uav_pos[i] = mgr.dict()
        uav_pos[i][0], uav_pos[i][1] = [init_x[i]], [init_y[i]]
        # for final result plot
        uav_res[i] = mgr.dict()
        uav_res[i][0], uav_res[i][1] = [init_x[i]], [init_y[i]]
    
    jobs = [ mp.Process(target=(uavs[i].pathplan), args=(uav_pos, uav_res))
             for i in range(n) 
            ]
    
    for j in jobs:
        j.start()
    for j in jobs:
        j.join()

    print('Excution time: ', time.time()-start_time)
    plots2()





