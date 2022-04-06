import math
import random
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cm


class uav():
    def __init__(self, init, des, theta, v0, r):
        self.cur_pos = init
        self.des_pos = des
        self.nxt_pos = [0,0]
        self.theta = theta
        self.v0 = v0
        self.r = r
        self.x_list = [init[0]]
        self.y_list = [init[1]]

    def update_nxt(self, theta):
        self.nxt_pos[0] = self.cur_pos[0] + self.v0*math.cos(theta)
        self.nxt_pos[1] = self.cur_pos[1] + self.v0*math.sin(theta)

    def update_cur(self, theta):
        self.cur_pos[0] += self.v0*math.cos(theta)
        self.cur_pos[1] += self.v0*math.sin(theta)
        self.x_list.extend([self.cur_pos[0]])
        self.y_list.extend([self.cur_pos[1]])
      

def finished():
    for i in range(n):
        x,y = uavs[i].cur_pos[0], uavs[i].cur_pos[1]
        dx,dy = uavs[i].des_pos[0], uavs[i].des_pos[1]
        if ((x-dx)**2+(y-dy)**2 > 4**2):
            return False
    return True

class particle():
    def __init__(self):
        self.position = np.random.uniform(VarMin, VarMax, (nPop, nVar))
        self.velocity = np.zeros((nPop, nVar))
        self.cost = np.ones((nPop))
        self.best_position = np.copy(self.position)
        self.best_cost = np.ones((nPop))
        self.globalbest_cost = math.inf
        self.globalbest_pos = np.zeros((nVar))
    

def ObjectiveFunction():

    F1, F2, F3 = 0, 0, 0
    for i in range(n):
        xc, yc = uavs[i].cur_pos[0], uavs[i].cur_pos[1]
        xg, yg = uavs[i].des_pos[0], uavs[i].des_pos[1]
        xn, yn = uavs[i].nxt_pos[0], uavs[i].nxt_pos[1]
        F1 += ((xc-xn)**2 + (yc-yn)**2)**(0.5) + ((xn-xg)**2 + (yn-yg)**2)**(0.5)
        # print((xc-xg)*(xn-xg)+(yc-yg)*(yn-yg))
        # print(math.acos((xc-xg)*(xn-xg)+(yc-yg)*(yn-yg)))
        # if (xc-xg)*(xn-xg)+(yc-yg)*(yn-yg) <= 1 and (xc-xg)*(xn-xg)+(yc-yg)*(yn-yg) >= -1:
        #     F3 += math.acos((xc-xg)*(xn-xg)+(yc-yg)*(yn-yg)) / ((xc-xn)**2 + ((yc-yn)**2)**(0.5))*(((xn-xg)**2 + (yn-yg)**2)**(0.5))
        for j in range(n):
            if i!=j:
                xn2, yn2 = uavs[j].nxt_pos[0], uavs[j].nxt_pos[1]
                if ((xn-xn2)**2 + (yn-yn2)**2)**(0.5) <= 2*uavs[i].r:
                    F2 += 1e5

    return 1.0*F1 + 1.0*F2 + 0.5*F3


def pso(ptcle):
    w = 1
    for i in range(nPop):
        for j in range(n):
            uavs[j].update_nxt(ptcle.position[i,j])
        ptcle.cost[i] = ObjectiveFunction()
        ptcle.best_cost[i] = ptcle.cost[i]

        if ptcle.best_cost[i] < ptcle.globalbest_cost:
            ptcle.globalbest_cost = ptcle.best_cost[i]
            ptcle.globalbest_pos = ptcle.best_position[i,:]
            # print(ptcle.globalbest_cost)
    
    # PSO main loop
    for it in range(MaxIt):
        for i in range(nPop):
            for j in range(n):
                # update velocity
                ptcle.velocity[i,j] = (w*ptcle.velocity[i,j] +
                                    c1*random.random()*(ptcle.best_position[i,j] -ptcle.position[i,j]) +
                                    c2*random.random()*(ptcle.globalbest_pos[j] -ptcle.position[i,j]))
                # apply velocity limits
                ptcle.velocity[i,j] = min(max(ptcle.velocity[i,j], VelMin), VelMax)
                # update position
                ptcle.position[i,j] += ptcle.velocity[i,j]
                # velocity mirror effect
                if ptcle.position[i,j] < VarMin or ptcle.position[i,j] > VarMax:
                    ptcle.velocity[i,j] *= -1
                # apply position limits
                ptcle.position[i,j] = min(max(ptcle.position[i,j], VarMin), VarMax)
                uavs[j].update_nxt(ptcle.position[i,j])
            ptcle.cost[i] = ObjectiveFunction()
            # update personal best
            # print(ptcle.cost[i], ptcle.best_cost[i], ptcle.globalbest_cost)
            if ptcle.cost[i] < ptcle.best_cost[i]:
                ptcle.best_position[i,:] = ptcle.position[i,:]
                ptcle.best_cost[i] = ptcle.cost[i]
                # update global best
                if ptcle.best_cost[i] < ptcle.globalbest_cost:
                    ptcle.globalbest_cost = ptcle.best_cost[i]
                    ptcle.globalbest_pos = ptcle.best_position[i,:]
                    # print(ptcle.globalbest_cost)

        w *= wdamp
                    


if __name__ == '__main__':

    random.seed(3)
    np.random.seed(3)

    # initialize
    # init_x = [0, 0]
    # init_y = [-30, 30]
    # des_x = [0, 0]
    # des_y = [30, -30]
    init_x = [0, 30, 30, 0]
    init_y = [0, 0, 30, 30]
    des_x = [30, 0, 0, 30]
    des_y = [30, 30, 0, 0]
    n = len(init_x) # number of uav

    # Problem Definition
    nVar = n                # Number of Decision Variables
    # VarSize = [1 nVar]      # Size of Decision Variables Matrix
    VarMin = 0              # Lower Bound of Variables
    VarMax = 2*math.pi      # Upper Bound of Variables

    # PSO parameters
    MaxIt = 100       # Maximum Number of Iterations
    nPop = 100        # Population Size (Swarm Size)
    w = 1             # Inertia Weight
    wdamp = 0.99      # Inertia Weight Damping Ratio
    c1 = 1.5          # Personal Learning Coefficient
    c2 = 2.0          # Global Learning Coefficient

    # Velocity Limits
    VelMax = 0.1*(VarMax-VarMin)
    VelMin = -VelMax

    # 速度 & 安全距離設定 velocity and safty range
    v0, r = 5, 5

    uavs = {}
    for i in range(n):
        uavs[i] = uav([init_x[i], init_y[i]], [des_x[i], des_y[i]], 0.0, v0, r)

    start_time = time.time()
    for i in range(1000):
        print('iteration: ', i)
        ptcle = particle()
        pso(ptcle)
        for j in range(n):
            uavs[j].update_cur(ptcle.globalbest_pos[j])
        
        if finished():
            print('done')
            break

    print('Excution time: ', time.time()-start_time)

    # Plotting
    fig = plt.figure()
    norm = colors.Normalize(vmin=0, vmax=n)
    led =[]
    for j in range(n):
        plt.scatter(uavs[j].des_pos[0], uavs[j].des_pos[1], s=400, color = cm.hsv(norm(j)), marker="*")
        plt.plot(uavs[j].x_list, uavs[j].y_list, color = cm.hsv(norm(j)))
        led.append('uav'+str(j))
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.legend(led)
    plt.grid()
    plt.show()





