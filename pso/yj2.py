import math
import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy
# import random
# import matplotlib.colors as colors
# import matplotlib.cm as cm
# import multiprocessing
# from multiprocessing import Pool

# def plots():
#     # Plotting
#     fig = plt.figure()
#     norm = colors.Normalize(vmin=0, vmax=n)
#     led =[]
#     for j in range(n):
#         plt.scatter(uavs[j].des_pos[0], uavs[j].des_pos[1], s=400, color = cm.hsv(norm(j)), marker="*")
#         plt.plot(uavs[j].x_list, uavs[j].y_list, color = cm.hsv(norm(j)))
#         plt.scatter(uavs[j].x_list, uavs[j].y_list, s=40, color = cm.hsv(norm(j)), marker=".")
#         led.append('uav'+str(j))
#     plt.xlabel('X')
#     plt.ylabel('Y')
#     plt.axis('equal')
#     plt.legend(led)
#     plt.grid()
#     plt.show()

def plots_3d():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot3D(flight.x_list, flight.y_list, flight.z_list, 'blue')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.grid()
    plt.show()

class uav():
    def __init__(self, init, des, theta0, v0):
        self.ini_pos = copy.deepcopy(init)
        self.cur_pos = copy.deepcopy(init)
        self.des_pos = copy.deepcopy(des)
        self.nxt_pos = np.asarray([0,0])
        self.cur_vel = np.asarray([v0, v0, v0])
        self.nxt_vel = np.asarray([v0, v0, v0])
        self.cur_theta = copy.deepcopy(theta0) # phi, gamma, psi
        self.nxt_theta = copy.deepcopy(theta0)
        self.x_list = [init[0]]
        self.y_list = [init[1]]
        self.z_list = [init[2]]

    def update_nxt(self, ang):
        self.nxt_theta[0] = ang[0]
        self.nxt_theta[1] = ang[1]
        self.nxt_theta[2] = max(min(self.cur_theta[2] + (g/v0)*math.tan(ang[0])*dt, math.pi), -math.pi)
        self.nxt_vel[0] =  v0*math.cos(self.nxt_theta[2])*math.cos(self.nxt_theta[1])
        self.nxt_vel[1] =  v0*math.sin(self.nxt_theta[2])*math.cos(self.nxt_theta[1])
        self.nxt_vel[2] =  v0*math.sin(self.nxt_theta[1])
        self.nxt_pos = self.cur_pos + self.nxt_vel*dt

    def update_cur(self, ang):
        self.cur_theta[0] = ang[0]
        self.cur_theta[1] = ang[1]
        self.cur_theta[2] = max(min(self.cur_theta[2] + (g/v0)*math.tan(ang[0])*dt, math.pi), -math.pi)
        self.cur_vel[0] =  v0*math.cos(self.cur_theta[2])*math.cos(self.cur_theta[1])
        self.cur_vel[1] =  v0*math.sin(self.cur_theta[2])*math.cos(self.cur_theta[1])
        self.cur_vel[2] =  v0*math.sin(self.cur_theta[1])
        self.cur_pos += self.cur_vel*dt
        self.x_list.extend([self.cur_pos[0]])
        self.y_list.extend([self.cur_pos[1]])
        self.z_list.extend([self.cur_pos[2]])

def finished():
    x, y, z = flight.cur_pos[0], flight.cur_pos[1], flight.cur_pos[2]
    dx, dy, dz = flight.des_pos[0], flight.des_pos[1], flight.des_pos[2]
    if ((x-dx)**2+(y-dy)**2+(z-dz)**2 > 50**2):
        return False
    return True

class particle():
    def __init__(self):
        self.position = [] 
        for i in range(nPop):
            self.position.extend([[np.random.uniform(VarMin[0], VarMax[0]), np.random.uniform(VarMin[1], VarMax[1])]])
        self.position = np.asarray(self.position)
        self.velocity = np.zeros((nPop, nCom))
        self.cost = np.ones((nPop))
        self.best_position = copy.deepcopy(self.position)
        self.best_cost = np.ones((nPop))
        self.globalbest_cost = math.inf
        self.globalbest_pos = np.zeros((nCom))
    
def ObjectiveFunction():
    F1, F2, F3, F4 = 0, 0, 0, 0
    xg, yg, zg = flight.des_pos[0], flight.des_pos[1], flight.des_pos[2]
    xn, yn, zn = flight.nxt_pos[0], flight.nxt_pos[1], flight.nxt_pos[2]
    F1 += ((xn-xg)**2 + (yn-yg)**2 + (zn-zg)**2)**(0.5)
    return 1.0*F1 + 0.0*F2 + 0.0*F3 + 0.0*F4


def pso():
    w = 1
    # for j in range(n):
    #     a = uavs[j].des_pos[0] - uavs[j].cur_pos[0]
    #     b = uavs[j].des_pos[1] - uavs[j].cur_pos[1]
    #     ptcle.position[0,j,:] = [a, -b]
    #     ptcle.best_position[0,j,:] = [a, -b]
    #     # print(ptcle.position[0,j,:])
    for i in range(nPop):
        flight.update_nxt(ptcle.position[i,:])
        ptcle.cost[i] = ObjectiveFunction()
        ptcle.best_cost[i] = ptcle.cost[i]
        ptcle.best_position[i,:] = ptcle.best_position[i]

        if ptcle.best_cost[i] < ptcle.globalbest_cost:
            ptcle.globalbest_cost = ptcle.best_cost[i]
            ptcle.globalbest_pos = ptcle.best_position[i,:]
            # print(ptcle.globalbest_cost)
    
    # PSO main loop
    for it in range(MaxIt):
        for i in range(nPop):
            for k in range(nCom):
                # update velocity
                ptcle.velocity[i,k] = (w*ptcle.velocity[i,k] +
                                    c1*np.random.random()*(ptcle.best_position[i,k] - ptcle.position[i,k]) +
                                    c2*np.random.random()*(ptcle.globalbest_pos[k] - ptcle.position[i,k]))
                # apply velocity limits
                ptcle.velocity[i,k] = min(max(ptcle.velocity[i,k], VelMin[k]), VelMax[k])
                # update position
                ptcle.position[i,k] += ptcle.velocity[i,k]
                # velocity mirror effect
                if ptcle.position[i,k] < VarMin[k] or ptcle.position[i,k] > VarMax[k]:
                    ptcle.velocity[i,k] *= -1
                # apply position limits
                ptcle.position[i,k] = min(max(ptcle.position[i,k], VarMin[k]), VarMax[k])
            flight.update_nxt(ptcle.position[i,:])
            ptcle.cost[i] = ObjectiveFunction()
            # update personal best
            if ptcle.cost[i] < ptcle.best_cost[i]:
                ptcle.best_position[i,:] = ptcle.position[i,:]
                ptcle.best_cost[i] = ptcle.cost[i]
                # update global best
                if ptcle.best_cost[i] < ptcle.globalbest_cost:
                    ptcle.globalbest_cost = ptcle.best_cost[i]
                    ptcle.globalbest_pos = ptcle.best_position[i,:]
                    # print('a',ptcle.globalbest_cost) #, ptcle.best_position[i,:,:])

        w *= wdamp

     


if __name__ == '__main__':

    # random.seed(3)
    np.random.seed(3)

    # initialize
    init =  [0.0, 0.0, 0.0]
    des = [2000.0, 2000.0, 2000.0]

    # Problem Definition
    nCom = 2                    # Number of Components (roll, pitch)
    # VarMax = 1.0*math.pi        # Upper Bound of Variables, 2*math.pi
    VarMax = np.asarray([1.0*math.pi/3, math.pi/4])
    VarMin = -VarMax            # Lower Bound of Variables, 0
    # VarMax = math.pi/6.0      
    # VarMin = -VarMax     
    # Velocity Limits
    VelMax = 0.5*(VarMax-VarMin)
    VelMin = -VelMax

    # PSO parameters
    MaxIt = 100       # Maximum Number of Iterations
    nPop = 100        # Population Size (Swarm Size)
    w = 1             # Inertia Weight
    wdamp = 0.99      # Inertia Weight Damping Ratio
    c1 = 2.0          # Personal Learning Coefficient
    c2 = 2.0          # Global Learning Coefficient

    g, dt = 9.8, 2
    v0 = 80
    theta0 = [0.0, 0.0, 0.0]
    flight = uav(init, des, theta0, v0)

    start_time = time.time()
    for i in range(1000):
        try: 
            print('Iteration: ', i)
            ptcle = particle()
            pso()
            flight.update_cur(ptcle.globalbest_pos)
            
            if finished():
                print('Done!')
                break
        except KeyboardInterrupt:
            plots_3d()
            print('KeyboardInterrupt')
            num = input('1 to continue, others to stop')
            if int(num) != 1:
                break

    print('Excution time: ', time.time()-start_time)
    plots_3d()




