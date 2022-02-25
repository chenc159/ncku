import csv
import sys
import os
import matplotlib.pyplot as plt
import matplotlib.patches as ptch
import matplotlib.cm as cm
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import art3d
from mpl_toolkits.mplot3d import proj3d
import pandas as pd
import numpy as np


def plot_xy(x, y, lx, ly):
    fig = plt.figure()
    plt.plot(x, y)
    plt.grid()
    plt.xlabel(lx)
    plt.ylabel(ly)
    plt.show()

def plot_xyz(lat, lon, alt):
    global ax
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(lat, lon, alt)

    ax.set_xlabel('Lat')
    ax.set_ylabel('Lon')
    ax.set_zlabel('Alt')
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) == 1 or len(sys.argv) == 2: #2m24d11h35m33s
        print('1st arg for file name, 2nd arg for data-to-plot (3D/vx/vy/vz/roll/pitch/yaw)!!')
    else:
        file_name = sys.argv[1]
        print('Inputed file name is: ', file_name)
        address = os.path.dirname(os.path.realpath('__file__')) + '/result/' + file_name + ".csv"
        df = pd.read_csv(address)
        if '3D' in sys.argv[2]:
            plot_xyz(df['lat'],df['lon'],df['alt'])
        elif 'vx' in sys.argv[2]: 
            plot_xy(df['time'],df['vx'], 'time', 'vx')
        elif 'vy' in sys.argv[2]: 
            plot_xy(df['time'],df['vy'], 'time', 'vy')
        elif 'vz' in sys.argv[2]: 
            plot_xy(df['time'],df['vz'], 'time', 'vz')
        elif 'roll' in sys.argv[2]: 
            plot_xy(df['time'],df['roll'], 'time', 'roll')
        elif 'pitch' in sys.argv[2]: 
            plot_xy(df['time'],df['pitch'], 'time', 'pitch')
        elif 'yaw' in sys.argv[2]: 
            plot_xy(df['time'],df['yaw'], 'time', 'yaw')

    