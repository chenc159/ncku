import csv
import sys
import os
import matplotlib

def read_csv(address):
    global trans_time, exe_time, all_pos, goals, n
    trans_time, exe_time, all_pos, goals = [], [], [], []
    with open(address) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            if 'Trans_Time' not in row:
                if 'Goals_x' in row:
                    goals.append([float(num) for num in row[2:]])
                else:
                    trans_time.append(float(row[0]))
                    exe_time.append(float(row[1]))
                    all_pos.append([float(num) for num in row[2:]])
        # print(len(trans_time))
        # print(np.shape(all_pos))
        goals = np.array(goals)
        all_pos = np.array(all_pos)
    n = int(len(all_pos[1])/nd)
    # print(len(all_pos))
    print('Length of video: ', trans_time[-1])

    global time, mode, lat, lon, alt, vx, vy, vz, roll, pitch, yaw
    time, mode, lat, lon, alt, vx, vy, vz, roll, pitch, yaw = [],[],[],[],[],[],[],[],[],[],[]
    with open(address) as csvfile:
        readCSV = csv.reader(csvfile, delimiter=',')
        for row in readCSV:
            if 'time' not in row:
                time.append(float(row[0]))
                mode.append(row[1])
                lat.append(float(row[2]))
                lon.append(float(row[3]))
                alt.append(float(row[4]))
                vx.append(float(row[5]))
                vy.append(float(row[6]))
                vz.append(float(row[7]))
                roll.append(float(row[8]))
                pitch.append(float(row[9]))
                yaw.append(float(row[10]))




if __name__ == '__main__':
    if len(sys.argv) == 1:
        print('File name needed!!')
    else:
        file_name = sys.argv[1]
        print('Inputed file name is: ', file_name)
    address = os.path.dirname(os.path.realpath('__file__')) + '/result/' + file_name + ".csv"
    read_csv(address)