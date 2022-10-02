import multiprocessing
import time
import copy
import numpy as np




def worker(d, key):
    for i in range(16):
        # print('before', key, d[0][0], d[key][0])
        if i%3 == 0:
            if key == 0:
                d[key][0] += 1
                print('\n', key, d[0][0], d[key])
            else:
                d[key][0] = d[0][0]
                print(key, d[0][0], d[key])

        # if key == 0:
        #     d[key]['pos'] += 1
        # else:
        #     d[key]['pos'] = d[0]['pos'] 
        # print(key, d[key]['pos'])

        # if d[key].id == 0:
        #     print('bb', d[0].pos)
        #     d[0].update(9)
        #     print('aA', d[0].pos)
        # else:
        #     d[key].pos = d[key-1].pos
        # # print(d[key].id, d[key].pos)

if __name__ == '__main__':
    class uav(object):
        def __init__(self, id, pos):
            self.id = id
            self.pos = pos
        def update(self, pos):
            self.pos = pos
            print(self.pos)
    
    mgr = multiprocessing.Manager()
    d = mgr.dict()
    for i in range(5):
        d[i] = mgr.list([1,2,3])
        # d[i] = uav(i, 1)
        # d[i] = mgr.dict()
        # d[i]['pos'] = 0
    jobs = [ multiprocessing.Process(target=worker, args=(d, i))
             for i in range(5) 
             ]
    for j in jobs:
        j.start()
    for j in jobs:
        j.join()