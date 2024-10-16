#!/usr/bin/env python

import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os
import subprocess

window = 200
first_run = 1

def animate(i):
    try:
        result = subprocess.check_output(['wc', '-l', 'data.csv'])
    except subprocess.CalledProcessError as e:
        return

    result_str = result.decode('utf-8')
    result_parts = result_str.split(' ')
    result_num = int(result_parts[0])


    if result_num > window:
        os.system('head -n 1 data.csv > data_short.csv')
        os.system('tail -n '+str(window)+' data.csv >> data_short.csv')
    else:
        os.system('tail -n '+str(window)+' data.csv > data_short.csv')

    data = pd.read_csv('data_short.csv')
    t = data['SYSTEM_TIME']
    ch4 = data['CH4']
    lat = data['LATITUDE']
    lon = data['LONGITUDE']
    alt = data['ALTITUDE']

    end = len(t)
    start = len(t) - window
    if start < 0:
      start = 0

    axs[0].cla()
    axs[0].plot(t[start:end], ch4[start:end], label='CH4 ppb')
    axs[0].legend(loc='upper left')

    axs[1].cla()

#    ax[1].scatter(lon[start:end], lat[start:end], alt[start:end], s=20, c=ch4[start:end])
    sp = axs[1].scatter(lon[start:end], lat[start:end], c=ch4[start:end])
    axs[1].axis('equal')

#    plt.colorbar()


fig, axs = plt.subplots(1,2)

ani = FuncAnimation(fig, animate, interval=1000)

plt.show()
