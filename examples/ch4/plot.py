#!/usr/bin/env python

import random
from itertools import count
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

window = 30
first_run = 1

def animate(i):
    os.system('head -n 1 data.csv > data_short.csv')
    os.system('tail -n 100 data.csv >> data_short.csv')

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
