#*=+--+=#=+--            SwiftPilot Autopilot Software            --+=#=+--+=#*#
#               Copyright (C) 2019 Black Swift Technologies LLC.               #
#                             All Rights Reserved.                             #
#                                                                              #
#    This program is free software: you can redistribute it and/or modify      #
#    it under the terms of the GNU General Public License version 2 as         #
#    published by the Free Software Foundation.                                #
#                                                                              #
#    This program is distributed in the hope that it will be useful,           #
#    but WITHOUT ANY WARRANTY; without even the implied warranty of            #
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
#    GNU General Public License for more details.                              #
#                                                                              #
#    You should have received a copy of the GNU General Public License         #
#    along with this program.  If not, see <http://www.gnu.org/licenses/>.     #
#                                                                              #
#                                 Jack Elston                                  #
#                          elstonj@blackswifttech.com                          #
#                                                                              #
#*=+--+=#=+--                 --+=#=+--+=#=+--                    --+=#=+--+=#*#

from bstpacket import BSTPacket

from comm_packets import *
from payload import *
from fixedwing import *

from downstream import *

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

telemetry_control  = TelemetryControl();
telemetry_orientation = TelemetryOrientation();
telemetry_position = TelemetryPosition();
telemetry_pressure = TelemetryPressure();
telemetry_system = TelemetrySystem();

user_payload = UserPayload();

downstream = Downstream();

def standard_handler(pkt):

    if((pkt.FROM & 0xFF000000) == 0x41000000):

        #-----[ TELEMETRY ]-----%
        if pkt.TYPE is PacketTypes.TELEMETRY_CONTROL:
            try:
                telemetry_control.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        if pkt.TYPE is PacketTypes.TELEMETRY_ORIENTATION:
            try:
                telemetry_orientation.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        if pkt.TYPE is PacketTypes.TELEMETRY_POSITION:
            try:
                telemetry_position.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        if pkt.TYPE is PacketTypes.TELEMETRY_PRESSURE:
            try:
                telemetry_pressure.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        if pkt.TYPE is PacketTypes.TELEMETRY_SYSTEM:
            try:
                telemetry_system.parse(pkt.DATA)
            except BufferError as ErrorMessage:
                print(ErrorMessage)

        #-----[ PAYLOAD ]-----%
        if pkt.TYPE is PacketTypes.PAYLOAD_CHANNEL_0:
            try:
                user_payload.buffer = [None] * 64;
                user_payload.parse(pkt.DATA)

                try:
                    downstream.parse(bytearray(user_payload.buffer))
                    updatePlots();
                except BufferError as ErrorMessage:
                    print(ErrorMessage)

            except BufferError as ErrorMessage:
                print(ErrorMessage)

plt.rcParams['toolbar'] = 'None'
fig = plt.figure(num=None, figsize=(20, 10), dpi=80, facecolor='w', edgecolor='k')

h2s_plt      = plt.subplot2grid( (5,5), (0,0) )
so2_020_plt  = plt.subplot2grid( (5,5), (1,0) )
so2_200_plt  = plt.subplot2grid( (5,5), (2,0) )
so2_500_plt  = plt.subplot2grid( (5,5), (3,0) )
co2_plt      = plt.subplot2grid( (5,5), (4,0) )
pres_plt     = plt.subplot2grid( (3,5), (0,1) )
temp_plt     = plt.subplot2grid( (3,5), (1,1) )
rh_plt       = plt.subplot2grid( (3,5), (2,1) )
part_003_plt = plt.subplot2grid( (6,5), (0,2) )
part_005_plt = plt.subplot2grid( (6,5), (1,2) )
part_010_plt = plt.subplot2grid( (6,5), (2,2) )
part_025_plt = plt.subplot2grid( (6,5), (3,2) )
part_050_plt = plt.subplot2grid( (6,5), (4,2) )
part_100_plt = plt.subplot2grid( (6,5), (5,2) )
pos_plt      = plt.subplot2grid( (1,5), (0,3) , colspan=2, projection='3d')

plt.subplots_adjust(wspace=0.5, hspace=0.5);

plt.ion()
plt.show()

system_time = [];
h2s = [];
so2_020 = [];
so2_200 = [];
so2_500 = [];
co2_ppm = [];
part_003 = [];
part_005 = [];
part_010 = [];
part_025 = [];
part_050 = [];
part_100 = [];
lat = [];
lon = [];
alt = [];
temp = [];
rh = [];
pres = [];

def updatePlots():
    global system_time, h2s, so2_020, so2_200, so2_500, co2_ppm, part_003, part_005, part_010, part_025, part_050, part_100, lat, lon, alt, temp, rh, pres

    system_time = addData(system_time,downstream.system_time);
    h2s         = addData(h2s,downstream.gas_ppm[0]);
    so2_020     = addData(so2_020,downstream.gas_ppm[1]);
    so2_200     = addData(so2_200,downstream.gas_ppm[2]);
    so2_500     = addData(so2_500,downstream.gas_ppm[3]);
    co2_ppm     = addData(co2_ppm,downstream.co2_ppm);
    part_003    = addData(part_003,downstream.particle_counts[0]);
    part_005    = addData(part_005,downstream.particle_counts[1]);
    part_010    = addData(part_010,downstream.particle_counts[2]);
    part_025    = addData(part_025,downstream.particle_counts[3]);
    part_050    = addData(part_050,downstream.particle_counts[4]);
    part_100    = addData(part_100,downstream.particle_counts[5]);
    lat         = addData(lat,downstream.latitude);
    lon         = addData(lon,downstream.longitude);
    alt         = addData(alt,downstream.altitude);
    temp        = addData(temp,telemetry_pressure.air_temperature);
    rh          = addData(rh,telemetry_pressure.humidity);
    pres        = addData(pres,telemetry_pressure.static_pressure);

    plotData(system_time, h2s, h2s_plt, 'H2S [ppm]', 0, 100);
    plotData(system_time, so2_020, so2_020_plt, 'SO2 [ppm]', 0, 20);
    plotData(system_time, so2_200, so2_200_plt, 'SO2 [ppm]', 0, 200);
    plotData(system_time, so2_500, so2_500_plt, 'SO2 [ppm]', 0, 500);
    plotData(system_time, co2_ppm, co2_plt, 'CO2 [ppm]', 0, 10000);
    plotData(system_time, part_003, part_003_plt, '< 0.3 um', 0, 0);
    plotData(system_time, part_005, part_005_plt, '< 0.5 um', 0, 0);
    plotData(system_time, part_010, part_010_plt, '< 1.0 um', 0, 0);
    plotData(system_time, part_025, part_025_plt, '< 2.5 um', 0, 0);
    plotData(system_time, part_050, part_050_plt, '< 5.0 um', 0, 0);
    plotData(system_time, part_100, part_100_plt, '< 10.0 um', 0, 0);
    plotData(system_time, temp, temp_plt, 'Temperature [deg C]', -20, 40);
    plotData(system_time, rh, rh_plt, 'Humidity [%]', 0, 100);
    plotData(system_time, pres, pres_plt, 'Pressure [Pa]', 60000, 100000);

    pos_plt.clear();
    pos_plt.plot3D(lon, lat, alt, 'gray')

    plt.draw()
    plt.pause(0.001)


def addData(array,data):
    array.append(data);
    array = array[-100:]

    return array

def plotData(time, data, plot, ylabel, ymin, ymax):

    plot.clear()
    plot.plot(system_time,data)

    if(ymin != ymax):
        plot.set_ylim(ymin,ymax)

    if(isinstance(data[-1],float)):
        text_num = round(data[-1],2)
    else:
        text_num = data[-1]
    plot.text(0.99,0.95, text_num,
            horizontalalignment='right',
            verticalalignment='top',
            transform=plot.transAxes)

    plot.set_ylabel(ylabel)
    plot.set_xlabel('Time [s]')
