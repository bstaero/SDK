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

import sys
import os.path
import binascii
import struct

from bst_python_sdk.handler import *
from bst_python_sdk.listener import *

import socket
import serial

import time, threading

#import handler
import plotting_handler


def initializeCommunications(host,port):
    s.connect((host, port));

def stopCommunications():
    s.close();

def communicationsThread(handler):
    while True:
        readUserData(handler);
        time.sleep(0.01);

def printValues():
    print("lla: %+07.03f %+08.03f %06.01f | %08.01f Pa %+05.01f deg %04.01f %%" % (
          telemetry_position.latitude,
          telemetry_position.longitude,
          telemetry_position.altitude,
          telemetry_pressure.static_pressure,
          telemetry_pressure.air_temperature,
          telemetry_pressure.humidity));

    threading.Timer(0.1, printingThread).start()

def sendTest():
    message = "this is a test %07.01f\n\r" % (time.time() - start_time);
    data = [ord(c) for c in message];

    sendUserData(data,len(data));
    print("sent message");

    threading.Timer(1.0, sendThread).start()

def readUserData(handler):
    data = s.recv(1024)
    #data = s.read(1024)
    if(len(data) > 0):
        pkt = parse_stream(data, handler);

def sendUserData(data,size):
    tx_payload = UserPayload();

    pkt = BSTPacket();
    pkt.set_addressing(True);
    pkt.TYPE=PacketTypes.PAYLOAD_CHANNEL_0;
    pkt.ACTION=0
    pkt.SIZE=tx_payload.SIZE;
    pkt.TO=0xFFFFFFFF;
    pkt.FROM=0;

    ptr = 0;

    while(ptr < size):
        if(size-ptr > 64):
            tx_payload.size = 64;
        else:
            tx_payload.size = size-ptr;

        for i in range(0,tx_payload.size):
            tx_payload.buffer[i] = data[ptr+i];

        for i in range(tx_payload.size,64):
            tx_payload.buffer[i] = 0;

        pkt.DATA = tx_payload.serialize();
        pkt.set_fletcher_16();

        s.send(pkt.serialize(),pkt.SIZE);

        ptr += tx_payload.size;
        tx_payload.size = 0;
        tx_payload.buffer = [None] * 64;


start_time = time.time();

HOST = '127.0.0.1';  # The server's hostname or IP address
#HOST = '192.168.1.1';  # The server's hostname or IP address
PORT = 55555;        # The port used by the server

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM);

initializeCommunications(HOST,PORT);

#DEVICE = '/dev/ttyUSB1';
#BAUD = 115200;
#
#s = serial.Serial(DEVICE, BAUD);

#communicationsThread(handler);
communicationsThread(plotting_handler);
