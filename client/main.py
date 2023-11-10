#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: main.py
# Author: github.com/annadostoevskaya
# Date: 11/10/2023 20:13:09
# Last Modified Date: 11/10/2023 22:33:13

import bluetooth
import tkinter
import json
import time

TARGET_DEVICE_ADDR = 'D8:55:75:A4:9A:1A'

if __name__ == '__main__':
    # devices = bluetooth.discover_devices(lookup_names=True);
    # print("client: Found {} devices.".format(len(devices)));

    # target_device = '';
    # for addr, name in devices:
    #    if (addr == TARGET_DEVICE_ADDR):
    #        target_device = addr;
    #    print('client:  {} - {}'.format(addr, name));

    # if (not target_device):
    #     print('client: 404 target device!');
    #     exit(-1);
    
    root = tkinter.Tk();
    slider = tkinter.Scale(root, from_=0, to=100, length=600, tickinterval=10, orient=tkinter.HORIZONTAL)
    slider.pack();

    addr = TARGET_DEVICE_ADDR;
    port = -1;
    service = bluetooth.find_service(uuid="00001101-0000-1000-8000-00805F9B34FB", address=TARGET_DEVICE_ADDR).pop();
    print(json.dumps(service, sort_keys=True, indent=4));
    port = service['port'];

    if (port == -1):
        print('client: 404 service not found!');
        exit(-1);

    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM);
    sock.connect((TARGET_DEVICE_ADDR, port));
    sock.setblocking(False);

    while True:
        power = slider.get();
        sock.send("{}\n".format(power));
        time.sleep(30 / 1000);
        root.update();
    
    sock.close();
    exit(0);


