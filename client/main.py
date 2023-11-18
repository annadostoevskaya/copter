#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: main.py
# Author: github.com/annadostoevskaya
# Date: 11/10/2023 20:13:09
# Last Modified Date: 11/10/2023 22:33:13

import bluetooth
import config as cfg
import tkinter
import json

if __name__ == '__main__':    
    root = tkinter.Tk();
    slider = tkinter.Scale(root, from_=0, to=100, 
        length=600, tickinterval=10, 
        orient=tkinter.HORIZONTAL)
    slider.pack();

    addr = cfg.TARGET_DEVICE_ADDR;
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM);
    sock.connect((cfg.TARGET_DEVICE_ADDR, cfg.TARGET_DEVICE_PORT));
    sock.setblocking(False);

    while True:
        power = slider.get();
        sock.send("{}\n".format(power));
        time.sleep(30 / 1000);
        root.update();
    
    sock.close();
    exit(0);


