#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# File: main.py
# Author: github.com/annadostoevskaya
# Date: 11/10/2023 20:13:09
# Last Modified Date: 11/10/2023 22:33:13

import bluetooth
import config as cfg
import time
import tkinter
import json

g_events = [];

def e_callback(e):
    g_events.append(e);

def e_handling(root, slider, sock, events):
    while True:
        if (len(events) == 0):
            return

        e = events.pop();
        
        if (e.keycode == 32 or e.keycode == 38):
            slider.set(slider.get() + 10);
        elif (e.keycode == 40):
            slider.set(slider.get() - 10);
        elif (e.keycode == 27 or e.keycode == 115):
            root.destroy();
            sock.close();
            exit(0)

        print(e);

if __name__ == '__main__':    
    root = tkinter.Tk();
    root.bind("<Alt-F4>", e_callback);
    root.bind("<Escape>", e_callback);
    root.bind("<space>", e_callback);
    root.bind("<Down>", e_callback);
    root.bind("<Up>", e_callback);

    slider = tkinter.Scale(root, from_=0, to=100, 
        length=600, tickinterval=10, 
        orient=tkinter.HORIZONTAL)
    slider.pack();

    addr = cfg.TARGET_DEVICE_ADDR;
    sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM);
    sock.connect((cfg.TARGET_DEVICE_ADDR, cfg.TARGET_DEVICE_PORT));
    sock.setblocking(False);
    while True:
        e_handling(root, slider, sock, g_events);

        gas = slider.get();
        sock.send("{}\n".format(gas));
        time.sleep(30 / 1000);
        root.update();
    
    sock.close();
    exit(0);


