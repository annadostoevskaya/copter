#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import bluetooth
import config as cfg

if __name__ == '__main__':
    devices = bluetooth.discover_devices(lookup_names=True);
    print("client: Found {} devices.".format(len(devices)));

    target_device = '';
    for addr, name in devices:
       if (addr == cfg.TARGET_DEVICE_ADDR):
           target_device = addr;
           print('client:  * {} - {}'.format(addr, name));
           continue
       print('client:  {} - {}'.format(addr, name));

    if (not target_device):
        print('client: 404 target device!');
        exit(-1);