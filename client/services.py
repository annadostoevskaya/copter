#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import bluetooth
import config as cfg
import json

if __name__ == '__main__':
    service = bluetooth.find_service(address=cfg.TARGET_DEVICE_ADDR);
    print(service); # TODO(annad): Beautify

