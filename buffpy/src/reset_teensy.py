#!/usr/bin/env python3

import hid

teensy_vid = 0x16C0
teensy_pid = 0x0486

with hid.Device(teensy_vid, teensy_pid) as device:
    buffer = [0x4]
    device.write(bytes(buffer))
    device.close()