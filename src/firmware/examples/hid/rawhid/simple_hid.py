#! /usr/local/bin/python3.9

import hid
import time

for device_dict in hid.enumerate():
    keys = list(device_dict.keys())
    keys.sort()
    for key in keys:
        print("%s : %s" % (key, device_dict[key]))
    print()

try:
    print("Opening the device")

    h = hid.device()
    h.open(5824, 1158)  # TREZOR VendorID/ProductID (see prints above to find)

    print("Manufacturer: %s" % h.get_manufacturer_string())
    print("Product: %s" % h.get_product_string())
    print("Serial No: %s" % h.get_serial_number_string())

    # enable non-blocking mode
    h.set_nonblocking(1)

    # write some data to the device
    print("Write the data")
    h.write([1] * 64)

    # wait
    time.sleep(0.001)

    # read back the answer
    print("Read the data")
    while True:
        h.write([1] * 64)
        # wait
        time.sleep(0.001)
        d = h.read(64)
        if d:
            print(d)
        else:
            break

    print("Closing the device")
    h.close()

except IOError as ex:
    print(ex)
    print("You probably don't have the hard-coded device.")
    print("Update the h.open() line in this script with the one")
    print("from the enumeration list output above and try again.")

print("Done")