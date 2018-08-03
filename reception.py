
#!/usr/bin/python

import glob
import serial
import sys
import platform

DEBUG_PRINT = 0

class Reception:

    ###############################################################################
    def __init__(self):
        if DEBUG_PRINT: print("init")
        self.port = self.serial_init()


    ###############################################################################
    def serial_init(self):
        PLATFORM = platform.system()
        if "Linux" in PLATFORM:
            SERIAL_PATH = "/dev/ttyUSB*"
        elif "Darwin" in PLATFORM:
            SERIAL_PATH = "/dev/tty.usb*"   # TODO: test it
        else: # Windows
            self.port = serial.Serial('COM5', 115200 * 2)
            SERIAL_PATH = 'WIN_WORKARAOUND'

        if SERIAL_PATH != 'WIN_WORKARAOUND':
            devices = glob.glob(SERIAL_PATH)
            self.port = serial.Serial(devices[0], 115200 * 2)

        success = self.port.isOpen()
        if success:
            if DEBUG_PRINT: print("Port open.")
            self.lookForHeader()
        else:
            print("\n!!! Error: serial device not found !!!")
            exit(-1)
        return self.port


    ###############################################################################
    def lookForHeader(self):
        if DEBUG_PRINT: print("seeking header\n")

        # packets structure:
        # 2 headers + 1 base_axis + (4 photodiodes * 2 bytes) + (3 accel * 2 bytes)

        while True:

            while self.readByte() != 255:
                pass # consume

            if self.readByte() != 255:
                continue

            break


    ###############################################################################
    def readByte(self):
        byte = ord(self.port.read(1))
        if DEBUG_PRINT:
            print(byte)
        return byte


    ###############################################################################
    def parse_data(self):
        centroidNum = 4
        accelerationNum = 3

        base_axis = self.readByte()
        base = (base_axis >> 1) & 1
        axis = (base_axis >> 0) & 1

        if DEBUG_PRINT: print("\nbase, axis:", base, axis)

        centroids = [0 for i in range(centroidNum)]

        for i in range(centroidNum):
            centroids[i] = self.decodeTime()
            if DEBUG_PRINT: print("centroids[", i, "] =", centroids[i])

        accelerations = [0 for i in range(accelerationNum)]
        for i in range(accelerationNum):
            accelerations[i] = self.decodeAccel()
            if DEBUG_PRINT: print("accelerations[", i, "] =", accelerations[i])

        # consumes header
        for i in range(2):
            b = self.readByte()
            if (b != 255):
                if DEBUG_PRINT: print("header problem", i)
                self.lookForHeader()
                break

        return base, axis, centroids, accelerations


    ###############################################################################
    def decodeTime(self):
        rxl = self.readByte()        # LSB first
        rxh = self.readByte()        # MSB last
        time = (rxh << 8) + rxl     # reconstruct packets
        time <<= 2                  # (non-significant) lossy decompression
        time /= 16.                 # convert to us

        if (time >= 6777 or time < 1222):
            time = 0
            if DEBUG_PRINT: print("INVALID TIME")

        return time


    ###############################################################################
    # github.com/adafruit/Adafruit_BNO055/blob/master/Adafruit_BNO055.cpp#L359-L361
    def decodeAccel(self):
        rxl = self.readByte()        # LSB first
        rxh = self.readByte()        # MSB last
        accel = (rxh << 8) + rxl    # reconstruct packets

        gravity = 9.81
        accel /= (1<<15) / (4.0 * gravity)  # normalize with max amplitude
        accel -= gravity * 2.0              # remove max negative value (-2g)...
                                            # ...to stay positive during tx
        return accel


###############################################################################
# This is just for debug, or to use as example
if __name__ == "__main__":

    if DEBUG_PRINT:
        print("default main")

    # create an object (it also does the port initialization)
    rx = Reception()

    while True:
        # base = 0 or 1 (B or C)
        # axis = 0 or 1 (horizontal or vertical)
        # centroids = array of 4 floats in microseconds
        # accelerations = array of 3 floats in G (AKA m/s^2)
        base, axis, centroids, accelerations = rx.parse_data()

        if not DEBUG_PRINT:
            print( base, axis, centroids, accelerations )
