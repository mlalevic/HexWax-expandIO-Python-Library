import time

from hw.commands import *
from hw.usbdevice import *

#use all defaults since that's the device I have
dev = UsbDevice.find()

#setup device
def show_firmware():
    #cmd is used for printing, otherwise you could've done: print GetFirmwareUnpack(dev.get_result(GetFirmware().pack()))
    cmd = GetFirmware()
    print cmd
    print FirmwareUnpack(dev.get_result(cmd.pack()))

def prepare_port():
    cmd = SetBit("TRISC", 6, 0)
    print cmd
    print SetBitUnpack(dev.get_result(cmd.pack()))

def set_bit(value):
    cmd = SetPortBit(Port.C, 6, value)
    print cmd
    print SetPortBitUnpack(dev.get_result(cmd.pack()))

show_firmware()
prepare_port()

#blinking - send 0 to turn the led on, 1 for off
for i in range(10):
    set_bit(0)
    time.sleep(1)
    set_bit(1)
    time.sleep(1)
