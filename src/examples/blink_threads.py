import time

from hw.commands import *
from hw.usbdevice import *

def handle_packet(packet):
    print Command.unpack(packet)

#use all defaults since that's the device I have
dev = UsbDevice.find()

#start a thread to listen to messages from the device
read_thread = UsbBackgroundRead(dev, handle_packet)
read_thread.start()

#show firmware and setup device
dev.send(GetFirmware().pack())
dev.send(SetBit("TRISC", 6, 0).pack())

#blinking - send 0 to turn the led on, 1 for off
for i in range(10):
    dev.send(SetPortBit(Port.C, 6, 0).pack())
    time.sleep(1)
    dev.send(SetPortBit(Port.C, 6, 1).pack())
    time.sleep(1)

read_thread.stop()