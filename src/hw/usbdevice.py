import usb.core
import usb.util
import threading

usb_timed_out = "Operation timed out"

class UsbDevice:
    packet_len = 64

    @staticmethod
    def find(idVendor=0x0b40, idProduct=0x0132, config = 1, write_timeout = 100, read_timeout = 100, write_endpoint = 1, read_endpoint = 0x81):
        dev = usb.core.find(idVendor=idVendor, idProduct=idProduct)

        # was it found?
        if dev is None:
            raise ValueError('Device not found')

        try:
            dev.detach_kernel_driver(0)
        except: # this usually mean that kernel driver has already been dettached
            pass

        # set the active configuration, the device has 2 configurations you can see them
        # by using: lsusb -vvv -d 0x0b40:
        # this device has configurations 1 and 2 I'm using the first one, not sure at the
        # moment if 2nd would work the same
        dev.set_configuration(config)

        return UsbDevice(dev, write_timeout, read_timeout, write_endpoint, read_endpoint)


    def __init__(self, dev, write_timeout = 100, read_timeout = 100, write_endpoint = 1, read_endpoint = 0x81):
        self.dev = dev
        self.write_timeout = write_timeout
        self.read_timeout = read_timeout
        self.write_endpoint = write_endpoint
        self.read_endpoint = read_endpoint

    @classmethod
    def _normalize(cls, packet):
        p_len = len(packet)
        if p_len > cls.packet_len:
            raise ValueError("Packet size too big %d" % p_len)
        else:
            return packet + [0x0]*(cls.packet_len - p_len)

    def _send(self, packet, timeout = None):
        if timeout is None:
            timeout = self.write_timeout
        self.dev.write(self.write_endpoint, packet, 0, timeout)

    def send(self, packet, timeout = None):
        self._send(self._normalize(packet), timeout)

    def read(self, timeout = None):
        if timeout is None:
            timeout = self.read_timeout
        return self.dev.read(self.read_endpoint, self.packet_len, 0, timeout)

    def get_result(self, packet):
        self.send(packet)
        return self.read()


class UsbBackgroundRead:
    def __init__(self, usb, callback, read_timeout = None):
        self.callback = callback
        self.usb = usb
        self.monitor_stop = False
        self.timeout = read_timeout
        self.thread = None

    def start(self):
        self.thread = threading.Thread(name="USB background read", target = self._loop)
        self.thread.start()

    def stop(self):
        if self.thread is None:
            return
        self.monitor_stop = True
        self.thread.join()
        self.thread = None
        self.monitor_stop = False

    def _loop(self):
        while(1):
            if self.monitor_stop:
                break

            #read the result
            try:
                if self.timeout is None:
                    bytes = self.usb.read()
                else:
                    bytes = self.usb.read(self.timeout)

                self.callback(bytes)
            except usb.core.USBError as ex:
                if ex.message != usb_timed_out: #this is a bit dummy, deficiency of PyUSB - it should have both errno and strerror
                    raise ex



if __name__ == "__main__":
    print 'Hello world!'