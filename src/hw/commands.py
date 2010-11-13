def _to_freq(number):
    suffix = ""
    if number > 1000:
        suffix = "kHz"
        number = number/1000

    if number > 1000:
        suffix = "MHz"
        number = number/1000

    return "%.3f%s"%(number, suffix)

class Wait:
    WAIT_ON_SET = 0x80
    WAIT_BIT_MASK = 0x07

class Analog:
    """Relevant bits"""
    CONTROL_MASK = 0x3F

    """Last 4 bits are determining analog input pin number"""
    PIN_MASK = 0x0F

    """Bits 4 and 5 are used as options pins"""
    OPTIONS_MASK = 0x30

    """Use VRef+ (AN3) as positive voltage reference"""
    USE_VREF_PLUS = 0x10

    """Use VRef- (AN2) as negative voltage reference"""
    USE_VREF_MINUS = 0x20

    """Maximum value returned by ADC"""
    MAX = 0x3FF

class Serial:
    """Two last bits indicate serial type"""
    INDICATOR_MASK = 0x03

    """Two last bits 0 indicate SPI"""
    SPI_INDICATOR = 0x00

    """Two last bits 10 indicate UNI/0"""
    UNIO_INDICATOR = 0x02

    """Last bit 1 indicates I2C"""
    I2C_INDICATOR = 0x01

    """Slew Rate - 1 = disabled (100kHz or 1MHz), 0 = enabled (400kHz)"""
    I2C_SLEW = 0x02

    """Only bits 6 and 7 are relevant"""
    SPI_MODE_MASK = 0xC0

    """ Relevant bits 5,4, 1,0 """
    SPI_CONTROL_MASK = 0x33

    """Bit indicating sampling for SPI - 1 = end, 0 = middle"""
    SPI_SAMPLE = 0x80

    """Bit indicating transition for SPI - 1 = active to idle, 0 = idle to active"""
    SPI_TRANSITION = 0x40

    """Bit indicating whether SPI is enabled or not"""
    SPI_ENABLED = 0x20

    """Bit indicating polarity - 1 = idle at high, 0 = idle at low"""
    SPI_CLOCK_POLARITY = 0x10

    """Last 2 bits indicate SPI speed"""
    SPI_SPEED_MASK = 0x03

    """Clock speed = reg.TMR2 / 2"""
    SPI_SPEED_TMR2_2 = 0x03

    """Clock speed = Fo / 64 = 375KHz (expand-IO - Fo=24MHz)"""
    SPI_SPEED_Fo_64 = 0x02

    """Clock speed = Fo / 16 = 1.5MHz (expand-IO - Fo=24MHz)"""
    SPI_SPEED_Fo_16 = 0x01

    """Clock speed = Fo / 4 = 6MHz (expand-IO - Fo=24MHz)"""
    SPI_SPEED_Fo_4 = 0x00

    """Speed indicated by formula: 1.5MHz / Divider (max speed 93.75kHz for expandIO)"""
    UNIO_SPEED_DIVIDER_MIN = 0x0F

    """Bits 7-4 indicate port number"""
    UNIO_PORT_MASK = 0xF0

    """Last 3 bits indicate pin to be used as SCIO"""
    UNIO_BIT_MASK = 0x07

    """expandIO = 24MHz"""
    F0_ExpandIO = 24000000

    """USB-XP = 48MHz"""
    F0_USB_XP = 48000000

    @classmethod
    def get_baude_rate(cls, freq, divider):
        return freq/(4 * (divider + 1))



#this is for PIC18F2455
registers = {
    "ADCON0" : 0xC2,
    "ADCON1" : 0xC1,
    "ADCON2" : 0XC0,
    "ADRESH" : 0xC4,
    "ADRESL" : 0xC3,
    "BAUDCON" : 0xB8,
    "CCP1AS" : 0xB7,
    "CCP1CON" : 0xBD,
    "CCP1DEL" : 0xB6,
    "CCP2CON" : 0xBA,
    "CCPR1H" : 0xBF,
    "CCPR1L" : 0xBE,
    "CCPR2H" : 0xBC,
    "CCPR2L" : 0xBB,
    "CMCON" : 0xB4,
    "CVRCON" : 0xB5,
    "HLVDCON" : 0xD2,
    "INTCON" : 0xF2,
    "INTCON2" : 0xF1,
    "INTCON3" : 0xF0,
    "IPR1" : 0x9F,
    "IPR2" : 0xA2,
    "LATA" : 0x89,
    "LATB" : 0x8A,
    "LATC" : 0x8B,
    "PIE1" : 0x9D,
    "PIE2" : 0xA0,
    "PIR1" : 0x9E,
    "PIR2" : 0xA1,
    "PORTA" : 0x80,
    "PORTB" : 0x81,
    "PORTC" : 0x82,
    "PORTE" : 0x84,
    "PR2" : 0xCB,
    "RCON" : 0xD0,
    "RCREG" : 0xAE,
    "RCSTA" : 0xAB,
    "SPBRG" : 0xAF,
    "SPBRGH" : 0xB0,
    "SSPADD" : 0xC8,
    "SSPBUF" : 0xC9,
    "SSPCON1" : 0xC6,
    "SSPCON2" : 0xC5,
    "SSPSTAT" : 0xC7,
    "STATUS" : 0xD8,
    "T0CON" : 0xD5,
    "T1CON" : 0xCD,
    "T2CON" : 0xCA,
    "T3CON" : 0xB1,
    "TMR0H" : 0xD7,
    "TMR0L" : 0xD6,
    "TMR1H" : 0xCF,
    "TMR1L" : 0xCE,
    "TMR2" : 0xCC,
    "TMR3H" : 0xB3,
    "TMR3L" : 0xB2,
    "TRISA" : 0x92,
    "TRISB" : 0x93,
    "TRISC" : 0x94,
    "TXREG" : 0xAD,
    "TXSTA" : 0xAC
}

def reg_name(id):
    for reg in registers.items():
        if reg[1] == id:
            return reg[0]
    return ''


class Port:
    A = 1
    B = 2
    C = 3
    D = 4
    E = 5
    Names = {1 : "A", 2 : "B", 3 : "C", 4 : "D", 5 : "E"}

devices = {
    0x25 : '2455',
    0x14 : '14K50',
    0x45 : '4455'
}

class Command:
    Null = 0x00
    Error = 0xFF
    GetReg = 0x98
    SetReg = 0x99
    GetBit = 0x9A
    SetBit = 0x9B
    GetPort = 0x9C
    SetPort = 0x9D
    GetPortBit = 0x9E
    SetPortBit = 0x9F
    GetAnalog = 0x96
    SetSerial = 0x93
    ExeSPI = 0xAF
    ExeUNIO = 0xB0
    ExeI2C = 0xA0
    Interrupt = 0x95
    Wait = 0xA9
    ScanMatrix = 0xAA
    CaMultiplex = 0xAB
    CcMultiplex = 0xAE
    MultiplexData = 0xAC
    Stream = 0xAD
    GetFirmware = 0x94

class Direction:
    INPUT = 0x01
    OUTPUT = 0x00

#### start pack
class Null:
    id = Command.Null
    len = 4

    def __init__(self):
        pass

    def pack(self):
        return [0x0]*Null.len

    def __str__(self):
        return "Null"

class GetReg:
    id = Command.GetReg
    len = 4

    def __init__(self, regname):
        self.regname = regname
        self.reg = registers[regname]

    def pack(self):
        return [GetReg.id, self.reg, 0x0, 0x0]

    def __str__(self):
        return "GetReg(%s)" % self.regname

class SetReg:
    id = Command.SetReg
    len = 4

    def __init__(self, regname, value):
        self.regname = regname
        self.reg = registers[regname]
        self.value = value

    def pack(self):
        return [self.id, self.reg, self.value, 0x0]

    def __str__(self):
        return "SetReg(%s) = 0x%X" %(self.regname, self.value)

class SetBit:
    id = Command.SetBit
    len = 4

    def __init__(self, regname, bitno, value):
        self.regname = regname
        self.reg = registers[regname]
        self.bitno = bitno
        self.value = value

    def pack(self):
        return [SetBit.id, self.reg, self.bitno, self.value]

    def __str__(self):
        return "Reg[%s].%i = %i" % (self.regname, self.bitno, self.value)

class GetBit:
    id = Command.GetBit
    len = 4

    def __init__(self, regname, bitno):
        self.regname = regname
        self.reg = registers[regname]
        self.bitno = bitno

    def pack(self):
        return [self.id, self.reg, self.bitno, 0x0]

    def __str__(self):
        return "GetBit[%s].%i" % (self.regname, self.bitno)

class SetPortBit:
    id = Command.SetPortBit
    len = 4

    def __init__(self, port, bitno, value):
        self.port = port
        self.bitno = bitno
        self.value = value

    def pack(self):
        return [SetPortBit.id, self.port, self.bitno, self.value]

    def __str__(self):
        return "Port[%s].%i = %i" % (Port.Names[self.port], self.bitno, self.value)

class GetPortBit:
    id = Command.GetPortBit
    len = 4

    def __init__(self, port, bitno, direction):
        self.port = port
        self.bitno = bitno
        self.direction = direction

    def pack(self):
        return [self.id, self.port, self.bitno, self.direction]

    def __str__(self):
        if self.direction == Direction.INPUT:
            dir = "Input"
        else:
            dir = "Output"

        direction
        return "GetPort[%s].%i as %s" % (Port.Names[self.port], self.bitno, dir)

class GetPort:
    id = Command.GetPort
    len = 4

    def __init__(self, port, mask):
        self.port = port
        self.mask = mask

    def pack(self):
        return [self.id, self.port, 0x0, self.mask]

    def __str__(self):
        return "Port(%s) with 0x%X" % (Port.Names[self.port], self.mask)

class SetPort:
    id = Command.SetPort
    len = 4

    def __init__(self, port, value, direction_mask):
        self.port = port
        self.value = value
        self.mask = direction_mask

    def pack(self):
        return [self.id, self.port, self.value, self.mask]

    def __str__(self):
        return "Port[%s] = 0x%X & 0x%X" % (Port.Names[self.port], self.value, self.mask)

class GetAnalog:
    id = Command.GetAnalog
    len = 4

    def __init__(self, pin, options = 0x0):
        """
        Pass options like Analog.USE_VREF_PLUS | Analog.USE_VREF_MINUS
        """
        self.pin = pin & Analog.PIN_MASK
        self.options = options

    def pack(self):
        return [self.id, self.pin | self.options, 0x0, 0x0]

    def __str__(self):
        return "GetAnalog[%i] with %s and %s" % (self.pin, self.mask)

class Wait:
    id = Command.Wait
    len = 4

    def __init__(self, regname, pin, value, timeout):
        self.regname = regname
        self.reg = registers[regname]
        self.pin = pin & 0x07 #only bits 0-2 relevant
        self.value = value & 0x01 #only bit 0 relevant
        self.timeout = timeout

    def pack(self):
        return [self.id, self.reg, self.pin | (self.value << 7), 0x0]

    def __str__(self):
        return "Wait[%s].%i = %i for %i ms" % (self.regname, self.pin, self.value, self.timeout)

class GetFirmware:
    id = Command.GetFirmware
    len = 4

    def __init__(self):
        pass

    def pack(self):
        return [GetFirmware.id, 0x0, 0x0, 0x0]

    def __str__(self):
        return "GetFirmware"



###### end pack
######  unpack
class Reader:
    @classmethod
    def can_handle(cls, packet):
        return cls.id == packet[0]


class FirmwareUnpack(Reader):
    id = Command.GetFirmware
    len = 4

    def __init__(self, packet):
        #maybe assert [0] is our id
        self.device = packet[1]
        self.version = (packet[2] << 8) | packet[3]

    def __str__(self):
        return "Firmware(%s, %d)" %(devices.get(self.device, "0x%X" % self.device), self.version)

class NullUnpack(Reader):
    id = Command.Null
    len = 4

    def __init__(self, packet):
        self.all_null = False
        self.len = 0
        while len(packet) > 0 and self.can_handle(packet):
            packet = packet[self.len:]
            self.len += 4
        if self.len == 64:
            self.all_null = True

    def __str__(self):
        if self.all_null:
            return "Null"
        else:
            return ""


class ErrorUnpack(Reader):
    id = Command.Error
    len = 4

    def __init__(self, packet):
        self.byte1 = packet[1]
        self.byte2 = packet[2]
        self.byte3 = packet[3]

    def __str__(self):
        return "Error(0x%X 0x%X 0x%X)" %(self.byte1, self.byte2, self.byte3)

class SetPortBitUnpack(Reader):
    id = Command.SetPortBit
    len = 4

    def __init__(self, packet):
        self.port = packet[1]
        self.bitno = packet[2]
        self.value = packet[3]

    def __str__(self):
        return "Done: Port[%s].%i = %i" % (Port.Names[self.port], self.bitno, self.value)

class GetPortBitUnpack(Reader):
    id = Command.GetPortBit
    len = 4

    def __init__(self, packet):
        self.port = packet[1]
        self.bitno = packet[2]
        self.value = packet[3]

    def __str__(self):
        return "Port[%s].%i = %i" % (Port.Names[self.port], self.bitno, self.value)

class SetBitUnpack(Reader):
    id = Command.SetBit
    len = 4

    def __init__(self, packet):
        self.reg = packet[1]
        self.bitno = packet[2]
        self.value = packet[3]

    def __str__(self):
        return "Done: Reg[%s].%i = %i" % (reg_name(self.reg), self.bitno, self.value)

class GetBitUnpack(Reader):
    id = Command.GetBit
    len = 4

    def __init__(self, packet):
        self.reg = packet[1]
        self.bitno = packet[2]
        self.value = packet[3]

    def __str__(self):
        return "Reg(%s).%i = %i" %(regname(self.reg), self.bitno, self.value)

class GetRegUnpack(Reader):
    id = Command.GetReg
    len = 4

    def __init__(self, packet):
        self.reg = packet[1]
        self.value = packet[2]

    def __str__(self):
        return "GetReg(%s) = 0x%X" % (reg_name(self.reg), self.value)

class SetRegUnpack(Reader):
    id = Command.SetReg
    len = 4

    def __init__(self, packet):
        self.reg = packet[1]
        self.value = packet[2]

    def __str__(self):
        return "Done SetReg(%s) = 0x%X" %(regname(self.reg), self.value)

class GetPortUnpack(Reader):
    id = Command.GetPort
    len = 4

    def __init__(self, packet):
        self.port = packet[1]
        self.value = packet[2]
        self.mask = packet[3]

    def __str__(self):
        return "Port(%s) = 0x%X & 0x%X" % (Port.Names[self.port], self.value, self.mask)

class SetPortUnpack(Reader):
    id = Command.SetPort
    len = 4

    def __init__(self, packet):
        self.port = packet[1]
        self.value = packet[2]
        self.mask = packet[3]

    def __str__(self):
        return "Done SetPort(%s) = 0x%X & 0x%X" % (Port.Names[self.port], self.value, self.mask)


class GetAnalogUnpack(Reader):
    id = Command.GetAnalog
    len = 4

    def __init__(self, packet):
        self.pin = packet[1] & Analog.PIN_MASK
        self.use_vref_plus = (packet[1] & Analog.USE_VREF_PLUS) != 0
        self.use_vref_minus = (packet[1] & Analog.USE_VREF_MINUS) != 0
        self.value = (packet[2] << 8) | packet[3]

    def __str__(self):
        if self.use_vref_plus:
            vref = "VRef+ "
        else:
            vref = "AVdd"

        if self.use_vref_minus:
            vref = "(" + vref + "- Vref-) "
            vref_add = "Vref- + "
        else:
            vref_add = ""

        return "Analog(AN%d) = %s%s*(0x%X/0x%X)" % (self.pin, vref_add, vref, \
                                                    self.value, Analog.MAX)

class SetSerialUnpack(Reader):
    id = Command.SetSerial
    len = 4

    def __init__(self, packet):
        self._unpack_type(packet[1])
        self._unpack_config(packet)


    def _unpack_type(self, byte):
        self.spi = (packet[1] & Serial.INDICATOR_MASK) == Serial.SPI_INDICATOR
        self.unio = (packet[1] & Serial.INDICATOR_MASK) == Serial.UNIO_INDICATOR
        self.i2c = (packet[1] & Serial.I2C_INDICATOR) != 0

    def _unpack_config(self, packet):
        if self.spi:
            self._unpack_spi(packet)
        elif self.unio:
            self._unpack_unio(packet)
        elif self.i2c:
            self._unpack_i2c(packet)
        else:
            raise ValueError('Unknown serial packet type! %s', str(packet))

    def _unpack_spi(self, packet):
        self.mode = (packet[1] & Serial.SPI_MODE_MASK) >> 6
        self.enabled = (packet[2] & Serial.SPI_ENABLED) != 0
        self.polarity_high = (packet[2] & Serial.SPI_CLOCK_POLARITY) != 0
        self.speed = packet[2] & Serial.SPI_SPEED

    def _unpack_unio(self, packet):
        self.speed = packet[2]
        self.port = (packet[3] & Serial.UNIO_PORT_MASK) >> 4
        self.scio_pin = (packet[3] & Serial.UNIO_BIT_MASK)

    def _unpack_i2c(self, packet):
        self.slew_rate_on = (packet[1] & Serial.I2C_SLEW) == 0
        self.speed = packet[2]

    def __str__(self):
        if self.spi:
            if self.mode & 0x02 > 0:
                sample = "Sample End"
            else:
                sample = "Sample Mid"

            if self.mode & 0x01 > 0:
                tran = "Transition A->I"
            else:
                tran = "Transition I->A"

            if self.polarity_high:
                pol = "Hight"
            else:
                pol = "Low"

            if self.enabled:
                en = "Enabled"
            else:
                en = "Disabled"

            # do speed
            if self.speed == Serial.SPI_SPEED_TMR2_2:
                speed = "TMR2/2"
            elif self.speed == Serial.SPI_SPEED_Fo_64:
                speed = _to_freq(Serial.F0_ExpandIO / 64)
            elif self.speed == Serial.SPI_SPEED_Fo_16:
                speed = _to_freq(Serial.F0_ExpandIO / 16)
            elif self.speed == Serial.SPI_SPEED_Fo_4:
                speed = _to_freq(Serial.F0_ExpandIO / 4)

            message = "SPI(%s, %s %s, Polarity %s) @ %s" % (en, sample, tran, pol, speed)
        elif self.unio:
            if self.speed == 0:
                freq = "unknown"
            else:
                freq = _to_freq(3000000/2/self.speed)

            message = "UNI/O(%d.%d) @ %s" % (self.port, self.scio_pin, freq)
        elif self.i2c:
            if self.slew_rate_on:
                slew = "ON"
            else:
                slew = "OFF"

            message = "I2C (Slew %s) @ %d " % (slew, Serial.get_baude_rate(Serial.F0_ExpandIO, self.speed))
        else:
            message = "Unknown"

        return "Done SetSerial: " + message


class ExeSpiUnpack(Reader):
    id = Command.ExeSPI

    def __init__(self, packet):
        self.payload_length = packet[1]
        self.len = self.payload_length + 2
        self.payload = packet[2:self.payload_lengt + 2]

    def __str__(self):
        return "Done SendSPI(%d) = %s" % (self.payload_length, str(self.payload))

class ExeUNIOUnpack(Reader):
    id = Command.ExeUNIO
    len = 64 #I'm still not sure how do you determine response size if you don't do request response
             # code using this has to handle that

    def __init__(self, packet):
        self.status = packet[1]
        self.response = packet[2:]

    def __str__(self):
        return "Done ExecUNIO(%d) = %s" % (self.status, str(self.response))

class ExeI2CUnpack(Reader):
    id = Command.ExeI2C
    len = 64 #I'm still not sure how do you determine response size if you don't do request response
             # code using this has to handle that

    def __init__(self, packet):
        self.status = packet[1]
        self.response = packet[2:]

    def __str__(self):
        return "Done ExecI2C(%d) = %s" % (self.status, str(self.response))

class InterruptUnpack(Reader):
    id = Command.Interrupt
    len = 4

    def __init__(self, packet):
        #nothing to do for interrupt, state of the controller has to be queried
        pass

    def __str__(self):
        return "Interrupt"

class WaitUnpack(Reader):
    id = Command.Wait
    len = 4

    def __init__(self, packet):
        self.reg = packet[1]
        self.bit = packet[2] & Wait.WAIT_BIT_MASK
        self.wait_on_set = (packet[2] & Wait.WAIT_ON_SET) != 0
        self.remaining = packet[3]

    def __str__(self):
        if self.wait_on_set:
            set = "SET"
        else:
            set = "CLEARED"

        if self.remaining == 0:
            timeout = "Timeout Occured!"
        else:
            timeout = ""

        return "Wait(%s).%d %s Done %s" % (reg_name(self.reg), self.bit, set, timeout)


###### end unpack

Command.packers = [Null, GetReg, SetBit, SetPortBit, GetFirmware, GetPortBit, \
                   SetReg, GetBit, GetPort, SetPort, GetAnalog, Wait]
Command.unpackers = [FirmwareUnpack, NullUnpack, ErrorUnpack, GetPortBitUnpack,\
                     SetPortBitUnpack, GetBitUnpack, SetBitUnpack, GetRegUnpack,\
                     SetRegUnpack, GetPortUnpack, SetPortUnpack, GetAnalogUnpack, \
                     SetSerialUnpack, ExeSpiUnpack, ExeUNIOUnpack, ExeI2CUnpack, \
                     InterruptUnpack, WaitUnpack]

if __name__ == "__main__":
    print "Hello World"
