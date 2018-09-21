# is31fl3218.py - driver for the I2C based ISSI 31fl3218 LED controller

"""This module allows driving the I2C LED controller"""
import smbus


class Sensor(object):
    """Sensor([bus]) -> Sensor
    Return a new is31fl3218 object that is connected to the
    specified I2C device interface.
    """
    REG_ADDR_SHUTDOWN = 0x00
    REG_ADDR_PWM = tuple(range(0x01, 0x13))
    REG_ADDR_CTRL = tuple(range(0x013, 0x16))
    REG_ADDR_UPDATE = 0x16
    REG_ADDR_RESET = 0x17
    REG_ADDR_LAST = REG_ADDR_RESET
    _bus = -1
    _debug = False
    _i2c_addr = 0b1010100

    def __init__(self, bus=0, debug=False):
        # 0 = /dev/i2c-0 (port I2C0), 1 = /dev/i2c-1 (port I2C1), etc
        if debug:
            print "using bus {0}, address {1}".format(bus, self._i2c_addr)
        self._bus = smbus.SMBus(bus)
        self._debug = debug

    def close(self):
        """close()
        Disconnects the object from the bus.
        """
        self._bus.close()
        self._bus = -1

    def __gamma_correct(self, duty):
        gamma_correction = (0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 20,
                            22, 24, 26, 29, 32, 35, 38, 41, 44, 47, 50, 53, 57,
                            61, 65, 69, 73, 77, 81, 85, 89, 94, 99, 104, 109,
                            114, 119, 124, 129, 134, 140, 146, 152, 158, 164,
                            170, 176, 182, 188, 195, 202, 209, 216, 223, 230,
                            237, 244, 251, 255)
        if duty > 100:
            duty = 100
        duty = duty * 63 / 100
        return gamma_correction[int(duty)]

    def __write_registers(self, reg_addr, values):
        """ this takes advantage of the  auto increment feature of the
        controller
        """
        if reg_addr+len(values) > self.REG_ADDR_LAST:
            raise IOError("Writing {0} bytes from reg {1} would go over the last register".format(len(values), reg_addr))

        self._bus.write_i2c_block_data(self._i2c_addr, reg_addr, values)

    def shutdown(self, off=True):
        """shutdown()
        enables/disable the LED controller at software level. You first need to have SDB pin high to enable in software.
        When SDB is high, you still need to set shutdown to false to be able to control the LEDs/
        """
        reg_val = 0
        if not off:
            reg_val = 0xff

        self.__write_registers(self.REG_ADDR_SHUTDOWN, [reg_val])

    def set_led(self, led, duty, immediately=True):
        """set_led()
        sets the duty cycle for a specific led. If immediately is False, the
        value is only stored in the controller but not applied to the LED until
        an update command is sent
        """
        if led >= len(self.REG_ADDR_PWM):
            raise IOError("Invalid LED n.{0}".format(led))
        gamma_corrected_dc = self.__gamma_correct(duty[0])
        self.__write_registers(self.REG_ADDR_PWM[led], [gamma_corrected_dc])
        if immediately:
            self.__write_registers(self.REG_ADDR_UPDATE, [1])

    def set_multiple_leds(self, first_led, duty_list, immediately=True):
        """set_multiple_leds()
        sets the duty cycle for multiple leds. duty_list needs to contain less
        than 18 items. If immediately is False, the values are only stored in
        the controller but not applied to the LED until a update() is called
        """
        if first_led+len(duty_list) > len(self.REG_ADDR_PWM):
            raise IOError("Setting {0} LEDS from {1} would go over the last LED".format(len(duty_list), first_led))
        gamma_corrected_dc_list = map(self.__gamma_correct, duty_list)
        self.__write_registers(self.REG_ADDR_PWM[first_led], gamma_corrected_dc_list)
        if immediately:
            self.__write_registers(self.REG_ADDR_UPDATE, [1])

    def enable_leds(self, ena_list):
        """enable_leds()
        enables/disables all leds. ena_list needs to contain exactly 18 items
        """
        if len(ena_list) != len(self.REG_ADDR_PWM):
            raise ValueError("ena_list needs to have {0} items".format(
                                len(self.REG_ADDR_PWM)))
        # each register only uses the 6 LSB to represent 6 LED pwm channels
        ena_bytes = [0, 0, 0]
        for idx in range(0, 6):
            if ena_list[idx]:
                ena_bytes[0] |= 1 << idx
            if ena_list[6+idx]:
                ena_bytes[1] |= 1 << idx
            if ena_list[12+idx]:
                ena_bytes[2] |= 1 << idx
        # send the actual enable bitmap
        self.__write_registers(self.REG_ADDR_CTRL[0], ena_bytes)

    def update(self):
        """update()
        Applies the duty cycles stored in the controller to the LED outputs
        """
        self.__write_registers(self.REG_ADDR_UPDATE, [1])
