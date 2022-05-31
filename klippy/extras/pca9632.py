# PCA9632 LED controller on i2c
#
# Copyright (C) 2021 Jacob Dockter <dockterj@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from . import bus
from mcp4018 import SoftwareI2C

#note that the address is actually 0b11000100 but the i2c library expects
#     7 bit addresses.  We can't read from the chip (requires 0b11000101)
PCA9632_I2C_ADDR = 0b1100010

LED_AUTO_INC_ALL = 0b10000000
# Set up the control byte to start with reg 0 and autoincrement through all
PCA9632CONTROL = [0x00 | LED_AUTO_INC_ALL]
# Set up the default data to turn off blink and set PWM to 0 for all LEDs
# (blink duty cycle set to 50% - GRPPWM)
PCA9632DATA = [ 0b00000001,                       # reg0 - responds to all call
                                                  #    bus address
            0b00100000 | 0b00010000 | 0b00000100, # reg1 - grp control blink,
                                                  #    out inverted, totem pole
            0,                                    # reg2 - led0
            0,                                    # reg3 - led1
            0,                                    # reg4 - led2
            0,                                    # reg5 - led3
            128,                                  # reg6 - blink duty cycle
            0,                                    # reg7 - blink frequency
            0b10101010]                           # reg8 - led control (set 
                                                  #    all 4 to individual 
                                                  #    pwm control)

BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000

class PCA9632:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        # Configure PCA9632
        if config.get("scl_pin", 0) == 0:
            # hardware I2C
            self.i2c = bus.MCU_I2C_from_config(
                config, default_addr=PCA9632_I2C_ADDR, default_speed=100000)        
        else:
            # software I2C
            self.i2c = SoftwareI2C(config, PCA9632_I2C_ADDR)
        # color order
        formats = {v: v for v in ["RBG", "BGR"]}
        self.color_order = config.getchoice("color_order", formats, "RBG")
        # Initial Color
        blue = config.getfloat("initial_BLUE",0.,minval=0.,maxval=1.)
        green = config.getfloat("initial_GREEN",0.,minval=0.,maxval=1.)
        red = config.getfloat("initial_RED",0.,minval=0.,maxval=1.)
        white = config.getfloat("initial_WHITE",0.,minval=0.,maxval=1.)
        blink = config.getfloat("initial_BLINK",0.,minval=0.,maxval=1.)
        self.update_color_data(red, green, blue, white, blink)
        # Register commands
        self.printer.register_event_handler("klippy:connect",
                                            self.send_data)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command("SET_LED", "LED", self.name,
                                    self.cmd_SET_LED,
                                    desc=self.cmd_SET_LED_help)
    def update_color_data(self, red, green, blue, white, blink):
        red = int(red * 255. + .5)
        green = int(green * 255. + .5)
        blue = int(blue * 255. + .5)
        white = int(white * 255. + .5)
        blink = int(blink * 255. + .5)
        if self.color_order == "RBG":
            self.led0 = red
            self.led1 = blue
            self.led2 = green
            self.led3 = 0
            self.led_blink = blink
        elif self.color_order == "BGR":
            self.led0 = blue
            self.led1 = green
            self.led2 = red
            self.led3 = 0
            self.led_blink = blink
    def send_data(self, print_time=None):
        minclock = 0
        if print_time is not None:
            minclock = self.i2c.get_mcu().print_time_to_clock(print_time)
        data = PCA9632DATA
        data[2] = self.led0
        data[3] = self.led1
        data[4] = self.led2
        data[5] = self.led3
        data[7] = self.led_blink
        if self.led_blink > 0:
            data[8] = 0b11111111 # reg8 - led control (set all to individual 
            #                             pwm and group blink control)
        else:
            data[8] = 0b10101010 # reg8 - led control (set all to individual
            #                             pwm and no blink)
        self.i2c.i2c_write((PCA9632CONTROL + data), minclock=minclock,
                          reqclock=BACKGROUND_PRIORITY_CLOCK)
    cmd_SET_LED_help = "Set the color of an LED.  SET_LED LED=my_pca9632 "
    cmd_SET_LED_help += "RED=0.1 GREEN=0.1 BLUE=0.1 WHITE=0.1 BLINK=0.5"
    def cmd_SET_LED(self, gcmd):
        # Parse parameters
        red = gcmd.get_float('RED', 0., minval=0., maxval=1.)
        green = gcmd.get_float('GREEN', 0., minval=0., maxval=1.)
        blue = gcmd.get_float('BLUE', 0., minval=0., maxval=1.)
        white = gcmd.get_float('WHITE', 0., minval=0., maxval=1.)
        blink = gcmd.get_float('BLINK', 0., minval=0., maxval=1.)
        self.update_color_data(red, green, blue, white, blink)
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback(self.send_data)
def load_config_prefix(config):
    return PCA9632(config)
