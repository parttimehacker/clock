#!/usr/bin/python3
""" DIYHA Motion Controller:
    Detect PIR motion and create a queue to manage them.
"""

# The MIT License (MIT)
#
# Copyright (c) 2019 parttimehacker@gmail.com
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import queue

from Adafruit_GPIO import GPIO

class MotionController:
    """ Motion detection device driver """

    def __init__(self, pin):
        """ Setup interrupts on the pGPIO pin and the motion queue. """
        self.gpio = GPIO.get_platform_gpio()
        self.queue = queue.Queue()
        self.pin = pin
        self.gpio.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        self.last_reading = 0

    def pir_interrupt_handler(self, gpio):
        """ Motion interrupt handler adds 1 or 0 to queue. """
        state = self.gpio.input(gpio)
        if state == 1:
            message = "1"
        else:
            message = "0"
        if state != self.last_reading:
            self.queue.put(message)
        self.last_reading = state

    def enable(self,):
        """ Enable interrupts and prepare the callback. """
        self.gpio.add_event_detect(self.pin, GPIO.RISING, callback=self.pir_interrupt_handler)

    def detected(self,):
        """ Has motion been detected? True or false based on queue contents. """
        return not self.queue.empty()

    def get_motion(self,):
        """ Return the last value either 1 or 0. """
        return self.queue.get(False)

    def wait_for_motion(self,):
        """ Blocking wait for the next interrupt 1 or 0. """
        return self.queue.get(True)