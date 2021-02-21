#!/usr/bin/python3
""" DIYHA clocks
    Display time and interesting LED matrix while sending motion to MQTT message
    broker. Monitor alarm status and create audible and visible warnings.
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

import os
import time
import logging
import logging.config

# imported third party classes

import paho.mqtt.client as mqtt
from Adafruit_LED_Backpack import BicolorMatrix8x8

# imported clocks classes

from pkg_classes.mqttlocationtopic import MqttLocationTopic
from pkg_classes.ledclock import LedClock
from pkg_classes.led8x8controller import Led8x8Controller
from pkg_classes.alarmcontroller import AlarmController
from pkg_classes.motioncontroller import MotionController
from pkg_classes.intervaltimer import IntervalTimer

#imported constants

from pkg_classes.ledclock import TIME_MODE, WHO_MODE
from pkg_classes.led8x8controller import IDLE_STATE, DEMO_STATE, SECURITY_STATE
from pkg_classes.led8x8controller import FIRE_MODE, PANIC_MODE, FIBONACCI_MODE

# Constants for GPIO pins and the I2C bus for the 8x8 matrix LED

MOTION_GPIO = 24
ALARM_GPIO = 4
MATRIX_I2C_ADDRESS = 0x70


# Seven Segment LED Clock constants

#TIME_MODE = 0
#WHO_MODE = 1
#COUNT_MODE = 2

# Start logging and enable imported classes to log appropriately.

logging.config.fileConfig(fname='/home/an/clocks/logging.ini',
                          disable_existing_loggers=False)
LOGGER = logging.getLogger("clocks")
LOGGER.info('Application started')

# Location provided by MQTT broker at runtime and managed by this class.

TOPIC = MqttLocationTopic() # Location MQTT topic

CLOCK = LedClock() # Seven segment LED backpack from Adafruit
CLOCK.run()

DISPLAY = BicolorMatrix8x8.BicolorMatrix8x8(address=MATRIX_I2C_ADDRESS)
DISPLAY.begin()

MATRIX = Led8x8Controller(DISPLAY) # 8x8 LED matrix from Adafruit
MATRIX.run()

ALARM = AlarmController(ALARM_GPIO) # Alarm or light controller
ALARM.sound_alarm(False)

# Start timer which controls LED devices based on time of day

TIMER = IntervalTimer(CLOCK, MATRIX)

# Process MQTT messages using a dispatch table algorithm.

#pylint: disable=too-many-branches

def system_message(client, msg):
    """ Log and process system messages. """
    LOGGER.info(msg.topic+" "+msg.payload.decode('utf-8'))
    if msg.topic == 'diy/system/fire':
        if msg.payload == b'ON':
            MATRIX.set_mode(FIRE_MODE)
            ALARM.sound_alarm(True)
        else:
            MATRIX.set_mode(FIBONACCI_MODE, True)
            ALARM.sound_alarm(False)
    elif msg.topic == 'diy/system/panic':
        if msg.payload == b'ON':
            MATRIX.set_mode(PANIC_MODE)
            ALARM.sound_alarm(True)
        else:
            MATRIX.set_mode(FIBONACCI_MODE, True)
            ALARM.sound_alarm(False)
    elif msg.topic == 'diy/system/who':
        if msg.payload == b'ON':
            CLOCK.set_mode(WHO_MODE)
        else:
            CLOCK.set_mode(TIME_MODE)
    elif msg.topic == 'diy/system/demo':
        if msg.payload == b'ON':
            TIMER.control_lights("Turn On")
            MATRIX.set_state(DEMO_STATE)
        else:
            TIMER.control_lights("Turn Off")
            MATRIX.set_state(IDLE_STATE)
    elif msg.topic == 'diy/system/security':
        if msg.payload == b'ON':
            MATRIX.set_state(SECURITY_STATE)
        else:
            TIMER.control_lights("Turn On")
            MATRIX.set_state(IDLE_STATE)
    elif msg.topic == 'diy/system/silent':
        if msg.payload == b'ON':
            TIMER.control_lights("Turn Off")
            MATRIX.set_state(IDLE_STATE)
        else:
            TIMER.control_lights("Turn On")
            MATRIX.set_state(DEMO_STATE)
    elif msg.topic == TOPIC.get_setup():
        topic = msg.payload.decode('utf-8') + "/motion"
        TOPIC.set(topic)
    if msg.topic == 'diy/system/who':
        if msg.payload == "ON":
            if not TOPIC.waiting_for_location:
                client.publish(TOPIC.get_status(), TOPIC.get_location(), 0, True)

#pylint: disable=unused-argument

def topic_message(client, msg):
    """ Set the sensors location topic. Used to publish measurements. """
    LOGGER.info(msg.topic+" "+msg.payload.decode('utf-8'))
    topic = msg.payload.decode('utf-8') + "/motion"
    TOPIC.set(topic)


#  A dictionary dispatch table is used to parse and execute MQTT messages.

TOPIC_DISPATCH_DICTIONARY = {
    "diy/system/demo":
        {"method":system_message},
    "diy/system/fire":
        {"method":system_message},
    "diy/system/panic":
        {"method":system_message},
    "diy/system/security":
        {"method":system_message},
    "diy/system/silent":
        {"method":system_message},
    "diy/system/who":
        {"method":system_message},
    TOPIC.get_setup():
        {"method":topic_message}
    }


def on_message(client, userdata, msg):
    """ dispatch to the appropriate MQTT topic handler """
    #pylint: disable=unused-argument
    if "motion" in msg.topic:
        MATRIX.update_motion(msg.topic)
    else:
        TOPIC_DISPATCH_DICTIONARY[msg.topic]["method"](client, msg)


def on_connect(client, userdata, flags, rc_msg):
    """ Subscribing in on_connect() means that if we lose the connection and
        reconnect then subscriptions will be renewed.
    """
    #pylint: disable=unused-argument
    client.subscribe("diy/system/demo", 1)
    client.subscribe("diy/system/fire", 1)
    client.subscribe("diy/system/panic", 1)
    client.subscribe("diy/system/security", 1)
    client.subscribe("diy/system/silent", 1)
    client.subscribe("diy/system/who", 1)
    client.subscribe(TOPIC.get_setup(), 1)
    client.subscribe("diy/+/+/motion", 1)


def on_disconnect(client, userdata, rc_msg):
    """ Subscribing on_disconnect() tilt """
    #pylint: disable=unused-argument
    client.connected_flag = False
    client.disconnect_flag = True


if __name__ == '__main__':

    # Setup MQTT handlers then wait for timed events or messages

    CLIENT = mqtt.Client()
    CLIENT.on_connect = on_connect
    CLIENT.on_disconnect = on_disconnect
    CLIENT.on_message = on_message

    # NOTE: Environment variable contains Mosquitto IP address.

    BROKER_IP = os.environ.get('MQTT_BROKER_IP')

    CLIENT.connect(BROKER_IP, 1883, 60)
    CLIENT.loop_start()

    # Message broker will send the location and set waiting to false.

    while TOPIC.waiting_for_location:
        time.sleep(5.0)

    MOTION = MotionController(MOTION_GPIO)
    MOTION.enable()

    # Loop forever checking for timed events every 10 seconds.

    while True:
        time.sleep(10.0)
        if MOTION.detected():
            CLIENT.publish(TOPIC.get_location(), MOTION.get_motion(), 0, True)
        TIMER.check_for_timed_events()
