# basic robot version 2.0
#   This is the main control script. Funtions are found in included files
#   Internet/HTTP controlled basic robot
#   RJM
#
#
# most functionality now moved into the NodeBot class found in nodebot.py
# frees up area to play with actual actions in main code
# TODO: add i2c for screen (done it before i'm sure)
#       add interaction with twitter bot
#       test all interactions
#
#

from machine import I2C, Pin
from time import sleep
import ssd1306
from nodebot import NodeBot
from myESP import *

NETWORK="MYNETWORK"
PASS="MYPASS"



def main():
    i2c_scl=12
    i2c_sda=14
    # main function
    sleep(2)
    # bot(drivepin, ledw, ledb, servo, buzzer)
    bot=NodeBot(15, 16, 5, 4, 0)
    bot.blink("w", 2, 1)
    if connect_network(NETWORK, PASS):
        bot.blink("w", 20, 0.1)
    else:
        bot.blink("b", 5, 1)
        while 1:
            sleep(1)
    i2c=I2C(scl=Pin(i2c_scl), sda=Pin(i2c_sda), freq=400000)
    screen=start_screen(128, 32, i2c)

    listen_for_command()        # pass pins so can continue execution from listen loop
