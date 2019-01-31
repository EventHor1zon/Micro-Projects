from machine import PWM, Pin
from time import sleep, sleep_ms


class NodeBot:
    #""" implements several robotic features on the ESP
    #    basic robot board. Lights, buzzer, screen, motor
    #    and servo if room. """
    # TODO:  exit gracefully if pins are bad - check how to do this?
    #       tone/buzzer function
    #
    # simplest html page to show working
    # some useful value
    #
    #
    # On second thoughts - dont do networking in here - use this library for
    #                      control of the NodeBot board only!

    def __init__(self, dpin, ledbpin,ledwpin, servopin, buzpin):
        # initialises pins
        avail_pins=[0,2,4,5,12,13,14,15,16]
        self.ledbpin=ledbpin
        self.dpin=dpin
        self.ledwpin=ledwpin
        self.servopin=servopin
        self.buzpin=buzpin
        print("Robot initialising!")
        if self.ledbpin not in avail_pins or \
           self.dpin not in avail_pins or \
           self.ledwpin not in avail_pins or \
           self.servopin not in avail_pins or \
           self.buzpin not in avail_pins:
            print("Invalid pins selected!")

        self.dPin=PWM(Pin(self.dpin, Pin.OUT), duty=0, freq=500)
        self.led_bPin=Pin(self.ledbpin, Pin.OUT)
        self.led_wPin=Pin(self.ledwpin, Pin.OUT)
        self.servoPin=PWM(Pin(self.servopin, Pin.OUT), duty=0, freq=500)
        self.buzPin=PWM(Pin(self.buzpin, Pin.OUT),freq=500)

    def drive(self, dval):          # pin for the drive transistor, duty cycle
        # function to set the drive values
        if dval > 1023:       # constrain the value
            dval = 1023
        elif dval < 0:
            dval = 0
        duty=1023-dval      # due to hardware layout, high duty cycle = low speed. Inverted here for simplicity
        self.dPin.duty(duty)     #                                                       on the web-panel

    def echo(self, args):
        for arg in args:
            print(arg)

    def lights(self):
        return

    def networking(self, args):
        """ will use this function to modify the webpage to show information on network
            possibly also address changes, online reset & networking commands - later"""
        return

    def blink(self, col, n, t):
        # col = white or blue , n=number of blinks, t=wait time
        if col=="b" or col=="blue":
            led=self.led_bPin
        elif col=="w" or col=="white":
            led=self.led_wPin
        else:
            return
        for i in range(n):
            led.value(1)
            sleep(t)
            led.value(0)
            sleep(t)

    def siren(self, n,t):
        #alternating siren, n=times, t=wait time
        for i in range(n):
            self.led_bPin.value(1)
            self.led_wPin.value(0)
            sleep(t)
            self.led_bPin.value(0)
            self.led_wPin.value(1)
            sleep(t)
        self.led_bPin.value(0)
        self.led_wPin.value(0)

    def connect_network(self, NETWORK, PASS):
        # connect to the network specified using network and pass
        radio=network.WLAN(network.STA_IF)
        radio.active(True)
        err_c=0
        while not radio.isconnected():              # begin network connection
            radio.connect(NETWORK, PASS)
            sleep(2)
            err_c=err_c+1
            if err_c > 10:
                print("Unable to connect...")
                return False
        print("Connected!")
        radio.ifconfig()
        return True
