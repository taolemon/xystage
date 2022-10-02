import RPi.GPIO as GPIO
import threading
from time import sleep
import sys

MAX_PULSE_FREQ = 200*1000
MIN_PULSE_PERIOD = 1.0/MAX_PULSE_FREQ

AXIS_X = 1
AXIS_Y = 2
DIR_POS = 1
DIR_NEG = 2

# Stepper motor
GPIO_PUL_X = 6
GPIO_DIR_X = 13
GPIO_PUL_Y = 19
GPIO_DIR_Y = 26
SLEEP_SEC = 2e-4

# Solenoid
GPIO_SOL = 5

def gpio_config():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_PUL_X, GPIO.OUT)
    GPIO.setup(GPIO_DIR_X, GPIO.OUT)
    GPIO.setup(GPIO_PUL_Y, GPIO.OUT)
    GPIO.setup(GPIO_DIR_Y, GPIO.OUT)
    GPIO.setup(GPIO_SOL  , GPIO.OUT)

def run_motor(axis, direction, nPulse, time):
    nPulse = int(nPulse)
    
    if(axis == AXIS_X):
        gpio_dir = GPIO_DIR_X
        gpio_pul = GPIO_PUL_X
    elif(axis == AXIS_Y):
        gpio_dir = GPIO_DIR_Y
        gpio_pul = GPIO_PUL_Y
    else:
        sys.stderr.write("INVALID AXIS")
        exit()
    
    T = calc_T(nPulse, time)
    
    GPIO.output(gpio_dir, direction==DIR_POS)
    dt = T/2
    for i in range(nPulse):
        GPIO.output(gpio_pul, True)
        sleep(dt)
        GPIO.output(gpio_pul, False)
        sleep(dt)

def en_solenoid(en):
    GPIO.output(GPIO_SOL, en)

def calc_T(nPulse, time):
    if(nPulse!=0):
        T = time/nPulse
    else:
        T = -1        
    if(T < MIN_PULSE_PERIOD):
        T = MIN_PULSE_PERIOD
    return(T)

class pos:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class motor:
    def __init__(self):
        self.pos_max = pos(100,100)
        self.pos = pos(0,0)
    
    # Core Functions
    def resetPos(self):
        self.pos.x = 0
        self.pos.y = 0
        
    def setMax(self, x, y):
        if(x > 0 and y > 0):
            self.pos_max.x = x
            self.pos_max.y = y
        else:
            pass
        
    def moveToRelX(self, dx, t):
        self.pos.x += dx
        if(dx >= 0):
            run_motor(AXIS_X, DIR_POS, abs(dx), t)
        else:
            run_motor(AXIS_X, DIR_NEG, abs(dx), t)

    def moveToRelY(self, dy, t):
        self.pos.y += dy
        if(dy >= 0):
            run_motor(AXIS_Y, DIR_POS, abs(dy), t)
        else:
            run_motor(AXIS_Y, DIR_NEG, abs(dy), t)

    def moveToRel(self, dx, dy, t):
        tx = threading.Thread(target=self.moveToRelX, args=(dx, t,))
        ty = threading.Thread(target=self.moveToRelY, args=(dy, t,))
        tx.start()
        ty.start()
        tx.join()
        ty.join()

    # Utility
    def setMaxAsPos(self):
        setMax(self.pos.x, self.pos.y)

    def moveToAbs(self, x, y, t):
        dx = x - self.pos.x
        dy = y - self.pos.y
        self.moveToRel(dx, dy, t)

    def returnToOrigin(self, t):
        self.moveToAbs(0, 0, t)
    
    def moveToMax(self, t):
        self.moveToAbs(self.pos_max.x, self.pos_max.y, t)
    
    def moveToRatio(self, xr, yr, t):
        x = self.pos_max.x * xr
        y = self.pos_max.y * yr
        self.moveToAbs(x, y, t)