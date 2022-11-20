import sys
import serial
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
from time import sleep
from threading import Thread
from enum import Enum

Version = '0.98-191122r1'

# General configuration
serialPort = '/dev/ttyAMA0'
baudRate = 115200
CW = False
CCW = True
azZeroDir = CW
elDown = False
el_steps_back = 1
minElevation = 0.0
maxElevation = 90.0
fullRevolution = 11250
azStepsPerDeg = (fullRevolution / 360)
elFullRange = ((fullRevolution / 4) / 2)
elStepsPerDeg = (elFullRange / 90)
azimuth_sensor = False
elevation_sensor = False

# Pin assignment
az_direction = 23
az_step = 18
az_EN_pin = 24
az_mode = (4, 27, 22)
azimuth_sensor_pin = 26
el_direction = 6
el_step = 13
el_EN_pin = 5
el_mode = (25, 8, 7)
elevation_sensor_pin = 21
led1 = 3

# Motor driver & GPIO pin configuration
azimuth = RpiMotorLib.A4988Nema(az_direction, az_step, az_mode, "A4988")
elevation = RpiMotorLib.A4988Nema(el_direction, el_step, el_mode, "A4988")
GPIO.setup(az_EN_pin, GPIO.OUT)
GPIO.setup(el_EN_pin, GPIO.OUT)
GPIO.setup(led1, GPIO.OUT)
GPIO.output(led1, GPIO.LOW)
GPIO.output(az_EN_pin, GPIO.HIGH)
GPIO.output(el_EN_pin, GPIO.HIGH)
GPIO.setup(azimuth_sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(elevation_sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
if not GPIO.input(azimuth_sensor_pin):
    azimuth_sensor = True
if not GPIO.input(elevation_sensor_pin):
    elevation_sensor = True


class RunStates(Enum):
    HOMING = 1
    READY = 2
    RUNNING = 3
    ERROR = 4


class MSpeed(float, Enum):
    FAST = 0.0003
    NORMAL = 0.0004
    SLOW = 0.0005


def drive_azimuth(direction, steps_needed, mode_override):
    global az_speed, killSig, az_EN_pin
    GPIO.output(az_EN_pin, GPIO.LOW)
    if steps_needed >= 30:
        az_microstep = "Full"
        az_stepcount = steps_needed
    else:
        az_microstep = "Half"
        az_stepcount = steps_needed * 2
    if not killSig:
        if mode_override:
            az_microstep = "Full"
            az_stepcount = steps_needed
        azimuth.motor_go(direction, az_microstep, int(az_stepcount), az_speed, False, 0)
    GPIO.output(az_EN_pin, GPIO.HIGH)


def drive_elevation(direction, steps_needed):
    global elevation_sensor, el_EN_pin, currPosition, nextTarget, lostElTreshold, el_speed, killSig
    GPIO.output(el_EN_pin, GPIO.LOW)
    el_microstep = "Half"
    el_stepcount = steps_needed * 2
    if not killSig:
        elevation.motor_go(direction, el_microstep, int(el_stepcount), el_speed, False, 0)
    GPIO.output(el_EN_pin, GPIO.HIGH)


def azsens_callback(channel):
    global azimuth_sensor_pin, azimuth_sensor
    if GPIO.input(azimuth_sensor_pin):
        azimuth_sensor = False
    else:
        azimuth_sensor = True


def elsens_callback(channel):
    global elevation_sensor_pin, elevation_sensor, lostElTreshold, currPosition, elDown, runState, killSig, killSigMsg
    if GPIO.input(elevation_sensor_pin):
        elevation_sensor = False
    else:
        elevation_sensor = True
        if elDown and runState == RunStates.RUNNING:
            GPIO.output(el_EN_pin, GPIO.HIGH)
            killSigMsg = "[EL] Unexpected limit switch activation!"


def elevation_to_home(stop):
    global el_speed, elevation_sensor, el_EN_pin, el_steps_back
    GPIO.output(el_EN_pin, GPIO.LOW)
    el_speed = MSpeed.FAST.value
    drive_elevation(CCW, 40)
    while not elevation_sensor:
        if stop():
            break
        el_speed = MSpeed.NORMAL.value
        drive_elevation(CW, 5)
    el_speed = MSpeed.SLOW.value
    while elevation_sensor:
        if stop():
            break
        drive_elevation(CCW, 1)
    el_speed = MSpeed.FAST.value
    if not stop():
        drive_elevation(CW, el_steps_back)
        currPosition[1] = minElevation
    GPIO.output(el_EN_pin, GPIO.HIGH)


def azimuth_to_home(stop):
    global azZeroDir, azimuth_sensor, az_speed, az_EN_pin, az_hall_act_width
    GPIO.output(az_EN_pin, GPIO.LOW)
    az_speed = MSpeed.FAST.value
    if azZeroDir:
        local_cw = CW
        local_ccw = CCW
    else:
        local_cw = CCW
        local_ccw = CW
    # Start rotating azimuth, stop within sensor active zone
    while not azimuth_sensor:
        if stop():
            break
        drive_azimuth(local_cw, 100, True)
    az_speed = MSpeed.NORMAL.value
    # Keep going at slower rate until reaching end of sensor active zone
    while azimuth_sensor:
        if stop():
            break
        drive_azimuth(local_cw, 10, True)
    # Drive slowly back to the sensor active zone
    az_speed = MSpeed.SLOW.value
    while not azimuth_sensor:
        if stop():
            break
        drive_azimuth(local_ccw, 1, True)
    # Drive forward and measure the size of sensor active zone
    az_speed = MSpeed.NORMAL.value
    while azimuth_sensor:
        if stop():
            break
        az_hall_act_width += 1
        drive_azimuth(local_ccw, 1, True)
    # Rotate to the middle of the sensor active zone & set as initial zero position
    az_speed = MSpeed.FAST.value
    if not stop():
        drive_azimuth(local_cw, (az_hall_act_width / 2), False)
    GPIO.output(az_EN_pin, GPIO.HIGH)


def target_handler(stop):
    global currPosition, nextTarget, killSig, el_speed, az_speed, runState, elDown, azZeroDir
    azimuth_updated = False
    direction = CW
    steps = 0
    while True:
        if stop():
            break
        sleep(0.2)
        new_target = [0.0, 0.0]
        if 0.0 <= currPosition[0] != nextTarget[0] >= 0.0:
            new_target[0] = nextTarget[0]
            nextTarget[0] = -1.0
            if currPosition[0] != new_target[0]:
                azdegscw = abs(((new_target[0] - currPosition[0] + 360) % 360))
                if azdegscw < 180:
                    direction = CW
                    azZeroDir = CCW
                    azdeg = azdegscw
                    p = currPosition[0] + azdeg
                    if p > 360:
                        p -= 360
                    currPosition[0] = p
                else:
                    direction = CCW
                    azZeroDir = CW
                    azdeg = (360 - azdegscw)
                    p = currPosition[0] - azdeg
                    if p < 0:
                        p += 360
                    currPosition[0] = p
                steps = round(abs(((azdeg * fullRevolution) / 360)), 2)
                if steps > 100:
                    az_speed = MSpeed.FAST.value
                elif steps < 10:
                    az_speed = MSpeed.SLOW.value
                else:
                    az_speed = MSpeed.NORMAL.value
                azimuth_updated = True
        if nextTarget[1] != currPosition[1] and minElevation <= nextTarget[1] <= maxElevation:
            new_target[1] = nextTarget[1]
            nextTarget[1] = -1.0
            if new_target[1] > currPosition[1]:
                elsteps = float(round((new_target[1] - currPosition[1]) * (elFullRange / 90), 2))
                elDown = False
                elevation_to_target = Thread(target=drive_elevation, args=(CCW, elsteps))
                currPosition[1] = float(round(currPosition[1] + (elsteps / elStepsPerDeg), 2))
            else:
                elsteps = float(round((currPosition[1] - new_target[1]) * (elFullRange / 90), 2))
                elDown = True
                elevation_to_target = Thread(target=drive_elevation, args=(CW, elsteps))
                currPosition[1] = float(round(currPosition[1] - (elsteps / elStepsPerDeg), 2))
            if elsteps > 30:
                el_speed = MSpeed.FAST.value
            elif elsteps > 10:
                el_speed = MSpeed.NORMAL.value
            else:
                el_speed = MSpeed.SLOW.value
            runState = RunStates.RUNNING
            elevation_to_target.start()
            if azimuth_updated:
                drive_azimuth(direction, steps, False)
                azimuth_updated = False
            if elevation_to_target.is_alive():
                elevation_to_target.join()
        if azimuth_updated:
            runState = RunStates.RUNNING
            drive_azimuth(direction, steps, False)
            azimuth_updated = False
        runState = RunStates.READY
        sleep(0.2)


def sta_led_indicator(stop):
    global led1, runState
    while True:
        match runState:
            case RunStates.HOMING:
                GPIO.output(led1, GPIO.LOW)
                sleep(0.2)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.2)
            case RunStates.READY:
                GPIO.output(led1, GPIO.LOW)
                sleep(0.95)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.05)
            case RunStates.RUNNING:
                GPIO.output(led1, GPIO.LOW)
                sleep(0.2)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.15)
            case RunStates.ERROR:
                GPIO.output(led1, GPIO.LOW)
                sleep(0.05)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.05)
        if stop():
            for x in range(25):
                GPIO.output(led1, GPIO.LOW)
                sleep(0.025)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.025)
            break
        sleep(0.1)


def serial_handler(stop):
    global serialPort, baudRate, runState, killSig, killSigMsg, nextTarget, az_hall_act_width
    zeropmsg = f'\n\r'
    busymsg = f'BUSY\n\r'
    try:
        ser = serial.Serial(port=serialPort, baudrate=baudRate, rtscts=True)
    except serial.SerialException:
        killSig = True
    if not killSig:
        ser.flush()
        sleep(0.1)
        initmsg = f'[ZdBrg:Main] v{Version}\n\r[ZdBrg:Main] Homing ...\n\r'
        zeropmsg = f'\n\rAZ0.0 EL0.0\n\r'
        ser.write(initmsg.encode())
    while runState == RunStates.HOMING:
        sleep(0.1)
    ser.flush()
    if killSigMsg == '':
        homingok = f'[ZdBrg:Main] Ready! AZ HALL ACT: {az_hall_act_width}/{fullRevolution}\n\r'
        ser.write(homingok.encode())
        ser.write(zeropmsg.encode())
        while True:
            data_str = ser.readline().decode('ascii')
            sleep(0.1)
            ser.flush()
            if data_str.startswith('SA SE') and runState == RunStates.READY:
                saseresp = f'SA SE\n\r'
                ser.write(saseresp.encode())
                if currPosition[0] > 0.0 or currPosition[1] > 0.0:
                    nextTarget = [0.0, minElevation]
            elif data_str.startswith('HOME') and runState == RunStates.READY:
                hresp = f'HOME\n\r'
                ser.write(hresp.encode())
                runState = RunStates.HOMING
                elevation_to_home(lambda: killSig,)
                currPosition[1] = minElevation
                azimuth_to_home(lambda: killSig,)
                currPosition[0] = 0
                runState = RunStates.READY
            elif data_str.startswith('TERM') and runState == RunStates.READY:
                tresp = f'TERM\n\r'
                ser.write(tresp.encode())
                if currPosition[0] > 0.0 or currPosition[1] > minElevation:
                    runState = RunStates.HOMING
                    elevation_to_home(lambda: killSig,)
                    azimuth_to_home(lambda: killSig,)
                killSig = True 
            elif data_str.startswith('SETAZ') and runState == RunStates.READY:
                set_az = data_str.removeprefix('SETAZ')
                if not set_az.replace('.', '', 1).isdigit():
                    if set_az >= 0 and set_az <= 360:
                        setazresp = f'AZ{currPosition[0]} => AZ{str(set_az)} OK\n\r'
                        currPosition[0] = float(set_az)
                        ser.write(setazresp.encode())
                    else:
                        setazerr = f'SETAZ: VALUE OUT OF RANGE\n\rAZ{currPosition[0]}\n\r'
                        ser.write(setazerr.encode())
                else:
                    setazresp = f'SETAZ: INVALID INPUT\n\rAZ{currPosition[0]}\n\r'
                    ser.write(setazresp.encode())
            elif data_str.startswith('SETEL') and runState == RunStates.READY:
                set_el = data_str.removeprefix('SETEL')
                if not set_el.replace('.', '', 1).isdigit():
                    if float(set_el) > currPosition[1] and float(set_el) >= minElevation and float(set_el) <= maxElevation:
                        setelerr = f'SETEL: VALUE OUT OF RANGE\n\rEL{currPosition[1]}\n\r'
                        ser.write(setelerr.encode())
                    else:
                        setelresp = f'EL{currPosition[1]} => EL{str(set_el)} OK\n\r'
                        currPosition[1] = float(set_el)
                        ser.write(setelresp.encode())
                else:
                    setelresp = f'SETEL: INVALID INPUT\n\rEL{currPosition[1]}\n\r'
                    ser.write(setelresp.encode())
            elif data_str.startswith('AZ EL'):
                elresp = f'AZ{str(currPosition[0])} EL{str(currPosition[1])}\n\r'
                ser.write(elresp.encode())
            elif data_str.startswith('AZ') and runState == RunStates.READY:
                splitcmd = data_str.split()
                if len(splitcmd) == 2:
                    fmt_az = splitcmd[0].removeprefix('AZ')
                    if not fmt_az.replace('.', '', 1).isdigit() or fmt_az == '':
                        azresp = f'ERROR: AZ{str(fmt_az)}\n\r'
                        ser.write(azresp.encode())
                        fmt_az = currPosition[0]
                    if splitcmd[1].startswith("EL"):
                        fmt_el = splitcmd[1].removeprefix("EL")
                        if not fmt_el.replace('.', '', 1).isdigit() or fmt_el == '' or float(fmt_el) < minElevation or \
                                float(fmt_el) > maxElevation:
                            elresp = f'ERROR: EL{str(fmt_el)}\n\r'
                            ser.write(elresp.encode())
                            fmt_el = currPosition[1]
                        nextTarget = [float(fmt_az), float(fmt_el)]
                        azelresp = f'AZ{str(nextTarget[0])} EL{str(nextTarget[1])}\n\r'
                        ser.write(azelresp.encode())
            elif runState != RunStates.READY:
                ser.write(busymsg.encode())
            if stop():
                if killSigMsg != '':
                    ser.write(f'[ZdBrg:Main] {killSigMsg}\n\r'.encode())
                ser.write('[ZdBrg:Main] Program terminating\n\r'.encode())
                ser.flush()
                ser.close()
                break


az_hall_act_width: int = 0
el_speed: float = MSpeed.NORMAL.value
az_speed: float = MSpeed.FAST.value
killSig: bool = False
killSigMsg: str = ''
nextTarget = [-1.0, -1.0]
currPosition = [-1.0, -1.0]
GPIO.add_event_detect(elevation_sensor_pin, GPIO.BOTH, callback=elsens_callback)
GPIO.add_event_detect(azimuth_sensor_pin, GPIO.BOTH, callback=azsens_callback)
serialListenerThread = Thread(target=serial_handler, args=(lambda: killSig,))
elHomeThread = Thread(target=elevation_to_home, args=(lambda: killSig,))
azZeroPosThread = Thread(target=azimuth_to_home, args=(lambda: killSig,))
statusLedThread = Thread(target=sta_led_indicator, args=(lambda: killSig,))
targetHandlerThread = Thread(target=target_handler, args=(lambda: killSig,))
runState = RunStates.HOMING
statusLedThread.start()
serialListenerThread.start()
elHomeThread.start()
azZeroPosThread.start()
elHomeThread.join(timeout=10.0)
if elHomeThread.is_alive():
    killSigMsg = 'Homing failed (EL:timeout=>10s)'
    runState = RunStates.ERROR
if killSigMsg == '':
    azZeroPosThread.join(timeout=40.0)
else:
    azZeroPosThread.join(timeout=1)
if azZeroPosThread.is_alive():
    if killSigMsg == '':
        killSigMsg = 'Homing failed (AZ:timeout=>45s)'
        runState = RunStates.ERROR
if not killSig and not killSigMsg:
    runState = RunStates.READY
    currPosition = [0.0, minElevation]
    targetHandlerThread.start()

# Main loop
while True:
    if killSig:
        sleep(1)
        break
    if not targetHandlerThread.is_alive():
        sleep(0.1)
        killSigMsg = 'Target handler died!'
    if not statusLedThread.is_alive():
        sleep(0.1)
        killSigMsg = 'Status indicator died!'
    if not serialListenerThread.is_alive():
        sleep(0.1)
        killSigMsg = 'Serial listener died!'
    if killSigMsg:
        killSig = True
        sleep(2)
    sleep(0.4)
        
# This is the end...
if targetHandlerThread.is_alive():
    targetHandlerThread.join(timeout=2)
if statusLedThread.is_alive():
    statusLedThread.join(timeout=2)
if serialListenerThread.is_alive():
    serialListenerThread.join(timeout=2)

# Disable motors
GPIO.output(led1, GPIO.LOW)
GPIO.output(az_EN_pin, GPIO.HIGH)
GPIO.output(el_EN_pin, GPIO.HIGH)
if killSigMsg:
    sys.exit(1)
sys.exit(0)
