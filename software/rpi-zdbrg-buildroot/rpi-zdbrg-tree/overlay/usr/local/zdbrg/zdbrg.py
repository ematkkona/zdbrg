import sys
import serial
import RPi.GPIO as GPIO
from RpiMotorLib import RpiMotorLib
from time import sleep
from threading import Thread
from enum import Enum

Version = '0.823-311022'

# Config
serialPort = '/dev/ttyAMA0'
baudRate = 115200
runZeroPos = False
elZeroToPark = False
parkAndExit = False
elevation_sensor = False
azimuth_sensor = False
CW = False
CCW = True
spd_norm = 0.0006
spd_fast = 0.0004
spd_slow = 0.0008
spd_el_slow = 0.002
lostElTreshold = 6.0
minElevation = 0.0
maxElevation = 90.0
fullRevolution = 11250
azStepsPerDeg = (fullRevolution / 360)
elFullRange = ((fullRevolution / 4) / 2)
elStepsPerDeg = (elFullRange / 90)

# Pin assignment
azZeroDir = CW
az_speed = spd_fast
az_direction = 23
az_step = 18
az_EN_pin = 24
az_mode = (4, 27, 22)
azimuth_sensor_pin = 26
el_direction = 6
el_speed = spd_norm
el_step = 13
el_EN_pin = 5
el_mode = (25, 8, 7)
elevation_sensor_pin = 21
led1 = 3

# Motor & GPIO setup
azimuth = RpiMotorLib.A4988Nema(az_direction, az_step, az_mode, "A4988")
elevation = RpiMotorLib.A4988Nema(el_direction, el_step, el_mode, "A4988")
GPIO.setup(az_EN_pin, GPIO.OUT)
GPIO.setup(el_EN_pin, GPIO.OUT)
GPIO.setup(led1, GPIO.OUT)
GPIO.output(led1, GPIO.LOW)
GPIO.output(az_EN_pin, GPIO.LOW)
GPIO.output(el_EN_pin, GPIO.LOW)
GPIO.setup(azimuth_sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(elevation_sensor_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.add_event_detect(elevation_sensor_pin, GPIO.FALLING)
GPIO.add_event_detect(azimuth_sensor_pin, GPIO.FALLING)


class RunStates(Enum):
    HOMING = 1
    READY = 2
    RUNNING = 3
    ERROR = 4


def drive_azimuth(direction, steps_needed):
    global az_speed
    GPIO.output(az_EN_pin, GPIO.LOW)
    az_microstep = "1/8"
    az_stepcount = steps_needed * 8
    if steps_needed > 100:
        az_microstep = "Full"
        az_stepcount = steps_needed
    elif steps_needed >= 60:
        az_microstep = "Half"
        az_stepcount = steps_needed * 2
    elif steps_needed >= 20:
        az_microstep = "1/4"
        az_stepcount = steps_needed * 4
    azimuth.motor_go(direction, az_microstep, int(az_stepcount), az_speed, False, 0)
    GPIO.output(az_EN_pin, GPIO.HIGH)


def drive_elevation(direction, steps_needed):
    global elevation_sensor
    global currPosition
    global nextTarget
    global runZeroPos
    global lostElTreshold
    global el_speed
    GPIO.output(el_EN_pin, GPIO.LOW)
    el_microstep = "1/4"
    el_stepcount = steps_needed * 4
    elevation.motor_go(direction, el_microstep, int(el_stepcount), el_speed, False, 0)
    if elevation_sensor and currPosition[1] > lostElTreshold and not runZeroPos:
        nextTarget[1] = currPosition[1]
        runZeroPos = True
        elevation_to_home()
        currPosition[1] = 0.0
        runZeroPos = False
    GPIO.output(el_EN_pin, GPIO.HIGH)


def sensor_handler(channel):
    global azimuth_sensor, elevation_sensor, lostElTreshold, currPosition, runZeroPos, killSig, killSigMsg
    if not GPIO.input(channel):
        if channel != azimuth_sensor:
            if currPosition[1] > lostElTreshold and not runZeroPos:
                killSig = True
                killSigMsg = "Unexpected (el) limit switch activation"
            elevation_sensor = True
        else:
            azimuth_sensor = True
    else:
        if channel != azimuth_sensor:
            elevation_sensor = False
        else:
            azimuth_sensor = False


def azimuth_sensor_callback(stop):
    global azimuth_sensor_pin
    while True:
        if stop():
            break
        sensor_handler(azimuth_sensor_pin)
        sleep(0.05)


def elevation_sensor_callback(stop):
    global elevation_sensor_pin
    while True:
        if stop():
            break
        sensor_handler(elevation_sensor_pin)
        sleep(0.05)


def elevation_to_home():
    global el_speed
    global spd_norm
    global spd_slow
    global elPosition
    global elevation_sensor
    global elZeroToPark
    el_speed = spd_fast
    drive_elevation(CCW, 60)
    while not elevation_sensor:
        drive_elevation(CW, 10)
    el_speed = spd_slow
    while elevation_sensor:
        drive_elevation(CCW, 1)
    el_speed = spd_norm
    if not elZeroToPark:
        drive_elevation(CW, 50)
        elPosition = minElevation
        currPosition[1] = minElevation
    else:
        drive_elevation(CW, 60)
        elPosition = -1.0
        currPosition[1] = -1.0


def azimuth_to_home():
    global spd_slow
    global spd_norm
    global spd_fast
    global azZeroDir
    global azPosition
    global azimuth_sensor
    global az_speed
    az_hall_act_width = 0

    az_speed = spd_fast
    if azZeroDir:
        local_cw = CW
        local_ccw = CCW
    else:
        local_cw = CCW
        local_ccw = CW
    while not azimuth_sensor:
        drive_azimuth(local_cw, 60)
    while azimuth_sensor:
        drive_azimuth(local_cw, 6)
    while not azimuth_sensor:
        drive_azimuth(local_ccw, 3)
    while azimuth_sensor:
        az_hall_act_width += 1
        drive_azimuth(local_ccw, 1)
    az_speed = spd_norm
    drive_azimuth(local_ccw, (az_hall_act_width / 2))
    az_speed = spd_norm
    azPosition = 0.0


def target_handler(stop):
    global currPosition, runZeroPos, nextTarget, killSig, el_speed, az_speed, elZeroToPark, parkAndExit, runState

    ignore_azimuth = False
    ignore_elevation = False
    elevation_updated = False
    azimuth_updated = False
    elevation_to_target = Thread(target=drive_elevation, args=(CW, 0), )

    while True:
        new_target = [0.0, 0.0]
        if 0.0 <= currPosition[0] != nextTarget[0] >= 0.0 and not azimuth_updated:
            if runZeroPos or parkAndExit:
                if currPosition[0] != 0.0:
                    nextTarget = [0.0, 0.0]
                    new_target[0] = nextTarget[0]
                else:
                    ignore_azimuth = True
                azimuth_updated = True
            else:
                new_target[0] = nextTarget[0]
                nextTarget[0] = -1.0
            if currPosition[0] != new_target[0]:
                azdegscw = abs(((new_target[0] - currPosition[0] + 360) % 360))
                if azdegscw < 180:
                    direction = CW
                    azdeg = azdegscw
                    p = currPosition[0] + azdeg
                    if p > 360:
                        p -= 360
                    currPosition[0] = p
                else:
                    direction = CCW
                    azdeg = (360 - azdegscw)
                    p = currPosition[0] - azdeg
                    if p < 0:
                        p += 360
                    currPosition[0] = p
                steps = round(abs(((azdeg * fullRevolution) / 360)), 2)
                if steps > 100:
                    az_speed = spd_fast
                elif steps > 20:
                    az_speed = spd_norm
                else:
                    az_speed = spd_slow
                azimuth_updated = True
        if nextTarget[1] != currPosition[1] and minElevation <= nextTarget[1] <= maxElevation and not elevation_updated:
            new_target[1] = nextTarget[1]
            nextTarget[1] = -1.0
            if not elevation_to_target.is_alive():
                if new_target[1] > currPosition[1]:
                    elsteps = float(round((new_target[1] - currPosition[1]) * (elFullRange / 90), 2))
                    elevation_to_target = Thread(target=drive_elevation, args=(CCW, elsteps))
                    currPosition[1] = float(round(currPosition[1] + (elsteps / elStepsPerDeg), 2))
                else:
                    elsteps = float(round((currPosition[1] - new_target[1]) * (elFullRange / 90), 2))
                    elevation_to_target = Thread(target=drive_elevation, args=(CW, elsteps))
                    currPosition[1] = float(round(currPosition[1] - (elsteps / elStepsPerDeg), 2))
                if elsteps > 30:
                    el_speed = spd_fast
                elif elsteps > 10:
                    el_speed = spd_slow
                else:
                    el_speed = spd_el_slow
                el_speed = spd_norm
                elevation_updated = True
                if not ignore_elevation:
                    runState = RunStates.RUNNING
                    elevation_to_target.start()
        if azimuth_updated:
            if not ignore_azimuth:
                runState = RunStates.RUNNING
                drive_azimuth(direction, steps)
                runState = RunStates.READY
            if runZeroPos:
                runState = RunStates.HOMING
                azimuth_to_home()
                runState = RunStates.READY
            azimuth_updated = False
            ignore_azimuth = False
        if elevation_updated:
            if not ignore_elevation:
                if elevation_to_target.is_alive():
                    elevation_to_target.join()
                    runState = RunStates.READY
            if runZeroPos:
                runState = RunStates.HOMING
                elevation_to_home()
                runState = RunStates.READY
            runZeroPos = False
            elZeroToPark = False
        if parkAndExit:
            runState = RunStates.ERROR
            if currPosition != [-1.0, -1.0]:
                elZeroToPark = True
                elevation_to_home()
            killSig = True
            break
        ignore_elevation = False
        elevation_updated = False
        if stop():
            break


def sta_led_indicator(stop):
    global led1, runState
    while True:
        match runState:
            case RunStates.HOMING:
                GPIO.output(led1, GPIO.LOW)
                sleep(0.4)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.2)
            case RunStates.READY:
                GPIO.output(led1, GPIO.LOW)
                sleep(0.1)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.8)
            case RunStates.RUNNING:
                GPIO.output(led1, GPIO.LOW)
                sleep(0.2)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.1)
            case RunStates.ERROR:
                GPIO.output(led1, GPIO.LOW)
                sleep(0.8)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.1)
        if stop():
            for x in range(10):
                GPIO.output(led1, GPIO.LOW)
                sleep(0.1)
                GPIO.output(led1, GPIO.HIGH)
                sleep(0.1)
            break
        sleep(0.1)
    GPIO.output(led1, GPIO.LOW)


def serial_handler(stop):
    global serialPort
    global baudRate
    global runState
    global killSig
    global killSigMsg
    global nextTarget
    global runZeroPos
    global parkAndExit
    global elZeroToPark
    ignore_position = False
    try:
        ser = serial.Serial(port=serialPort, baudrate=baudRate, rtscts=True)
    except serial.SerialException:
        killSig = True
    if not killSig:
        ser.flush()
        sleep(0.1)
        initmsg = f'[ZdBrg:Main] v{Version}\n\r[ZdBrg:Main] Homing ...\n\r'
        zeropmsg = f'[ZdBrg:Main] Ready!\n\rAZ0.0 EL0.0\n\r'
        ser.write(initmsg.encode())
    while runState == RunStates.HOMING:
        if killSig:
            break
        sleep(0.5)
    if not killSig:
        ser.write(zeropmsg.encode())
        sleep(0.1)
    ser.flush()
    while True:
        data_str = ser.readline().decode('ascii')
        sleep(0.1)
        ser.flush()
        if data_str.startswith('SA SE'):
            saseresp = f'SA SE\n\r'
            ser.write(saseresp.encode())
            if currPosition[0] > 0 or currPosition[1] > 0:
                nextTarget = [0.0, minElevation]
        elif data_str.startswith('HOME'):
            hresp = f'HOME\n\r'
            ser.write(hresp.encode())
            if currPosition[0] > 0 or currPosition[1] > minElevation:
                nextTarget = [0.0, minElevation]
                sleep(8)
            runZeroPos = True
        elif data_str.startswith('TERM'):
            tresp = f'TERM\n\r'
            ser.write(tresp.encode())
            if currPosition[0] > 0 or currPosition[1] > minElevation:
                nextTarget = [0.0, minElevation]
                sleep(8)
            elZeroToPark = True
            parkAndExit = True
        elif data_str.startswith('SETAZ'):
            set_az = data_str.removeprefix('SETAZ')
            ignore_position = True
            if not set_az.replace('.', '', 1).isdigit():
                currPosition[0] = float(set_az)
                setazresp = f'AZ{str(set_az)}\n\r'
                ser.write(setazresp.encode())
            else:
                setazresp = f'AZSET FAIL {str(set_az)}\n\r'
                ser.write(setazresp.encode())
        elif data_str.startswith('SETEL'):
            set_el = data_str.removeprefix('SETEL')
            ignore_position = True
            if not set_el.replace('.', '', 1).isdigit():
                currPosition[1] = float(set_el)
                setelresp = f'EL{str(set_el)}\n\r'
                ser.write(setelresp.encode())
            else:
                setelresp = f'EL SET FAILED {str(set_el)}\n\r'
                ser.write(setelresp.encode())
        elif data_str.startswith('AZ EL'):
            elresp = f'AZ{str(currPosition[0])} EL{str(currPosition[1])}\n\r'
            ser.write(elresp.encode())
            ignore_position = True
        elif data_str.startswith('AZ') and not ignore_position:
            splitcmd = data_str.split()
            if runZeroPos is False and parkAndExit is False and len(splitcmd) == 2:
                fmt_az = splitcmd[0].removeprefix('AZ')
                if not fmt_az.replace('.', '', 1).isdigit() or fmt_az == '':
                    azresp = f'AZ{str(currPosition[0])}\n\r'
                    ser.write(azresp.encode())
                else:
                    nextTarget[0] = float(fmt_az)
                if splitcmd[1].startswith("EL"):
                    fmt_el = splitcmd[1].removeprefix("EL")
                    splitcmd[1] = ''
                    splitcmd[0] = ''
                    if not fmt_el.replace('.', '', 1).isdigit() or fmt_el == '':
                        elresp = f'EL{str(currPosition[1])}\n\r'
                        ser.write(elresp.encode())
                    elif float(fmt_el) < minElevation or float(fmt_el) > maxElevation:
                        elresp = f'EL-1.0\n\r'
                        ser.write(elresp.encode())
                    else:
                        nextTarget[1] = float(fmt_el)
            ser.reset_output_buffer()
            ignore_position = False
        if stop():
            break
    if killSigMsg != '':
        ser.write(f'[ZdBrg:Main] {killSigMsg}\n\r'.encode())
    ser.write('[ZdBrg:Main] Program terminating\n\r'.encode())
    ser.close()


elPosition = -1.0
azPosition = -1.0
killSig = False
killSigMsg = ''
nextTarget = [-1.0, -1.0]
currPosition = [-1.0, -1.0]
statusLedThread = Thread(target=sta_led_indicator, args=(lambda: killSig,))
elSensorThread = Thread(target=elevation_sensor_callback, args=(lambda: killSig,))
azSensorThread = Thread(target=azimuth_sensor_callback, args=(lambda: killSig,))
serialListenerThread = Thread(target=serial_handler, args=(lambda: killSig,))
targetHandlerThread = Thread(target=target_handler, args=(lambda: killSig,))
runState = RunStates.HOMING
statusLedThread.start()
serialListenerThread.start()
azSensorThread.start()
elSensorThread.start()
elHomeThread = Thread(target=elevation_to_home, )
azZeroPosThread = Thread(target=azimuth_to_home, )
elHomeThread.start()
azZeroPosThread.start()
sleep(0.5)
elHomeThread.join(timeout=10.0)
if elHomeThread.is_alive():
    killSigMsg = 'Homing failed (EL:timeout=>10s)'
    killSig = True
    runState = RunStates.ERROR
else:
    azZeroPosThread.join(timeout=25.0)
    if azZeroPosThread.is_alive():
        killSigMsg = 'Homing failed (AZ:timeout=>25s)'
        killSig = True
        runState = RunStates.ERROR
currPosition = [0.0, minElevation]
targetHandlerThread.start()
if not killSig:
    runState = RunStates.READY

while True:
    if killSig:
        azimuth.motor_stop()
        elevation.motor_stop()
        runState = RunStates.ERROR
        sleep(1)
        break
    if not targetHandlerThread.is_alive():
        sleep(0.2)
        killSigMsg = 'targetHandlerThread died'
        break
    if not serialListenerThread.is_alive():
        sleep(0.2)
        killSigMsg = 'serialListenerThread died'
        break
    sleep(.2)
if not killSig:
    elZeroToPark = True
    elevation_to_home()
killSig = True
azSensorThread.join(timeout=0.5)
elSensorThread.join(timeout=0.5)
serialListenerThread.join(timeout=0.5)
targetHandlerThread.join(timeout=0.5)
GPIO.cleanup()
if killSigMsg:
    sys.exit(1)
sys.exit(0)
