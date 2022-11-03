import RPi.GPIO as GPIO 
import sys
import serial
from RpiMotorLib import RpiMotorLib
from time import sleep, perf_counter
from threading import Thread
from enum import Enum

Version = '0.823-311022'
serialPort = '/dev/ttyAMA0'
baudRate = 115200

CW = False
CCW = True
spd_norm = 0.0006
spd_fast = 0.0004
spd_slow = 0.0008
spd_elsslow = 0.002

azZeroDir = CW
az_speed = spd_fast
az_direction = 23
az_step = 18
az_EN_pin = 24
az_mode = (4,27,22)
az_sensor = 26
fullRevolution = 11250
azStepsPerDeg = (fullRevolution / 360)

el_direction = 6
el_speed = spd_norm
el_step = 13
el_EN_pin = 5
el_mode = (25, 8, 7)
el_sensor = 21

led1 = 3

lostElTreshold = 6.0
minElevation = 0.0
maxElevation = 90.0
elFullRange = ((fullRevolution / 4) / 2)
elStepsPerDeg = (elFullRange / 90)
runZeroPos = False
elZeroToPark = False
parkAndExit = False

class RunStates(Enum):
    HOMING = 1
    READY = 2
    RUNNING = 3
    ERROR = 4

runState = RunStates.HOMING

azimuth = RpiMotorLib.A4988Nema(az_direction, az_step, az_mode, "A4988")
elevation = RpiMotorLib.A4988Nema(el_direction, el_step, el_mode, "A4988")

GPIO.setup(az_EN_pin,GPIO.OUT)
GPIO.setup(el_EN_pin,GPIO.OUT)
GPIO.setup(led1,GPIO.OUT)
GPIO.output(led1,GPIO.LOW)
GPIO.output(az_EN_pin,GPIO.LOW)
GPIO.output(el_EN_pin,GPIO.LOW)

GPIO.setup(az_sensor, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(el_sensor, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect

def azRun(direction, steps_needed):
    global az_speed
    GPIO.output(az_EN_pin,GPIO.LOW)
    runMode = "1/8"
    runMult = 8
    runSteps = steps_needed * 8
    if steps_needed > 100:
        runMode = "Full"
        runMult = 1
        runSteps = steps_needed
    elif steps_needed >= 60:
        runMode = "Half"
        runMult = 2
        runSteps = steps_needed * 2
    elif steps_needed >= 20:
        runMode = "1/4"
        runMult = 4
        runSteps = steps_needed * 4    
    azimuth.motor_go(direction, runMode, int(runSteps), az_speed, False, 0)
    GPIO.output(az_EN_pin,GPIO.HIGH)

def elRun(direction, steps_needed):
    global elSensor
    global currPosition
    global nextTarget
    global runZeroPos
    global lostElTreshold
    global el_speed

    GPIO.output(el_EN_pin,GPIO.LOW)
    runMode = "1/4"
    runSteps = steps_needed * 4
    elevation.motor_go(direction, runMode, int(runSteps), el_speed, False, 0)
    if (elSensor and currPosition[1] > lostElTreshold and not runZeroPos):
        nextTarget[1] = currPosition[1]
        runZeroPos = True
        elZeroPos()
        currPosition[1] = 0.0
        runZeroPos = False
    GPIO.output(el_EN_pin,GPIO.HIGH)

def sensorCallback(channel):
    global azSensor
    global elSensor
    global lostElTreshold
    global currPosition
    global runZeroPos
    global killSig
    global killSigMsg
    if not GPIO.input(channel):
        if (channel == az_sensor):
            azSensor = True
        else:
            if (currPosition[1] > lostElTreshold and not runZeroPos):
                killSig = True
                killSigMsg = "Unexpected (el) limit switch activation"
            elSensor = True
    else:
        if (channel == az_sensor):
            azSensor = False
        else:
            elSensor = False


def azSensor(stop):
    while True:
        if stop():
            break
        sensorCallback(26)
        sleep(0.05)

def elSensor(stop):
    while True:
        if stop():
            break
        sensorCallback(21)
        sleep(0.05)

def elZeroPos():
    global el_speed
    global spd_norm
    global spd_slow
    global elPosition
    global elSensor
    global elZeroToPark
    el_speed = spd_fast
    elRun(CCW, 60)
    while not elSensor:
        elRun(CW, 10)
    el_speed = spd_slow
    while elSensor:
        elRun(CCW, 1)
    el_speed = spd_norm
    if not elZeroToPark:
        elRun(CW, 50)
        elPosition = minElevation
        currPosition[1] = minElevation
    else:
        elRun(CW, 60)
        elPosition = -1.0
        currPosition[1] = -1.0

def azZeroPos():
    global spd_slow
    global spd_norm
    global spd_fast
    sensorWidth = 0
    global azZeroDir
    global azPosition
    global azSensor
    az_speed = spd_fast
    if(azZeroDir):
        CWz = CW
        CCWz = CCW
    else:
        CWz = CCW
        CCWz = CW
    while not azSensor:
        azRun(CWz, 60)
    while azSensor:
        azRun(CWz, 6)
    while not azSensor:
        azRun(CCWz, 3)
    while azSensor:
        sensorWidth += 1 
        azRun(CCWz, 1)
    az_speed = spd_norm
    azRun(CCWz, (sensorWidth/2))
    az_speed = spd_norm
    azPosition = 0.0

def targetHandler(stop):
    global currPosition
    global runZeroPos
    global nextTarget
    global killSig
    global el_speed
    global az_speed
    global spd_fast
    global spd_norm
    global spd_slow
    global spd_elsslow
    global elZeroToPark
    global rebootFile
    global parkAndExit
    global runState

    omitAzRun = False
    omitElRun = False
    elUpdated = False
    azUpdated = False
    elToTarget = Thread(target = elRun, args=(CW, 0), )

    while (True):
        setTarget = [0.0, 0.0]
        if (currPosition[0] >= 0.0 and nextTarget[0] >= 0.0 and currPosition[0] != nextTarget[0] and not azUpdated):
            if runZeroPos or parkAndExit:
                if (currPosition[0] != 0.0):
                    nextTarget = [0.0, 0.0]
                    setTarget[0] = nextTarget[0]
                else:
                    omitAzRun = True
                azUpdated = True
            else:
                setTarget[0] = nextTarget[0]
                nextTarget[0] = -1.0
            if currPosition[0] != setTarget[0]:
                azdegscw = abs(((setTarget[0] - currPosition[0] + 360) % 360))
                if (azdegscw < 180):
                    direction = CW
                    azdeg = azdegscw
                    p = currPosition[0] + azdeg
                    if(p>360):
                        p -= 360
                    currPosition[0] = p
                else:
                    direction = CCW
                    azdeg = (360 - azdegscw)
                    p = currPosition[0] - azdeg
                    if (p<0):
                        p += 360
                    currPosition[0] = p
                steps = round(abs(((azdeg * fullRevolution) / 360)), 2)
                if (steps > 100):
                    az_speed = spd_fast
                elif (steps > 20):
                    az_speed = spd_norm
                else:
                    az_speed = spd_slow
                azUpdated = True
        if (nextTarget[1] != currPosition[1] and nextTarget[1] >= minElevation and nextTarget[1] <= maxElevation and not elUpdated):
            setTarget[1] = nextTarget[1]
            nextTarget[1] = -1.0
            if (not elToTarget.is_alive()):
                if (setTarget[1] > currPosition[1]):
                    elsteps = float(round((setTarget[1] - currPosition[1]) * (elFullRange / 90), 2))
                    elToTarget = Thread(target = elRun, args=(CCW, elsteps))
                    currPosition[1] = float(round(currPosition[1] + (elsteps / elStepsPerDeg), 2))
                else:
                     elsteps = float(round((currPosition[1] - setTarget[1]) * (elFullRange / 90), 2))
                     elToTarget = Thread(target = elRun, args=(CW, elsteps))
                     currPosition[1] = float(round(currPosition[1] - (elsteps / elStepsPerDeg), 2))
                if (elsteps > 30):
                    el_speed = spd_fast
                elif (elsteps > 10):
                    el_speed = spd_slow
                else:
                    el_speed = spd_elsslow
                el_speed = spd_norm
                elUpdated = True
                if not omitElRun:
                    runState = RunStates.RUNNING
                    elToTarget.start()
        if azUpdated:
            if not omitAzRun:
                runState = RunStates.RUNNING
                azRun(direction, steps)
                runState = RunStates.READY
            if runZeroPos:
                runState = RunStates.HOMING
                azZeroPos()
                runState = RunStates.READY
            azUpdated = False
            omitAzRun = False
        if elUpdated:
            if not omitElRun:
                if elToTarget.is_alive():
                   elToTarget.join()
                   runState = RunStates.READY
            if runZeroPos:
                runState = RunStates.HOMING
                elZeroPos()
                runState = RunStates.READY
            runZeroPos = False
            elZeroToPark = False
        if parkAndExit:
            runState = RunStates.ERROR
            if (currPosition != [-1.0, -1.0]):
                elZeroToPark = True
                elZeroPos()
            killSig = True
            break
        omitElRun = False
        elUpdated = False
        if stop():
            break

def StatusLED(stop):
    global runState
    global led1
    while (True):
        match runState:
            case RunStates.HOMING:
                GPIO.output(led1,GPIO.LOW)
                sleep(0.4)
                GPIO.output(led1,GPIO.HIGH)
                sleep(0.2)
            case RunStates.READY:
                GPIO.output(led1,GPIO.LOW)
                sleep(0.1)
                GPIO.output(led1,GPIO.HIGH)
                sleep(0.8)
            case RunStates.RUNNING:
                GPIO.output(led1,GPIO.LOW)
                sleep(0.2)
                GPIO.output(led1,GPIO.HIGH)
                sleep(0.1)
            case RunStates.ERROR:
                GPIO.output(led1,GPIO.LOW)
                sleep(0.8)
                GPIO.output(led1,GPIO.HIGH)
                sleep(0.1)
        if stop():
            for x in range(10):
                GPIO.output(led1,GPIO.LOW)
                sleep(0.1)
                GPIO.output(led1,GPIO.HIGH)
                sleep(0.1)
            break
        sleep(0.1)
    GPIO.output(led1,GPIO.LOW)

def serialListener(stop):
    global serialPort
    global baudRate
    global runState
    global killSig
    global killSigMsg
    global nextTarget
    global runZeroPos
    global parkAndExit
    global elZeroToPark
    
    data_str = ''
    omitPos = False
    ser = serial.Serial(port=serialPort, baudrate=baudRate, rtscts=True)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    sleep(0.1)
    ser.write(0x1b)
    initmsg = (f'[ZdBrg:Main] v{Version} - Homing...\n\r')
    zeropmsg = (f'AZ0.0 EL0.0\n\r')
    ser.write(initmsg.encode())
    while(runState == RunStates.HOMING):
        if (killSig):
            break
        sleep(0.5)
    if (not killSig):
        ser.write(zeropmsg.encode())
        sleep(0.1)
    ser.flush()
    while (True):
        if (ser.in_waiting > 0):
            try:
                data_fetch = ser.readline().decode('ascii')
            except:
                data_fetch = ''
            sleep(0.1)
            data_str = data_fetch
            data_fetch = ''
            if (data_str.startswith('SA SE')):
                saseresp = (f'SA SE\n\r')
                ser.write(saseresp.encode())
                if (currPosition[0] > 0 or currPosition[1] > 0):
                    nextTarget = [0.0, minElevation]
                data_str = ''
            elif (data_str.startswith('HOME')):
                hresp = (f'HOME\n\r')
                ser.write(hresp.encode())
                if (currPosition[0] > 0 or currPosition[1] > minElevation):
                    nextTarget = [0.0, minElevation]
                    sleep(8)
                runZeroPos = True
                data_str = ''
            elif (data_str.startswith('TERM')):
                tresp = (f'TERM\n\r')
                ser.write(tresp.encode())
                if (currPosition[0] > 0 or currPosition[1] > minElevation):
                    nextTarget = [0.0, minElevation]
                    sleep(8)
                elZeroToPark = True
                parkAndExit = True
                data_str = ''
            elif (data_str.startswith('SETAZ')):
                setAz = data_str.removeprefix('SETAZ')
                data_str = ''
                omitPos = True
                if (not setAz.replace('.', '', 1).isdigit()):
                    currPosition[0] = float(setAz)
                    setazresp = (f'AZ{str(setAz)}\n\r')
                    ser.write(setazresp.encode())
                else:
                    setazresp = (f'AZSET FAIL {str(setAz)}\n\r')
                    ser.write(setazresp.encode())
                setAz = ''
            elif (data_str.startswith('SETEL')):
                setEl = data_str.removeprefix('SETEL')
                data_str = ''
                omitPos = True
                if (not setEl.replace('.', '', 1).isdigit()):
                    currPosition[1] = float(setEl)
                    setelresp = (f'EL{str(setEl)}\n\r')
                    ser.write(setelresp.encode())
                else:
                    setelresp = (f'EL SET FAILED {str(setEl)}\n\r')
                    ser.write(setelresp.encode())
                setEl = ''
            elif (data_str.startswith('AZ EL')):
                elresp = (f'AZ{str(currPosition[0])} EL{str(currPosition[1])}\n\r')
                ser.write(elresp.encode())
                data_str = ''
                omitPos = True
            elif (data_str.startswith('AZ') and not omitPos): 
                splitcmd = data_str.split()
                data_str = ''
                if (runZeroPos == False and parkAndExit == False and len(splitcmd) == 2):
                    nopfixAz = splitcmd[0].removeprefix('AZ')
                    if (not nopfixAz.replace('.', '', 1).isdigit() or nopfixAz == ''):
                        azresp = (f'AZ{str(currPosition[0])}\n\r')
                        ser.write(azresp.encode())
                    else:
                        nextTarget[0] = float(nopfixAz)
                    if (splitcmd[1].startswith("EL")):
                        nopfixEl = splitcmd[1].removeprefix("EL")
                        splitcmd[1] = ''
                        splitcmd[0] = ''
                        if (not nopfixEl.replace('.', '', 1).isdigit() or nopfixEl == ''):
                            elresp = (f'EL{str(currPosition[1])}\n\r')
                            ser.write(elresp.encode())
                        elif (float(nopfixEl) < minElevation or float(nopfixEl) > maxElevation):
                            elresp = (f'EL-1.0\n\r')
                            ser.write(elresp.encode())
                        else:
                            nextTarget[1] = float(nopfixEl)
                data_str = ''
            ser.reset_output_buffer()
            omitPos = False
        if stop():
            break
    if (killSigMsg != ''):
        ser.write(f'[ZdBrg:Main] {killSigMsg}\n\r'.encode()) 
    ser.write('[ZdBrg:Main] Terminating...\n\r'.encode())
    ser.close()

elPosition = -1.0
azPosition = -1.0
killSig = False
killSigMsg = ''
nextTarget = [-1.0, -1.0]
currPosition = [-1.0, -1.0]

statusLedThread = Thread(target=StatusLED, args=(lambda: killSig, ))
elSensorThread = Thread(target=elSensor, args=(lambda: killSig, ))
azSensorThread = Thread(target=azSensor, args=(lambda: killSig, ))
serialListenerThread = Thread(target=serialListener, args=(lambda: killSig, ))
targetHandlerThread = Thread(target=targetHandler, args=(lambda: killSig, ))
statusLedThread.start()
serialListenerThread.start()
azSensorThread.start()
elSensorThread.start()
elHomeThread = Thread(target=elZeroPos, )
azZeroPosThread = Thread(target=azZeroPos, )
elHomeThread.start()
azZeroPosThread.start()
sleep(0.5)
elHomeThread.join(timeout=10.0)
if elHomeThread.is_alive():
    killSigMsg = 'Homing failed (el-timeout=>10s)'
    killSig = True
    runState = RunStates.ERROR
else:
    azZeroPosThread.join(timeout=25.0)
    if azZeroPosThread.is_alive():
        killSigMsg = 'Homing failed (az-timeout=>25s)'
        killSig = True
        runState = RunStates.ERROR
currPosition = [0.0, minElevation]
targetHandlerThread.start()
if(not killSig):
    runState = RunStates.READY

while (True):
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
if(not killSig):
    elZeroToPark = True
    elZeroPos()
killSig = True
azSensorThread.join(timeout=0.5)
elSensorThread.join(timeout=0.5)
serialListenerThread.join(timeout=0.5)
targetHandlerThread.join(timeout=0.5)
GPIO.cleanup()
if (killSigMsg):
    sys.exit(1)
sys.exit(0)
