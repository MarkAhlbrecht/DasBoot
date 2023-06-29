#
# Das Boot Circuit Python Controller
#
import time
import board
from adafruit_seesaw import seesaw, rotaryio, digitalio

import displayio
import terminalio
from adafruit_display_text import label
import adafruit_displayio_sh1107

from digitalio import DigitalInOut, Direction, Pull

import busio

from math import cos, pi

import PID

###
### ENCODER INIT
###
seesaw = seesaw.Seesaw(board.I2C(), addr=0x36)

seesaw_product = (seesaw.get_version() >> 16) & 0xFFFF
#print("Found product {}".format(seesaw_product))
if seesaw_product != 4991:
    print("Wrong firmware loaded?  Expected 4991")

seesaw.pin_mode(24, seesaw.INPUT_PULLUP)
button = digitalio.DigitalIO(seesaw, 24)
button_held = False

encoder = rotaryio.IncrementalEncoder(seesaw)
last_position = -1

buttonPress = False
buttonLongPress = False
longPressTime = 1.0

inputState = "IDLE"

######
# MODE INIT
######
modes = ["STANDBY", "AUTO","MANUAL","COMM","CONTROL"]
nModes = 5
targetMode = 0

mode = "STANDBY"
modeNum = 1

###
### Display Init
###

displayio.release_displays()
# oled_reset = board.D9

# Use for I2C
i2c = board.I2C()
display_bus = displayio.I2CDisplay(i2c, device_address=0x3C)

# SH1107 is vertically oriented 64x128
WIDTH = 128
HEIGHT = 64
BORDER = 2

display = adafruit_displayio_sh1107.SH1107(
    display_bus, width=WIDTH, height=HEIGHT, rotation=0
)

# Make the display context
splash = displayio.Group()
display.show(splash)


line1 = label.Label(terminalio.FONT, text=" "*20, color=0xFFFFFF, x=8, y=8+0)
splash.append(line1)
line1.text = "Line1"

line2 = label.Label(terminalio.FONT, text=" "*20, color=0xFFFFFF, x=8, y=8+12)
splash.append(line2)
line2.text = "Line2"

line3 = label.Label(terminalio.FONT, text=" "*20, color=0xFFFFFF, x=8, y=8+2*12)
splash.append(line3)
line3.text = "Line3"

line4 = label.Label(terminalio.FONT, text=" "*20, color=0xFFFFFF, x=8, y=8+3*12)
splash.append(line4)
line4.text = "Line4"

line5 = label.Label(terminalio.FONT, text=" "*20, color=0xFFFFFF, x=8, y=8+4*12)
splash.append(line5)
line5.text = "Line5"



time.sleep(0.5)

blankLines = [" ", " ", " ", " ", " "]
lines = [" ", " ", " ", " ", " "]
nLines = 5


line1.text = blankLines[0]
line2.text = blankLines[1]
line3.text = blankLines[2]
line4.text = blankLines[3]
line5.text = blankLines[4]

###
### MOTOR INIT
###
led = DigitalInOut(board.D13)
led.direction = Direction.OUTPUT

motorEnable = DigitalInOut(board.D10)
motorEnable.direction = Direction.OUTPUT

motorDrive1 = DigitalInOut(board.D11)
motorDrive1.direction = Direction.OUTPUT

motorDrive2 = DigitalInOut(board.D12)
motorDrive2.direction = Direction.OUTPUT

def motor_forward():
    motorEnable.value = True
    motorDrive1.value = True
    motorDrive2.value = False
    return

def motor_reverse():
    motorEnable.value = True
    motorDrive1.value = False
    motorDrive2.value = True
    return

def motor_stop():
    motorEnable.value = False
    motorDrive1.value = False
    motorDrive2.value = False
    return

#motor_forward()
#time.sleep(2.0)
#motor_reverse()
#time.sleep(2.0)
motor_stop()

motorFwd = True
motorOn = False
motorTimeManual = 1.0
motorDriveTime = 0

###
### SERIAL INIT
###
uart = busio.UART(board.TX, board.RX, baudrate=9600,timeout=0.01)

commSimOn = True
firstSim = True
simHeadingBase = 270.0
simHeadingSwing = 30.0
simHeadingPeriod = 20.0
simTurnRate = 1.5
simHeading = 0.0
prevSimHeading = 270.0
prevSimTR = 0.0
prevSimTime = time.monotonic()
simSpeed = 4.5
K_TR = 36*0.12*0.044*simSpeed

words = [""]
sentence = ""
commBuffer = ""
measHeading = 0
measHeadingTime = 0
measTurnRate = 0.0

###
### AP INIT
###
hdgTarget = 0
turnRateTarget = 0

###
### CONTROLLER INIT
###
###
### Controller
###
trMax = 5.0 # deg/sec
trHdgMax = 45.0
trDeadZone = 0.25
hdgDeadZone = 1.0
trHdgCoef = -1.0*trMax/trHdgMax

KpHdg = 0.1
KiHdg = 0.0
KdHdg = 0.000
hdgController = PID.PID()
hdgController.SetKp(KpHdg)
hdgController.SetKi(KiHdg)
hdgController.SetKd(KdHdg)
hdgControllerDelay = 50
ctrlOutput = 0

turnKp = -0.22
turnKi = 0
turnKd = 0
turnController = PID.PID()
turnController.SetKp(turnKp)
turnController.SetKi(turnKi)
turnController.SetKd(turnKd)
turnCtrlOutput = 0

# Control Modify Mode
ctlModes = ["trMax", "trHdgMax", "trDeadZone", "turn Prop", "Turn Intg"];
nCtlModes = 5
ctlMode = 0
ctlUIState = "Select"

###
### MAIN LOOP
###

prevTime = time.monotonic()
nextSendTime = time.monotonic()
nextCtrlSampleTime = time.monotonic()
driveDelayCoef = 5.0
rcvdHeading =0
motorSpeed = 0

while True:

    elapsedTime = time.monotonic()
    deltaTime = time.monotonic() - prevTime
    prevTime = elapsedTime
    #motorSpeed = 0
    #print(deltaTime)

    #
    # ENCODER HANDLING
    #
    # negate the position to make clockwise rotation positive
    position = -encoder.position

    buttonPress = False
    buttonLongPress = False
    buttonRelease = False

    if position != last_position:
        encoderCnt = position
        prevEncoderCnt = last_position
        last_position = position
        #print("Position: {}".format(position))
    else:
        prevEncoderCnt = encoderCnt

    if not button.value and not button_held:
        button_held = True
        buttonTime = elapsedTime
#        print("Button pressed")

    if button.value and button_held:
        button_held = False
        buttonRelease = True
#        print("Button Released")
    if button.value:
        buttonTime = elapsedTime

    if (inputState == "IDLE"):
        if button_held and (elapsedTime - buttonTime > longPressTime):
            inputState = "LONG"
            buttonLongPress = True
        elif buttonRelease:
            buttonPress = True

    elif (inputState == "LONG"):
        if buttonRelease:
            inputState = "IDLE"

    #line1.text = "Position: {}".format(position)

    #
    # COMM SIMULATOR
    #
    if (commSimOn):

        deltaTR = K_TR*(motorSpeed/100)*deltaTime
        simTurnRate = prevSimTR + deltaTR
        prevSimTR = simTurnRate
        simDeltaHeading = simTurnRate*deltaTime
        simHeading = prevSimHeading + simDeltaHeading
        if simHeading > 360.0:
            simHeading = simHeading - 360
        if simHeading < 0:
            simHeading = simHeading + 360
        prevSimHeading = simHeading
        prevSimTime = elapsedTime

        if (elapsedTime > nextSendTime):
            nextSendTime = elapsedTime + 0.5
            #simDeltaHeading = simHeadingSwing * cos(2*pi*(elapsedTime/simHeadingPeriod))
            #simHeading = simHeadingBase + simDeltaHeading
            #print(deltaHeading)
            #simTurnRate = (simHeading - prevSimHeading)/(elapsedTime - prevSimTime)
            #prevSimHeading = simHeading
            #prevSimTime = elapsedTime
            if firstSim:
                firstSim = False
            else:
                msg = "$INHDM,{0},T,{1},A*FF\n".format(simHeading,simTurnRate)
                binMsg = bytes(msg,"ascii")
                uart.write(binMsg)

    #
    # COMM READ
    #
    measUpdate = 0
    data = uart.read(32)  # read up to 32 bytes
    # print(data)  # this is a bytearray type

    if data is not None:
        led.value = True

        data_string = ''.join([chr(b) for b in data])
        nCommChar = len(data)
        nCommBuffer = len(commBuffer)

        #print(f"[{data}]")
        #print(f"{nCommChar}:{nCommBuffer}\n" )
        commBuffer += data_string

        idx = commBuffer.rfind('$')
        if (idx > 0):
            commBuffer = commBuffer[idx:]
        idx = commBuffer.rfind('\n')
        if (idx > 0):
            commBuffer = commBuffer[:idx]
            sentence = commBuffer
            words = sentence.split(',')
            if (words[0] == "$INHDM" ):
                try:
                    rcvdHeading = float(words[1])
                    measTurnRate = float(words[3])
                    measHeadingTime = elapsedTime
                    measHeading = rcvdHeading
                    measUpdate = 1
                except:
                    pass
        #print(words)
        #print("->{0} {1} {2}".format(measHeadingTime,measHeading,measTurnRate))

        # convert bytearray to string
        #data_string = ''.join([chr(b) for b in data])
        #print(data_string, end="")

        led.value = False

        if nCommBuffer > 256:
            #print("Resetting")
            commBuffer = ""
    else:
        data_string = ""
        nCommChar = 0
    if measUpdate == 0:
        measDelay = elapsedTime - measHeadingTime
        coast = measTurnRate * measDelay
        measHeading = rcvdHeading + coast
        #print("-- Heading Coast --")
        #print(f"  {measDelay} {rcvdHeading} {coast} {measHeading}")

    #
    # MODE STATE MACHINE
    #
    if buttonLongPress:
        mode="MODE_SEL"

    if (mode == "MODE_SEL"):
        modeNum =0
#         print(f"Encoder: {encoderCnt}")
#         if (encoderCnt > prevEncoderCnt):
        if (encoderCnt > prevEncoderCnt):
            targetMode +=1
            if(targetMode >= nModes):
                targetMode = 0
        if (encoderCnt < prevEncoderCnt):
            targetMode -=1
            if(targetMode < 0):
                targetMode = nModes-1
        #print(f"Mode: {mode} -> TargetMode: {modes[targetMode]}")
        for m in range(nModes):
            if(m==targetMode):
                lines[m] = ("==> " + modes[m])
            else:
                lines[m] = ("    " + modes[m])

        if (buttonPress):
            mode = modes[targetMode]
        #prevEncoderCnt = encoderCnt
    elif (mode == "STANDBY"):
        modeNum=1
        #         print(f"Mode: {mode}")

        #display.text(mode, 20, 20, 0, size=2)

        for m in range(nLines):
            lines[m] = blankLines[m]
        lines[4] = mode

        if (encoderCnt > prevEncoderCnt):
            hdgTarget += 10*(encoderCnt - prevEncoderCnt)
            if(hdgTarget >180):
                hdgTarget -= 360

        if (encoderCnt < prevEncoderCnt):
            hdgTarget += 10*(encoderCnt - prevEncoderCnt)
            if(hdgTarget <-179):
                hdgTarget += 360

        hdgTarget360 = hdgTarget
        if(hdgTarget360 < 0):
            hdgTarget360 = 360 + hdgTarget360
        targetString = "    {0:03d}      {1:+03d} ".format(int(hdgTarget360), int(turnRateTarget))

        hdg360 = measHeading
        if(hdg360 < 0):
            hdg360 = 360 + hdg360
        hdgString = "  - {0:03d} -  - {1:+03d} -".format(int(hdg360),int(measTurnRate))

        lines[1] = targetString
        lines[2] = hdgString

        #print(hdgString)
        #print(targetString)

        #display.text(hdgString, 40, 40, 0, size=8)
        #display.text(targetString, 40, 120, 0, size=8)

        #if (encoderBtn == False and encoderBtnCnt == 1):
        #    hdgTarget = att_deg[2]

        #if (encoderBtn == False and encoderBtnCnt > encoderShortPress and standbyDelay == 0):
        #    mode = "AUTO"
        #    standbyDelay = 15
        if buttonPress:
            mode = "AUTO"

        pass

    elif (mode == "AUTO"):
        modeNum = 2
        for m in range(nLines):
            lines[m] = blankLines[m]
        lines[4] = mode

        if (encoderCnt > prevEncoderCnt):
            hdgTarget += 10*(encoderCnt - prevEncoderCnt)
            if(hdgTarget >180):
                hdgTarget -= 360

        if (encoderCnt < prevEncoderCnt):
            hdgTarget += 10*(encoderCnt - prevEncoderCnt)
            if(hdgTarget <-179):
                hdgTarget += 360




        # Controller
        error = measHeading - hdgTarget
        if error > 180:
            error = error - 360
        elif error <= -180:
            error = error + 360

        turnRateTarget = error*trHdgCoef
        if turnRateTarget > trMax:
            turnRateTarget = trMax
        if turnRateTarget < -1*trMax:
            turnRateTarget = -1*trMax

        trErr = measTurnRate - turnRateTarget
        turnCtrlOutput = turnController.GenOut(trErr)

        if abs(trErr) > trDeadZone and abs(error) > hdgDeadZone:
            motorDriveTime = elapsedTime + abs(turnCtrlOutput)
            if turnCtrlOutput > 0:
                motorFwd = True
            else:
                motorFwd = False


        #hdgDrive = abs(turnCtrlOutput)
        #ctrlTurnOutput = turnController.GenOut(error);

        #if (abs(error) < 45) and elapsedTime > nextCtrlSampleTime:
        #    motorDriveTime = elapsedTime + hdgDrive
        #    nextCtrlSampleTime = motorDriveTime + driveDelayCoef * hdgDrive
        #    if ctrlOutput > 0:
        #        motorFwd = True
        #    else:
        #        motorFwd = False
        #

        hdgTarget360 = hdgTarget
        if(hdgTarget360 < 0):
            hdgTarget360 = 360 + hdgTarget360
        targetString = "    {0:03d}      {1:+03d} ".format(int(hdgTarget360), int(turnRateTarget))

        hdg360 = measHeading
        if(hdg360 < 0):
            hdg360 = 360 + hdg360
        #hdgString = "  - {0:03d} -  - {1:+03d} -".format(int(hdg360),int(measTurnRate))
        #print(f"e:{error} t:{elapsedTime} {motorDriveTime} {nextCtrlSampleTime}")

#        if (abs(measTurnRate)> 1.0):
#            turnDriveTime = abs(measTurnRate)/50
#            print(turnDriveTime)
#            motorDriveTime = elapsedTime + turnDriveTime
#            motorOn = True
#            if(measTurnRate > 0):
#                motorFwd = False
#                trPreString = "  "
#                trPostString = "<="
#            else:
#                motorFwd = True
#                trPreString = "=>"
#                trPostString = "  "

        #hdgString = "  {0}{1:03d}{2}  {3}{4:+03d}{5}".format(ctlPreString,int(hdg360),ctlPostString,trPreString,int(measTurnRate),trPostString)

        #lines[1] = targetString
        #lines[2] = hdgString

        ctlPreString = "  "
        ctlPostString = "  "
        trPreString = "  "
        trPostString = "  "

        if (motorDriveTime > elapsedTime):
            if motorFwd:
                motorSpeed = 100
                motor_forward()
                ctlPreString = "=>"
            else:
                motorSpeed = -100
                motor_reverse()
                ctlPostString = "<="
        else:
            motor_stop()
            motorSpeed = 0
            motorOn = False

        hdgString = "  {0}{1:03d}{2}  {3}{4:+03d}{5}".format(ctlPreString,int(hdg360),ctlPostString,trPreString,int(measTurnRate),trPostString)

        #print(targetString)
        #print(hdgString)

        lines[1] = targetString
        lines[2] = hdgString

        if buttonPress:
            mode = "STANDBY"
            motor_stop()
        pass

    elif (mode == "MANUAL"):
        modeNum = 3
        if (encoderCnt > prevEncoderCnt):
            motorDriveTime = elapsedTime + motorTimeManual
            motorFwd = True
            motorOn = True

        if (encoderCnt < prevEncoderCnt):
            motorDriveTime = elapsedTime + motorTimeManual
            motorFwd = False
            motorOn = True

        if buttonPress:
            motorDriveTime = elapsedTime
            motorFwd = False
            motorOn = False

        for m in range(nLines):
            lines[m] = blankLines[m]
        lines[4] = mode

        if (motorDriveTime > elapsedTime):
            if motorFwd:
                motorSpeed = 100
                motor_forward()
                lines[0] = "--->Forward"
            else:
                motorSpeed = -100
                motor_reverse()
                lines[0] = "    Reverse<---"
        else:
            motorSpeed = 0
            motor_stop()

        #prevEncoderCnt = encoderCnt
        pass

    elif (mode == "COMM"):
        modeNum = 4
        for m in range(nLines):
            lines[m] = blankLines[m]
        lines[0] = data_string
        lines[1] = str(nCommChar)
        lines[2] = "{0} {1}".format(measHeading,measTurnRate)
        lines[3] = str(elapsedTime - measHeadingTime)
        lines[4] = mode
        pass

    elif (mode == "CONTROL"):
        modeNum = 5

        for m in range(nLines):
            lines[m] = blankLines[m]
        lines[4] = mode


#         if (encoderCnt > prevEncoderCnt):
        if ctlUIState == "Select":
            if (encoderCnt > prevEncoderCnt):
                ctlMode +=1
                if(ctlMode >= nCtlModes):
                    ctlMode = 0
            if (encoderCnt < prevEncoderCnt):
                ctlMode -=1
                if(ctlMode < 0):
                    ctlMode = nCtlModes-1
            #print(f"Mode: {mode} -> TargetMode: {modes[targetMode]}")

            for m in range(nCtlModes):
                if(m==ctlMode):
                    lines[m] = ("==> " + ctlModes[m])
                else:
                    lines[m] = ("    " + ctlModes[m])

            if (buttonPress):
                #edit
                ctlUIState = "Entry"

        elif ctlUIState == "Entry":
            lines[0] = ctlModes[ctlMode]
            if ctlMode == 0:
                cValue = trMax
            elif ctlMode == 1:
                cValue = trHdgMax
            elif ctlMode == 2:
                cValue = trDeadZone
            elif ctlMode == 3:
                cValue = turnController.Kp
            elif ctlMode == 4:
                cValue = turnController.Ki

            #lines[1] = "{0}".format(cValue)

            if (encoderCnt > prevEncoderCnt):
                cValue = cValue*1.2
            elif (encoderCnt < prevEncoderCnt):
                cValue = cValue*0.83

            lines[1] = "{0}".format(cValue)

            if ctlMode == 0:
                trMax = cValue
                trHdgCoef = -1.0*trMax/trHdgMax
            elif ctlMode == 1:
                trHdgMax = cValue
                trHdgCoef = -1.0*trMax/trHdgMax
            elif ctlMode == 2:
                trDeadZone = cValue
            elif ctlMode == 3:
                turnController.Kp = cValue
            elif ctlMode == 4:
                turnController.Ki = cValue

            if (buttonPress):
                #return
                ctlUIState = "Select"
        #prevEncoderCnt = encoderCnt
        print(f"CTL SET: {encoderCnt} {ctlUIState} {ctlMode}")
        pass

    print(f"1,{elapsedTime},{modeNum},{measHeading},{measTurnRate},{measUpdate},{hdgTarget360},{turnRateTarget},{ctrlOutput},{turnCtrlOutput},{motorSpeed}")

    line1.text = lines[0]
    line2.text = lines[1]
    line3.text = lines[2]
    line4.text = lines[3]
    line5.text = lines[4]
    prevEncoderCnt = encoderCnt
    #time.sleep(0.2)