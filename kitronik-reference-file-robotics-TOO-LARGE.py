########################################
##                                    ##
##   THIS FILE IS FOR REFERENCE ONLY  ##
## USE OF COMMENTS MAKES FILE TOO BIG ##
##    FOR UPLOADING TO BBC MICROBIT   ##
##  USE 'robotics.py' FOR ACTUAL USE  ##
##                                    ##
########################################
from microbit import *
#Class for driving the Kitronik All-in-one Robotics Board
class KitronikRoboticsBoard:
    PRESCALE_REG = 0xFE #The prescale register address
    MODE_1_REG = 0x00 #The mode 1 register address
    # If you wanted to write some code that stepped through
    # the servos or motors then this is the Base and size to do that
    SRV_REG_BASE = 0x08
    MOT_REG_BASE = 0x28
    REG_OFFSET = 4
    # To get the PWM pulses to the correct size and zero
    # offset these are the default numbers.
    SERVO_MULTIPLIER = 226
    SERVO_ZERO_OFFSET = 0x66
    
    #The Robotics board can be configured to use different I2C addresses: 0x6C, 0x6D, 0x6E, 0x6F.
    #0x6C is the default value for the All-in-one Robotics Board (set as the chipAddress)
    chipAddress = 0x6C
    #A flag to allow us to initialise without explicitly calling the secret incantation
    initialised = False
    #a flag to establish whether a stepper motor call has been made before
    stepInit = False
    #Initialise stepStage, used along with stepInit for initialising the stepper motor function
    stepStage = 0
    stepper1Steps = 200 #Default value for the majority of stepper motors; can be altered if neccessary for a particular stepper motor
    stepper2Steps = 200 #Default value for the majority of stepper motors; can be altered if neccessary for a particular stepper motor
    
    #This initialisation function sets up the PCA9865 I2C driver chip to be running at 50Hz pulse repetition, and then sets the 16 output registers 0.
    #It should not need to be called directly be a user - the first servo or motor write will call it automatically.
    def __init(self):
            
        buf = bytearray(2)

        #First set the prescaler to 50 hz
        buf[0] = self.PRESCALE_REG
        buf[1] = 0x85 #50Hz
        i2c.write(self.chipAddress, buf, False)
        #Block write via the all leds register to turn off all outputs
        
        for blockReg in range(0xFA, 0xFE, 1):
            buf[0] = blockReg
            buf[1] = 0x00
            i2c.write(self.chipAddress, buf, False)
        #Set the mode 1 register to come out of sleep
        buf[0] = self.MODE_1_REG
        buf[1] = 0x01
        i2c.write(self.chipAddress, buf, False)
        #set the initalised flag so we dont come in here again automatically
        self.initialised = True
    
    #sets the requested servo to the reguested angle.
    #if the PCA has not yet been setup calls the initialisation routine
    def servoWrite(self, servo, degrees):
        if self.initialised is False:
            self.__init(self)
        buf = bytearray(2)
        calcServo = self.SRV_REG_BASE + ((servo - 1) * self.REG_OFFSET)
        HighByte = False
        PWMVal = (degrees * 100 * self.SERVO_MULTIPLIER) / (10000 + self.SERVO_ZERO_OFFSET)
        
        if (PWMVal > 0xFF):
            HighByte = True
        buf[0] = calcServo
        buf[1] = int(PWMVal)
        i2c.write(self.chipAddress, buf, False)
        buf[0] = calcServo + 1
        if (HighByte):
            buf[1] = 0x01
        else:
            buf[1] = 0x00
        i2c.write(self.chipAddress, buf, False)
    
    #Function to set the requested motor running in chosen direction at a set speed.
    #if the PCA chip has not yet been initialised calls the initialisation routine.
    def motorOn(self, motor, direction, speed):
        if self.initialised is False:
            self.__init(self)
        buf = bytearray(2)
        motorReg = self.MOT_REG_BASE + (2 * (motor - 1) * self.REG_OFFSET)
        HighByte = False
        OutputVal = speed * 40
        
        if direction == "forward":
            if OutputVal > 0xFF:
                HighByte = True
                HighOutputVal = int(OutputVal/256)
            buf[0] = motorReg
            buf[1] = int(OutputVal)
            i2c.write(self.chipAddress, buf, False)
            buf[0] = motorReg + 1
            if HighByte:
                buf[1] = HighOutputVal
            else:
                buf[1] = 0x00
            i2c.write(self.chipAddress, buf, False)
            
            for offset in range(4, 6, 1):
                buf[0] = motorReg + offset
                buf[1] = 0x00
                i2c.write(self.chipAddress, buf, False)
            
        elif direction == "reverse":
            if OutputVal > 0xFF:
                HighByte = True
                HighOutputVal = int(OutputVal/256)
            buf[0] = motorReg + 4
            buf[1] = int(OutputVal)
            i2c.write(self.chipAddress, buf, False)
            buf[0] = motorReg + 5
            if HighByte:
                buf[1] = HighOutputVal
            else:
                buf[1] = 0x00
            i2c.write(self.chipAddress, buf, False)
            
            for offset2 in range(0, 2, 1):
                buf[0] = motorReg + offset2
                buf[1] = 0x00
                i2c.write(self.chipAddress, buf, False)

    #Function to turn off the specified motor.
    def motorOff(self, motor):
        buf = bytearray(2)
        motorReg = self.MOT_REG_BASE + (2 * (motor - 1) * self.REG_OFFSET)
        
        for offset3 in range(0, 2, 1):
            buf[0] = motorReg + offset3
            buf[1] = 0x00
            i2c.write(self.chipAddress, buf, False)
        
        for offset4 in range(4, 6, 1):
            buf[0] = motorReg + offset4
            buf[1] = 0x00
            i2c.write(self.chipAddress, buf, False)

    #Function to turn off all motors and servos.
    def allOff(self):
        buf = bytearray(2)
        servoOffCount = 0
        servoRegCount = 0
        
        for motors in range(1, 5, 1):
            self.motorOff(self, motors)

        while servoOffCount < 8:
            for offset5 in range(0, 2, 1):
                buf[0] = self.SRV_REG_BASE + servoRegCount + offset5
                buf[1] = 0x00
                i2c.write(self.chipAddress, buf, False)

            servoRegCount += 4
            servoOffCount += 1

    #Sets the requested stepper motor to a chosen angle relative to the start position.
    #if the PCA chip has not yet been initialised calls the initialisation routine.
    def stepperMotorTurnAngle(self, stepper, direction, angle):
        angleToSteps = 0

        if self.initialised is False: 
            self.__init(self)

        #convert angle to motor steps, depends on which stepper is being turned to set the number of steps for a full rotation
        if stepper == "Stepper1":
            angleToSteps = ((angle - 1) * (self.stepper1Steps - 1)) / (360 - 1) + 1
        else:
            angleToSteps = ((angle - 1) * (self.stepper2Steps - 1)) / (360 - 1) + 1

        angleToSteps = int(angleToSteps)
        self._turnStepperMotor(self, stepper, direction, angleToSteps)

    #Sets the requested stepper motor to turn a set number of steps.
    #if the PCA chip has not yet been initialised calls the initialisation routine.
    def stepperMotorTurnSteps(self, stepper, direction, stepperSteps):
        if self.initialised is False: 
            self.__init(self)

        self._turnStepperMotor(self, stepper, direction, stepperSteps)

    #The function called to actually turn the stepper motor a set number of steps
    #This function uses a finite state machine (stepStage) to set each motor output to energise the coils of the stepper motor
    #in the correct sequence in order to continuously drive the stepper motor in a set direction
    #Each stepStage value (1-4) corresponds to particular motor outputs and directions being active (for either stepper output)
    def _turnStepperMotor(self, stepper, direction, steps):
        stepCounter = 0
        #This sets stepStage to 1 the FIRST time _turnStepperMotor is called - every other time it will 'remember' the previous position
        if self.stepInit is False:
            self.stepStage = 1 #stepStage determines which coils in the stepper motor will be energised (order is very important to ensure actual turning)
            self.stepInit = True

        #Loop to run until the number of motor steps set by the user is reached
        while stepCounter < steps:
            #This section uses the current stepStage and user selected Stepper motor to set which Robotics Board motor Output Address should be used
            if stepStage == 1 or stepStage == 3:
                if stepper == "Stepper1":
                    currentMotor = 1
                else:
                    currentMotor = 3
            else:
                if stepper == "Stepper1":
                    currentMotor = 2
                else:
                    currentMotor = 4

            #This section uses the current stepStage to set which direction the Robotics Board motor Output should be driven
            if stepStage == 1 or stepStage == 4:
                 currentDirection = "forward"
            else:
                currentDirection = "reverse"

            #Function call for the Robotics Board motor drive with the previously set currentmotor and currentDirection
            self.motorOn(self, currentMotor, currentDirection, 100)
            sleep(20)

            #This section progresses the stepStage depending on the user selected Stepper motor direction and previous stepStage
            if direction == "forward":
                if stepStage == 4: 
                    stepStage = 1
                else:
                    stepStage += 1
            elif direction == "reverse":
                if stepStage == 1: 
                    stepStage = 4
                else:
                    stepStage -= 1
            
            stepCounter += 1

#Test program will run forever
#On pressing button A on the micro:bit: 
#     A stepper motor connected to motor outputs 1 & 2 will turn 180 degrees
#     Motors connected to outputs 3 & 4 will turn in opposite directions at different speeds
#     4 servos will turn one direction, 4 will turn the other
#On pressing button B on the micro:bit: 
#     A stepper motor connected to motor outputs 1 & 2 will turn 100 steps in the opposite direction to button A
#     Motors connected to outputs 3 & 4 will turn off
#     8 servos will set to the 90 degree position
#On pressing button A+B on the micro:bit:
#     All motor and servo outputs will be turned off         
while True:
    theBoard = KitronikRoboticsBoard
    if button_a.is_pressed():
        theBoard.stepperMotorTurnAngle(theBoard, "Stepper1", "forward", 180)
        theBoard.motorOn(theBoard, 3, "forward", 10)
        theBoard.motorOn(theBoard, 4, "reverse", 100)
        theBoard.servoWrite(theBoard, 1, 180)
        theBoard.servoWrite(theBoard, 2, 180)
        theBoard.servoWrite(theBoard, 3, 180)
        theBoard.servoWrite(theBoard, 4, 180)
        theBoard.servoWrite(theBoard, 5, 0)
        theBoard.servoWrite(theBoard, 6, 0)
        theBoard.servoWrite(theBoard, 7, 0)
        theBoard.servoWrite(theBoard, 8, 0)
    if button_b.is_pressed():
        theBoard.stepperMotorTurnSteps(theBoard, "Stepper1", "reverse", 100)
        theBoard.motorOff(theBoard, 3)
        theBoard.motorOff(theBoard, 4)
        theBoard.servoWrite(theBoard, 1, 90)
        theBoard.servoWrite(theBoard, 2, 90)
        theBoard.servoWrite(theBoard, 3, 90)
        theBoard.servoWrite(theBoard, 4, 90)
        theBoard.servoWrite(theBoard, 5, 90)
        theBoard.servoWrite(theBoard, 6, 90)
        theBoard.servoWrite(theBoard, 7, 90)
        theBoard.servoWrite(theBoard, 8, 90)
    if button_a.is_pressed() and button_b.is_pressed():
        theBoard.allOff(theBoard)