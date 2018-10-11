# micropython-microbit-kitronik-robotics
Example MicroPython (for BBC micro:bit) code for the Kitronik All-in-one Robotics Board ( www.kitronik.co.uk/5641 )

## Motors

This package contains a function for driving standard motors forwards and backwards, with a speed setting of 0-100%:
```blocks
KitronikRoboticsBoard.motorOn(KitronikRoboticsBoard, 3, "forward", 10)
KitronikRoboticsBoard.motorOn(KitronikRoboticsBoard, 4, "reverse", 100)
```
There are also functions for driving stepper motors forwards and backwards by either a set number of steps, or to an angle relative to the start position:
```blocks
KitronikRoboticsBoard.stepperMotorTurnAngle(KitronikRoboticsBoard, "Stepper1", "forward", 180)
KitronikRoboticsBoard.stepperMotorTurnSteps(KitronikRoboticsBoard, "Stepper1", "reverse", 100)
```
Individual motor outputs can also be turned off:
```blocks
KitronikRoboticsBoard.motorOff(KitronikRoboticsBoard, 3)
```

## Servos

This package contains a function for driving both 180 and continuous rotation servos, with the ability to set the drive angle between 0 and 180 degrees:
```blocks
KitronikRoboticsBoard.servoWrite(KitronikRoboticsBoard, 4, 180)
KitronikRoboticsBoard.servoWrite(KitronikRoboticsBoard, 5, 0)
```
(Note: For continuous rotation servos, setting the drive angle to 0 will turn the servo at max speed in one direction, and setting it to 180 will turn it at max speed in the other direction)

## Other

This package also contains an 'emergency stop' function which turns off all motor and all servo outputs at the same time:
```blocks
KitronikRoboticsBoard.allOff(KitronikRoboticsBoard)
```

## License

MIT

## Supported Targets

BBC micro:bit