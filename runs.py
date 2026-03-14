from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import * 
from functions import *
from test_functions import *

hub = PrimeHub(Axis.Y, Axis.Z)
hub.display.orientation(Side.BOTTOM)

def run1():
    rob = Robot(kp=5, ki=0, kd=10, turnKp=0.08, turnKi=0, turnKd=0, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=5, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    rob.accelDecel(65, -400)
    wait(10)
    rob.turn(-15, 125)
    wait(10)

    arm.run(-700)
    wait(400)
    arm.stop()
    wait(500)

    rob.sivuv(1000, 350)
    wait(10)
    arm.run_time(1000, 1000)
    wait(300)
    rob.pid(6, 100)
    wait(10)
    # rob.turn(35, 125)
    rob.sivuv(-300, 500)
    wait(300)
    rob.pid(10, -100)
    wait(10)
    shell.run(1000)
    wait(1000)
    shell.stop()
    wait(300)
    rob.pid(10, 200)
    wait(10)
    rob.turn(50, 125)
    wait(300)
    rob.pid(75, 525)
    wait(300)
    rob.shellButton(-180)

####

def run2():
    rob = Robot(kp=1, ki=0, kd=1, turnKp=0.5, turnKi=0, turnKd=0, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    # rob.pid(44, -525)
    rob.accelDecel(45, -525)
    wait(200)
    for i in range(4):
        arm.run_time(-700, 700)
        wait(200)
        arm.run_time(700, 700)
    wait(200)
    rob.pid(50, 525)

####

def run3():
    rob = Robot(kp=1, ki=0, kd=2, turnKp=0.5, turnKi=0, turnKd=0, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    rob.accelDecel(42, -525)
    wait(10)
    arm.run(-1000)
    wait(1000)
    arm.stop()
    wait(10)
    rob.pid(13, 525)
    wait(100)
    rob.pid(7, -100)
    wait(10)
    arm.run(1000)
    wait(1300)
    arm.stop()
    wait(100)
    rob.pid(4, -300)
    wait(10)
    shell.run_time(1100, 2000)
    wait(10)
    rob.pid(35, 525)
    wait(10)
    arm.run(700)
    wait(10)
    arm.stop()
    rob.stopColor("stopYellow", 180)
####

def run4():
    rob = Robot(kp=1, ki=0, kd=2, turnKp=0.5, turnKi=0, turnKd=0,shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # H igh G
    wait(400)

    rob.accelDecel(22, 525)
    wait(300)
    rob.turn(90, 125)
    wait(300)
    rob.accelDecel(62, 525)
    wait(300)
    rob.turnWhileShell(-90, 90, 1000, 125)
    wait(300)
    # rob.pid(8, 150)
    rob.accelDecel(4, 300)
    wait(300)
    rob.shellTurn(-10, -400)
    wait(300)
    arm.run_time(-660, 3000)
    wait(300)
    rob.pid(10,-300)
    wait(400)
    rob.turn(-53, 200)
    wait(300)
    rob.pid(7, -100)
    wait(1000)
    rob.pid(2, 100)
    wait(1000)
    rob.turn(-20, 125)
    wait(200)
    rob.accelDecel(100, 525)
    wait(200)
    rob.stopColor("stopYellow", 365, 800)

###

def run5():
    rob = Robot(kp=2, ki=0, kd=5, turnKp=1, turnKi=0, turnKd=2,shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    rob.accelDecel(70, -525)
    wait(300)
    rob.turn(48, 125)
    wait(300)

    rob.ziun(1, 800, 660)
    arm.run_time(1000, 1500)
    wait(300)
    rob.pid(10, 1000)
    wait(300)
    arm.run_time(1000, 600)
    wait(300)
    rob.turn(-50, 125)
    wait(300)
    rob.pid(10, 525)
    wait(300)
    rob.turn(-90, 125)
    wait(300)
    rob.pid(10, 525)
    wait(300)
    rob.turn(90, 500)
    wait(300)
    rob.turn(-40, 500)
    wait(300)
    rob.turn(40, 125)

###

def run6():
    rob = Robot(kp=1, ki=0, kd=2, turnKp=0.5, turnKi=0, turnKd=0, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100)
    hub.speaker.beep(800, 80)
    wait(10)
    hub.speaker.beep(1100, 120)
    wait(400)

    rob.accelDecel(50, -400)
    wait(10)
    rob.shellTurn(-180, 600)
    wait(10)
    rob.accelDecel(45, -400)
    wait(10)
    rob.pid(2, 100)
    wait(10)
    arm.run_time(-1000, 1000)
    wait(10)
    rob.turn(78, 125)
    wait(100)
    rob.pid(10, 50)
    wait(10)
    arm.run_time(400, 2000)
    wait(10)
    arm.run_time(-400, 1500)
    # wait(10)
    # rob.pid(5, 50)
    # wait(10)
    # arm.run_time(400, 1000)
    wait(10)
    rob.pid(10, -50)
    wait(10)
    rob.turn(-81, 125)
    wait(10)
    rob.accelDecel(60, 525)
    wait(10)
    rob.turn(-50, 125)
    wait(100)
    rob.accelDecel(35, 300)
    wait(10)
    rob.shellTurn(-180, 600)
    # wait(10)
    # arm.run_time(1000, 2000)

###

def run7():
    rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)

    rob.pid(50,-500)
    wait(400)
    rob.pid(3,500)
    wait(400)
    arm.run_time(660, 1000)
    wait(400)
    rob.pid(4,650)
    wait(200)
    rob.pid(5, -650)
    wait(200)
    # rob.pid(15,525)
    # wait(200)
    rob.pid(45, 525)

###

def run8():
    rob = Robot(kp=2, ki=0, kd=10, turnKp=1.1, turnKi=0, turnKd=0,shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    rob.accelDecel(21, -525)
    wait(300)
    rob.turn(-90, 125)
    wait(300)
    rob.accelDecel(55, -525)
    wait(300)
    rob.turn(110, 150)
    wait(300)
    rob.pid(8, -100)
    wait(300)
    rob.turn(-20, 125)
    wait(300)
    arm.run_time(700, 1500)
    wait(300)
    rob.turn(15, 125)
    wait(300)
    rob.pid(5, -100)
    wait(300)
    rob.pid(10, 525)
    wait(300)
    rob.turn(-40, 125)
    wait(300)
    rob.accelDecel(80, -525)

###

def test():
    # rob = Robot(kp=0.08, ki=0, kd=0.1, shellKp=2, shellKi=0, shellKd=10, shellTol=0, tol=10, wait_time=1)
    rob = Robot(kp=2, ki=0, kd=5, turnKp=1, turnKi=0, turnKd=0,shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    # rob.turnWhileShell(90, -90, 600, 125)
    # arm.run_time(-660, 3000)
    # rob.pid()
    # arm.run_time(1000, 1500)
    # wait(300)
    # arm.run_time(-1000, 300)
    # wait(300)
    # rob.pid(10, 100)
    # rob.moveWhileShell(-90, 80, 600, -525)
    # shell.run(800)
    wheels.drive(1000, 0)
    wait(100000)
    # wait(100000)

###

def battery():
    rob = Robot()
    rob.battery_percent()
    # hub.display.icon(Icon.HAPPY)
    

######

# run1()

# run2()

# run3()

# run4()

# run5()

# run6()

# run7()

# run8()

# test()

# battery()
