from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis, Icon
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import * 
from functions import *

hub = PrimeHub(Axis.Y, Axis.Z)
hub.display.orientation(Side.BOTTOM)

def run1():
    # rob = Robot(kp=5, ki=0, kd=10, turnKp=0.08, turnKi=0, turnKd=0, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=5, turn_wait_time=1)
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=10, turnKi=0, turnKd=25, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(60)
    hub.speaker.beep(600, 80)
    wait(100)

    rob.pid(62, -50)
    wait(10)
    rob.turn(-15, 50)
    wait(10)

    arm.run(-700)
    wait(400)
    arm.brake()
    wait(500)

    # rob.pid(2, -30)
    # wait(10)
    rob.turn(-40, 50)
    wait(10)
    arm.run_time(1000, 2000)
    wait(300)
    rob.pid(6, 30)
    wait(10)
    rob.turn(45, 50)
    wait(300)
    rob.pid(9, -50)
    wait(10)
    shell.run(1000)
    wait(1000)
    shell.stop()
    wait(300)
    rob.pid(10, 30)
    wait(10)
    rob.turn(50, 50)
    wait(300)
    rob.pid(75, 75)
    wait(10)
    rob.shellButton(-180)

####

def run2():
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=6, turnKi=0, turnKd=15, shellKp=2, shellKi=0, shellKd=10, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(40)
    hub.speaker.beep()
    wait(100)

    rob.pid(44, -50)
    wait(200)
    for i in range(4):
        arm.run_time(-700, 700)
        wait(300)
        arm.run_time(700, 700)
    wait(200)
    rob.pid(50, 50)

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

    # rob.accelDecel(42, -525)
    rob.pid(42, -50)
    wait(10)
    arm.run(-1000)
    wait(1000)
    arm.stop()
    wait(10)
    rob.pid(13, 50)
    wait(100)
    rob.pid(7, -30)
    wait(10)
    arm.run(1000)
    wait(1300)
    arm.stop()
    wait(100)
    rob.pid(1, -30)
    wait(10)
    shell.run_time(1100, 2000)
    wait(10)
    rob.pid(35, 50)
    wait(10)
    arm.run(700)
    wait(10)
    arm.stop()
    # rob.stopColor("stopYellow", 180)

####

def run4():
    rob = Robot(kp=2, ki=0, kd=10, turnKp=7, turnKi=0, turnKd=15, shellKp=40, shellKi=0, shellKd=80, shellTol=2, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(60)
    hub.speaker.beep(600, 80)
    wait(100)

    # rob.turn(90, 30)
    # rob.shellTurn(90)

    rob.pid(22, -50)
    wait(100)
    rob.turn(-90, 40)
    wait(100)
    rob.pid(65, 50)
    wait(100)
    rob.turnWhileShell(-90, 90, 100, 50)
    wait(100)
    rightwheel.dc(30)
    leftwheel.dc(30)
    wait(750)
    wheels.brake()
    wait(100)
    rob.shellTurn(20)
    wait(100)
    arm.run_time(-660, 3000)
    wait(10)
    rob.shellTurn(-20)
    wait(10)
    rob.pid(10, -30)
    wait(10)
    rob.turn(-50, 40)
    wait(10)
    rob.pid(11, -30)
    wait(600)
    # rob.turn(-20, 50)
    # wait(10)
    rob.pid(15, 60)
    wait(10)
    rob.turn(-45, 50)
    wait(10)
    rob.pid(20, 50)
    wait(10)
    rob.turn(30, 50)
    wait(10)
    rob.pid(70, 50)
    wait(10)
    # rob.stopColor("stopYellow", 180, 100r)
    rob.shellTurn(90 )


###

def run5():
    rob = Robot(kp=2.5, ki=0, kd=16, turnKp=8, turnKi=0, turnKd=20, shellKp=12, shellKi=0, shellKd=0, shellTol=5, turnTol=5, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100)

    rob.pid(30, -50)
    wait(10)
    rob.shellTurn(90)
    wait(10)
    rob.pid(70,-50)
    wait(10)
    rob.pid(2,25) #המהירות המינמלית של הרובוט
    wait(10) 
    rob.turn(90,40)
    wait(10)
    arm.run_time(-1000,1000)
    wait(10)
    rob.pid(17,30)
    wait(10)
    arm.run_time(1000,1000)
    wait(10)
    rob.pid(17,-30)
    wait(10)
    rob.turn(-90)
    wait(10)
    rob.pid(100,70)

    
###

def run6():
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=6.5, turnKi=0, turnKd=20, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)
    wait(100)


    arm.run_time(-1000, 1100)
    wait(10)
    arm.run_time(1000, 700)
    wait(10)
    rob.pid(10, -30)
    wait(10)
    rob.turn(-22, 40)
    wait(10)
    rob.pid(58, -50)
    wait(10)
    rob.turn(68, 40)
    wait(300)
    rob.pid(40, -100)
    wait(40)
    arm.run_time(1000, 2000)
    wait(300)
    rob.pid(18, 30)
    wait(10)
    rob.turn(-70, 40)
    wait(10)
    rob.pid(30, 50)
    wait(10)
    rob.turn(25, 50)
    wait(10)
    rob.pid(15, -40)
    wait(300)
    rob.turn(-55, 50)
    wait(10)
    rob.turn(55, 40)
    wait(10)
    rob.pid(50, 60)

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
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=9, turnKi=0, turnKd=18, shellKp=0, shellKi=0, shellKd=0, shellTol=0, turnTol=10, turn_wait_time=1)

    hub.speaker.volume(20)
    hub.speaker.beep(600, 80)  # Low C
    wait(100)
    hub.speaker.beep(800, 80)  # Mid E
    wait(10)
    hub.speaker.beep(1100, 120) # High G
    wait(400)   

    # arm.run_time(-1000, 2000)
    # wait(30)
    # rob.pid(12,-50)
    # wait(30)
    # rob.turn(-85,40)
    # wait(30)
    # rob.pid(67,-50)
    # # wait(30)
    # # rob.turn(-31, 40)
    # # wait(30)
    # # rob.pid(50, -60)
    # wait(30)
    # rob.turn(120,40)
    # wait(30)
    # # rob.turn(-30,40)
    # # wait(30)
    # rob.turn(-5,40)
    # wait(30)
    # rob.pid(5, -50)
    # wait(30)
    # arm.run_time(1000, 2000)
    # # wait(30)
    # # rob.turn(30, 50)
    # wait(30)
    # rob.pid(10, -50)
    # wait(30)
    # rob.pid(10, 50)

    # rob.turn(90, 50)
    #arm.run_time(1000,1000)
    
    rob.pid(20, -50)
    wait(30)
    rob.turn(-30, 30)
    wait(30)
    rob.pid(5, -50)
    wait(30)
    rob.turn(-20, 30)
    wait(30)
    rob.pid(15, -50)
    wait(30)
    rob.pid(15, 50)
    wait(30)
    rob.turn(-48, 40)
    wait(10)
    rob.pid(65, -50)
    wait(10)
    arm.run_time(-1000, 2000)
    wait(30)
    rob.turn(130, 50)
    wait(10)
    rob.pid(15, -50)
    

###

def test():
    rob = Robot(kp=1, ki=0, kd=0.1, turnKp=8, turnKi=0, turnKd=15, shellKp=22, shellKi=0, shellKd=20, shellTol=2, turnTol=10, turn_wait_time=1)
    # rob_t = Robot_test(kp=1, ki=0, kd=0.1, turnKp=1, turnKi=0, turnKd=0, shellKp=22, shellKi=0, shellKd=20, shellTol=2, turnTol=10, turn_wait_time=1)
    # rob.pid(100000,100)
    wheels.drive(1000, 0)
    wait(1000000)


###

def battery():
    rob = Robot()
    rob.battery_percent()
    # for i in range(10):
    #     print(hub.battery.voltage())
    #     wait(100)
    # hub.display.icon(Icon.HAPPY)
    

######


#run1()

# run2()

# run3()

# run4()

# run5()

# run6()

# run7()

# run8()

# test()

battery()
