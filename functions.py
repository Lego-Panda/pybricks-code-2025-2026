from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import *

hub = PrimeHub(Axis.Y, Axis.Z)

def singnum(value):
    return value / abs(value)

class Robot:
    def __init__(self,kp=0, ki=0, kd=0,turnKp=0, turnKi=0, turnKd=0,shellKp=0,shellKi=0,shellKd=0, shellTol=0, turnTol=0, turn_wait_time=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.turnKp = turnKp
        self.turnKi = turnKi
        self.turnKd = turnKd
        self.turnTol = turnTol
        self.turn_wait_time = turn_wait_time
        self.shellKp = shellKp
        self.shellKi = shellKi
        self.shellKd = shellKd
        self.shellTol= shellTol

        self.errorSum = 0
        self.lastError = 0

    def pid(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        while abs(leftwheel.angle()) < distance / CIRCUMFERENCE * 360:
            error = 0 - hub.imu.heading()

            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * (error - self.lastError)

            rightwheel.run(int(speed + pidValue))
            leftwheel.run(int(speed - pidValue))

            self.lastError = error
            self.errorSum += error

            wait(10)

        leftwheel.brake()
        rightwheel.brake()

    def accelDecel(self, distance, speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        min_speed = 100 if speed >= 0 else -100

        twenty_percent_dist = (distance / CIRCUMFERENCE * 360) * 0.2

        while abs(leftwheel.angle()) < distance / CIRCUMFERENCE * 360:

            remaining_distance = (distance / CIRCUMFERENCE * 360) - abs(leftwheel.angle())

            error = 0  - hub.imu.heading()

            if abs(leftwheel.angle()) <= twenty_percent_dist and twenty_percent_dist > 0:
                current_speed = min_speed + (speed - min_speed) * (abs(leftwheel.angle()) / twenty_percent_dist)

            elif remaining_distance <= twenty_percent_dist and twenty_percent_dist > 0:
                current_speed = min_speed + (speed - min_speed) * (remaining_distance / twenty_percent_dist)

            else:
                current_speed = speed
                
            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * (error - self.lastError)

            rightwheel.run(int(current_speed + pidValue))
            leftwheel.run(int(current_speed - pidValue))

            self.lastError = error
            self.errorSum += error

            wait(10)

        leftwheel.brake()
        rightwheel.brake()

    def turn(self, degrees, speed):
        hub.imu.reset_heading(-degrees)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        on_setpoint = True
        time_at_setpoint = 0
        wait(100)

        while time_at_setpoint < self.turn_wait_time:

            error = hub.imu.heading()

            pidValue = self.turnKp * error + self.turnKi * self.errorSum + self.turnKd * (error - self.lastError)

            rightwheel.run(int(speed * singnum(degrees) + pidValue))
            leftwheel.run(int(-speed * singnum(degrees) - pidValue))

            self.lastError = error
            self.errorSum += error

            if not on_setpoint: time_at_setpoint += 0.02
            else: time_at_setpoint = 0

            on_setpoint = abs(hub.imu.heading()) >= self.turnTol

        leftwheel.brake()
        rightwheel.brake()

    def shellTurn(self, degrees, speed=800):
        
        self.errorSum = 0
        self.lastError = 0

        shell.reset_angle(0)

        while abs(shell.angle() * SHELL_RATIO) <= abs(degrees):

            error = shell.angle() * SHELL_RATIO

            pidValue = self.shellKp * error + self.shellKi * self.errorSum + self.shellKd * (error - self.lastError)

            shell.run(int(speed * singnum(degrees) - pidValue))

            self.lastError = error
            self.errorSum += error

            on_setpoint = True
            time_at_setpoint = 0

            if not on_setpoint: time_at_setpoint += 0.02
            else: time_at_setpoint = 0

            on_setpoint = abs(shell.angle() * SHELL_RATIO) >= self.shellTol

            wait(100)

        shell.stop()

    def turnWhileShell(self, shellDegrees, turnDegrees, shellSpeed=1000, turnSpeed=125):
        hub.imu.reset_heading(-turnDegrees)
        shell.reset_angle(0)
        self.errorSum = 0
        turnLastError = 0
        turnErrorSum = 0
        shellLastError = 0
        shellErrorSum = 0

        turnAtSetPoint = False
        shellAtSetPoint = False
        turn_on_setpoint = True
        turn_time_at_setpoint = 0
        shell_on_setpoint = True
        shell_time_at_setpoint = 0
        wait(100)

        while not turnAtSetPoint and not shellAtSetPoint:

            if not turnAtSetPoint:
                turnError = hub.imu.heading()

                turnPidValue = self.turnKp * turnError + self.turnKi * turnErrorSum + self.turnKd * (turnError - turnLastError)

                rightwheel.run(int(turnSpeed * singnum(turnDegrees) - turnPidValue))
                leftwheel.run(int(-turnSpeed * singnum(turnDegrees) + turnPidValue))

                turnLastError = turnError

                if not turn_on_setpoint: turn_time_at_setpoint += 0.02
                else: turn_time_at_setpoint = 0
                
                if not turn_on_setpoint:
                    turnAtSetPoint = True

                turn_on_setpoint = abs(hub.imu.heading()) >= self.turnTol

            if turnAtSetPoint:
                leftwheel.stop()
                rightwheel.stop()
                turnAtSetPoint = True

            if not shellAtSetPoint:
                shellError = shell.angle() * SHELL_RATIO

                shellPidValue = self.kp * shellError + self.ki * shellErrorSum + self.kd * (shellError - shellLastError)

                shell.run(int(shellSpeed * singnum(shellDegrees) - shellPidValue))

                shellLastError = shellError

                shell_on_setpoint = True
                shell_time_at_setpoint = 0

                if not shell_on_setpoint: shell_time_at_setpoint += 0.02
                else: shell_time_at_setpoint = 0

                if not shell_on_setpoint:
                    shellAtSetPoint = True

                shell_on_setpoint = abs(hub.imu.heading()) >= self.turnTol

                wait(100)

            if shellAtSetPoint:
                shell.stop()
                shellAtSetPoint = True
        
        shell.stop()
        leftwheel.stop()
        rightwheel.stop()

    def stopColor(self, preset, degrees=365, speed=500):
        self.errorSum = 0
        self.lastError = 0

        stop_color, slow_color = COLOR_PRESETS[preset]

        shell.reset_angle(0)

        while abs(shell.angle() * SHELL_RATIO) <= abs(degrees):

            error = shell.angle() * SHELL_RATIO

            pidValue = self.shellKp * error + self.shellKp * self.errorSum + self.shellKd * (error - self.lastError)

            shell.run(int(speed * singnum(degrees) - pidValue))

            self.lastError = error

            on_setpoint = True
            time_at_setpoint = 0

            current_color = colorS.color()

            if current_color == stop_color:
                wait(100)
                shell.stop()
                break
            elif current_color == slow_color:
                speed = 400

            if not on_setpoint:
                time_at_setpoint += 0.02
            else:
                time_at_setpoint = 0

            on_setpoint = (shell.angle() * SHELL_RATIO) >= self.shellTol 

            wait(100)

        shell.stop()

    def shellButton(self, degrees=365):
        while True:
            pressed = hub.buttons.pressed()
            if Button.RIGHT in pressed or Button.LEFT in pressed:
                self.stopColor("stopYellow", degrees)
                break
            

    def shellPitch(self):
        while True:
            if hub.imu.tilt()[0] < 90:
                Robot.stopColor(self, "stopYellow")
                break

    def battery_percent(self):
        voltage = hub.battery.voltage()

        min_v = 6000
        max_v = 8300

        percent = (voltage - min_v) / (max_v - min_v) * 100
        percent = max(0, min(100, percent))  # clamp 0–100

        print(percent)

    def resetShell(self, preset, speed):
        stop_color, slow_color = COLOR_PRESETS[preset]

        shell.run(speed)

        while True:
            current_color = colorS.color()

            if current_color == stop_color:
                wait(150)
                if colorS.color() == stop_color:
                    shell.stop()
                    break

            elif current_color == slow_color:
                speed = 400
            else:
                speed = speed
