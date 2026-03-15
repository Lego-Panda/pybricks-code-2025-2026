from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop, Axis
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, hub_menu
from robot_conf import *

hub = PrimeHub(Axis.Y, Axis.Z)

def signnum(value):
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

            rightwheel.run(int(speed * signnum(degrees) + pidValue))
            leftwheel.run(int(-speed * signnum(degrees) - pidValue))

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

            shell.run(int(speed * signnum(degrees) - pidValue))

            self.lastError = error
            self.errorSum += error

            on_setpoint = True
            time_at_setpoint = 0

            if not on_setpoint: time_at_setpoint += 0.02
            else: time_at_setpoint = 0

            on_setpoint = abs(shell.angle() * SHELL_RATIO) >= self.shellTol

            wait(100)

        shell.stop()

    def turnWhileShell(self, preset, shellDegrees, turnDegrees, shellSpeed=1000, turnSpeed=125):
        hub.imu.reset_heading(-turnDegrees)
        stop_color, slow_color = COLOR_PRESETS[preset]
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

                rightwheel.run(int(turnSpeed * signnum(turnDegrees) - turnPidValue))
                leftwheel.run(int(-turnSpeed * signnum(turnDegrees) + turnPidValue))

                turnLastError = turnError

                if not turn_on_setpoint: turn_time_at_setpoint += 0.02
                else: turn_time_at_setpoint = 0
                
                if not turn_on_setpoint:
                    turnAtSetPoint = True

                turn_on_setpoint = abs(hub.imu.heading()) >= self.turnTol

            if turnAtSetPoint:
                leftwheel.brake()
                rightwheel.brake()
                turnAtSetPoint = True

            if not shellAtSetPoint:
                shellError = shell.angle() * SHELL_RATIO

                shellPidValue = self.kp * shellError + self.ki * shellErrorSum + self.kd * (shellError - shellLastError)

                shell.run(int(shellSpeed * signnum(shellDegrees) - shellPidValue))

                shellLastError = shellError

                shell_on_setpoint = True
                shell_time_at_setpoint = 0

                current_color = colorS.color()

                if current_color == stop_color:
                    wait(100)
                    shell.stop()
                    break
                elif current_color == slow_color:
                    speed = 400

                if not shell_on_setpoint: shell_time_at_setpoint += 0.02
                else: shell_time_at_setpoint = 0

                if not shell_on_setpoint:
                    shellAtSetPoint = True

                shell_on_setpoint = abs(hub.imu.heading()) >= self.turnTol

                wait(100)

            if shellAtSetPoint:
                shell.brake()
                shellAtSetPoint = True
        
        shell.brake()
        leftwheel.brake()
        rightwheel.brake()

    
    def stopColor(self, preset, degrees=365, speed=500):
        self.errorSum = 0
        self.lastError = 0

        stop_color, slow_color = COLOR_PRESETS[preset]

        shell.reset_angle(0)

        while abs(shell.angle() * SHELL_RATIO) <= abs(degrees):

            error = shell.angle() * SHELL_RATIO

            pidValue = self.shellKp * error + self.shellKp * self.errorSum + self.shellKd * (error - self.lastError)

            shell.run(int(speed * signnum(degrees) - pidValue))

            self.lastError = error

            on_setpoint = True
            time_at_setpoint = 0

            current_color = colorS.color()

            if current_color == stop_color:
                wait(100)
                shell.brake()
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

    def testcolor(self, preset, speed=500, blind_time=300):
        stop_color, slow_color = COLOR_PRESETS[preset]
        
        # 1. Start the motor
        shell.run(speed)

        # 2. The Blind Drive: Wait for 'blind_time' milliseconds before checking colors.
        # This forces the robot to drive off the starting position or over an early line.
        if blind_time > 0:
            wait(blind_time)

        # 3. Now begin the high-precision scanning
        while True:
            current_color = colorS.color()

            if current_color == stop_color:
                shell.brake()
                break 
                
            elif current_color == slow_color:
                shell.run(400 if speed > 0 else -400)

            wait(10)

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

    def moveWhileShell(self, shellDegrees, moveDistance, shellSpeed=500, moveSpeed=150):

        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)
        shell.reset_angle(0)
        self.errorSum = 0
        moveLastError = 0
        moveErrorSum = 0
        shellLastError = 0
        shellErrorSum = 0

        moveAtSetPoint = False
        shellAtSetPoint = False
        shell_on_setpoint = True
        shell_time_at_setpoint = 0
        wait(10)

        while not moveAtSetPoint or not shellAtSetPoint:
            if not moveAtSetPoint:
                if abs(leftwheel.angle()) >= moveDistance / CIRCUMFERENCE * 360:
                    moveAtSetPoint = True
                    wheels.brake()
                else:
                    moveError = 0 - gyro.heading() 
                    movePidValue = self.kp * moveError + self.ki * moveErrorSum + self.kd * (moveError - moveLastError)

                    rightwheel.run(int(moveSpeed + movePidValue))
                    leftwheel.run(int(moveSpeed - movePidValue))

                    moveLastError = moveError
                    moveErrorSum += moveError

            if not shellAtSetPoint:
                currentShellAngle = shell.angle() * SHELL_RATIO
                
                if abs(currentShellAngle) >= abs(shellDegrees):
                    shellAtSetPoint = True
                    shell.brake()
                else:
                    direction = 1 if shellDegrees > 0 else -1
                    shell.run(shellSpeed * direction)

            wait(10)

    def ziun(self, times_of_backshot, speed, behind_speed):
        
        for i in range(times_of_backshot):
            wheels.drive(-speed, 0)
            wait(1500)
            wheels.brake()
            wait(300)
            wheels.drive(behind_speed, 0)
            wait(400)
            wheels.brake()
            wait(300)
            wheels.drive(-behind_speed, 0)
            wait(400)
            wheels.brake()

    def sivuv(self, speed, time):
        leftwheel.run(speed)
        rightwheel.run(-speed)
        wait(time)
        leftwheel.brake()
        rightwheel.brake()

    def test_pid(self, distance,speed):
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)
        base_power = speed/10
        self.errorSum = self.lastError = 0

        target_angle = (distance / CIRCUMFERENCE) * 360
        
        timer = StopWatch()
        last_time = timer.time()

        while (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2 < target_angle:
            current_time = timer.time()
            dt = max((current_time - last_time) / 1000, 0.001)
            last_time = current_time

            error = -hub.imu.heading() # "0 -" is mathematically the same as just "-"
            self.errorSum = max(-50, min(50, self.errorSum + (error * dt)))
            derivative = (error - self.lastError) / dt
            
            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * derivative

            left_power = max(-100, min(100, base_power - pidValue))
            right_power = max(-100, min(100, base_power + pidValue))

            leftwheel.dc(left_power)
            rightwheel.dc(right_power)

            self.lastError = error
            wait(10)

        leftwheel.brake()
        rightwheel.brake()

    def accelDecel_test(self, distance, speed):

        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0 
        self.lastError = 0

        min_power = 10 if speed >= 0 else -10
        
        target_angle = (distance / CIRCUMFERENCE) * 360
        twenty_percent_angle = target_angle * 0.2

        timer = StopWatch()
        last_time = timer.time()

        while (abs(leftwheel.angle) + abs(rightwheel.angle) / 2) >= target_angle: 

            remaining_angle = target_angle - (abs(leftwheel.angle) + abs(rightwheel.angle) / 2)

            if (abs(leftwheel.angle) + abs(rightwheel.angle) / 2) <= twenty_percent_angle and twenty_percent_angle > 0:
                current_base = min_power + (speed - min_power) * ((abs(leftwheel.angle) + abs(rightwheel.angle) / 2) / twenty_percent_angle)
            
            elif remaining_angle <= twenty_percent_angle and twenty_percent_angle > 0:
                current_base = min_power + (speed - min_power) * (remaining_angle / twenty_percent_angle)
            
            else:
                current_base = speed

            current_time = timer.time()
            dt = max((current_time - last_time) / 1000, 0.001)
            last_time = current_time

            error = -hub.imu.heading()
            self.errorSum = max(-50, min(50, self.errorSum + (error * dt)))
            derivative = (error - self.lastError) / dt
            
            pidValue = self.kp * error + self.ki * self.errorSum + self.kd * derivative

            left_power = max(-100, min(100, current_base - pidValue))
            right_power = max(-100, min(100, current_base + pidValue))

            leftwheel.dc(left_power)
            rightwheel.dc(right_power)

            self.lastError = error
            wait(10)

        leftwheel.brake()
        rightwheel.brake()
