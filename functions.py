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

        target_angle = (distance / CIRCUMFERENCE) * 360

        self.errorSum = 0
        self.lastError = 0

        while (abs(leftwheel.angle()) + abs(rightwheel.angle())) / 2 < target_angle:

            error = -hub.imu.heading()

            self.errorSum = max(-50, min(50, self.errorSum + error))
            
            pidValue = (self.kp * error) + (self.ki * self.errorSum) + (self.kd * (error - self.lastError))

            leftwheel.dc(max(-100, min(100, speed - pidValue)))
            rightwheel.dc(max(-100, min(100, speed + pidValue)))

            self.lastError = error
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
        hub.imu.reset_heading(0)
        leftwheel.reset_angle(0)
        rightwheel.reset_angle(0)

        self.errorSum = 0
        self.lastError = 0

        time_at_setpoint = 0
        wait(100)

        while time_at_setpoint < self.turn_wait_time:

            current_heading = hub.imu.heading()
            error = degrees - current_heading
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.turnKp * error + self.turnKi * self.errorSum + self.turnKd * (error - self.lastError)

            left_power = int(max(-speed, min(speed, -pidValue)))
            right_power = int(max(-speed, min(speed, pidValue)))

            print("Error:", error, "| PID:", pidValue, "| L_PWR:", left_power, "| R_PWR:", right_power)

            leftwheel.dc(left_power)
            rightwheel.dc(right_power)

            self.lastError = error

            if abs(error) <= self.turnTol: 
                time_at_setpoint += 0.02
            else: 
                time_at_setpoint = 0
            
            wait(20)

        leftwheel.brake()
        rightwheel.brake()
    
    def shellTurn(self, degrees, speed=100):
        
        self.errorSum = 0
        self.lastError = 0
        
        shell.reset_angle(0)
        
        time_at_setpoint = 0
        wait(100) # Let the physical mechanism settle before moving

        while True:

            current_angle = shell.angle() * SHELL_RATIO
            error = degrees - current_angle
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.shellKp * error + self.shellKi * self.errorSum + self.shellKd * (error - self.lastError)

            shell_power = int(max(-speed, min(speed, pidValue)))

            print("Error:", error, "| Pwr:", shell_power, "| Timer:", time_at_setpoint)

            shell.dc(shell_power)

            self.lastError = error

            if abs(error) <= self.shellTol: 
                time_at_setpoint += 20
            else: 
                time_at_setpoint = 0
            
            if time_at_setpoint >= 100:
                break
            
            wait(20)

        shell.brake()

    def turnWhileShell(self, shellDegrees, turnDegrees, shellSpeed=50, turnSpeed=50):
        hub.imu.reset_heading(0)
        shell.reset_angle(0)

        self.errorSum = 0
        turnLastError = 0
        turnErrorSum = 0
        shellLastError = 0
        shellErrorSum = 0

        turn_time_at_setpoint = 0
        shell_time_at_setpoint = 0
        wait(100)

        # THE FIX: The loop continues until BOTH timers reach 100ms at the exact same time.
        while turn_time_at_setpoint < 100 or shell_time_at_setpoint < 100:

            # ==========================================
            #              Robot Base PID
            # ==========================================
            turn_error = turnDegrees - hub.imu.heading()
            
            if abs(turn_error) < 15:
                turnErrorSum = max(-50, min(50, turnErrorSum + turn_error))
            else:
                turnErrorSum = 0 

            turnPid = self.turnKp * turn_error + self.turnKi * turnErrorSum + self.turnKd * (turn_error - turnLastError)

            left_power = int(max(-turnSpeed, min(turnSpeed, -turnPid)))
            right_power = int(max(-turnSpeed, min(turnSpeed, turnPid)))

            leftwheel.dc(left_power)
            rightwheel.dc(right_power)

            turnLastError = turn_error

            if abs(turn_error) <= self.turnTol: 
                turn_time_at_setpoint += 20
            else: 
                turn_time_at_setpoint = 0

            # ==========================================
            #                 Shell PID
            # ==========================================
            shell_error = shellDegrees - (shell.angle() * SHELL_RATIO)
            
            if abs(shell_error) < 15:
                shellErrorSum = max(-50, min(50, shellErrorSum + shell_error))
            else:
                shellErrorSum = 0 

            shellPid = self.shellKp * shell_error + self.shellKi * shellErrorSum + self.shellKd * (shell_error - shellLastError)

            shell_power = int(max(-shellSpeed, min(shellSpeed, shellPid)))

            shell.dc(shell_power)

            shellLastError = shell_error

            if abs(shell_error) <= self.shellTol: 
                shell_time_at_setpoint += 20
            else: 
                shell_time_at_setpoint = 0
            
            print("TurnErr:", turn_error, "TurnPwr:", left_power, "|| ShellErr:", shell_error, "ShellPwr:", shell_power)

            # A single wait controls the timing for both
            wait(20)
        
        # Emergency brakes when both are perfectly stable
        shell.brake()
        leftwheel.brake()
        rightwheel.brake()

    
    def stopColor(self, preset, degrees, speed=100):
        
        stop_color, slow_color = COLOR_PRESETS[preset]

        self.errorSum = 0
        self.lastError = 0
        
        shell.reset_angle(0)
        
        time_at_setpoint = 0
        wait(100) 

        while True:

            current_angle = shell.angle() * SHELL_RATIO
            error = degrees - current_angle
            
            if abs(error) < 15:
                self.errorSum = max(-50, min(50, self.errorSum + error))
            else:
                self.errorSum = 0 

            pidValue = self.shellKp * error + self.shellKi * self.errorSum + self.shellKd * (error - self.lastError)

            shell_power = int(max(-speed, min(speed, pidValue)))

            print("Error:", error, "| Pwr:", shell_power, "| Timer:", time_at_setpoint)

            shell.dc(shell_power)

            current_color = colorS.color()

            if current_color == stop_color:
                wait(100)
                shell.brake()
                break
            elif current_color == slow_color:
                speed = 40

            self.lastError = error

            if abs(error) <= self.shellTol: 
                time_at_setpoint += 20 
            else: 
                time_at_setpoint = 0
            
            if time_at_setpoint >= 100:
                break
            
            wait(20)

        shell.brake()

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

    def arm(self, time, motor_speed):
        arm.reset_angle(0)
        
        arm.dc(motor_speed)
        wait(time)
        arm.brake()
