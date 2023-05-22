from time import sleep
import defSA
from motor import Motor
from machine import Pin, I2C, ADC
from OLED import SSD1306_I2C
from apds9960 import uAPDS9960 as APDS9960
import def_us as us
import ultrasonic as ultra
import def_motorcontrol as mc

motor_left = Motor("left", 8, 9, 6)
motor_right = Motor("right", 10, 11, 7)
R = ADC(Pin(26))  # Right Sensor
M = ADC(Pin(27))  # Middle Sensor
L = ADC(Pin(28))  # Left Sensor
x0 = (-15)
x1 = 0
x2 = 15
NR = 0
NM = 0
NL = 0
i = 0
i2c = I2C(0, sda=Pin(12), scl=Pin(13))
bus = I2C(1, sda=Pin(18), scl=Pin(19))
oled = SSD1306_I2C(128, 64, i2c)
apds = APDS9960(bus)
apds.enableProximitySensor()
apds.setProximityIntLowThreshold(50)
proximity_measurement = apds.readProximity()

left_pwm_lost = ()
right_pwm_lost = ()

TRIG = 3
ECHO = 2
uss = ultra.sonic(TRIG, ECHO)
dist = uss.distance_mm()
servo = us.Servo()  # initialize Class Servo
mcontrol = mc.Motorcontrol()

adc_A0 = ADC(Pin(26))
adc_A1 = ADC(Pin(27))
adc_A2 = ADC(Pin(28))
w0 = adc_A0.read_u16()
w1 = adc_A1.read_u16()
w2 = adc_A2.read_u16()
numerator = w0 * x0 + w1 * x1 + w2 * x2
denominator = w0 + w1 + w2

l_state = []
r_state = []
calc_state = []
line_dist = []
last = (len(line_dist) - 1)
line_error = []

states = ['begin_park', 'start_parking', 'parking', 'line', 'corridor', 'start']

state = 'start'
while True:
    if w0 > 4000 or w1 > 4000 or w2 > 4000:
        state = 'line'
        while state == 'line':
            line_dist = []
            line_error = []
            w0 = adc_A0.read_u16()
            w1 = adc_A1.read_u16()
            w2 = adc_A2.read_u16()
            numerator = w0 * x0 + w1 * x1 + w2 * x2
            denominator = w0 + w1 + w2
            line_dist.append(numerator / denominator)
            last = len(line_dist) - 1
            print("{:3d}, {:4d}, {:4d}, {:3.2f}".format(w0, w1, w2, line_dist[last]))
            sleep(0.1)
        line_error.append(0 - line_dist[last])
        Kp = 1.5
        i = 0
        avg_pwm = 38

        while state == 'line' and i < 15:
            line_dist = []
            line_error = []
            line_dist.append(numerator / denominator)
            line_error.append(0 - line_dist[last])
            left_pwm = avg_pwm - Kp * line_error[len(line_error) - 1]
            right_pwm = avg_pwm + Kp * line_error[len(line_error) - 1]
            motor_left.control(1, 0)
            motor_right.control(1, 0)
            motor_left.control(1, round(left_pwm))
            motor_right.control(1, round(right_pwm))
            sleep(0.1)
            motor_left.control(1, 0)
            motor_right.control(1, 0)
            i += 1
            oled.fill(0)
            oled.text(str(i), 0, 0)
            oled.text(str(line_error[len(line_error) - 1]), 20, 20)
            oled.show()
            w0 = adc_A0.read_u16()
            w1 = adc_A1.read_u16()
            w2 = adc_A2.read_u16()
            numerator = w0 * x0 + w1 * x1 + w2 * x2
            denominator = w0 + w1 + w2

        while state == 'line' and (w0 > 4000 or w1 > 4000 or w2 > 4000):
            line_dist = []
            line_error = []
            line_dist.append(numerator / denominator)
            line_error.append(0 - line_dist[last])
            left_pwm_lost = avg_pwm - 1000 * Kp * line_error[len(line_error) - 1]
            right_pwm_lost = avg_pwm + 1000 * Kp * line_error[len(line_error) - 1]
            left_pwm = 34 - Kp * line_error[len(line_error) - 1]
            right_pwm = 34 + Kp * line_error[len(line_error) - 1]
            motor_left.control(1, round(left_pwm))
            motor_right.control(1, round(right_pwm))
            oled.text('following', 50, 50)
            oled.show()
            w0 = adc_A0.read_u16()
            w1 = adc_A1.read_u16()
            w2 = adc_A2.read_u16()
            numerator = w0 * x0 + w1 * x1 + w2 * x2
            denominator = w0 + w1 + w2

        while state == 'line' and (2000 < w0 < 4000 and 2000 < w1 < 4000 and 2000 < w2 < 4000):
            if left_pwm_lost >= 80:
                left_pwm_lost = 80
                right_pwm_lost = 0
            elif right_pwm_lost >= 80:
                right_pwm_lost = 80
                left_pwm_lost = 0
            motor_left.control(1, 0)
            motor_right.control(1, 0)
            motor_left.control(1, round(left_pwm_lost))
            motor_right.control(1, round(right_pwm_lost))
            sleep(0.1)
            motor_left.control(1, 0)
            motor_right.control(1, 0)
            sleep(0.1)
            w0 = adc_A0.read_u16()
            w1 = adc_A1.read_u16()
            w2 = adc_A2.read_u16()
            oled.fill(0)
            oled.text('lost', 0, 0)
            oled.show()

    elif (l_state[len(l_state) - 1] < 250 and lv2 < 500) or (r_state[len(r_state) - 1] < 250 and rv2 < 500):
        state = 'corridor'
        while state == 'corridor':
            import time
            from machine import Pin, I2C, ADC
            import def_us as us
            import def_pdcontrol as pdc
            import def_motorcontrol as mc
            from OLED import SSD1306_I2C as ss
            from apds9960 import uAPDS9960 as APDS9960

            """define global variables and arrays"""
            bus = I2C(1, sda=Pin(18), scl=Pin(19))
            apds = APDS9960(bus)
            apds.enableProximitySensor()
            apds.setProximityIntLowThreshold(50)

            adc_A0 = ADC(Pin(26))
            adc_A1 = ADC(Pin(27))
            adc_A2 = ADC(Pin(28))
            w0 = adc_A0.read_u16()
            w1 = adc_A1.read_u16()
            w2 = adc_A2.read_u16()

            i2c = I2C(0, sda=Pin(12), scl=Pin(13))
            rv = [0, 0]  # right us array
            lv = [0, 0]  # left us array
            dta = [0, 1]  # code run time array

            dt = time.time()
            dta.append(dt)

            """Servo motor + us sensor control"""
            servo = us.Servo()  # initialize Class Servo
            servo.sweep(direction='right', angle=155, slp=0.04)  # sweep right
            rv1 = servo.rv
            servo.sweep(direction='left', angle=25, slp=0.04)  # sweep left
            lv1 = servo.lv
            servo.sweep(direction='right', angle=130, slp=0.04)  # sweep right
            rv2 = servo.rv
            rv.append((rv1 + rv2) / 2)
            servo.sweep(direction='left', angle=45, slp=0.04)  # sweep left
            lv2 = servo.lv
            lv.append((lv1 + lv2) / 2)

            """activate us pd code"""
            control = pdc.PDControl()
            uspd = control.us_pdcontrol(lv[len(lv) - 1], lv[len(lv) - 2], rv[len(rv) - 1], rv[len(rv) - 2],
                                        dta[len(dta) - 1], dta[len(dta) - 2])
            """pd limit control"""
            if - 20 < uspd < 20:
                pd = int(uspd)
            elif - 20 > uspd:
                pd = -20
            else:
                pd = 20
            """Control motor speed based off distance to wall"""
            if (lv[len(lv) - 1]) < 40 or (rv[len(rv) - 1]) < 40:
                vl = 43
                vr = vl
                slp = 0.1
                cv = 1
            elif (lv[len(lv) - 1]) > 200 or (rv[len(rv) - 1]) > 200:
                vl = 43
                vr = vl
                slp = 0.1
                cv = 1
            elif (lv[len(lv) - 1]) < 70 or (rv[len(rv) - 1]) < 70:
                vl = 43
                vr = vl
                slp = 0.16
                cv = 1
            elif (lv[len(lv) - 1]) < 100 or (rv[len(rv) - 1]) < 100:
                vl = 43
                vr = vl
                slp = 0.28
                cv = 1
            else:
                vl = 43
                vr = vl
                slp = 0.4
                cv = 1

            """Activate motor control"""
            mcontrol = mc.Motorcontrol()
            mcontrol.set_motor1(cv, pd, vl, vr, slp)

            """Activate Oled and print desired values"""
            oled = ss(128, 64, i2c)
            text1 = "pd = {}".format(pd)
            text2 = "r_s = {}".format(rv[len(rv) - 1])
            text3 = "l_s = {}".format(lv[len(lv) - 1])
            oled.text(str(text1), 0, 0)
            oled.text(str(text2), 0, 10)
            oled.text(str(text3), 0, 20)
            oled.show()
    else:
        state = 'start'
        mcontrol.set_motor2(cv=1, pd=0, vl=40, vr=40, slp=0.1)
        while state == 'start':
            oled.fill(0)
            oled.text(state, 50, 50)
            oled.show()

            l_state = []
            r_state = []
            calc_state = []
            line_dist = []
            """set motor to 0"""
            mcontrol.set_motor2(cv=1, pd=0, vl=0, vr=0, slp=0)
            """sweep us sensor take left and right reading"""
            servo.sweep(direction='right', angle=35, slp=0.04)
            r_state.append(dist)
            servo.sweep(direction='left', angle=60, slp=0.04)  # sweep left
            lv2 = servo.lv
            servo.sweep(direction='left', angle=145, slp=0.04)
            l_state.append(dist)
            servo.sweep(direction='right', angle=120, slp=0.04)  # sweep right
            rv2 = servo.rv
            mcontrol.set_motor2(cv=1, pd=0, vl=40, vr=40, slp=0)
            """check for change of state"""

        while state == 'start_parking':
            oled.fill(0)
            oled.text(state, 50, 50)
            oled.show()
            servo.sweep(direction='left', angle=145, slp=0.08)
            if dist < 300:
                mcontrol.set_motor2(cv=1, pd=0, vl=0, vr=0, slp=0)
                state = 'parking'
                oled.text(str(state), 0, 0)
                oled.text(str(dist), 30, 30)
                oled.show()
            else:
                mcontrol.set_motor2(cv=1, pd=0, vl=40, vr=40, slp=0)
                oled.text(str(state), 0, 0)
                oled.text(str(dist), 30, 30)
                oled.show()
            sleep(0.08)
        while state == 'parking':
            oled.fill(0)
            oled.text(state, 50, 50)
            oled.show()
            while 220 < proximity_measurement <= 255:
                mcontrol.set_motor2(cv=0, pd=0, vl=0, vr=0, slp=0)
                sleep(0.05)
                servo.sweep(direction='left', angle=35, slp=0.08)  # sweep left
                r_state.append(dist)
                servo.sweep(direction='right', angle=140, slp=0.08)  # sweep right
                l_state.append(dist)
                calc = l_state[len(l_state) - 1] - r_state[len(r_state) - 1]
                calc_state.append(int(calc))
                oled.fill(0)
                oled.text(str(calc_state[len(calc_state) - 1]), 20, 20)
                oled.show()
                defSA.set_state(u_state=calc_state[len(calc_state) - 1])
            while 220 < proximity_measurement <= 255:
                mcontrol.set_motor2(cv=0, pd=0, vl=0, vr=0, slp=0)
                print(proximity_measurement)
                sleep(0.2)
