import time
from machine import Pin, ADC, I2C
from motor import Motor
from time import sleep
from OLED import SSD1306_I2C

motor_left = Motor("left", 8, 9, 6)
motor_right = Motor("right", 10, 11, 7)
i2c = I2C(0, sda=Pin(12), scl=Pin(13))
oled = SSD1306_I2C(128, 64, i2c)
x0 = -15
x1 = 0
x2 = 15

adc_A0 = ADC(Pin(26))
adc_A1 = ADC(Pin(27))
adc_A2 = ADC(Pin(28))

w0 = 0
w1 = 0
w2 = 0

while True:
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

    while i < 15:
        line_dist = []
        line_error = []
        line_dist.append(numerator / denominator)
        line_error.append(0 - line_dist[last])
        error = line_error[len(line_error) - 1]

        left_pwm = avg_pwm - Kp * error
        right_pwm = avg_pwm + Kp * error

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
        oled.text(str(error), 20, 20)
        oled.show()

        w0 = adc_A0.read_u16()
        w1 = adc_A1.read_u16()
        w2 = adc_A2.read_u16()
        numerator = w0 * x0 + w1 * x1 + w2 * x2
        denominator = w0 + w1 + w2

    while w0 > 4000 or w1 > 4000 or w2 > 4000:
        line_dist = []
        line_error = []
        line_dist.append(numerator / denominator)
        line_error.append(0 - line_dist[last])
        error = line_error[len(line_error) - 1]

        left_pwm_lost = avg_pwm - 1000 * Kp * error
        right_pwm_lost = avg_pwm + 1000 * Kp * error
        left_pwm = 34 - Kp * error
        right_pwm = 34 + Kp * error

        motor_left.control(1, round(left_pwm))
        motor_right.control(1, round(right_pwm))
        oled.text('following', 50, 50)
        oled.show()

        w0 = adc_A0.read_u16()
        w1 = adc_A1.read_u16()
        w2 = adc_A2.read_u16()
        numerator = w0 * x0 + w1 * x1 + w2 * x2
        denominator = w0 + w1 + w2

    while 2000 < w0 < 4000 and 2000 < w1 < 4000 and 2000 < w2 < 4000:
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
        while
