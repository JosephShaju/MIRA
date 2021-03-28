import machine
import sys
import utime
import micropython
import math

micropython.alloc_emergency_exception_buf(100)

from machine import Timer

tim0 = Timer(0)

counter_1 = 0
counter_2 = 0
repl_button = machine.Pin(0, machine.Pin.IN, machine.Pin.PULL_UP)

encoder1_pinA = machine.Pin(21, machine.Pin.IN)
encoder1_pinB = machine.Pin(14, machine.Pin.IN)
encoder2_pinA = machine.Pin(33, machine.Pin.IN)
encoder2_pinB = machine.Pin(22, machine.Pin.IN)

pwm_pin_1 = machine.Pin(27, machine.Pin.OUT)
pwm_pin_2 = machine.Pin(5, machine.Pin.OUT)
INA1 = machine.Pin(26, machine.Pin.OUT)
INB1 = machine.Pin(25, machine.Pin.OUT)
INA2 = machine.Pin(13, machine.Pin.OUT)
INB2 = machine.Pin(26, machine.Pin.OUT)

global previous_time = 0
global previous_tim = 0
global previous_error = 0
global previous_erro = 0
Integral = 0
Integra = 0

final_enc_1 = 0
final_enc_2 = 0

IR_Proxy_1 = machine.Pin(19, machine.Pin.IN, machine.Pin.PULL_UP)
# #print("I am here4")
IR_Proxy_2 = machine.Pin(15, machine.Pin.IN, machine.Pin.PULL_UP)
# #print("I am here7")
IR_Proxy_3 = machine.Pin(4, machine.Pin.IN, machine.Pin.PULL_UP)
# print("I am here8")
IR_Proxy_4 = machine.Pin(14, machine.Pin.IN, machine.Pin.PULL_UP)
# #print("I am here5")

pwm_1 = machine.PWM(pwm_pin_1)
# #print("I am here6")
pwm_2 = machine.PWM(pwm_pin_2)
# print("I am here9")
pwm_duty_1 = 0
# #print("I am here0")
pwm_duty_2 = 0

def pid_1(encoder_count):
    global Kp
    global Ki
    global kd

    error = encoder_count - counter_1

    if (previous_time == 0):
        previous_time = utime.time()

    current_time = utime.time()
    delta_time = current_time - previous_time
    delta_error = error - previous_error

    Pout = Kp/10 * error

    Integral += (error * delta_time)

    if Integral > 10:
        Integral = 10
    if Integral < -10:
        Integral = -10

    Iout = (Ki/10) * Integral

    Derivative = delta_error/delta_time
    previous_error = error
    previous_time = current_time

    Dout = (Kd/1000) * Derivative

    output = Pout + Iout + Dout

    if (output > pwm_duty_1 and pwm_duty_1 < 1024):
        pwm_duty_1 += 5

    if (output < pwm_duty_1):
        pwm_duty_1 -= 10

    return pwm_duty_1, error

def pid_2(encoder_count):
    erro = encoder_count - counter_2

    if (previous_time == 0):
        previous_tim = utime.time()
    
    current_time = utime.time()
    delta_time = current_time - previous_tim
    delta_error = error - previous_erro

    Pout = Kp/10 * error

    Integra += (error * delta_time)

    if Integra > 10:
        Integra = 10
    if Integra < -10:
        Integra = -10
    
    Iout = (Ki/10) * Integra

    Derivative = delta_error/delta_time
    previous_erro = error
    previous_tim = current_time

    Dout = (Kd/1000) * Derivative

    output = Pout + Iout + Dout

    if (output > pwm_duty_2 and pwm_duty_2 < 1024):
        pwm_duty_2 += 5

    if (output < pwm_duty_2):
        pwm_duty_2 -= 10

    return pwm_duty_2, erro


def move_motors(encoder_count1, encoder_count2):
    duty_cycle_1, error_1 = pid_1(encoder_count1)
    duty_cycle_2, error_2 = pid_2(encoder_count2)

    while (error_1 <= 10 and error_2<= 10):
        if (duty_cycle_1 >= 0):
            INA1.value(0)
            INB1.value(1)
            pwm_1.duty(pwm_duty_1)
        else:
            INA1.value(1)
            INB1.value(0)
            pwm_1.duty(-1*pwm_duty_1)
        duty_cycle_1, error_1 = pid_1(encoder_count1)

        if (duty_cycle_2 >= 0):
            INA2.value(0)
            INB2.value(1)
            pwm_2.duty(pwm_duty_2)
        else:
            INA2.value(1)
            INB2.value(0)
            pwm_2.duty(-1*pwm_duty_2)
        duty_cycle_2, error_2 = pid_2(encoder_count2)


def timer_callback():
    while (IR_Proxy_1.value() == 0 or IR_Proxy_2.value() == 0 or IR_Proxy_3.value() == 0 or IR_Proxy_4.value() == 0):
        pass
    else:
        move_motors(final_enc_1, final_enc_2)
    
def isr_init():
    encoder1_pinA.irq(trigger=machine.Pin.IRQ_RISING, handler=handle_interrupt)
    encoder1_pinB.irq(trigger=machine.Pin.IRQ_RISING, handler=handle_interrupt)		
    encoder2_pinA.irq(trigger=machine.Pin.IRQ_RISING, handler=handle_interrupt)
    encoder2_pinB.irq(trigger=machine.Pin.IRQ_RISING, handler=handle_interrupt)	

    IR_Proxy_1.irq(trigger=machine.Pin.IRQ_FALLING, handler=handler_ir_interrupt)
    IR_Proxy_2.irq(trigger=machine.Pin.IRQ_FALLING, handler=handler_ir_interrupt)		
    IR_Proxy_3.irq(trigger=machine.Pin.IRQ_FALLING, handler=handler_ir_interrupt)
    IR_Proxy_4.irq(trigger=machine.Pin.IRQ_FALLING, handler=handler_ir_interrupt)

def stop():
    INA1.value(0)
    INA2.value(0)
    INB1.value(0)
    INB2.value(0)
    print("\nSTOP CONDITION")

def handle_interrupt(pin):
    if pin == 21 or pin == 14:
        if encoder1_pinA.value() == 0 and encoder1_pinB.value() == 1:
            counter_1 += 1
        else:
            counter_1 -= 1
    
    elif pin == 33 or pin == 22:
        if encoder2_pinA.value() == 0 and encoder2_pinB.value() == 1:
            counter_1 += 1
        else:
            counter_1 -= 1
    

def handler_ir_interrupt(pin):
    stop()
    tim0.init(period=1000, mode=Timer.ONE_SHOT, callback=timer_callback)

Kp = 20
Ki = 0.01
Kd = 10

# Slowly fade LED brightness
print("I am behind While")
while True:
    print("I am inside While")
    if repl_button.value() == 0:
        sys.exit()
    
    final_enc_1 = 400
    final_enc_2 = 500

    move_motors(final_enc_1, final_enc_2)
