from brian.sensors import SensorPort
from brian.sensors.EV3 import TouchSensorEV3
from brian.motors import EV3LargeMotor, EV3MediumMotor, MotorPort
from brian.uicontrol import UiEventsListener
import time

#motors
mL = EV3LargeMotor(MotorPort.C)
mR = EV3LargeMotor(MotorPort.D)
mM = EV3MediumMotor(MotorPort.A) # M jako masturbator
#sensors
button = TouchSensorEV3(SensorPort.S1)

#global var
listener = UiEventsListener()
#listener.enable_turn_off_by_buttons(False)
buttons = listener.buttons_event_since_last()
knob = listener.knob_event_since_last()
clock = time.time_ns()

speed = 100

I = 0.0
kp = 5
ki = 0
kd = 0
anti_windup = 0
prev_error = 0

masturbator_h = 600  #celkova vyska masturbatoru
masturbator_edging_h = masturbator_h - 200            #cekaci vyska masturbatoru
edging = False
prev_angle = 0
stroke = False

############### methods ###############
def init():
    while not mL.is_ready() or not mR.is_ready() or not mM.is_ready() or not button.is_ready():
        print("intializing...")
        if not mL.is_connected():
            print("mL <not connected>")
        if not mR.is_connected():
            print("mR <not connected>")
        if not mM.is_connected():
            print("mM <not connected>")
        if not button.is_connected():
            print("button <not connected>")
        time.sleep(0.2)
        masturbator_pre_game()

def masturbator_pre_game(): #inicializace
    print("pregaming...")
    mM.run_unregulated(-1.0)
    button.wait_for_press()

    mM.brake()
    mM.reset_angle(0)
    print("ready to iniciate")

def render(dt):
    global buttons, knob
    buttons = listener.buttons_event_since_last()
    knob = listener.knob_event_since_last()
    update(dt)

def pid(error: float):
    global kp, ki, kd, I, anti_windup, prev_error
    P = kp * error
    I += ki * error
    D = kd * (error - prev_error)

    if abs(I) > anti_windup:
        I = anti_windup * (abs(I)/I)
    prev_error = error

    return P+I+D

def motors_update(dt):
    # movement
    #pid_value = pid(0)
    #mL.run_at_speed(int(speed - pid_value))
    #mR.run_at_speed(int(speed + pid_value))

    # masturbator
    masturbation()

def update(dt):
    global prev_angle
    motors_update(dt)
    if buttons.bottom_right.just_pressed:
        print(mM.current_angle())
    if not prev_angle == mM.current_angle():
        print(mM.current_angle())
        prev_angle = mM.current_angle()

def masturbation():
    global stroke
    #detekce koule a horniho tlacitka (kalibrace)
    if button.is_pressed():
        print("jede dolu - kalibrovan")
        time.sleep(0.1)
        mM.reset_angle(0)
        stroke = False #jestli se ma masturbator vratit nahoru
        mM.rotate_to_angle_without_speed_control(masturbator_edging_h)
    if mM.current_angle() > masturbator_h-2 and stroke: # is_stalled() ma inverted vystup
        print("zpatky nahoru")
        mM.run_unregulated(-1.0)
        stroke = False
    if buttons.bottom_left.just_pressed:
        mM.rotate_to_angle_without_speed_control(masturbator_h)
        stroke = True

############### zacatek programu ###############
init()
while True:
    dt = time.time_ns() - clock
    clock = time.time_ns()
    render(dt)
    time.sleep(0.001)


