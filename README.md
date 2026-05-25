[code.py](https://github.com/user-attachments/files/28234779/code.py)
# robot
Έργο για το μάθημα Ενσωματωμένα Συστήματα 
 
import machine
import utime

l_pwm = machine.PWM(machine.Pin(8)); l_dir = machine.Pin(9, machine.Pin.OUT)
r_pwm = machine.PWM(machine.Pin(10)); r_dir = machine.Pin(11, machine.Pin.OUT)
l_pwm.freq(1000); r_pwm.freq(1000)

btn1 = machine.Pin(20, machine.Pin.IN, machine.Pin.PULL_UP)
btn2 = machine.Pin(21, machine.Pin.IN, machine.Pin.PULL_UP)

dig_l = machine.Pin(0, machine.Pin.IN, machine.Pin.PULL_UP)
dig_r = machine.Pin(1, machine.Pin.IN, machine.Pin.PULL_UP)
sens_l = machine.ADC(27); sens_c = machine.ADC(26); sens_r = machine.ADC(28)

kp = 25.0
kd = 35.0
BASE_SPEED_FAST = 30000
BASE_SPEED_SLOW = 18000
last_error = 0
stop_counter = 0
WHITE_LIMIT = 0
BLACK_LIMIT = 0
BLACK_OFFSET = 7000
WHITE_OFFSET = 3000

def drive(left, right):
    if left >= 0:
        l_dir.value(0); l_pwm.duty_u16(max(0, min(65535, int(left))))
    else:
        l_dir.value(1); l_pwm.duty_u16(max(0, min(65535, int(abs(left)))))
    if right >= 0:
        r_dir.value(0); r_pwm.duty_u16(max(0, min(65535, int(right))))
    else:
        r_dir.value(1); r_pwm.duty_u16(max(0, min(65535, int(abs(right)))))

def calibrate():
    global WHITE_LIMIT, BLACK_LIMIT
    print("Step 1: CENTER on BLACK. Starting in 2s...")
    utime.sleep(2)
    s_b = 0
    for i in range(40):
        s_b += sens_c.read_u16()
        utime.sleep_ms(20)
    avg_black = s_b / 40

    print("Step 2: CENTER on WHITE. Starting in 3s...")
    utime.sleep(3)
    s_w = 0
    for i in range(40):
        s_w += sens_c.read_u16()
        utime.sleep_ms(20)
    avg_white = s_w / 40

    BLACK_LIMIT = avg_black + BLACK_OFFSET
    WHITE_LIMIT = avg_white - WHITE_OFFSET
    print("Done. BLACK_LIMIT:{} WHITE_LIMIT:{}".format(int(BLACK_LIMIT), int(WHITE_LIMIT)))

def zig_zag_search():
    print("Searching...")
    search_speed = 20000
    for duration in [150, 350, 550, 800]:
        drive(-search_speed, search_speed)
        start_t = utime.ticks_ms()
        while utime.ticks_diff(utime.ticks_ms(), start_t) < duration:
            if sens_c.read_u16() < BLACK_LIMIT or dig_l.value() == 0 or dig_r.value() == 0:
                return
        search_speed = -search_speed
    drive(0, 0)
    while True: utime.sleep(1)

def all_white(v_l, v_c, v_r):
    return v_l > WHITE_LIMIT and v_c > WHITE_LIMIT and v_r > WHITE_LIMIT

def any_black(v_l, v_c, v_r):
    return v_l < BLACK_LIMIT or v_c < BLACK_LIMIT or v_r < BLACK_LIMIT

def track_1():
    global last_error
    last_error = 0
    # Two separate timers:
    # white_ms = total ms all-3-white has been seen (never resets)
    # black_ms = total ms any sensor saw black since last all-white
    # Stop only if white_ms gets large AND black hasn't been seen for a long time
    all_white_start = None
    last_black_time = utime.ticks_ms()

    print("Track 1 running...")
    while True:
        v_l = sens_l.read_u16()
        v_c = sens_c.read_u16()
        v_r = sens_r.read_u16()
        d_l = dig_l.value()
        d_r = dig_r.value()
        now = utime.ticks_ms()

        if any_black(v_l, v_c, v_r):
            last_black_time = now  # reset whenever we see the line

        # Stop condition: all 3 white AND no black seen for 800ms
        # This means we are genuinely past the end, not mid-curve
        if all_white(v_l, v_c, v_r):
            time_since_black = utime.ticks_diff(now, last_black_time)
            if time_since_black >= 800:
                drive(0, 0)
                print("Stopped.")
                return
       
        # Drive
        if d_l == 0:
            drive(-15000, BASE_SPEED_FAST)
        elif d_r == 0:
            drive(BASE_SPEED_FAST, -15000)
        else:
            error = v_l - v_r
            correction = (kp * error) + (kd * (error - last_error))
            last_error = error
            speed = BASE_SPEED_FAST if v_c < BLACK_LIMIT else BASE_SPEED_SLOW
            drive(speed - correction, speed + correction)
        utime.sleep_ms(15)

def track_2_3():
    global last_error, stop_counter
    last_error = 0
    stop_counter = 0
    last_black_time = utime.ticks_ms()

    print("Track 2/3 running...")
    while True:
        v_l = sens_l.read_u16()
        v_c = sens_c.read_u16()
        v_r = sens_r.read_u16()
        d_l = dig_l.value()
        d_r = dig_r.value()
        now = utime.ticks_ms()

        if any_black(v_l, v_c, v_r):
            last_black_time = now

        # Stop condition: all 3 white AND no black seen for 800ms
        if all_white(v_l, v_c, v_r):
            time_since_black = utime.ticks_diff(now, last_black_time)
            if time_since_black >= 800:
                drive(0, 0)
                print("Stopped.")
                return

        # Line lost — search
        if v_c > WHITE_LIMIT and d_l == 1 and d_r == 1:
            stop_counter += 1
            if stop_counter > 15:
                zig_zag_search()
                stop_counter = 0
        else:
            stop_counter = 0
            if d_l == 0:
                drive(-11000, BASE_SPEED_SLOW + 15000)
            elif d_r == 0:
                drive(BASE_SPEED_SLOW + 15000, -11000)
            else:
                error = v_l - v_r
                correction = (kp * error) + (kd * (error - last_error))
                last_error = error
                speed = BASE_SPEED_FAST if v_c < BLACK_LIMIT else BASE_SPEED_SLOW
                drive(speed - correction, speed + correction)
        utime.sleep_ms(5)

print("Ready. GP20=Track1 GP21=Track2/3")
while True:
    if btn1.value() == 0:
        utime.sleep_ms(200)
        calibrate()
        track_1()
    if btn2.value() == 0:
        utime.sleep_ms(200)
        calibrate()
        track_2_3()
    utime.sleep_ms(50)
