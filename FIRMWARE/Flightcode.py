from machine import Pin, PWM
from utime import sleep
import math
import machine
import utime as time
import MPU6050
from bmp280 import *
def quat_to_angle_axis(w, x, y, z):
    angle = 2 * math.acos(w)
    sin_half_angle = math.sqrt(1 - w * w)

    if sin_half_angle < 1e-6:
        # No rotation or invalid, return zero vector
        return (0.0, 0.0, 0.0)

    # Normalize axis and scale by angle
    axis_x = x / sin_half_angle
    axis_y = y / sin_half_angle
    axis_z = z / sin_half_angle

    # Final angle-axis vector (axis * angle)
    return (axis_x * angle, axis_y * angle, axis_z * angle)
# Servo setup
servoy = PWM(Pin(15))
servoy.freq(50)
servox = PWM(Pin(10))
servox.freq(50)
def set_servo_x(cmmd,servo):
    if cmmd > 5 :
        cmmd = 5
    if cmmd < -5:
        cmmd = -5
    cmmd = cmmd * 16 * -1
    angle = cmmd +90
    angle = angle + 12
    angle = angle 
    pulse_width = 1000 + (angle / 180) * 1000
    duty_cycle = int(pulse_width * (65535 / 20000))
    servo.duty_u16(duty_cycle)

def set_servo_y(cmmd,servo):
    if cmmd > 5 :
        cmmd = 5
    if cmmd < -5:
        cmmd = -5
    cmmd = cmmd * 7.4
    angle = cmmd + 90
    angle = angle +5
    angle = angle *1.5
    pulse_width = 1000 + (angle / 180) * 1000
    duty_cycle = int(pulse_width * (65535 / 20000))
    servo.duty_u16(duty_cycle)

set_servo_y(0,servoy)
set_servo_x(0,servox)
# Pyro output
pyro = Pin(12, Pin.OUT)
led = Pin(13, Pin.OUT)
buzzer = Pin(2, Pin.OUT)
# I2C setup for BMP280
sdaPIN = machine.Pin(0)
sclPIN = machine.Pin(1)
bus = machine.I2C(0, sda=sdaPIN, scl=sclPIN, freq=400000)
time.sleep(0.1)
led.on()
buzzer.off()
time.sleep(10)
buzzer.on()
# BMP280 setup
bmp = BMP280(bus)
bmp.use_case(BMP280_CASE_INDOOR)
sea_level_pressure = 100820


# Datalogger setup
led.off()
logfile = open("datalog.txt", "w")
logfile.write("time_ms   xpos    ypos  roll zaccel  xout    yout    altitude \n")

def set_servo_x(cmmd,servo):
    if cmmd > 5 :
        cmmd = 5
    if cmmd < -5:
        cmmd = -5
    cmmd = cmmd * 16 * -1
    angle = cmmd +90
    angle = angle + 12
    angle = angle 
    pulse_width = 1000 + (angle / 180) * 1000
    duty_cycle = int(pulse_width * (65535 / 20000))
    servo.duty_u16(duty_cycle)

def set_servo_y(cmmd,servo):
    if cmmd > 5 :
        cmmd = 5
    if cmmd < -5:
        cmmd = -5
    cmmd = cmmd * 7.4
    angle = cmmd + 90
    angle = angle +5
    angle = angle *1.5
    pulse_width = 1000 + (angle / 180) * 1000
    duty_cycle = int(pulse_width * (65535 / 20000))
    servo.duty_u16(duty_cycle)

set_servo_y(0,servoy)
set_servo_x(0,servox)

def to_euler_angles(w, x, y, z):
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = math.sqrt(1 + 2 * (w * y - x * z))
    cosp = math.sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class Quaternion:
    def __init__(self, w, x, y, z):
        self.w, self.x, self.y, self.z = w, x, y, z

    def __mul__(self, other):
        w1, x1, y1, z1 = self.w, self.x, self.y, self.z
        w2, x2, y2, z2 = other.w, other.x, other.y, other.z
        return Quaternion(
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2
        )

    def normalize(self):
        mag = math.sqrt(self.w**2 + self.x**2 + self.y**2 + self.z**2)
        self.w /= mag
        self.x /= mag
        self.y /= mag
        self.z /= mag

# MPU6050 setup
i2c = machine.I2C(0, sda=machine.Pin(0), scl=machine.Pin(1))
mpu = MPU6050.MPU6050(i2c)

# Calibration
print("Calibration")
mpu.wake()
time.sleep(3)
xoff = yoff = zoff = 0

time.sleep(1)
for _ in range(5000):
    x, y, z = mpu.read_gyro_data()
    xoff += x
    yoff += y
    zoff += z
xoff /= 5000
yoff /= 5000
zoff /= 5000
print(f"x{xoff} y{yoff} z{zoff}")

#xoff = -0.0008994192
#yoff = 0.000619975
#zoff = 0.0005649991



led.on()
baseline_altitude = 44330 * (1 - (bmp.pressure / sea_level_pressure) ** 0.1903)
buzzer.off()
def detect_launch():
    threshold = -1.5
    print("Waiting for launch...")
    while True:
        time.sleep(0.01)
        x, y, z = mpu.read_accel_data()
        print(f"Accel (g): x={x:.2f}, y={y:.2f}, z={z:.2f}")
        if x < threshold:
            print("Launch detected!")
            return

detect_launch()
wholetime = time.ticks_ms()

# PID and quaternion initialization
xpos = ypos = 0
p, i, d = 0.4, 0.2, 0.13
xp = xi = xd = yp = yi = yd = 0
prevx = prevy = 0
cycletime = time.ticks_ms()
time.sleep(0.01)
start_time = time.ticks_ms()
statequat = Quaternion(1, 0, 0, 0)

# Control loop
while time.ticks_ms() - start_time < 2800:
    elapsed = (time.ticks_ms() - cycletime) / 1000
    cycletime = time.ticks_ms()
    s = elapsed
    acelx, acely, acelz = mpu.read_accel_data()
    x, y, z = mpu.read_gyro_data()
    x -= xoff
    y -= yoff
    z -= zoff

    xpos += x * elapsed
    ypos += y * elapsed
    zpos = z * elapsed

    xr = math.radians(x * elapsed)
    yr = math.radians(y * elapsed)
    zr = math.radians(z * elapsed)
    detasq = xr**2 + yr**2 + zr**2
    deta = math.sqrt(detasq)
    if deta != 0:
        ux, uy, uz = xr/deta, yr/deta, zr/deta
    else:
        ux = uy = uz = 0

    w = math.cos(deta/2)
    s = math.sin(deta/2)
    rotationquat = Quaternion(w, s*ux, s*uy, s*uz)
    statequat = statequat * rotationquat
    statequat.normalize()

    xq, yq, roll = quat_to_angle_axis(statequat.w,statequat.x,statequat.y,statequat.z)
    xpos = math.degrees(roll)
    roll = math.degrees(xq)
    ypos = math.degrees(yq)
    print(f"x{xpos}  y{ypos}")
    xp = xpos * p
    xi += xpos * elapsed * i
    xd = ((xpos - prevx) / elapsed) * d
    xout = xp + xi + xd
 
    prevx = xpos
    xout = -1* xout
    set_servo_x(xout, servox)

    yp = ypos * p
    yi += ypos * elapsed * i
    yd = ((ypos - prevy) / elapsed) * d
    yout = yp + yi + yd
    prevy = ypos
    set_servo_y(yout, servoy)

    # Logging
    pressure = bmp.pressure
    altitude = 44330 * (1 - (pressure / sea_level_pressure) ** 0.1903) - baseline_altitude
    temperature = bmp.temperature
    logfile.write(f"{time.ticks_ms() - wholetime }        {xpos:<7.2f} {ypos:<7.2f} {roll} {acelx} {xout:<7.2f} {yout:<7.2f} {altitude}\n")
print(s)
# Apogee detection and logging
highest = altitude
while True:
    pressure = bmp.pressure
    temperature = bmp.temperature
    altitude = 44330 * (1 - (pressure / sea_level_pressure) ** 0.1903) - baseline_altitude
    logfile.write(f"{time.ticks_ms() - wholetime} {'NA':<7} {'NA':<7} {'NA':<7} {'NA':<7} {altitude}\n")

    if altitude > highest:
        highest = altitude
    if altitude < highest - 0.5:
        print("Parachute deployed.")
        break
    time.sleep(0.1)
led.off()
set_servo_y(0, servoy)
set_servo_x(0, servox)
pyro.on()
time.sleep(3)
pyro.off()
logfile.close()
print("Flight complete. Data logged to datalog.txt.")




