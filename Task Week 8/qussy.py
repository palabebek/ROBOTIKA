from controller import Robot

# Konfigurasi dasar
TIME_STEP = 64
MAX_SPEED = 6.28

# Inisialisasi robot
robot = Robot()

# Inisialisasi sensor jarak inframerah (ps0 hingga ps7)
prox_sensor = []
sensor_names = ["ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"]

for name in sensor_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    prox_sensor.append(sensor)

# Inisialisasi motor
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Threshold jarak sensor
FRONT_THRESHOLD = 150.0  # Jarak untuk deteksi dinding di depan
SIDE_THRESHOLD = 80.0    # Jarak untuk deteksi dinding di samping kiri
TURN_THRESHOLD = 100.0   # Jarak untuk deteksi berbelok di ujung lorong

# Loop utama
while robot.step(TIME_STEP) != -1:
    # Membaca nilai sensor jarak
    front = prox_sensor[0].getValue()   # Sensor depan (ps0)
    front_right = prox_sensor[1].getValue()  # Sensor depan kanan (ps1)
    front_left = prox_sensor[6].getValue()  # Sensor depan kiri (ps6)
    left_side = prox_sensor[7].getValue()   # Sensor samping kiri (ps7)

    # Mengatur kecepatan dasar
    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED

    # Logika untuk zig-zag
    if front > FRONT_THRESHOLD or front_left > FRONT_THRESHOLD:
        # Jika ada dinding di depan, berbelok ke kanan
        left_speed = 0.5 * MAX_SPEED
        right_speed = -0.2 * MAX_SPEED
    elif left_side < SIDE_THRESHOLD:
        # Jika tidak ada dinding di samping kiri, berbelok kiri sedikit
        left_speed = 0.3 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED
    elif front_right > TURN_THRESHOLD:
        # Jika berada di ujung lorong, berbelok ke atas atau bawah
        left_speed = -0.2 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED
    else:
        # Berjalan lurus mengikuti dinding di samping kiri
        left_speed = 0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED

    # Mengatur kecepatan motor
    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
