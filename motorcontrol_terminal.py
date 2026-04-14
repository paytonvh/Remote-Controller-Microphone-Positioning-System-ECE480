import serial
import tty
import sys
import termios
import math

# ESP32 Serial Connection
ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, timeout=1)

# Mic state 
HOME_POS = [19.0, 43.0, 52.0]   # X, Y, Z (inches)
mic_pos  = HOME_POS.copy()

# Room geometry
CORNERS = [
    ( 2.0,  0.0, 56.0),  # Motor 1 — front left
    (36.0,  0.0, 56.0),  # Motor 2 — front right
    ( 2.0, 86.0, 56.0),  # Motor 3 — rear left
    (36.0, 86.0, 56.0),  # Motor 4 — rear right
]

BOUNDS = {
    'x': (8.0,  28.0),
    'y': (8.0,  78.0),
    'z': (12.0, 50.0),
}

PULLEY_CIRC = 5.25  # inches

# Speed
CRUISE_SPEED_IPS    = 3.0   # inches per second cruise speed
HOME_SPEED_IPS      = 2.0
EMERGENCY_SPEED_IPS = 4.0

# Step sizes (inches)
STEP_SIZES  = [0.5, 1.0, 3.0, 6.0, 12.0]
step_index  = 2
step_size   = STEP_SIZES[step_index]

# 
def clamp(value, lo, hi):
    return max(lo, min(hi, value))

# Calculate cable length for 
def cable_length(pos, corner):
    return math.sqrt(sum((pos[i] - corner[i])**2 for i in range(3)))

def distance(a, b):
    return math.sqrt(sum((a[i] - b[i])**2 for i in range(3)))

"""
Calculate move duration in ms from distance and speed.
"""
def duration_from_dist(dist, speed_ips):
    if dist < 0.001:
        return 0
    ms = int((dist / speed_ips) * 1000)
    return max(ms, 100)   # 100ms minimum so ramp has room to run

"""
Build packet to be translated by ESP32
Format: "M1:12.50,M2:-8.30,M3:5.10,M4:-3.90,DUR:500,BLOCK"
Rotations are signed — positive reels out, negative reels in.
"""
def build_packet(prev_pos, new_pos, duration_ms, blocking=False):
    parts = []
    for i, corner in enumerate(CORNERS):
        dL  = cable_length(new_pos, corner) - cable_length(prev_pos, corner)
        rot = (dL / PULLEY_CIRC) * 360.0
        parts.append(f"M{i+1}:{rot:.2f}")
    parts.append(f"DUR:{duration_ms}")
    if blocking:
        parts.append("BLOCK")
    return ",".join(parts)

def save_position(pos):
    with open("position.txt", 'w') as f:
        f.write(f"{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}")

def load_position():
    global mic_pos
    try:
        with open("position.txt") as f:
            x, y, z = map(float, f.read().strip().split(', '))
            mic_pos = [x, y, z]
            print(f"Restored position: X={x:.2f}  Y={y:.2f}  Z={z:.2f}")
    except FileNotFoundError:
        print("No position file — starting at home.")
        mic_pos = HOME_POS.copy()
    except ValueError:
        print("Position file corrupted — starting at home.")
        mic_pos = HOME_POS.copy()

# Communication
def wait_for_done(timeout=60.0):
    import time
    ser.timeout = 0.05
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode('utf-8').strip()
        if line == "DONE":
            return True
        elif line:
            print(f"  ESP32: {line}")
    print("Warning: timed out waiting for DONE")
    return False

def send_blocking(packet, new_pos):
    """Send packet, block until DONE, then update position."""
    ser.write((packet + '\n').encode('utf-8'))
    wait_for_done()
    mic_pos[:] = new_pos
    save_position(mic_pos)

"""
Compute Jacobian factor for the Z-axis movement
"""
def z_amplification_factor(pos):
    total = 0
    for corner in CORNERS:
        dx = corner[0] - pos[0]
        dy = corner[1] - pos[1]
        dz = corner[2] - pos[2]
        L  = math.sqrt(dx*dx + dy*dy + dz*dz)

        # how much cable changes per unit Z
        total += abs(dz / L)
    return total / 4

"""
Move mic by (dx, dy, dz)
Blocks until ESP32 confirms done
"""
def move_mic(dx, dy, dz):
    global mic_pos
    prev = mic_pos.copy()

    # Ensure movement won't go out of bounds
    new = [
        clamp(prev[0] + dx, *BOUNDS['x']),
        clamp(prev[1] + dy, *BOUNDS['y']),
        clamp(prev[2] + dz, *BOUNDS['z']),
    ]

    dist = distance(prev, new)
    if dist < 0.001:
        print("Already at boundary in that direction.")
        return

    dur    = duration_from_dist(dist, CRUISE_SPEED_IPS)
    packet = build_packet(prev, new, dur, blocking=True)

    print(f"Moving to X={new[0]:.2f}  Y={new[1]:.2f}  Z={new[2]:.2f}  "
          f"dist={dist:.2f}\"  dur={dur}ms")
    send_blocking(packet, new)

def move_left():
    move_mic(-step_size, 0.0, 0.0)

def move_right():    
    move_mic( step_size, 0.0, 0.0)

def move_forward():  
    move_mic(0.0,  -step_size, 0.0)

def move_backward(): 
    move_mic(0.0, step_size, 0.0)

def move_up():
    factor = z_amplification_factor(mic_pos)
    move_mic(0.0, 0.0, step_size / factor)

def move_down():
    factor = z_amplification_factor(mic_pos)
    move_mic(0.0, 0.0, -step_size / factor)

def mic_home():
    global mic_pos

    safe_z  = BOUNDS['z'][1] - 4.0   # near top of Z range
    prev    = mic_pos.copy()
    step1   = [prev[0], prev[1], safe_z]
    dist1   = distance(prev, step1)

    if dist1 > 0.001:
        dur1   = duration_from_dist(dist1, HOME_SPEED_IPS)
        pkt1   = build_packet(prev, step1, dur1, blocking=True)
        print("Homing")
        send_blocking(pkt1, step1)

    prev2   = mic_pos.copy()
    target  = HOME_POS.copy()
    dist2   = distance(prev2, target)

    if dist2 > 0.001:
        dur2   = duration_from_dist(dist2, HOME_SPEED_IPS)
        pkt2   = build_packet(prev2, target, dur2, blocking=True)
        send_blocking(pkt2, target)
    print("Home complete")

def mic_emergency():
    global mic_pos
    prev   = mic_pos.copy()
    target = [prev[0], prev[1], BOUNDS['z'][1]]
    dist   = distance(prev, target)
    if dist < 0.001:
        print("Already at max Z.")
        return
    dur    = duration_from_dist(dist, EMERGENCY_SPEED_IPS)
    packet = build_packet(prev, target, dur, blocking=True)
    print("EMERGENCY RAISE")
    send_blocking(packet, target)
    print("Emergency raise complete.")

# Step size control
def step_up():
    global step_index, step_size
    if step_index < len(STEP_SIZES) - 1:
        step_index += 1
        step_size   = STEP_SIZES[step_index]
    print(f"Step size: {step_size}\"")

def step_down():
    global step_index, step_size
    if step_index > 0:
        step_index -= 1
        step_size   = STEP_SIZES[step_index]
    print(f"Step size: {step_size}\"")
 
def get_key():
    fd  = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

def main():
    load_position()

    print("\nControls:")
    print("  W / S    -- forward / backward  (Y)")
    print("  A / D    -- left / right        (X)")
    print("  U / J    -- up / down           (Z)")
    print("  H        -- home (blocking)")
    print("  E        -- emergency raise (blocking)")
    print("  + / -    -- step size up / down")
    print(f"  Step sizes: {STEP_SIZES}\"")
    print(f"  Current step: {step_size}\"\n")

    KEYMAP = {
        'a': move_left,
        'd': move_right,
        'w': move_forward,
        's': move_backward,
        'u': move_up,
        'j': move_down,
        'h': mic_home,
        'e': mic_emergency,
        '+': step_up,
        '=': step_up,
        '-': step_down,
    }

    try:
        while True:
            key = get_key()
            if key == 'q':
                break
            action = KEYMAP.get(key)
            if action:
                action()
    finally:
        ser.close()
        print("\nConnection closed.")

if __name__ == "__main__":
    main()
