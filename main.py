import pygame
import math
import sys
from collections import deque

# --- Parameters / scaling ---
pygame.init()
WIDTH, HEIGHT = 1000, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Crane — full physics (meter-based)")

clock = pygame.time.Clock()
FPS = 120
dt = 1.0 / FPS

# physics units: meters
PIXELS_PER_M = 40.0
BASE_X_PX = 200
BASE_Y_PX = HEIGHT - 120

# initial crane geometry (in meters)
boom_length_m = 5.0
boom_angle_deg = 45.0
hoist_length_m = 4.0

# control speeds (physical units)
boom_angle_rate_deg_per_s = 30.0
boom_length_rate_m_per_s = 1
hoist_rate_m_per_s = 0.75

payload_radius_px = int(0.5 * PIXELS_PER_M)  # 0.5 m radius visually

# pendulum state
theta = 0.0
theta_dot = 0.0

g = 9.81

# smoothing buffers for numerical derivatives (meters)
vel_buf_x = deque(maxlen=4)
vel_buf_y = deque(maxlen=4)
acc_buf_x = deque(maxlen=4)
acc_buf_y = deque(maxlen=4)
L_dot_buf = deque(maxlen=4)
L_ddot_buf = deque(maxlen=4)

# previous pivot pos/length (in meters)
prev_pivot_m = None
prev_pivot_vx = 0.0
prev_pivot_vy = 0.0
prev_hoist_length_m = hoist_length_m
prev_hoist_v = 0.0

theta_damping = 0.995

font = pygame.font.SysFont("Consolas", 18)

def world_to_screen(x_m, y_m):
    """convert world meters (x right, y down) relative to base_pos to screen pixels"""
    px = BASE_X_PX + x_m * PIXELS_PER_M
    py = BASE_Y_PX + y_m * PIXELS_PER_M
    return int(px), int(py)

def draw_crane(boom_tip_m, payload_m):
    screen.fill((245,245,245))
    # base
    pygame.draw.rect(screen, (130,130,130), (BASE_X_PX-20, BASE_Y_PX, 40, 120))
    # boom
    pygame.draw.line(screen, (240,200,0), (BASE_X_PX, BASE_Y_PX),
                     world_to_screen(*boom_tip_m), 8)
    # hoist cable
    pygame.draw.line(screen, (40,40,40),
                     world_to_screen(*boom_tip_m),
                     world_to_screen(*payload_m), 2)
    # payload
    payload_px = world_to_screen(payload_m[0], payload_m[1])
    pygame.draw.circle(screen, (200,40,40),
                       (payload_px[0], payload_px[1] + payload_radius_px),
                       payload_radius_px)
    # overlay text
    lines = [
        f"boom_length = {boom_length_m:.2f} m",
        f"boom_angle = {boom_angle_deg:.2f} deg",
        f"hoist_length = {hoist_length_m:.2f} m",
        f"theta  (swing) = {math.degrees(theta):.2f} deg",
        f"theta_dot = {math.degrees(theta_dot):.2f} deg/s",
        "Controls: Q/E angle up/down | D/A extend/retract boom | W/S hoist in/out | Esc to quit"
    ]
    for i, l in enumerate(lines):
        screen.blit(font.render(l, True, (10,10,10)), (20, 20 + 22*i))
    pygame.display.flip()

def avg(buf):
    if len(buf) == 0: return 0.0
    return sum(buf) / len(buf)

# main loop
running = True
while running:
    # handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        running = False

    # controls: set commanded velocities (deg/s or m/s)
    if keys[pygame.K_q]:
        boom_angle_deg += boom_angle_rate_deg_per_s * dt
    if keys[pygame.K_e]:
        boom_angle_deg -= boom_angle_rate_deg_per_s * dt
    if keys[pygame.K_d]:
        boom_length_m += boom_length_rate_m_per_s * dt
    if keys[pygame.K_a]:
        boom_length_m -= boom_length_rate_m_per_s * dt
    if keys[pygame.K_w]:
        hoist_length_m -= hoist_rate_m_per_s * dt
    if keys[pygame.K_s]:
        hoist_length_m += hoist_rate_m_per_s * dt

    # clamp geometry to safe ranges
    boom_angle_deg = max(0.0, min(90.0, boom_angle_deg))
    boom_length_m = max(1.0, min(15.0, boom_length_m))
    hoist_length_m = max(0.25, min(10.0, hoist_length_m))

    # compute pivot (boom tip) in meters relative to base:
    phi_rad = math.radians(-boom_angle_deg) 
    pivot_x_m = boom_length_m * math.cos(phi_rad)
    pivot_y_m = boom_length_m * math.sin(phi_rad)
    pivot_m = (pivot_x_m, pivot_y_m)

    # numerical differentiation for pivot velocity & acceleration (in meters/sec)
    if prev_pivot_m is None:
        vx = vy = 0.0
    else:
        prev_x_m, prev_y_m = prev_pivot_m
        vx = (pivot_x_m - prev_x_m) / dt
        vy = (pivot_y_m - prev_y_m) / dt

    vel_buf_x.append(vx)
    vel_buf_y.append(vy)
    vx_f = avg(vel_buf_x)
    vy_f = avg(vel_buf_y)

    ax = (vx_f - prev_pivot_vx) / dt
    ay = (vy_f - prev_pivot_vy) / dt
    acc_buf_x.append(ax)
    acc_buf_y.append(ay)
    ax_f = avg(acc_buf_x)
    ay_f = avg(acc_buf_y)
    prev_pivot_vx = vx_f
    prev_pivot_vy = vy_f

    # hoist length derivatives (m/s and m/s^2) with smoothing
    L_dot = (hoist_length_m - prev_hoist_length_m) / dt
    L_dot_buf.append(L_dot)
    L_dot_f = avg(L_dot_buf)
    L_ddot = (L_dot_f - prev_hoist_v) / dt
    L_ddot_buf.append(L_ddot)
    L_ddot_f = avg(L_ddot_buf)
    prev_hoist_v = L_dot_f
    prev_hoist_length_m = hoist_length_m

    # pendulum physics all in meters, radians
    L = hoist_length_m
    theta_ddot = -(g / L) * math.sin(theta) - (1.0 / L) * (
        ax_f * math.cos(theta) + ay_f * math.sin(theta) + 2.0 * L_dot_f * theta_dot
    )
    theta_dot += theta_ddot * dt
    theta_dot *= theta_damping
    theta += theta_dot * dt

    # compute payload position in meters
    boom_tip_m = (pivot_x_m, pivot_y_m)
    payload_x_m = pivot_x_m + L * math.sin(theta)
    payload_y_m = pivot_y_m + L * math.cos(theta)
    payload_m = (payload_x_m, payload_y_m)

    draw_crane(boom_tip_m, payload_m)

    prev_pivot_m = pivot_m

    clock.tick(FPS)

pygame.quit()
sys.exit()
