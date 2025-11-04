import pygame
import pymunk
import math
import sys
import numpy as np
from collections import deque

from crane import Crane
from fis import CraneFIS

pygame.init()
WIDTH, HEIGHT = 1280, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Crane")
clock = pygame.time.Clock()
FPS = 120

PIXELS_PER_M = 6.0   # zoomed-out scale (you can change)
BASE_X, BASE_Y = 100, HEIGHT - 100

def to_pygame(pos):
    x, y = pos
    return int(BASE_X + x * PIXELS_PER_M), int(BASE_Y - y * PIXELS_PER_M)

space = pymunk.Space()
space.gravity = (0.0, -9.81)
space.iterations = 50

crane = Crane(space)
fis = CraneFIS()

# --- Control & GUI state ---
angle_history = deque(maxlen=800)
sim_time = 0.0

boom_target_angle = crane.boom_bodies[0].angle
was_rotating = False
boom_torque_up = 250000
boom_torque_down = 10000
hoist_speed = 0.03       # m per frame
extend_speed = 0.03      # m per frame
boom_min_len = 3.0
boom_max_len = 100.0

boom_target_angle = crane.boom_bodies[0].angle
was_rotating = False
k_p = 750000.0
k_d = 200000.0


def draw_ui(panel_rect, inputs, outputs, cg_px, pivot_px):
    x, y, w, h = panel_rect
    pygame.draw.rect(screen, (240,240,240), panel_rect)
    pygame.draw.rect(screen, (200,200,200), (x+6, y+6, w-12, h-12))

    font = pygame.font.SysFont("Consolas", 16)
    big = pygame.font.SysFont("Consolas", 18, bold=True)

    screen.blit(big.render("FIS — CraneSafetyFIS", True, (10,10,10)), (x + 18, y + 18))

    label_y = y + 52
    for k,v in inputs.items():
        screen.blit(font.render(f"{k}: {v:.2f}", True, (10,10,10)), (x + 18, label_y))
        label_y += 20

    # control adjustment bar (0..3)
    ca = outputs.get('ControlAdjustment', 0.0)
    ofb = outputs.get('OperatorFeedback', 0.0)
    bar_w = w - 60
    bar_x = x + 18

    screen.blit(font.render("ControlAdjustment", True, (10,10,10)), (bar_x, label_y))
    label_y += 18
    pygame.draw.rect(screen, (180,180,180), (bar_x, label_y, bar_w, 18))
    filled = int((ca / 3.0) * bar_w)
    pygame.draw.rect(screen, (80,140,220), (bar_x, label_y, filled, 18))
    screen.blit(font.render(f"{ca:.3f} / 3.0", True, (10,10,10)), (bar_x + bar_w + 8, label_y))
    label_y += 26

    screen.blit(font.render("OperatorFeedback", True, (10,10,10)), (bar_x, label_y))
    label_y += 18
    pygame.draw.rect(screen, (180,180,180), (bar_x, label_y, bar_w, 18))
    filled2 = int((ofb / 3.0) * bar_w)
    pygame.draw.rect(screen, (220,110,80), (bar_x, label_y, filled2, 18))
    screen.blit(font.render(f"{ofb:.3f} / 3.0", True, (10,10,10)), (bar_x + bar_w + 8, label_y))
    label_y += 36

    # draw CG marker and pivot marker inside panel for emphasis
    pygame.draw.circle(screen, (0,0,255), (x + 60, y + h - 80), 6)
    screen.blit(font.render("System CG", True, (10,10,10)), (x + 72, y + h - 86))
    pygame.draw.circle(screen, (0,0,0), (x + 60, y + h - 50), 6)
    screen.blit(font.render("Pivot", True, (10,10,10)), (x + 72, y + h - 56))



# --- Main loop ---
running = True
while running:
    dt = 1.0 / FPS
    sim_time += dt

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        running = False

    # Boom angle
    rotating = False
    if keys[pygame.K_q]:
        crane.boom_bodies[0].torque += boom_torque_up * abs(math.cos(crane.boom_bodies[0].angle)) + crane.gravity_moment()
        rotating = True
    if keys[pygame.K_e]:
        crane.boom_bodies[0].torque -= boom_torque_down * abs(math.cos(crane.boom_bodies[0].angle)) + crane.gravity_moment()
        rotating = True

    # Hoist
    if keys[pygame.K_w]:
        crane.hoist_length = max(0.5, crane.hoist_length - hoist_speed)
        crane.boom_springs[-1].rest_length = crane.hoist_length
    if keys[pygame.K_s]:
        crane.hoist_length = min(20.0, crane.hoist_length + hoist_speed)
        crane.boom_springs[-1].rest_length = crane.hoist_length

    # Boom extend
    if keys[pygame.K_d]:
        crane.telescope(direction=+1, speed=extend_speed)
    if keys[pygame.K_a]:
        crane.telescope(direction=-1, speed=extend_speed)

    if was_rotating and not rotating:
        boom_target_angle = crane.boom_bodies[0].angle

    if not rotating:
        error = boom_target_angle - crane.boom_bodies[0].angle
        ang_vel = crane.boom_bodies[0].angular_velocity

        hold_torque = k_p * error - k_d * ang_vel - crane.gravity_moment()
        crane.boom_bodies[0].torque += hold_torque

    was_rotating = rotating

    # --- Evaluate FIS each frame ---
    cg_pos = crane.compute_cg()
    CGD_input = abs(cg_pos.x - crane.base_pos.x)
    BL_input = crane.boom_length()
    PH_input = (crane.base_pos.y - crane.payload_body.position.y)

    ca, ofb = fis.evaluate(BL_input, CGD_input, PH_input)

    space.step(dt)

    boom_tip_world = crane.boom_tip_world()
    dx = crane.payload_body.position.x - boom_tip_world.x
    dy = crane.payload_body.position.y - boom_tip_world.y
    theta = math.atan2(dx, -dy) if (abs(dx) > 1e-9 or abs(dy) > 1e-9) else 0.0
    angle_history.append(theta)

    # --- Rendering ---
    screen.fill((250, 250, 250))

    base_px = to_pygame(crane.base_pos)
    pygame.draw.rect(screen, (120,120,120), (base_px[0] - 20, base_px[1], 40, 120))

    # boom
    prev_px = to_pygame(crane.base_pos) 
    for shape in crane.boom_shapes:
        body = shape.body
        p1 = body.position + shape.a.rotated(body.angle)
        p2 = body.position + shape.b.rotated(body.angle)

        # Convert to pixel coordinates
        boom_start_px = to_pygame(p1)
        boom_tip_px   = to_pygame(p2)

        color = getattr(shape, "color", (240, 200, 0, 255))[:3]  # fallback to yellow
        pygame.draw.line(screen, color, boom_start_px, boom_tip_px, int(max(1, crane.boom_thickness * PIXELS_PER_M)))

    # cable and payload
    boom_tip_px = to_pygame(crane.boom_tip_world())
    payload_px  = to_pygame(crane.payload_body.position)
    pygame.draw.line(screen, (40,40,40), boom_tip_px, payload_px, max(1, int(PIXELS_PER_M * 0.05)))
    pygame.draw.circle(screen, (200,40,40), payload_px, max(2, int(PIXELS_PER_M * crane.payload_radius)))


    # draw CG and pivot markers
    cg_px = to_pygame(cg_pos)
    pivot_px = to_pygame(crane.base_pos)
    pygame.draw.circle(screen, (0,0,255), cg_px, 6)    # CG (blue)
    pygame.draw.circle(screen, (0,0,0), pivot_px, 6)   # pivot (black)


    # draw FIS panel
    panel_rect = (WIDTH - 360, 20, 340, HEIGHT - 40)
    inputs = {'BL (m)': BL_input, 'CGD (m)': CGD_input, 'PH (m)': PH_input}
    outputs = {'ControlAdjustment': ca, 'OperatorFeedback': ofb}
    draw_ui(panel_rect, inputs, outputs, cg_px, pivot_px)

    # small instruction text
    font_sm = pygame.font.SysFont("Consolas", 16)
    lines = [
        "Controls: Q/E rotate, A/D extend/retract, W/S winch, Esc quit",
        f"Zoom (PIXELS_PER_M) = {PIXELS_PER_M:.2f}"
    ]
    for i,l in enumerate(lines):
        screen.blit(font_sm.render(l, True, (10,10,10)), (18, 18 + 20*i))

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
sys.exit()
