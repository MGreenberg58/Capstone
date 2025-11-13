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

PIXELS_PER_M = 6.0 
BASE_X, BASE_Y = 100, HEIGHT - 100

def to_pygame(pos):
    x, y = pos
    return int(BASE_X + x * PIXELS_PER_M), int(BASE_Y - y * PIXELS_PER_M)

space = pymunk.Space()
space.gravity = (0.0, -9.81)
space.iterations = 100
base_pos = (0.0, 0.0)

ground= pymunk.Segment(space.static_body, (-1000, base_pos[1]), (1000, base_pos[1]), 1.0)
ground.elasticity = 0.5
ground.friction = 1.0
space.add(ground)

crane = Crane(space, base_pos)
fis = CraneFIS()

angle_history = deque(maxlen=800)
sim_time = 0.0

boom_target_angle = crane.boom_bodies[0].angle
hoist_speed = 0.02       # m per frame
extend_speed = 0.01      # m per frame
boom_min_len = 3.0
boom_max_len = 100.0

payload_mass_min = 100.0
payload_mass_max = 2000.0
pslider_rect = pygame.Rect(WIDTH - 340 + 18, HEIGHT - 200, 260, 20)
pslider_dragging = False

boom_mass_min = 6000.0
boom_mass_max = 12000.0
bslider_rect = pygame.Rect(WIDTH - 340 + 18, HEIGHT - 250, 260, 20)
bslider_dragging = False

boom_target_angle = crane.boom_bodies[0].angle
k_p = 50000000.0
k_d = 25000000.0

def draw_slider(label, value, min_val, max_val, rect, color=(80,140,220)):
    pygame.draw.rect(screen, (180,180,180), rect)
    rel = (value - min_val) / (max_val - min_val)
    knob_x = rect.x + int(rel * rect.width)
    pygame.draw.circle(screen, color, (knob_x, rect.centery), 10)
    font = pygame.font.SysFont("Consolas", 16)
    screen.blit(font.render(f"{label}: {value:.1f} kg", True, (10,10,10)), (rect.x, rect.y - 24))

def draw_ui(panel_rect, inputs, outputs, cg_px, pivot_px, boom_cg_px):
    x, y, w, h = panel_rect
    pygame.draw.rect(screen, (240,240,240), panel_rect)
    pygame.draw.rect(screen, (200,200,200), (x+6, y+6, w-12, h-12))

    font = pygame.font.SysFont("Consolas", 16)
    big = pygame.font.SysFont("Consolas", 18, bold=True)

    screen.blit(big.render("CraneSafetyFIS", True, (10,10,10)), (x + 18, y + 18))

    draw_slider("Payload Mass", crane.payload_mass, payload_mass_min, payload_mass_max, pslider_rect)
    draw_slider("Boom Mass", sum(crane.boom_masses), boom_mass_min, boom_mass_max, bslider_rect)

    pygame.draw.circle(screen, (0,0,255), cg_px, 6) 
    pygame.draw.circle(screen, (0,0,0), pivot_px, 6)  
    pygame.draw.circle(screen, (0,200,0), boom_cg_px, 6)

    label_y = y + 52
    for k,v in inputs.items():
        screen.blit(font.render(f"{k}: {v:.2f}", True, (10,10,10)), (x + 18, label_y))
        label_y += 20

    ca = outputs.get('ControlAdjustment', 0.0)
    ofb = outputs.get('OperatorFeedback', 0.0)
    bar_w = w - 40
    bar_x = x + 18

    screen.blit(font.render("ControlAdjustment", True, (10,10,10)), (bar_x, label_y))
    label_y += 16
    pygame.draw.rect(screen, (180,180,180), (bar_x, label_y, bar_w, 18))
    filled = int((ca / 3.0) * bar_w)
    pygame.draw.rect(screen, (80,140,220), (bar_x, label_y, filled, 18))
    screen.blit(font.render(f"{ca:.3f}", True, (10,10,10)), (bar_x + 8, label_y + 3))
    label_y += 24

    screen.blit(font.render("OperatorFeedback", True, (10,10,10)), (bar_x, label_y))
    label_y += 16
    pygame.draw.rect(screen, (180,180,180), (bar_x, label_y, bar_w, 18))
    filled2 = int((ofb / 3.0) * bar_w)
    pygame.draw.rect(screen, (220,110,80), (bar_x, label_y, filled2, 18))
    screen.blit(font.render(f"{ofb:.3f}", True, (10,10,10)), (bar_x + 8, label_y + 3))
    label_y += 36

    screen.blit(font.render(f"Boom Moment: {boom_moment/1000:.1f} kNm", True, (10,10,10)), (x + 18, label_y))
    label_y += 20
    screen.blit(font.render(f"Payload Moment: {payload_moment/1000:.1f} kNm", True, (10,10,10)), (x + 18, label_y))
    label_y += 36

    # draw CG marker and pivot marker
    pygame.draw.circle(screen, (0,0,255), (x + 60, y + h - 80), 6)
    screen.blit(font.render("System CG", True, (10,10,10)), (x + 72, y + h - 86))
    pygame.draw.circle(screen, (0,0,0), (x + 60, y + h - 50), 6)
    screen.blit(font.render("Pivot", True, (10,10,10)), (x + 72, y + h - 56))
    pygame.draw.circle(screen, (0,200,0), (x + 60, y + h - 110), 6)
    screen.blit(font.render("Boom CG", True, (10,10,10)), (x + 72, y + h - 116))
    

running = True
while running:
    dt = 1.0 / FPS
    sim_time += dt

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN:
            if pslider_rect.collidepoint(event.pos):
                pslider_dragging = True
            if bslider_rect.collidepoint(event.pos):
                bslider_dragging = True

        if event.type == pygame.MOUSEBUTTONUP:
            pslider_dragging = False
            bslider_dragging = False

        if event.type == pygame.MOUSEMOTION:
            if pslider_dragging:
                x = event.pos[0]
                rel_x = np.clip((x - pslider_rect.x) / pslider_rect.width, 0, 1)
                payload_mass = payload_mass_min + rel_x * (payload_mass_max - payload_mass_min)
                crane.set_payload_mass(payload_mass)
            if bslider_dragging:
                x = event.pos[0]
                rel_x = np.clip((x - bslider_rect.x) / bslider_rect.width, 0, 1)
                boom_mass = boom_mass_min + rel_x * (boom_mass_max - boom_mass_min)
                crane.set_boom_mass(boom_mass)

    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        running = False

    # Boom angle
    rotating = False
    if keys[pygame.K_q]:
        boom_target_angle += 0.002
    if keys[pygame.K_e]:
        boom_target_angle -= 0.002

    max_diff = math.radians(2)
    boom_target_angle = np.clip(boom_target_angle, crane.boom_bodies[0].angle - max_diff, crane.boom_bodies[0].angle + max_diff)

    error = boom_target_angle - crane.boom_bodies[0].angle
    ang_vel = crane.boom_bodies[0].angular_velocity
    torque = k_p * error - k_d * ang_vel - crane.gravity_moment()
    crane.boom_bodies[0].torque += torque

    # Hoist
    if keys[pygame.K_w]:
        crane.hoist_length = max(0.5, crane.hoist_length - hoist_speed)
        pin, slide = crane.payload_rope
        slide.min = crane.hoist_length
        slide.max = crane.hoist_length

    if keys[pygame.K_s]:
        crane.hoist_length = min(50.0, crane.hoist_length + hoist_speed)
        pin, slide = crane.payload_rope
        slide.min = crane.hoist_length
        slide.max = crane.hoist_length

    # Boom extend
    length_changed = False
    if keys[pygame.K_d]:
        crane.telescope(-extend_speed)
    if keys[pygame.K_a]:
        crane.telescope(extend_speed)

    boom_cg = crane.compute_boom_cg()
    pivot = crane.base_pos

    boom_lever = abs(boom_cg.x - pivot.x)
    payload_lever = abs(crane.payload_body.position.x - pivot.x)

    boom_moment = sum(crane.boom_masses) * 9.81 * boom_lever
    payload_moment = crane.payload_mass * 9.81 * payload_lever

    # FIS
    cg_pos = crane.compute_cg()
    CGD_input = abs(cg_pos.x - crane.base_pos.x)
    BL_input = crane.boom_length()
    PH_input = (crane.payload_body.position.y - crane.base_pos.y)

    ca, ofb = fis.evaluate(BL_input, CGD_input, PH_input)

    space.step(dt)

    boom_tip_world = crane.boom_tip_world()
    dx = crane.payload_body.position.x - boom_tip_world.x
    dy = crane.payload_body.position.y - boom_tip_world.y
    theta = math.atan2(dx, -dy) if (abs(dx) > 1e-9 or abs(dy) > 1e-9) else 0.0
    angle_history.append(theta)

    # Rendering
    screen.fill((250, 250, 250))

    base_px = to_pygame(crane.base_pos)
    pygame.draw.rect(screen, (120,120,120), (base_px[0] - 20, base_px[1], 40, 120))

    # boom
    prev_px = to_pygame(crane.base_pos) 
    for body, length in zip(crane.boom_bodies, crane.boom_sections):
        start_world = body.position + pymunk.Vec2d(-length/2, 0).rotated(body.angle)
        end_world   = body.position + pymunk.Vec2d(length/2, 0).rotated(body.angle)
        start_px = to_pygame(start_world)
        end_px   = to_pygame(end_world)

        shape = next((s for s in body.shapes if hasattr(s, "color")), None)
        color = getattr(shape, "color", (240,200,0))
        pygame.draw.line(screen, color, start_px, end_px, max(4, int(PIXELS_PER_M * 0.8)))

    # cable and payload
    tip_body = crane.boom_tip_body
    tip_length = crane.boom_sections[-1]
    tip_anchor_local = (tip_length / 2, 0)
    tip_anchor_world = tip_body.local_to_world(tip_anchor_local)

    boom_tip_px = to_pygame(tip_anchor_world)
    payload_px  = to_pygame(crane.payload_body.position)

    pygame.draw.line(screen, (40,40,40), boom_tip_px, payload_px, max(1, int(PIXELS_PER_M * 0.05)))
    pygame.draw.circle(screen, (200,40,40), payload_px, max(2, int(PIXELS_PER_M * crane.payload_radius)))

    # draw FIS panel
    panel_rect = (WIDTH - 360, 20, 340, HEIGHT - 40)
    inputs = {'BL (m)': BL_input, 'CGD (m)': CGD_input, 'PH (m)': PH_input}
    outputs = {'ControlAdjustment': ca, 'OperatorFeedback': ofb}
    draw_ui(panel_rect, inputs, outputs, to_pygame(cg_pos), to_pygame(crane.base_pos), to_pygame(boom_cg))

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
