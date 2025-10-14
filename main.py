import pygame
import pymunk
import math
import sys

# --- Init PyGame ---
pygame.init()
WIDTH, HEIGHT = 1000, 700
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Crane — Real-World Physics")

clock = pygame.time.Clock()
FPS = 120

# --- Conversion factor ---
PIXELS_PER_M = 100.0
BASE_X, BASE_Y = 200, HEIGHT - 120

def to_pygame(pos):
    """Convert physics (meters) to screen pixels."""
    x, y = pos
    return int(BASE_X + x * PIXELS_PER_M), int(BASE_Y - y * PIXELS_PER_M)

# --- Pymunk Space ---
space = pymunk.Space()
space.gravity = (0.0, -9.81)  # m/s^2 downward (real units)

base_pos = (0, 0)
base_body = pymunk.Body(body_type=pymunk.Body.STATIC)
base_body.position = base_pos

# --- Crane Boom ---
boom_length = 5.0
boom_mass = 20.0
boom_moment = pymunk.moment_for_segment(boom_mass, (-boom_length/2,0), (boom_length/2,0), 0.1)
boom_body = pymunk.Body(boom_mass, boom_moment)

# Set COM so that left end aligns with base
pivot_world = base_body.position
boom_body.position = pivot_world + pymunk.Vec2d(boom_length/2, 0)

boom_shape = pymunk.Segment(boom_body, (-boom_length/2,0), (boom_length/2,0), 0.1)
boom_shape.color = (240,200,0,255)
space.add(boom_body, boom_shape)

# Pivot joint
pivot = pymunk.PinJoint(base_body, boom_body, (0,0), (-boom_length/2,0))
space.add(pivot)

# Limit rotation
rot_limit = pymunk.RotaryLimitJoint(base_body, boom_body, -math.pi/3, math.pi/2)
space.add(rot_limit)

# --- Payload ---
payload_mass = 3.0       # kg
payload_radius = 0.5     # m
payload_moment = pymunk.moment_for_circle(payload_mass, 0, payload_radius)
payload_body = pymunk.Body(payload_mass, payload_moment)

# Hoist cable
hoist_length = 4.0  # meters
payload_body.position = (boom_length/2, -hoist_length)
payload_shape = pymunk.Circle(payload_body, payload_radius)
payload_shape.color = (200, 40, 40, 255)
space.add(payload_body, payload_shape)

# Damped spring simulating hoist cable
spring = pymunk.DampedSpring(
    boom_body, payload_body,
    (boom_length/2, 0),  # boom tip
    (0, 0),              # payload center
    hoist_length,
    stiffness=2000.0,    # N/m
    damping=50.0          # N*s/m
)
space.add(spring)

def damped_velocity_func(body, gravity, damping, dt):
    pymunk.Body.update_velocity(body, gravity, body.damping, dt)

boom_body.velocity_func = damped_velocity_func
payload_body.velocity_func = damped_velocity_func

# --- User control parameters ---
boom_torque_mag = 800.0  # N*m
hoist_speed = 0.01          # m per frame
boom_body.damping = 0.99
payload_body.damping = 0.995

# --- Main Loop ---
running = True
while running:
    # Event handling
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        running = False

    # --- Controls ---
    if keys[pygame.K_q]:
        boom_body.torque += boom_torque_mag
    if keys[pygame.K_e]:
        boom_body.torque += -boom_torque_mag
    if keys[pygame.K_w]:
        hoist_length = max(0.5, hoist_length - hoist_speed)
        spring.rest_length = hoist_length
    if keys[pygame.K_s]:
        hoist_length = min(10.0, hoist_length + hoist_speed)
        spring.rest_length = hoist_length

    if keys[pygame.K_q] or keys[pygame.K_e]:
        boom_body.damping = 0.99
    else:
        boom_body.damping = 0.8

    # --- Step physics ---
    space.step(1.0 / FPS)

    # --- Rendering ---
    screen.fill((245, 245, 245))

    # Draw base
    pygame.draw.rect(screen, (130,130,130), (BASE_X-20, BASE_Y, 40, 120))

    # Boom
    boom_tip_world = boom_body.position + pymunk.Vec2d(boom_length/2, 0).rotated(boom_body.angle)
    boom_start_world = boom_body.position + pymunk.Vec2d(-boom_length/2, 0).rotated(boom_body.angle)
    boom_start_pixel = to_pygame(boom_start_world)
    boom_end_pixel = to_pygame(boom_tip_world)
    pygame.draw.line(screen, (240,200,0), boom_start_pixel, boom_end_pixel, 8)

    # Hoist cable
    payload_px = to_pygame(payload_body.position)
    pygame.draw.line(screen, (40,40,40), boom_end_pixel, payload_px, 2)

    # Payload
    pygame.draw.circle(screen, (200,40,40), payload_px, int(PIXELS_PER_M * payload_radius))

    # Overlay text
    font = pygame.font.SysFont("Consolas", 18)
    lines = [
        f"boom_angle = {math.degrees(boom_body.angle)%360:.2f} deg",
        f"hoist_length = {spring.rest_length:.2f} m",
        f"payload_pos = ({payload_body.position.x:.2f}, {payload_body.position.y:.2f}) m",
        "Controls: Q/E rotate boom, W/S up/down winch, Esc quit"
    ]
    for i, l in enumerate(lines):
        screen.blit(font.render(l, True, (10,10,10)), (20, 20 + 22*i))

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
sys.exit()
