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
    x, y = pos
    return int(BASE_X + x * PIXELS_PER_M), int(BASE_Y - y * PIXELS_PER_M)

# --- Pymunk Space ---
space = pymunk.Space()
space.gravity = (0.0, -9.81)  # m/s^2 downward (real units)

# --- Base (static) ---
base_body = pymunk.Body(body_type=pymunk.Body.STATIC)
base_body.position = (0.0, 0.0)  # world coords (meters)
space.add(base_body)

# --- Parameters ---
boom_length = 5.0
boom_mass = 20.0
boom_thickness = 0.1

payload_mass = 3.0
payload_radius = 0.3

hoist_length = 4.0  # meters

# placeholders for globals that functions will set
boom_body = None
boom_shape = None
pivot = None
rot_limit = None
spring = None
payload_body = None
payload_shape = None

# --- Create payload first (we attach spring to boom later) ---
payload_moment = pymunk.moment_for_circle(payload_mass, 0, payload_radius)
payload_body = pymunk.Body(payload_mass, payload_moment)
# set a temporary position; will set relative to boom tip after boom exists
payload_body.position = (boom_length + 0.5, -hoist_length)
payload_shape = pymunk.Circle(payload_body, payload_radius)
payload_shape.color = (200, 40, 40, 255)
space.add(payload_body, payload_shape)


def make_initial_boom(length):
    """Create the boom body (added once), shape, pivot joint and rot limit.
       boom_body.position is the COM world position (base + length/2, 0).
    """
    global boom_body, boom_shape, pivot, rot_limit

    # create body with moment for a centered segment (-L/2 .. +L/2)
    boom_moment = pymunk.moment_for_segment(boom_mass, (-length/2, 0), (length/2, 0), boom_thickness)
    boom_body = pymunk.Body(boom_mass, boom_moment)

    # place COM so left end sits exactly at base_body.position
    boom_body.position = base_body.position + (length / 2.0, 0.0)
    boom_body.angle = 0.0

    # shape centered on COM in local coordinates
    boom_shape = pymunk.Segment(boom_body, (-length/2, 0), (length/2, 0), boom_thickness)
    boom_shape.color = (240, 200, 0, 255)

    # add body+shape once
    space.add(boom_body, boom_shape)

    # pivot: attach base (local (0,0)) to left end of boom (local -length/2,0)
    pivot = pymunk.PinJoint(base_body, boom_body, (0, 0), (-length/2, 0))
    space.add(pivot)

    # rotation limit
    rot_limit = pymunk.RotaryLimitJoint(base_body, boom_body, -math.pi/3, math.pi/2)
    space.add(rot_limit)


def create_spring(length):
    """(Re)create spring connecting boom tip to payload center. Removes old spring if present."""
    global spring
    if spring is not None and spring in space.constraints:
        space.remove(spring)
    spring = pymunk.DampedSpring(
        boom_body, payload_body,
        (length/2.0, 0.0),  # anchor on boom in boom-local coords
        (0.0, 0.0),         # anchor on payload (its COM)
        hoist_length,
        stiffness=2000.0,
        damping=50.0
    )
    space.add(spring)


def create_boom(length):
    """Called when length changes. Replace only the shape and spring, update moment and pivot anchor.
       Keep the same boom_body in the space (don't re-add body).
    """
    global boom_shape, spring, pivot

    # preserve state
    old_angle = boom_body.angle
    old_ang_vel = boom_body.angular_velocity
    old_lin_vel = boom_body.velocity

    # remove old shape and spring (if present)
    if boom_shape in space.shapes:
        space.remove(boom_shape)
    if spring is not None and spring in space.constraints:
        space.remove(spring)

    # update moment for new centered segment
    boom_body.moment = pymunk.moment_for_segment(boom_mass, (-length/2, 0), (length/2, 0), boom_thickness)

    # move COM so left end still at base_world
    pivot_world = base_body.position
    boom_body.position = pivot_world + pymunk.Vec2d(length/2.0, 0.0).rotated(boom_body.angle)

    # restore state
    boom_body.angle = old_angle
    boom_body.angular_velocity = old_ang_vel
    boom_body.velocity = old_lin_vel

    # create new centered shape and add
    boom_shape = pymunk.Segment(boom_body, (-length/2, 0), (length/2, 0), boom_thickness)
    boom_shape.color = (240, 200, 0, 255)
    space.add(boom_shape)

    # update pivot's boom-local anchor so left end remains fixed
    pivot.anchor_b = (-length/2.0, 0.0)

    # recreate spring attached to new tip
    create_spring(length)

    # Optionally reposition payload if it ended up far away (we keep payload state so user can see continuity)


# --- initialize boom & spring properly (order matters) ---
make_initial_boom(boom_length)
create_spring(boom_length)

# After spring exists, set payload initial position relative to current boom tip (so it doesn't start stretched weirdly)
boom_tip_world = boom_body.position + pymunk.Vec2d(boom_length/2.0, 0.0).rotated(boom_body.angle)
payload_body.position = boom_tip_world + (0.0, -hoist_length)
payload_body.velocity = (0.0, 0.0)

# --- velocity_func damping wrapper so body.damping works ---
def damped_velocity_func(body, gravity, damping, dt):
    pymunk.Body.update_velocity(body, gravity, body.damping, dt)

boom_body.velocity_func = damped_velocity_func
payload_body.velocity_func = damped_velocity_func

# damping values (per-body)
boom_body.damping = 0.99
payload_body.damping = 0.995

# --- user control params ---
boom_torque_up = 700
boom_torque_down = 0
hoist_speed = 0.01       # m per frame
extend_speed = 0.02      # m per frame
boom_min_len = 3.0
boom_max_len = 12.0

# PD hold vars
boom_target_angle = boom_body.angle
was_rotating = False
k_p = 12000.0
k_d = 2500.0

# --- Main Loop ---
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        running = False

    # ----- Controls -----
    rotating = False
    if keys[pygame.K_q]:
        boom_body.torque += boom_torque_up * math.cos(boom_body.angle) + boom_length * math.cos(boom_body.angle) * (payload_mass + boom_mass)
        rotating = True
    if keys[pygame.K_e]:
        boom_body.torque -= boom_torque_down * math.cos(boom_body.angle) - boom_length * math.cos(boom_body.angle) * (payload_mass + boom_mass)
        rotating = True

    # hoist
    if keys[pygame.K_w]:
        hoist_length = max(0.5, hoist_length - hoist_speed)
        if spring is not None:
            spring.rest_length = hoist_length
    if keys[pygame.K_s]:
        hoist_length = min(10.0, hoist_length + hoist_speed)
        if spring is not None:
            spring.rest_length = hoist_length

    # extend / retract (A/D keys)
    length_changed = False
    if keys[pygame.K_d]:
        new_len = min(boom_max_len, boom_length + extend_speed)
        if new_len != boom_length:
            boom_length = new_len
            length_changed = True
    if keys[pygame.K_a]:
        new_len = max(boom_min_len, boom_length - extend_speed)
        if new_len != boom_length:
            boom_length = new_len
            length_changed = True

    if length_changed:
        create_boom(boom_length)

    if was_rotating and not rotating:
        boom_target_angle = boom_body.angle

    if not rotating:
        angle_error = boom_target_angle - boom_body.angle
        boom_body.torque += k_p * angle_error - k_d * boom_body.angular_velocity

    was_rotating = rotating

    # step physics
    space.step(1.0 / FPS)

    # rendering
    screen.fill((245, 245, 245))

    # draw base rectangle at base_body.position
    base_px = to_pygame(base_body.position)
    pygame.draw.rect(screen, (130, 130, 130), (base_px[0] - 20, base_px[1], 40, 120))

    # boom endpoints (centered shape: -L/2 .. +L/2)
    boom_start_world = boom_body.position + pymunk.Vec2d(-boom_length/2.0, 0.0).rotated(boom_body.angle)
    boom_tip_world = boom_body.position + pymunk.Vec2d(boom_length/2.0, 0.0).rotated(boom_body.angle)
    boom_start_px = to_pygame(boom_start_world)
    boom_tip_px = to_pygame(boom_tip_world)
    pygame.draw.line(screen, (240, 200, 0), boom_start_px, boom_tip_px, 8)

    # cable and payload
    payload_px = to_pygame(payload_body.position)
    pygame.draw.line(screen, (40, 40, 40), boom_tip_px, payload_px, 2)
    pygame.draw.circle(screen, (200, 40, 40), payload_px, int(PIXELS_PER_M * payload_radius))

    # overlays
    font = pygame.font.SysFont("Consolas", 18)
    lines = [
        f"boom_angle = {math.degrees(boom_body.angle)%360:.2f} deg",
        f"boom_length = {boom_length:.2f} m",
        f"hoist_length = {spring.rest_length:.2f} m",
        f"payload_pos = ({payload_body.position.x:.2f}, {payload_body.position.y:.2f}) m",
        "Controls: Q/E rotate, A/D extend/retract, W/S winch, Esc quit"
    ]
    for i, l in enumerate(lines):
        screen.blit(font.render(l, True, (10, 10, 10)), (20, 20 + 22*i))

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
sys.exit()
