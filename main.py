# crane_sim_with_skfuzzy.py
import pygame
import pymunk
import math
import sys
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from collections import deque

# --- Init PyGame ---
pygame.init()
WIDTH, HEIGHT = 1920, 1080
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Crane — Real-World Physics + scikit-fuzzy FIS")
clock = pygame.time.Clock()
FPS = 120

# --- Conversion factor ---
PIXELS_PER_M = 10.0
BASE_X, BASE_Y = 100, HEIGHT - 100

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
boom_length = 20.0
boom_mass = 50.0
boom_thickness = 0.5

payload_mass = 100.0
payload_radius = 1

hoist_length = 20.0  # meters

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
        stiffness=100000.0,
        damping=10.0
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

def visualizeFis():
    panel_x = 760
    panel_w = WIDTH - panel_x - 20
    panel_y = 20
    panel_h = HEIGHT - 40
    pygame.draw.rect(screen, (230, 230, 230), (panel_x, panel_y, panel_w, panel_h))
    pygame.draw.rect(screen, (200, 200, 200), (panel_x+6, panel_y+6, panel_w-12, panel_h-12))

    # Draw cg
    cg_px = to_pygame(cg_pos)
    pygame.draw.circle(screen, (0, 0, 255), cg_px, 8)   

    font = pygame.font.SysFont("Consolas", 16)
    bigfont = pygame.font.SysFont("Consolas", 18, bold=True)

    # show inputs
    label_x = panel_x + 20
    current_y = panel_y + 20
    screen.blit(bigfont.render("FIS — CraneSafetyFIS (scikit-fuzzy)", True, (10,10,10)), (label_x, current_y))
    current_y += 30
    lines = [
        f"BoomLength (BL): {BL_input:.2f} m",
        f"CGDistance (CGD): {CGD_input:.2f} m",
        f"PayloadHeight (PH): {PH_input:.2f} m"
    ]
    for l in lines:
        screen.blit(font.render(l, True, (10,10,10)), (label_x, current_y))
        current_y += 22

    current_y += 10
    # bars for outputs
    ca_x = label_x
    ca_y = current_y
    screen.blit(font.render("ControlAdjustment:", True, (10,10,10)), (ca_x, ca_y))
    ca_y += 20
    bar_w = panel_w - 60
    bar_h = 18
    pygame.draw.rect(screen, (180,180,180), (ca_x, ca_y, bar_w, bar_h))
    filled = int((control_adj / 3.0) * bar_w)
    pygame.draw.rect(screen, (80, 140, 220), (ca_x, ca_y, filled, bar_h))
    screen.blit(font.render(f"{control_adj:.3f} / 3.0", True, (10,10,10)), (ca_x + bar_w + 6, ca_y))

    ca_y += 36
    screen.blit(font.render("OperatorFeedback:", True, (10,10,10)), (ca_x, ca_y))
    ca_y += 20
    pygame.draw.rect(screen, (180,180,180), (ca_x, ca_y, bar_w, bar_h))
    filled2 = int((feedback_lvl / 3.0) * bar_w)
    pygame.draw.rect(screen, (220, 110, 80), (ca_x, ca_y, filled2, bar_h))
    screen.blit(font.render(f"{feedback_lvl:.3f} / 2.0", True, (10,10,10)), (ca_x + bar_w + 6, ca_y))

    # small swing angle plot at bottom of panel
    plot_h = 160
    plot_w = panel_w - 40
    plot_x = panel_x + 20
    plot_y = panel_y + panel_h - plot_h - 20
    pygame.draw.rect(screen, (245,245,245), (plot_x, plot_y, plot_w, plot_h))
    pygame.draw.rect(screen, (200,200,200), (plot_x, plot_y, plot_w, plot_h), 1)
    screen.blit(font.render("Payload Swing Angle (deg)", True, (10,10,10)), (plot_x + 6, plot_y + 6))

    if len(angle_history) > 1:
        arr = np.array(angle_history)
        arr_deg = arr * 180.0 / math.pi
        y_limit = max(10.0, np.max(np.abs(arr_deg)))
        Np = len(arr_deg)
        xs = np.linspace(plot_x + 10, plot_x + plot_w - 10, Np)
        ys = plot_y + plot_h/2 - (arr_deg / y_limit) * (plot_h/2 - 20)
        points = [(int(xs[i]), int(ys[i])) for i in range(Np)]
        if len(points) > 1:
            pygame.draw.lines(screen, (60, 160, 60), False, points, 2)
        zero_y = int(plot_y + plot_h/2)
        pygame.draw.line(screen, (180,180,180), (plot_x+6, zero_y), (plot_x+plot_w-6, zero_y), 1)
        screen.blit(font.render(f"+{y_limit:.0f}°", True, (10,10,10)), (plot_x + plot_w - 50, plot_y + 8))
        screen.blit(font.render(f"-{y_limit:.0f}°", True, (10,10,10)), (plot_x + plot_w - 50, plot_y + plot_h - 20))

    # overlays (left side)
    font_sm = pygame.font.SysFont("Consolas", 18)
    lines_left = [
        f"boom_angle = {math.degrees(boom_body.angle)%360:.2f} deg",
        f"boom_length = {boom_length:.2f} m",
        f"hoist_length = {spring.rest_length:.2f} m",
        f"payload_pos = ({payload_body.position.x:.2f}, {payload_body.position.y:.2f}) m",
        f"CGDistance (computed) = {CGD_input:.2f} m",
        "Controls: Q/E rotate, A/D extend/retract, W/S winch, Esc quit"
    ]
    for i, l in enumerate(lines_left):
        screen.blit(font_sm.render(l, True, (10, 10, 10)), (20, 20 + 22*i))


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
boom_torque_up = 20000
boom_torque_down = 7000
hoist_speed = 0.03       # m per frame
extend_speed = 0.03      # m per frame
boom_min_len = 3.0
boom_max_len = 100.0

# PD hold vars
boom_target_angle = boom_body.angle
was_rotating = False
k_p = 750000.0
k_d = 200000.0

# --- FIS using scikit-fuzzy ---
u_bl = np.arange(0, 120, 1)
u_cgd = np.arange(0, 40, 0.1)
u_ph = np.arange(-20, 40, 0.5)

BoomLength = ctrl.Antecedent(u_bl, 'BoomLength')
CGDistance = ctrl.Antecedent(u_cgd, 'CGDistance')
PayloadHeight = ctrl.Antecedent(u_ph, 'PayloadHeight')

u_ctrl = np.arange(0, 3.01, 0.01)   # ControlAdjustment 0..3
u_feed = np.arange(0, 2.01, 0.01)   # OperatorFeedback 0..2
ControlAdjustment = ctrl.Consequent(u_ctrl, 'ControlAdjustment')
OperatorFeedback = ctrl.Consequent(u_feed, 'OperatorFeedback')

# Membership functions (trapmf/trimf) matching MATLAB spec
BoomLength['Short']  = fuzz.trapmf(BoomLength.universe, [0, 5, 15, 20])
BoomLength['Medium'] = fuzz.trapmf(BoomLength.universe, [15, 20, 30, 50])
BoomLength['Long']   = fuzz.trapmf(BoomLength.universe, [40, 60, 100, 100])

CGDistance['Close']  = fuzz.trapmf(CGDistance.universe, [0, 2, 7, 10])
CGDistance['Medium'] = fuzz.trapmf(CGDistance.universe, [5, 9, 15, 20])
CGDistance['Far']    = fuzz.trapmf(CGDistance.universe, [15, 20, 35, 40])

PayloadHeight['Low']    = fuzz.trapmf(PayloadHeight.universe, [0, 0, 0, 5])
PayloadHeight['Medium'] = fuzz.trapmf(PayloadHeight.universe, [4, 10, 15, 20])
PayloadHeight['High']   = fuzz.trapmf(PayloadHeight.universe, [12, 20, 25, 30])

ControlAdjustment['NoCorrection']   = fuzz.trimf(ControlAdjustment.universe, [0.0, 0.0, 1.0])
ControlAdjustment['SmallCorrection']= fuzz.trimf(ControlAdjustment.universe, [0.5, 1.0, 1.5])
ControlAdjustment['StrongCorrection']=fuzz.trimf(ControlAdjustment.universe, [1.0, 2.0, 2.5])
ControlAdjustment['OverrideStop']   = fuzz.trimf(ControlAdjustment.universe, [2.0, 3.0, 3.0])

OperatorFeedback['Safe']    = fuzz.trimf(OperatorFeedback.universe, [0.0, 0.0, 1.0])
OperatorFeedback['Caution'] = fuzz.trimf(OperatorFeedback.universe, [0.5, 1.0, 1.5])
OperatorFeedback['Unsafe']  = fuzz.trimf(OperatorFeedback.universe, [1.0, 2.0, 2.5])
OperatorFeedback['Danger']  = fuzz.trimf(OperatorFeedback.universe, [2.0, 3.0, 3.0])

# Rules
rules = []
rules.append(ctrl.Rule(BoomLength['Long'] & CGDistance['Far'],
                      (ControlAdjustment['OverrideStop'], OperatorFeedback['Danger'])))
rules.append(ctrl.Rule(CGDistance['Close'],
                      (ControlAdjustment['NoCorrection'], OperatorFeedback['Safe'])))

rules.append(ctrl.Rule(BoomLength['Short'] & CGDistance['Medium'] & PayloadHeight['Low'],
                      (ControlAdjustment['NoCorrection'], OperatorFeedback['Safe'])))
rules.append(ctrl.Rule(BoomLength['Medium'] & CGDistance['Medium'] & PayloadHeight['Low'],
                      (ControlAdjustment['SmallCorrection'], OperatorFeedback['Caution'])))
rules.append(ctrl.Rule(BoomLength['Long'] & CGDistance['Medium'] & PayloadHeight['Low'],
                      (ControlAdjustment['SmallCorrection'], OperatorFeedback['Caution'])))
rules.append(ctrl.Rule(BoomLength['Short'] & CGDistance['Medium'] & PayloadHeight['Medium'],
                      (ControlAdjustment['NoCorrection'], OperatorFeedback['Safe'])))
rules.append(ctrl.Rule(BoomLength['Medium'] & CGDistance['Medium'] & PayloadHeight['Medium'],
                      (ControlAdjustment['SmallCorrection'], OperatorFeedback['Caution'])))
rules.append(ctrl.Rule(BoomLength['Long'] & CGDistance['Medium'] & PayloadHeight['Medium'],
                      (ControlAdjustment['SmallCorrection'], OperatorFeedback['Caution'])))
rules.append(ctrl.Rule(BoomLength['Short'] & CGDistance['Medium'] & PayloadHeight['High'],
                      (ControlAdjustment['NoCorrection'], OperatorFeedback['Safe'])))
rules.append(ctrl.Rule(BoomLength['Medium'] & CGDistance['Medium'] & PayloadHeight['High'],
                      (ControlAdjustment['SmallCorrection'], OperatorFeedback['Caution'])))
rules.append(ctrl.Rule(BoomLength['Long'] & CGDistance['Medium'] & PayloadHeight['High'],
                      (ControlAdjustment['StrongCorrection'], OperatorFeedback['Unsafe'])))

rules.append(ctrl.Rule(BoomLength['Short'] & CGDistance['Far'] & PayloadHeight['Low'],
                      (ControlAdjustment['SmallCorrection'], OperatorFeedback['Caution'])))
rules.append(ctrl.Rule(BoomLength['Medium'] & CGDistance['Far'] & PayloadHeight['Low'],
                      (ControlAdjustment['StrongCorrection'], OperatorFeedback['Unsafe'])))
rules.append(ctrl.Rule(BoomLength['Short'] & CGDistance['Far'] & PayloadHeight['Medium'],
                      (ControlAdjustment['SmallCorrection'], OperatorFeedback['Caution'])))
rules.append(ctrl.Rule(BoomLength['Medium'] & CGDistance['Far'] & PayloadHeight['Medium'],
                      (ControlAdjustment['StrongCorrection'], OperatorFeedback['Unsafe'])))
rules.append(ctrl.Rule(BoomLength['Short'] & CGDistance['Far'] & PayloadHeight['High'],
                      (ControlAdjustment['StrongCorrection'], OperatorFeedback['Unsafe'])))
rules.append(ctrl.Rule(BoomLength['Medium'] & CGDistance['Far'] & PayloadHeight['High'],
                      (ControlAdjustment['StrongCorrection'], OperatorFeedback['Danger'])))


crane_ctrl = ctrl.ControlSystem(rules)
crane_sim = ctrl.ControlSystemSimulation(crane_ctrl, flush_after_run=30)  # flush periodically for speed

# --- Logging & visualization buffers ---
angle_history = deque(maxlen=800)  
time_history = deque(maxlen=800)
sim_time = 0.0

def compute_cg():
    """Compute the center of gravity of the crane system (base, boom, payload)."""
    bodies = [base_body, boom_body, payload_body]
    valid_bodies = [b for b in bodies if b.mass > 0 and not np.isinf(b.mass)]

    if not valid_bodies:
        return base_body.position

    total_mass = sum(b.mass for b in valid_bodies)
    weighted_sum = sum((b.mass * b.position for b in valid_bodies), start=pymunk.Vec2d(0, 0))
    cg = weighted_sum / total_mass
    return cg


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

    # ----- Controls -----
    grav = (boom_mass * 9.81 * (boom_length/2) + payload_mass * 9.81 * boom_length) * math.cos(boom_body.angle)
    rotating = False
    if keys[pygame.K_q]:
        boom_body.torque += boom_torque_up * math.cos(boom_body.angle) + grav + 1000
        rotating = True
    if keys[pygame.K_e]:
        boom_body.torque -= boom_torque_down * math.cos(boom_body.angle) - grav + 1000
        rotating = True

    # hoist
    if keys[pygame.K_w]:
        hoist_length = max(0.5, hoist_length - hoist_speed)
        if spring is not None:
            spring.rest_length = hoist_length
    if keys[pygame.K_s]:
        hoist_length = min(50.0, hoist_length + hoist_speed)
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
        error = boom_target_angle - boom_body.angle
        boom_body.torque += k_p * error - k_d * boom_body.angular_velocity + grav

    was_rotating = rotating

    # --- Evaluate FIS each frame ---
    cg_pos = compute_cg()
    CGD_input = float(abs(cg_pos.x - base_body.position.x))
    BL_input = float(boom_length) 
    PH_input = float((math.sin(boom_body.angle) * (boom_length)) - hoist_length)

    # set inputs (clip to universes to avoid skfuzzy warnings)
    crane_sim.input['BoomLength'] = np.clip(BL_input, u_bl[0], u_bl[-1])
    crane_sim.input['CGDistance'] = np.clip(CGD_input, u_cgd[0], u_cgd[-1])
    crane_sim.input['PayloadHeight'] = np.clip(PH_input, u_ph[0], u_ph[-1])
    try:
        crane_sim.compute()
        control_adj = float(crane_sim.output['ControlAdjustment'])
        feedback_lvl = float(crane_sim.output['OperatorFeedback'])
    except Exception:
        # if compute fails for any reason, fallback to safe defaults
        control_adj = 0.0
        feedback_lvl = 0.0

    # step physics
    space.step(dt)

    # compute swing angle for visualization
    boom_tip_world = boom_body.position + pymunk.Vec2d(boom_length/2.0, 0.0).rotated(boom_body.angle)
    dx = payload_body.position.x - boom_tip_world.x
    dy = payload_body.position.y - boom_tip_world.y
    theta = math.atan2(dx, -dy) if (abs(dx) > 1e-9 or abs(dy) > 1e-9) else 0.0
    angle_history.append(theta)
    time_history.append(sim_time)

    # --- Rendering ---
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

    visualizeFis()

    pygame.display.flip()
    clock.tick(FPS)

pygame.quit()
sys.exit()
