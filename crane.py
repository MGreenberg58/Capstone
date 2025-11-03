# crane.py
import pymunk
import math

class Crane:
    def __init__(self, space, base_pos=(0.0, 0.0), boom_sections=(10.0, 7.0, 5.0), boom_masses=(25, 10, 5), payload_mass=50.0, payload_radius=1.0, hoist_length=15.0):
        self.space = space
        self.base_pos = pymunk.Vec2d(base_pos[0], base_pos[1])
        self.boom_sections = list(boom_sections)
        self.boom_masses = list(boom_masses)
        self.boom_thickness = 0.6
        self.payload_mass = payload_mass
        self.payload_radius = payload_radius
        self.hoist_length = hoist_length

        self.payload_body = None
        self.payload_shape = None

        self.boom_springs = []

        self._create_telescoping_boom()
        self._create_payload_rope()

    def _create_telescoping_boom(self):
        self.boom_bodies = []
        self.boom_shapes = []
        self.boom_joints = []
        self.boom_springs = []

        pivot = pymunk.Body(body_type=pymunk.Body.STATIC)
        pivot.position = self.base_pos

        prev_body = None
        prev_length = None
        total_x = self.base_pos.x

        for i, length in enumerate(self.boom_sections):
            mass = self.boom_masses[i]
            moment = pymunk.moment_for_segment(mass, (-length/2,0), (length/2,0), self.boom_thickness)
            body = pymunk.Body(mass, moment)
            body.position = pymunk.Vec2d(total_x + length/2, self.base_pos.y)
            shape = pymunk.Segment(body, (-length/2,0), (length/2,0), self.boom_thickness)
            shape.color = (240, 200, 0, 255)
            shape.filter = pymunk.ShapeFilter(group=1)  # prevent collisions between boom sections
            self.space.add(body, shape)

            if i == 0:
                joint = pymunk.PinJoint(body, pivot, (-length/2,0), (0,0))
                self.space.add(joint)
            else:
                # Keep sections aligned
                rot_limit = pymunk.RotaryLimitJoint(prev_body, body, 0, 0)
                self.space.add(rot_limit)
                self.boom_joints.append(rot_limit)

                # Allow sliding along previous section
                groove = pymunk.GrooveJoint(prev_body, body,
                                            (-prev_length/2,0), (prev_length/2,0),
                                            (-length/2,0))
                self.space.add(groove)
                self.boom_joints.append(groove)

                # Add spring for stiffness
                spring = pymunk.DampedSpring(prev_body, body,
                                            (prev_length/2,0),
                                            (-length/2,0),
                                            rest_length=0.0,
                                            stiffness=20000,
                                            damping=800)
                self.space.add(spring)
                self.boom_springs.append(spring)

            self.boom_bodies.append(body)
            self.boom_shapes.append(shape)
            prev_body = body
            prev_length = length
            total_x += length

        self.boom_tip_body = self.boom_bodies[-1]
        self.total_boom_length = sum(self.boom_sections)

    def _create_payload_rope(self):
        moment = pymunk.moment_for_circle(self.payload_mass, 0, self.payload_radius)
        self.payload_body = pymunk.Body(self.payload_mass, moment)
        tip_pos = self.boom_tip_body.position
        self.payload_body.position = tip_pos + (0, -self.hoist_length)
        self.payload_shape = pymunk.Circle(self.payload_body, self.payload_radius)
        self.payload_shape.color = (200, 40, 40, 255)
        self.space.add(self.payload_body, self.payload_shape)

        # Rope-like spring from last boom section to payload
        rope = pymunk.DampedSpring(self.boom_tip_body, self.payload_body,
                                   (self.boom_sections[-1]/2,0),
                                   (0,0),
                                   rest_length=self.hoist_length,
                                   stiffness=100000,
                                   damping=10)
        self.space.add(rope)
        self.boom_springs.append(rope)

    def compute_cg(self):
        bodies = self.boom_bodies + [self.payload_body]
        total_mass = sum(b.mass for b in bodies)
        weighted = sum((b.mass * b.position for b in bodies), start=pymunk.Vec2d(0,0))
        return weighted / total_mass
    
    def gravity_moment(self):
        g = pymunk.Vec2d(0, -9.81)
        pivot = self.base_pos
        total_torque = 0.0
        
        for body in self.boom_bodies:
            r = body.position - pivot  
            F = body.mass * g
            torque = r.cross(F)
            total_torque += torque
        
        if self.payload_body is not None:
            r = self.payload_body.position - pivot
            F = self.payload_body.mass * g
            total_torque += r.cross(F)
        
        return total_torque

    def boom_length(self):
        if not self.boom_bodies:
            return 0.0
        # Distance between base and last boom tip
        base_pos = self.base_pos
        tip_pos = self.boom_tip_body.position
        return (tip_pos - base_pos).length


    def boom_tip_world(self):
        return self.boom_tip_body.position

    # --- Dynamic telescoping methods ---
    def extend_section(self, section_idx, delta_length):
        if section_idx < 0 or section_idx >= len(self.boom_sections):
            return
        # Increase section length
        new_length = self.boom_sections[section_idx] + delta_length
        self.boom_sections[section_idx] = max(1.0, new_length)  # minimum 1 m

        body = self.boom_bodies[section_idx]
        # Update shape
        if self.boom_shapes[section_idx] in self.space.shapes:
            self.space.remove(self.boom_shapes[section_idx])
        new_shape = pymunk.Segment(body, (-new_length/2,0), (new_length/2,0), self.boom_thickness)
        new_shape.color = (240, 200, 0, 255)
        self.space.add(new_shape)
        self.boom_shapes[section_idx] = new_shape

        # Update moment
        body.moment = pymunk.moment_for_segment(self.boom_mass, (-new_length/2,0), (new_length/2,0), self.boom_thickness)

        # Update tip spring anchor if last section
        if section_idx == len(self.boom_sections)-1:
            rope = self.boom_springs[-1]
            rope.anchor_a = (new_length/2, 0)

    def retract_section(self, section_idx, delta_length):
        self.extend_section(section_idx, -delta_length)
