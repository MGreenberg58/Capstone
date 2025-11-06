# crane.py
import pymunk
import math

class Crane:
    def __init__(self, space, base_pos=(0.0, 0.0), boom_sections=(40.0, 25.0, 15.0), boom_masses=(6000, 2500, 500), payload_mass=500.0, payload_radius=1.0, hoist_length=15.0):
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

        colors = [(237, 86, 86, 255), (86, 237, 159, 255), (86, 176, 237, 255)]

        for i, length in enumerate(self.boom_sections):
            offset = 0.5
            mass = self.boom_masses[i]
            moment = pymunk.moment_for_segment(mass, (-length/2,0), (length/2,0), self.boom_thickness)
            body = pymunk.Body(mass, moment)
            body.position = pymunk.Vec2d(total_x + length/2 - offset*i, self.base_pos.y)
            shape = pymunk.Segment(body, (-length/2,0), (length/2,0), self.boom_thickness)
            shape.color = colors[i]
            shape.filter = pymunk.ShapeFilter(group=1)  # prevent collisions between boom sections
            self.space.add(body, shape)

            if i == 0:
                joint = pymunk.PinJoint(body, pivot, (-length/2,0), (0,0))
                self.space.add(joint)
            else:
                rot_limit = pymunk.RotaryLimitJoint(prev_body, body, -0.005, 0.005)
                self.space.add(rot_limit)
                self.boom_joints.append(rot_limit)

                groove = pymunk.GrooveJoint(prev_body, body,
                                            (-prev_length/2,0), (prev_length/2,0),
                                            (-length/2,0))
                self.space.add(groove)
                self.boom_joints.append(groove)

                initial_rest = offset
                spring = pymunk.DampedSpring(prev_body, body,
                                            (prev_length/2,0),
                                            (-length/2,0),
                                            rest_length=initial_rest,
                                            stiffness=100000,
                                            damping=10000)
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

        tip_body = self.boom_tip_body
        tip_length = self.boom_sections[-1]

        tip_anchor = (tip_length / 2, 0)
        tip_world = tip_body.local_to_world(tip_anchor)
        self.payload_body.position = tip_world + pymunk.Vec2d(0, -self.hoist_length)
        self.payload_shape = pymunk.Circle(self.payload_body, self.payload_radius)
        self.payload_shape.filter = pymunk.ShapeFilter(group=1)
        self.payload_shape.color = (200, 40, 40, 255)
        self.space.add(self.payload_body, self.payload_shape)

        pin = pymunk.PinJoint(tip_body, self.payload_body, tip_anchor, (0, 0))
        slide = pymunk.SlideJoint(tip_body, self.payload_body, tip_anchor, (0, 0), self.hoist_length, self.hoist_length)
        # spring = pymunk.DampedSpring(tip_body, self.payload_body, tip_anchor, (0, 0), rest_length=self.hoist_length, stiffness=2000, damping=200)

        self.space.add(pin, slide) 
        self.payload_rope = (pin, slide)

    def telescope(self, direction):
        for i, spring in enumerate(self.boom_springs):
            new_rest = max(self.boom_sections[i+1]/10, spring.rest_length + direction)
            new_rest = min(new_rest, self.boom_sections[i+1] * 9/10)
            spring.rest_length = new_rest

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

        base_pos = self.base_pos
        tip_pos = self.boom_tip_body.position
        return (tip_pos - base_pos).length

    def boom_tip_world(self):
        return self.boom_tip_body.position
    
    def set_payload_mass(self, new_mass):
        self.payload_mass = new_mass
        moment = pymunk.moment_for_circle(new_mass, 0, self.payload_radius)
        self.payload_body.mass = new_mass
        self.payload_body.moment = moment

    def set_boom_mass(self, new_mass):
        curr = sum(self.boom_masses)
        ratio = new_mass / curr
        for i in range(len(self.boom_bodies)):
            new = self.boom_masses[i] * ratio
            self.boom_masses[i] = new
            moment = pymunk.moment_for_segment(new, (-self.boom_sections[i]/2,0), (self.boom_sections[i]/2,0), self.boom_thickness)
            self.boom_bodies[i].mass = new_mass
            self.boom_bodies[i].moment = moment


