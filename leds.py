import xled
from xled_plus import highcontrol
import random
from io import BytesIO
from Box2D import b2World, b2PolygonShape, b2_staticBody, b2_dynamicBody, b2CircleShape, b2WeldJointDef, b2RevoluteJointDef, b2RayCastCallback
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import time
import math

# pygame
PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 1200


water_color = (0, 0, 50)
white_color = (100, 100, 100)
wood_color = (170, 100, 0)
empty_color = (0, 0, 0)
orange_color = (150, 50, 50)
# Define the size of the squares
square_size = 1.0  # 1 meter by 1 meter
source_points = []

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()
quarter = math.pi / 2.0

def my_draw_polygon(polygon, body, fixture):
    vertices = [(body.transform * v) * PPM for v in polygon.vertices]
    vertices = [(v[0], SCREEN_HEIGHT - v[1]) for v in vertices]
    pygame.draw.polygon(screen, body.userData['color'], vertices)

def my_draw_circle(circle, body, fixture):
    position = body.transform * circle.pos * PPM
    position = (position[0], SCREEN_HEIGHT - position[1])
    pygame.draw.circle(screen, body.userData['color'], [int(
        x) for x in position], int(circle.radius * PPM))

def create_dynamics(world, points):
    border_points = []
    # Find all the squares around the border of the points
    for x in range(60):
        for y in range(60):
            if (x, y) in points:
                continue
            if (x + 1, y) in points or (x, y + 1) in points or (x + 1, y + 1) in points or (x - 1, y) in points or (x, y - 1) in points or (x - 1, y - 1) in points or (x + 1, y - 1) in points or (x - 1, y + 1) in points:
                border_points.append((x, y))

    started = False
    source_x = 0
    lowest_y = 100
    for y in reversed(range(60)):
        found = False
        for bp in border_points:
            if bp[1] == y:
                if source_x == 0 and (bp[0] + 1, y) in points:
                    source_x = bp[0] + 1
                    started = True
                    found = True
                    if y < lowest_y:
                        lowest_y = y
                    source_points.append((source_x, y))
                    break
                else:
                    if source_x == bp[0] + 1 and (source_x, y) in points:
                        if y < lowest_y:
                            lowest_y = y
                        found = True
                        source_points.append((source_x, y))
                        break
        if started and not found:
            break

    # remove lowest source point - it will be blocked by a ramp
    del source_points[-1]

    # # remove the lowest border points
    # for bp in border_points:
    #     if bp[1] == lowest_y:

    positions = border_points

    # Create static squares
    for pos in positions:
        # Define the body
        body = world.CreateBody(position=pos, type=b2_staticBody)
        body.userData = {'color': white_color}
        # Define and attach the shape
        shape = b2PolygonShape(box=(square_size/2, square_size/2))
        body.CreateFixture(shape=shape, density=0.0)

    # create waterwheel
    wheel_pos = (17, 33)
    axel1 = world.CreateBody(position=wheel_pos, type=b2_dynamicBody)
    axel2 = world.CreateBody(position=wheel_pos, type=b2_dynamicBody)
    shape1 = b2PolygonShape(box=(5, 0.5))
    axel1.CreateFixture(shape=shape1, density=10.0, friction=10.0, restitution=0.1)
    shape2 = b2PolygonShape(box=(0.5, 5))
    axel2.CreateFixture(shape=shape2, density=10.0, friction=10.0, restitution=0.1)
    center = world.CreateBody(position=wheel_pos, type=b2_staticBody)
    # create a freely rotating joint between the wheel and the center
    joint1_def = b2RevoluteJointDef(bodyA=axel1, bodyB=center, localAnchorA=axel1.localCenter, localAnchorB=center.localCenter)
    joint2_def = b2WeldJointDef(bodyA=axel2, bodyB=axel1, localAnchorA=axel2.localCenter, localAnchorB=axel1.localCenter)
    joint1_def.collideConnected = False
    joint2_def.collideConnected = False
    # joint_def.enableMotor = True
    # joint_def.maxMotorTorque = 1000
    # joint_def.motorSpeed = 10.0
    wheel_joint = world.CreateJoint(joint1_def)
    world.CreateJoint(joint2_def)
    wood_bodies = [center]
    wood_bodies.append(axel1)
    wood_bodies.append(axel2)
    center.userData = {'color': empty_color}
    axel1.userData = {'color': wood_color}
    axel2.userData = {'color': wood_color}

    valve_hinge_pos = (29, 12)
    valve_length = 3.9
    for body in world.bodies:
        if body.type == b2_staticBody and body.position[0] == valve_hinge_pos[0] and body.position[1] <= valve_hinge_pos[1]:
            world.DestroyBody(body)

    # create ramp
    ramp = world.CreateBody(position=(4.5, 12.5), type=b2_staticBody, userData={'color': wood_color})
    ramp_shape = b2PolygonShape(vertices=[(0,0), (0,5), (20,0)])
    ramp.CreateFixture(shape=ramp_shape)
    # upper ramp
    ramp2 = world.CreateBody(position=(8.5, 44.5), type=b2_staticBody, userData={'color': wood_color})
    ramp_shape2 = b2PolygonShape(vertices=[(0,0), (0,1), (4,0)])
    ramp2.CreateFixture(shape=ramp_shape2)
    # right ramp
    ramp3 = world.CreateBody(position=(32.5, 28.5), type=b2_staticBody, userData={'color': wood_color})
    ramp_shape3 = b2PolygonShape(vertices=[(0,0), (0,1), (-4,0)])
    ramp3.CreateFixture(shape=ramp_shape3)

    # create valve
    # valve_hinge = world.CreateBody(position=valve_hinge_pos, type=b2_staticBody, userData = {'color': orange_color})
    valve_hinge = world.CreateBody(position=valve_hinge_pos, type=b2_staticBody, userData = {'color': orange_color})
    valve_flange = world.CreateBody(position=(valve_hinge_pos[0], valve_hinge_pos[1] - valve_length), type=b2_dynamicBody, userData = {'color': wood_color})
    valve_shape = b2PolygonShape(box=(0.5, valve_length))
    valve_flange.CreateFixture(shape=valve_shape, density=10.0, friction=10.0, restitution=0.1)
    flange_top = (valve_flange.localCenter[0] - 0.5, valve_flange.localCenter[1] + valve_length)
    hinge_top = (valve_hinge.localCenter[0] - 0.5, valve_hinge.localCenter[1] + 0.5)
    valve_joint_def = b2RevoluteJointDef(bodyA=valve_flange, bodyB=valve_hinge, localAnchorA=flange_top, localAnchorB=hinge_top)
    valve_joint_def.collideConnected = False
    valve_joint_def.enableMotor = True
    valve_joint_def.motorSpeed = 0.0
    valve_joint_def.maxMotorTorque = 100000000
    valve_joint_def.enableLimit = True
    valve_joint_def.lowerAngle = -math.pi
    valve_joint_def.upperAngle = 0
    valve_joint = world.CreateJoint(valve_joint_def)
    return {'wheel': wheel_joint, 'valve': valve_joint}

def main():
    # parser = argparse.ArgumentParser()
    # parser.add_argument("-l", "--local", help = "local display only, no xled")
    # args = parser.parse_args()
    # # if called with --local arg, don't run any xled functions
    # if not args.local:
    
    device = xled.discover.discover()

    # Connect to the LED device
    control = xled.ControlInterface(device.ip_address, device.hw_address)
    high_control = highcontrol.HighControlInterface(device.ip_address)
    print(device.ip_address)
    layout = control.get_led_layout()

    b2PolygonShape.draw = my_draw_polygon

    b2CircleShape.draw = my_draw_circle

    # Provided data
    data = layout.data["coordinates"]

    xs = [point['x'] for point in data]

    ys = [point['y'] for point in data]

    xs.sort()
    ys.sort()
    xmap = {}
    ymap = {}
    for x in xs:
        if x not in xmap:
            xmap[x] = len(xmap)
    for y in ys:
        if y not in ymap:
            ymap[y] = len(ymap)

    x_coords = [xmap[point['x']] for point in data]
    y_coords = [ymap[point['y']] for point in data]

    sink_y = min(y_coords)

    points = [(x_coords[i], y_coords[i]) for i, x in enumerate(x_coords)]
    # Initialize the Box2D world
    world = b2World(gravity=(0, -10), doSleep=True)

    # move all points up and to the right
    og_points = points
    for i, point in enumerate(og_points):
        points[i] = (point[0] + 5, point[1] + 5)

    joints = create_dynamics(world, points)

    num_leds = control.get_device_info()['number_of_led']
    print(num_leds)
    velocity_iterations = 10
    position_iterations = 10
    simulation_duration = 10  # in seconds
    valve_active_duration = 5
    valve_disabled_duration = 2
    movie_duration = 10  # in seconds
    sink_open_bodies = 400
    sink_close_bodies = 100
    speed = 0.1  # Time in seconds between frames
    drops_per_second = 20
    num_drops_created = 0
    num_drops = 0
    high_control.turn_off()
    high_control.turn_on()
    # high_control.set_mode('movie')
    high_control.clear_movies()
    res = high_control.get_movies()
    capacity = res["available_frames"] - 1
    print(capacity)
    led_lookup = {}
    for i, point in enumerate(points):
        led_lookup[point] = i
    frames = bytearray()
    frame_list = []
    lowest_y = 100
    wheel_spin = 0
    is_flowing = True
    for body in world.bodies:
        if body.type == b2_staticBody and body.position[1] < lowest_y:
                lowest_y = body.position[1]
    # kill the bottom layer of border
    for body in world.bodies:
        if body.position[1] <= lowest_y:
            world.DestroyBody(body)
            if body.type == b2_dynamicBody:
                num_drops -= 1

    cycles_complete = 0
    max_cycles = 3
    state = 'cycling'
    frame_idx = 0
    movie_ids = []
    movies = []
    durations = []
    # for i in range(int(simulation_duration / TIME_STEP)):
    while state != 'finished' and len(movies) < 8:
        # pygame requires us to listen for events, or it won't simulate
        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                # The user closed the window or pressed escape
                state = 'finished'

        screen.fill((0, 0, 0, 0))
        # Draw the world
        for body in world.bodies:
            for fixture in body.fixtures:
                fixture.shape.draw(body, fixture)

        world.Step(TIME_STEP, velocity_iterations, position_iterations)
        world.ClearForces()

        # Flip the screen and try to keep at the target FPS
        pygame.display.flip()
        clock.tick(TARGET_FPS)
        frame = bytearray()
        # time_elapsed = i * TIME_STEP
        valve_joint = joints['valve']
        wheel_joint = joints['wheel']
        wheel_spin += wheel_joint.speed * TIME_STEP
        if valve_joint.motorEnabled:
            valve_joint.motorSpeed = wheel_joint.speed / -5.0
            if wheel_spin > 27 or state != 'cycling':
                print('valve disabled')
                valve_joint.motorEnabled = False
                wheel_spin = 0
        else:
            if wheel_spin > 17 and state == 'cycling':
                print('valve enabled')
                cycles_complete += 1
                valve_joint.motorEnabled = True
                valve_joint.motorSpeed = 0
                wheel_spin = 0

        # if we haven't produced enough drops, produce one
        expected_drops = drops_per_second * frame_idx * TIME_STEP
        if num_drops_created < expected_drops and is_flowing:
            missing_drops = expected_drops - num_drops_created
            for _ in range(int(missing_drops)):
                # pick a random source_point
                s = random.choice(source_points)
                # Define the body
                body = world.CreateBody(position=s, type=b2_dynamicBody)
                body.userData = {'color': water_color}
                # Define and attach the shape
                # create a circle
                shape = b2CircleShape(radius=square_size/2)
                body.CreateFixture(shape=shape, density=100.0, friction=0.0, restitution=0.1)
                # give it a random positive x velocity and a small y velocity
                body.linearVelocity = (random.random() * 10, random.random())
                num_drops_created += 1
                num_drops += 1
        led_indexed = {k: empty_color for k in range(num_leds)}

        for body in world.bodies:
            if body.position[1] <= lowest_y:
                # world.DestroyBody(body)
                if body.type == b2_dynamicBody:
                    world.DestroyBody(body)
                    num_drops -= 1

        if cycles_complete == max_cycles and state == 'cycling':
            print('resetting')
            state = 'resetting'
            wheel_joint.enableLimit = True
            wheel_joint.upperAngle = (math.floor(wheel_joint.angle / quarter) + 1) * quarter
            wheel_joint.lowerAngle = 0
            is_flowing = False
        
        if state == 'resetting':
            print(wheel_joint.angle % quarter)
            if wheel_joint.angle % quarter < 0.05:
                print('draining')
                state = 'draining'
                wheel_joint.upperAngle = wheel_joint.angle + 0.1
                wheel_joint.lowerAngle = wheel_joint.angle

        if state == 'draining' and num_drops == 0 and wheel_joint.angle % quarter < 0.1:
            print('finished')
            state = 'finished'

        for (x, y) in points:
            block_center = (int(x * PPM), int(SCREEN_HEIGHT - (y * PPM)))
            colors = []
            offset = int(square_size / 4 * PPM)
            sample_ul = (block_center[0] - offset, block_center[1] - offset)
            sample_ur = (block_center[0] + offset, block_center[1] - offset)
            sample_ll = (block_center[0] - offset, block_center[1] + offset)
            sample_lr = (block_center[0] + offset, block_center[1] + offset)
            samples = [sample_ul, sample_ur, sample_ll, sample_lr]
            for sample in samples:
                c = screen.get_at(sample)
                color = (c.r, c.g, c.b)
                colors.append(color)
            color = tuple(int(sum(x) / len(x)) for x in zip(*colors))

            led_indexed[led_lookup[(x, y)]] = color

        for body in world.bodies:
            if body.type == b2_dynamicBody and body.position[1] < sink_y:
                world.DestroyBody(body)
        
        for i in range(num_leds):
            color = led_indexed[i]
            frame.extend(color)
        frame_list.append(frame)
        # high_control.show_rt_frame(BytesIO(frame))
        frames.extend(frame)
        frame_idx += 1

        # if frame_idx % int(simulation_duration / TIME_STEP) == 0:
        #     print('made movie')
        #     durations.append(simulation_duration)
        #     movie = BytesIO(frames)
        #     numframes = movie.seek(0, 2) // (high_control.led_bytes * high_control.num_leds)
        #     print(numframes)
        #     movie.seek(0)
        #     movies.append(movie)
            # movie_id = high_control.upload_movie(BytesIO(frames), fps=60, name=str(frame_idx), force=True)
            # movie_ids.append(movie_id)
            # high_control.show_movie(BytesIO(frames), fps=60)
            # frames = bytearray()
        # for i in range(int(simulation_duration / TIME_STEP)):
    # control.set_mode('rt')
    # control.set_movies_new('asg', int(uuid.uuid1()), 'rgb_raw', num_leds, len(frames), 30)
    # high_control.show_movie(BytesIO(frames), fps=60)
    frame_len = high_control.led_bytes * high_control.num_leds
    print(frame_idx)
    print(frame_len)
    i = 0
    while True:
        frame = frame_list[i % len(frame_list)]
        i += 1
    # for frame in frame_list:
        high_control.show_rt_frame(BytesIO(frame))
        time.sleep(TIME_STEP)
    # print('made last movie')
    # movies.append(BytesIO(frames))
    # duration = (frame_idx % int(simulation_duration / TIME_STEP)) * TIME_STEP
    # durations.append(duration)
    # movie_idx = 0
    # for movie in movies:
    #     movie_name = 'waterwheel' + str(movie_idx)
    #     duration = durations[movie_idx]
    #     movie_idx += 1
        # movie_id = high_control.upload_movie(movie, fps=30, name=movie_name)
        # print(movie_id)
        # movie_ids.append((movie_id, duration))
    # high_control.show_movie(BytesIO(frames), fps=60)
    # print(movie_ids)
    # worked = high_control.show_playlist(movie_ids)
    # print(worked)

    # control.set_mode('movie')
    # control.set_led_movie_config(0, len(frames), num_leds)
    # control.set_led_movie_full(frames)
    # for frame_file in frames:
    #     control.set_rt_frame_socket(frame_file, version=3, leds_number=num_leds)
    #     # control.set_rt_frame_rest(frame_file)
    #     time.sleep(speed)

class RaycastInterceptor(b2RayCastCallback):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.__type = None
        self.__point = None
        self.fixture = None

    def ReportFixture(self, fixture, point, normal, fraction):
        self.fixture = fixture
        self.__type = fixture.body.userData
        self.__point = point
        # stop at closest point
        return fraction

    @property
    def type(self):
        return self.__type

    @property
    def point(self):
        return self.__point


def createVoxels(world, center_pos, density, w, h):
    body = world.CreateBody(position=center_pos, type=b2_dynamicBody)
    # Divide the width and height into voxel size, create a shape for each voxel and attach it to the body
    for x in range(int(w*2)):
        for y in range(int(h*2)):
            pos = (x - w + center_pos[0], y - h + center_pos[1])
            # Define and attach the shape
            # shape = b2PolygonShape(box=(0.5, 0.5))
            shape = b2PolygonShape()
            shape.SetAsBox(0.5, 0.5)
            # shape.m_p.Set(pos[0], pos[1])
            shape.pos = pos
            # shape.m_p.Set(x - w/2, y - h/2)
            # shape.pos = (x - w/2, y - h/2)
            fixture = body.CreateFixture(shape=shape, density=density)
            # position the fixture relative to the body
            # fixture.shape.pos.set(pos[0], pos[1])
    return body


if __name__ == '__main__':
    main()  
