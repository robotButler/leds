import xled
from xled_plus import highcontrol
import random
from io import BytesIO
from Box2D import b2World, b2PolygonShape, b2_staticBody, b2_dynamicBody, b2CircleShape, b2WeldJointDef, b2RevoluteJointDef, b2RayCastCallback
import pygame
from pygame.locals import (QUIT, KEYDOWN, K_ESCAPE)
import argparse

# pygame
PPM = 20.0  # pixels per meter
TARGET_FPS = 60
TIME_STEP = 1.0 / TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT = 800, 1200


water_color = (0, 0, 50)
white_color = (100, 100, 100)
wood_color = (100, 100, 0)
empty_color = (0, 0, 0)
orange_color = (150, 50, 50)
# Define the size of the squares
square_size = 1.0  # 1 meter by 1 meter
source_points = []

# --- pygame setup ---
screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock = pygame.time.Clock()

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
    world.CreateJoint(joint1_def)
    world.CreateJoint(joint2_def)
    wood_bodies = [center]
    wood_bodies.append(axel1)
    wood_bodies.append(axel2)
    center.userData = {'color': (0, 0, 0)}
    axel1.userData = {'color': wood_color}
    axel2.userData = {'color': wood_color}

    valve_hinge_pos = (28, 12)
    for body in world.bodies:
        if body.type == b2_staticBody and body.position[0] == valve_hinge_pos[0] and body.position[1] <= valve_hinge_pos[1]:
            world.DestroyBody(body)

    # create valve
    # valve_hinge = world.CreateBody(position=valve_hinge_pos, type=b2_staticBody, userData = {'color': orange_color})
    valve_hinge = world.CreateBody(position=valve_hinge_pos, type=b2_staticBody, userData = {'color': orange_color})
    valve_hinge_shape = b2PolygonShape(box=(0.5, 0.5))
    valve_hinge.CreateFixture(shape=valve_hinge_shape)
    valve_flange = world.CreateBody(position=(valve_hinge_pos[0], valve_hinge_pos[1] - 4), type=b2_dynamicBody, userData = {'color': wood_color})
    valve_shape = b2PolygonShape(box=(0.5, 4))
    valve_flange.CreateFixture(shape=valve_shape, density=10.0, friction=10.0, restitution=0.1)
    flange_top = (valve_flange.localCenter[0] - 0.5, valve_flange.localCenter[1] + 4)
    hinge_top = (valve_hinge.localCenter[0] - 0.5, valve_hinge.localCenter[1] + 0.5)
    valve_joint = b2RevoluteJointDef(bodyA=valve_flange, bodyB=valve_hinge, localAnchorA=flange_top, localAnchorB=hinge_top)
    valve_joint.collideConnected = False
    valve_joint.enableMotor = True
    valve_joint.motorSpeed = -1.0
    valve_joint.maxMotorTorque = 1000000
    world.CreateJoint(valve_joint)

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

    create_dynamics(world, points)

    num_leds = control.get_device_info()['number_of_led']
    print(num_leds)
    velocity_iterations = 10
    position_iterations = 10
    simulation_duration = 30  # in seconds
    movie_duration = 10  # in seconds
    sink_open_bodies = 400
    sink_close_bodies = 100
    speed = 0.1  # Time in seconds between frames
    drops_per_second = 50
    num_drops_created = 0
    num_drops = 0
    high_control.turn_off()
    high_control.turn_on()
    high_control.set_mode('movie')
    high_control.clear_movies()
    led_lookup = {}
    for i, point in enumerate(points):
        led_lookup[point] = i
    frames = bytearray()
    sink_open = False
    running = True
    lowest_y = 100
    for body in world.bodies:
        if body.type == b2_staticBody and body.position[1] < lowest_y:
                lowest_y = body.position[1]
    # if num_drops > sink_open_bodies:
    for body in world.bodies:
        if body.position[1] <= lowest_y:
            world.DestroyBody(body)
            if body.type == b2_dynamicBody:
                num_drops -= 1
    for i in range(int(simulation_duration / TIME_STEP)):
        if not running:
            break

        for event in pygame.event.get():
            if event.type == QUIT or (event.type == KEYDOWN and event.key == K_ESCAPE):
                # The user closed the window or pressed escape
                running = False

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
        # if we haven't produced enough drops, produce one
        expected_drops = drops_per_second * i * TIME_STEP
        if num_drops_created < expected_drops and not sink_open:
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
        # if num_drops > sink_open_bodies and not sink_open:
        #     print('opening sink')
        #     sink_open = True
        #     # destroy the sink_border_bodies
        # lowest_y = 100
        # for body in world.bodies:
        #     if body.type == b2_staticBody and body.position[1] < lowest_y:
        #             lowest_y = body.position[1]
        # if num_drops > sink_open_bodies:
        for body in world.bodies:
            if body.position[1] <= lowest_y:
                # world.DestroyBody(body)
                if body.type == b2_dynamicBody:
                    world.DestroyBody(body)
                    num_drops -= 1
            # print(num_drops)
                
        # if num_drops < sink_close_bodies and sink_open:
        #     print('closing sink')
        #     sink_open = False
        #     for x in range(60):
        #         # Define and attach the shape
        #         body = world.CreateBody(position=(x, lowest_y), type=b2_staticBody)
        #         body.userData = {'color': white_color}
        #         shape = b2PolygonShape(box=(square_size/2, square_size/2))
        #         body.CreateFixture(shape=shape, density=0.0)
        # ray cast each coordinate in led_lookup to find objects
        # empty_count = 0
        # object_count = 0
        # for idx, coord in enumerate(led_lookup):
        #     # ray cast down
        #     coord_dst = (coord[0] + 1, coord[1] + 1)
        #     # print('origin' + str(coord))
        #     # print('dst ' + str(coord_dst))
        #     ri = RaycastInterceptor()
        #     world.RayCast(ri, coord, coord_dst)
        #     if ri.fixture != None:
        #         # print(ri.fixture)
        #         object_count += 1
        #         if ri.fixture.body in wood_bodies:
        #             led_indexed[led_lookup[coord]] = (255, 255, 0)
        #         elif ri.fixture.body.type == b2_dynamicBody:
        #             # print('water')
        #             # print(coord)
        #             led_indexed[led_lookup[coord]] = water_color
        #     else:
        #         empty_count += 1
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
            if body.type == b2_dynamicBody:
                if body.position[1] < sink_y:
                    world.DestroyBody(body)
                    # print('destroying body')
                    continue
                # for fixture in body.fixtures:
                #     if fixture.shape.type == 2:
                #         x0, y0 = fixture.shape.centroid
                #     else:
                #         x0, y0 = fixture.shape.pos
                #     (bodyx, bodyy) = body.position
                #     x = round(x0 + bodyx)
                #     y = round(y0 + bodyy)
                #     if (x, y) in led_lookup:
                #         num_neighbors = 0
                #         neighbors = [(i, j) for i in range(x-1, x+2) for j in range(y-1, y+2)]
                #         for n in neighbors:
                #             if n in led_lookup and led_lookup[n] in led_indexed and led_indexed[led_lookup[n]] != empty_color:
                #                 num_neighbors += 1
                #         # interpolate between white and water_color based on number of neighbors
                #         # interpolation = num_neighbors / 9.0
                #         # color = tuple(int(white_color[i] - ((interpolation ** 0.25) * (white_color[i] - water_color[i]))) for i in range(3))

                #         # sample 4 points around the center of the block
                #         block_center = (int(x * PPM), int(SCREEN_HEIGHT - (y * PPM)))
                #         colors = []
                #         offset = int(square_size / 4 * PPM)
                #         sample_ul = (block_center[0] - offset, block_center[1] - offset)
                #         sample_ur = (block_center[0] + offset, block_center[1] - offset)
                #         sample_ll = (block_center[0] - offset, block_center[1] + offset)
                #         sample_lr = (block_center[0] + offset, block_center[1] + offset)
                #         samples = [sample_ul, sample_ur, sample_ll, sample_lr]
                #         for sample in samples:
                #             c = screen.get_at(sample)
                #             color = (c.r, c.g, c.b)
                #             colors.append(color)
                #         # color = screen.get_at((int(x * PPM), int(SCREEN_HEIGHT - (y * PPM))))
                #         # # pick any non-black color. If multiple, pick the most common
                #         # color_counts = {}
                #         # for color in colors:
                #         #     c = (color.r, color.g, color.b)
                #         #     if c != empty_color:
                #         #         if c not in color_counts:
                #         #             color_counts[c] = 0
                #         #         color_counts[c] += 1
                #         # if len(color_counts) == 0:
                #         #     color = empty_color
                #         # else:
                #         #     color = max(color_counts, key=color_counts.get)

                #         # average all 4 samples
                #         color = tuple(int(sum(x) / len(x)) for x in zip(*colors))

                #         led_indexed[led_lookup[(x, y)]] = color
        
        for i in range(num_leds):
            color = led_indexed[i]
            frame.extend(color)
        frames.extend(frame)
    print('made movie')
    # control.set_mode('rt')
    # control.set_movies_new('asg', int(uuid.uuid1()), 'rgb_raw', num_leds, len(frames), 30)
    high_control.show_movie(BytesIO(frames), fps=30)
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
