from controller import Supervisor
import math
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

robot = Supervisor()
TIME_STEP = int(robot.getBasicTimeStep())

WHEEL_RADIUS = 0.033   # this is the turtlebot parameter as per webot documentation
WHEEL_SEPARATION = 0.160  # distance between wheels as per documentation

# This can be changed, it is the experiment parameters which I set for now
STRAIGHT_TIME = 5.0     # i am moving the robot for 5 seconds on each side of square
LINEAR_SPEED  = 0.20    # forward speed of the robot

# Wheel speed for turning in place
TURN_WHEEL_SPEED = 3.0  # this is the radian per second value

# we calculate robot angular speed for the chosen TURN_WHEEL_SPEED
# When we set left = +w, right = -w:
#   omega_robot = R * (omega_r - omega_l) / L = R * (-w - w) / L = -2 R w / L
omega_robot_turn = (WHEEL_RADIUS * (TURN_WHEEL_SPEED - (-TURN_WHEEL_SPEED))
                    / WHEEL_SEPARATION)
TURN_TIME = (math.pi / 2.0) / abs(omega_robot_turn)  # time to rotate 90 deg

# Total experiment time: 4 straight segments + 4 turns
NUM_SIDES = 4
MAX_TIME = NUM_SIDES * (STRAIGHT_TIME + TURN_TIME) + 1.0  #im adding a 1 second margin

# selecting motors

left_motor  = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

# velocity controls
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

# converting linear speed to wheel angular speed
target_omega = LINEAR_SPEED / WHEEL_RADIUS

# getting ground truth - using this value to calculate error with odometer reading

robot_node = robot.getSelf()
translation_field = robot_node.getField('translation')
rotation_field    = robot_node.getField('rotation')

# Initial ground truth position, we only need to consider x and y value since it is flat
initial_pos = translation_field.getSFVec3f()
x_true = initial_pos[0]
y_true = initial_pos[1]

#Odometry Initialization

#initialize odometry position to match true pose at t = 0
x_odom = x_true
y_odom = y_true

# Estimate initial heading theta_odom from the rotation field (axis-angle)
# Webots rotation: [ax, ay, az, angle]
axis_angle = rotation_field.getSFRotation()
ax, ay, az, angle = axis_angle

# For ENU with Z-up, yaw is rotation about z-axis; sign depends on axis direction.
if abs(az) > 0.9:
    theta_odom = angle if az > 0 else -angle
else:
    theta_odom = 0.0

# we plot the graph as it moves in real time - we also get average error value in the end

plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], marker='.')
ax.set_xlabel("Time [s]")
ax.set_ylabel("Odometry Error [m]")
ax.set_title("Euclidean Distance between Odometry and Ground Truth")
ax.grid(True)

time_data = []
error_data = []

#error accumulation

sum_error = 0.0
num_samples = 0

start_time = robot.getTime()

#square path state machine

STATE_STRAIGHT = 0
STATE_TURN     = 1

state = STATE_STRAIGHT
segment_start_time = start_time
sides_completed = 0

# main control of robot to move in square

while robot.step(TIME_STEP) != -1:
    current_time = robot.getTime()
    elapsed = current_time - start_time
    seg_elapsed = current_time - segment_start_time

    #control: choose wheel velocities based on state
    if state == STATE_STRAIGHT:
        # Go straight
        left_motor.setVelocity(target_omega)
        right_motor.setVelocity(target_omega)

        if seg_elapsed >= STRAIGHT_TIME:
            # Done one straight segment, now turn 90°
            state = STATE_TURN
            segment_start_time = current_time

    elif state == STATE_TURN:
        # Turn in place (left forward, right backward)
        left_motor.setVelocity(TURN_WHEEL_SPEED)
        right_motor.setVelocity(-TURN_WHEEL_SPEED)

        if seg_elapsed >= TURN_TIME:
            sides_completed += 1
            if sides_completed >= NUM_SIDES:
                # finished full square - now we stop moving
                left_motor.setVelocity(0.0)
                right_motor.setVelocity(0.0)
                break
            # start next straight segment
            state = STATE_STRAIGHT
            segment_start_time = current_time

    #ground truth position
    pos = translation_field.getSFVec3f()
    x_true = pos[0]
    y_true = pos[1]

    #odometry integration
    dt = TIME_STEP / 1000.0

    # Motor target velocities (rad/s)
    omega_l = left_motor.getVelocity()
    omega_r = right_motor.getVelocity()

    # Differential drive kinematics
    v = WHEEL_RADIUS * (omega_r + omega_l) / 2.0
    omega_robot = WHEEL_RADIUS * (omega_r - omega_l) / WHEEL_SEPARATION

    theta_odom += omega_robot * dt
    x_odom += v * math.cos(theta_odom) * dt
    y_odom += v * math.sin(theta_odom) * dt

    #Error computation
    error = math.sqrt((x_true - x_odom) ** 2 + (y_true - y_odom) ** 2)
    sum_error += error
    num_samples += 1

    #live plot update
    time_data.append(elapsed)
    error_data.append(error)
    line.set_data(time_data, error_data)
    ax.relim()
    ax.autoscale_view()
    plt.pause(0.001)


# printing average error results on the terminal after 20 sec

if num_samples > 0:
    avg_error = sum_error / num_samples
else:
    avg_error = float('nan')

print("Odometry-only square-path experiment summary")
print(f"Sides completed: {sides_completed}/{NUM_SIDES}")
print(f"Approx. duration: {elapsed:.2f} s")
print(f"Samples:          {num_samples}")
print(f"Average position error (odom vs true): {avg_error:.6f} m")

plt.ioff()
plt.show()
