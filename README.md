# BubbleRob Controller

## Node N1: Robot Controller

Implements controller to command BubbleRob robot in Coppelia Sim

- Subscribe to topic /trajectory to receive positions in trajectory
- Publish wheels speed corresponding to desired movement to topics /leftMotorSpeed and /leftMotorSpeed

### Path following algorithm:
1. Get next desired position
2. Calculate angle between vector (Xd - X0) and x-axis of robot in robot local frame (Xd: desired position, X0: current position)
3. If angle is greater than an arbitrary tolerance value => set opposite velocities in wheels to rotate and allign vectors; else => set max velocity to both wheels to move forward
4. Repeat 3 until arrive to desired position
5. If there is a next desired position => go to 1; else => stop

## Node N2: Path Generator 

Implement path generator that receives (x, y) positions from user, stores and publishes after receiving a pre-defined number of positions 

- Subscribe to topic /position to receive positions
- Publish to topic /trajectory to pass accumulated positions
