# Line Following Robot with Obstacle Avoidance (Webots)

## Overview

This project implements a **line-following robot with obstacle
avoidance** using the **Webots robotics simulator**.\
The controller uses **ground sensors** to detect a line on the floor and
**proximity sensors** to detect obstacles.\
A **finite state machine (FSM)** controls the robot's behavior, allowing
it to:

-   Follow a line path
-   Detect obstacles ahead
-   Avoid obstacles with a predefined maneuver
-   Return to the line after bypassing the obstacle

The controller is written in **Python using the Webots Robot API**.

------------------------------------------------------------------------

## Features

-   Line detection using three ground sensors
-   Obstacle detection using proximity sensors
-   State-machine based robot behavior
-   Dynamic speed control for turning and navigation
-   Obstacle bypass and line re-acquisition logic

------------------------------------------------------------------------

## Robot Sensors

### Ground Sensors

The robot uses three ground sensors:

  Sensor   Position   Purpose
  -------- ---------- ----------------------------
  gs0      Right      Detect line on right
  gs1      Center     Detect line directly ahead
  gs2      Left       Detect line on left

These sensors detect the contrast between the floor and the line.

A threshold (`> 600`) determines whether the sensor is over the line.

------------------------------------------------------------------------

### Proximity Sensors

The robot uses **8 proximity sensors (ps0--ps7)**.

Obstacle detection uses the front sensors:

-   `ps0`
-   `ps1`
-   `ps7`

If any of these sensors detect a value ≥ **80**, the robot assumes an
obstacle is present.

------------------------------------------------------------------------

## Robot Motors

Two wheel motors are used:

-   `left wheel motor`
-   `right wheel motor`

Motor velocities are adjusted depending on the robot state.

Maximum speed:

    MAX_SPEED = 6.28

------------------------------------------------------------------------

## State Machine

The robot behavior is controlled using a **finite state machine**.

### States

  State         Description
  ------------- -----------------------------------------------------
  forward       Move straight and follow the line
  left          Slight left correction when line detected on left
  right         Slight right correction when line detected on right
  right_turn    Sharp right turn to avoid obstacle
  forward2      Move forward past obstacle
  left_turn     Turn left to return toward line
  forward3      Short forward motion to reacquire line
  right_turn2   Final correction to realign with line

------------------------------------------------------------------------

## Obstacle Avoidance Strategy

When an obstacle is detected:

1.  Robot performs a **sharp right turn**
2.  Moves **forward to bypass the obstacle**
3.  Turns **left to return toward the line**
4.  Performs **final alignment** with the line


------------------------------------------------------------------------

## Key Logic

### Line Detection

    line_right = gsValues[0] > 600
    line_left  = gsValues[2] > 600
    line_forward = gsValues[1] > 600

### Obstacle Detection

    obstacle = (psValues[0] >= 80) or (psValues[1] >= 80) or (psValues[7] >= 80)

### Line Reacquisition

    on_line = (not line_left) or (not line_right) or (not line_forward)

------------------------------------------------------------------------

## Timing Control

State transitions rely on counters:

  Variable      Purpose
  ------------- --------------------------------
  counter       Counts simulation steps
  COUNTER_MAX   Duration for small corrections
  COUNTER_2     Timeout for line recovery

Counters ensure that turns and movements last for a fixed number of
simulation steps.

------------------------------------------------------------------------

## Running the Controller

### Requirements

-   Webots
-   Python controller support enabled
-   A robot model with:
    -   3 ground sensors
    -   8 proximity sensors
    -   differential drive motors

------------------------------------------------------------------------

### Steps

1.  Open the Webots world containing the robot.
2.  Attach this Python controller to the robot.
3.  Start the simulation.

The robot will automatically:

-   Follow the line
-   Detect obstacles
-   Avoid them
-   Rejoin the line

------------------------------------------------------------------------

## Example Console Output

    Counter: 12, State: forward
    Counter: 0, State: right_turn
    Counter: 35, State: forward2
    Counter: 0, State: left_turn

This helps with debugging the state transitions.

------------------------------------------------------------------------

## Citation

Github repo for setup, guide and FSM https://github.com/felipenmartins/Robotics-Simulation-Labs


## Author

Yusuf Solomon

Robotics \| AI \| Autonomous Systems
