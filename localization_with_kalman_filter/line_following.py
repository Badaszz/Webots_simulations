
COUNTER_MAX = 6
COUNTER_2 = 31


def compute_line_follow_action(state, line_left, line_right, line_forward, obstacle, on_line, counter, max_speed):
    leftSpeed = 0.0
    rightSpeed = 0.0

    if state == 'forward':
        leftSpeed = max_speed
        rightSpeed = max_speed

        if obstacle:
            state = 'right_turn'
            counter = 0
        elif line_left and not line_right:
            state = 'left'
            counter = 0
        elif line_right and not line_left:
            state = 'right'
            counter = 0

    elif state == 'right':
        leftSpeed = 0.8 * max_speed
        rightSpeed = 0.4 * max_speed

        if counter == COUNTER_MAX:
            state = 'forward'

    elif state == 'left':
        leftSpeed = 0.4 * max_speed
        rightSpeed = 0.8 * max_speed

        if counter == COUNTER_MAX:
            state = 'forward'

    elif state == 'right_turn':
        leftSpeed = max_speed
        rightSpeed = 0.0

        if counter == 50:
            if obstacle:
                counter = 34
            else:
                state = 'forward2'
                counter = 0

    elif state == 'forward2':
        leftSpeed = 0.4 * max_speed
        rightSpeed = 0.4 * max_speed

        if counter == 15:
            state = 'left_turn'
            counter = 0

    elif state == 'left_turn':
        leftSpeed = 0.2 * max_speed
        rightSpeed = 0.8 * max_speed

        if on_line and (counter > 16):
            state = 'right_turn2'
            counter = 0
        elif obstacle:
            state = 'right_turn'
            counter = 0
        elif counter == COUNTER_2:
            state = 'forward3'
            counter = 0

    elif state == 'forward3':
        leftSpeed = 0.6 * max_speed
        rightSpeed = 0.6 * max_speed

        if on_line:
            state = 'right_turn2'
            counter = 0
        elif counter == 13:
            state = 'left_turn'
            counter = 0

    elif state == 'right_turn2':
        leftSpeed = max_speed
        rightSpeed = 0.0

        if counter == 30:
            state = 'forward'
            counter = 0

    counter += 1
    return state, leftSpeed, rightSpeed, counter
