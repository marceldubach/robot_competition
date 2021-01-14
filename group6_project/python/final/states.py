"""
    main states of the state machine:
    0: starting
    1: moving
    2: obstacle avoidance
    3: catch bottle
    4: return to home
    5: empty container
    6: finished
"""
# define states as global variables
STARTING = 0
MOVING = 1
OBSTACLE = 2
CATCH = 3
RETURN = 4
EMPTY = 5
FINISH = 6