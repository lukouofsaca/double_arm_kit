# grasp little things, like a key
THUMB_READY_GRASP = [0,60000,60000,60000,60000,0]
THUMB_GRASP = [55000,60000,60000,60000,60000,0]
THUMB_GRASP_UNUSED = [40000,32767,0,0,0,60000]

# grasp a bottle, like a water bottle
HAND_READY_GRASP_BOTTLE = [0,0,0,0,0,65535]
HAND_GRASP_BOTTLE = [15500,30000,21000,19000,16000,65535]

# initial position of the hand
HAND_INITIAL_POS= [0,0,0,0,0,0]

# from gforce demo
GESTURES = {
    "REST":     [20000, 10000, 10000, 10000, 10000,     0],
    "FIST":     [45000, 65535, 65535, 65535, 65535,     0],
    "POINT":    [45000,     0, 65535, 65535, 65535,     0],
    "VICTORY":  [45000,     0,     0, 65535, 65535,     0],
    "SPREAD":   [    0,     0,     0,     0,     0,     0],
    "ROCK":     [    0,     0, 65535, 65535,     0,     0],
    "PINCH":    [45000, 45000, 45000, 45000, 45000,     0],
    "SHOOT":    [    0,     0, 65535, 65535, 65535,     0],
    "ITC":      [30000, 30000,     0,     0,     0, 65535],
    "SIX":      [    0, 65535, 65535, 65535,     0,     0],
    "TRIGGER":  [38000, 30000, 32000, 65535, 65535, 65535],
    "THUMBUP":  [    0, 65535, 65535, 65535, 65535,     0],
    "GRASP":    [27525, 29491, 32768, 27525, 24903, 65535]
}

__all__ = [
    'THUMB_GRASP',
    'HAND_READY_GRASP_BOTTLE_POS',
    'HAND_GRASP_BOTTLE_POS',
    'HAND_INITIAL_POS',
    'GESTURES',
]