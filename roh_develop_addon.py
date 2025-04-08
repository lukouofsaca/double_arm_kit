from Robotic_Arm import *
from Robotic_Arm.rm_robot_interface import *
from roh_registers_v1 import *
import time
import logging
import threading
R_ARM_IP = "192.168.1.18"
L_ARM_IP = "192.168.1.19"

COM_PORT = 1
ROH_ADDR = 2

RIGHT_HAND = 0
LEFT_HAND = 1

SHORT_MAX = 65535

ANGLE_THUMB_MAX = 90
ANGLE_SIDE_MAX = 31
ANGLE_HAND_MAX = 50
ANGLE_MAX = 180

# the following parameters are used to calculate the position of the object.
# which are in the original demo file provided by oymotion.
TABLE_HEIGHT = 0.72 #桌面高度/m
BASE_HEIGHT = 0.80 #+ 0.01 #底座高度/m
OBJECT_DISTANCE = 0.56 #底座接线口右侧中心离中间孔距离/m
HOLE_DISTANCE = 0.1775 #孔距/m
OFFSET_BASE_TABLE = TABLE_HEIGHT - BASE_HEIGHT #采用桌面底座高度差，计算Z偏移值

target_fingers = [
    ROH_FINGER_POS_TARGET0,
    ROH_FINGER_POS_TARGET1,
    ROH_FINGER_POS_TARGET2,
    ROH_FINGER_POS_TARGET3,
    ROH_FINGER_POS_TARGET4,
    ROH_FINGER_POS_TARGET5,
]

def L_BYTE(v):
    return v & 0xFF

def H_BYTE(v):
    return (v >> 8) & 0xFF

def create_hand_register_params_t(
    target,
    total_register_number,
):
    """warp input data. the input list of data must be warped before send to arm.
    the function is used to create a register params for hand.

    Args:
        target (int):start of target_register. please refer to roh_registers_v1.py.
        total_register_number (register number): num of continous registers to be write.
    Returns:
        rm_peripheral_read_write_params_t: rm_peripheral_read_write_params_t
    """
    return rm_peripheral_read_write_params_t(
        1, # COM_PORT=hand
        target,
        2, # ROH_ADDR=hand
        total_register_number
    )

def warp_bit(input_data):
    """warp input data. the input list of data must be warped before send to arm.

    Args:
        input_data (list): data need to be warped. the data should be a list of int[0-65535].

    Returns:
        input[list]: the warped data.
        len[int]: the length of input_data.should be passed to the function.
    """
    assert type(input_data) == list, "input_data must be a list"
    len = input_data.__len__()
    input = []
    for i in range(len):
        assert type(input_data[i]) == int, "input_data must be a list of int"
        assert 0<= input_data[i] and input_data[i] <= 65535, "input_data must be in range [0, 65535]"
        input.append(H_BYTE(input_data[i]))
        input.append(L_BYTE(input_data[i]))
    return input,len    

def register_write(robot,data,target=ROH_FINGER_POS_TARGET0, delay=0, num=None, warpped = False):
    """write registers.

    Args:
        robot (RoboticArm): robot to be written
        data (list[short]): data need to be moved. the data should be a list of int[0-65535].
        data should be warped before send to arm.
        target (int): start of target_register. default is ROH_FINGER_POS_TARGET0(equals to thumb circle).
        delay (int, optional): delay of the move. Defaults to 0.
        num (int, optional): num of continous registers to be write. Default to None.(To be calculated by data).
    """
    if not warpped:
        data,_ = warp_bit(data)
    if num is None:
        num = len(data) // 2 

    params = create_hand_register_params_t(target=target, total_register_number=num)
    ret = robot.rm_write_registers(params, data)
    
    if delay > 0:
        time.sleep(delay)
    return ret

# we defie the hand_control_b as the register_write function.
# Facutally you can use the same as you use hand_control_p/a as you may don't like to pass all the positions
#   to the function. Hence you can alternativeli pass target/num to the functions.

hand_control_b = register_write

def extract_data(data):
    """byte to short.
    get the data from arm. the data is a list of byte. the function will convert the data to a list of short.

    Args:
        data (byte): list of 1byte data.
        which follows the big endian format.
        we implictly assume that the len of data can be divided by 2.

    Returns:
        list[short]: list of short data.
    """
    exact_len = len(data)//2
    data_c = []
    for i in range(exact_len):
        data_c.append(data[i*2]<<8 | data[i*2+1])
    return data_c
        
def register_read_holding(robot: RoboticArm,target=ROH_FINGER_FORCE_LIMIT0,num: int=1):
    """
    read registers.

    Args:
        robot (RoboticArm): robot_hand to be read
        target (REGISTER, optional): . Defaults to ROH_FINGER_FORCE_LIMIT0.
        num (int, optional): num to read. Defaults to 1.

    Returns:
        status: read status
        data: read data
    """
    assert num <= 12
    params = create_hand_register_params_t(target=target,total_register_number=num)
    if num==1:
        status,data = robot.rm_read_holding_registers(params)
        data = [data]
    else:
        status,data = robot.rm_read_multiple_holding_registers(params)
        data = extract_data(data)
    if status != 0:
        logging.log(level=logging.ERROR,msg=f"error status {status} for hand")
    return status,data

def register_read_input(robot: RoboticArm,target=ROH_CALI_THUMB_POS0,num: int=1):
    """
    read registers.

    Args:
        robot (RoboticArm): robot_hand to be read
        target (REGISTER, optional): . Defaults to ROH_FINGER_FORCE_LIMIT0.
        num (int, optional): num to read. Defaults to 1.

    Returns:
        status: read status
        data: read data
    """
    assert num <= 12
    params = create_hand_register_params_t(target=target,total_register_number=num)
    if num==1:
        status,data = robot.rm_read_input_registers(params)
        data = [data]
    else:
        status,data = robot.rm_read_multiple_input_registers(params)
        data = extract_data(data)
    if status != 0:
        logging.log(level=logging.ERROR,msg=f"error status {status} for hand")
    return status,data



def arms_init(arm_ips:list[str], arm_port:int=8080,robots:list[RoboticArm]=[]):
    """get/connect hands.

    Args:
        arm_ips (list[str]): arm ips.
        arm_port (int, optional): arm port. Defaults to 8080.
        provide_arm (list[RoboticArm], optional): list of robotic arms. 
            if not provided, the function will use the provided arms instead of creating new copy.
            Defaults to empty list.
            should satisfy arm_ips correspond to the provided arms(but isn't need for the same number).

    Returns:
        list[RoboticArm]: list of robotic arms.
    """
    
    if len(robots) < len(arm_ips):
        logging.log(level=logging.INFO,msg="provided arms is less than arm_ips")
        for i in range(len(arm_ips)-len(robots)):
            robots.append(RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E))
    # elif len(provided_arm) > len(arm_ips):
    #     logging.log(level=logging.INFO,msg="provided arms is more than arm_ips")
    #     # we expect no behavior for the overflow arms.
        
    for arm,ip in zip(robots,arm_ips):
        handle = arm.rm_create_robot_arm(ip, arm_port)
        arm.rm_close_modbustcp_mode()
        arm.rm_set_modbus_mode(COM_PORT,115200,1)
        
        
    return robots

def disconnect(robots:list[RoboticArm]):
    """disconnect arms.

    Args:
        arms (list[RoboticArm]): list of robotic arms.
    """
    for arm in robots:
        arm.rm_delete_robot_arm()

## put this outside, so anyone won't pass 6 params could use this function to convert to percent.
def precent_to_short(x):
    percent = x/100*65535
    if percent > 65535:
        percent = 65535
    elif percent < 0:
        percent = 0
    return round(percent)

def angle_to_short(x):
    percent = x/ANGLE_MAX*65535
    if percent > 65535:
        ## need to log the error.
        percent = 65535
    elif percent < 0:
        percent = 0
    return round(percent)

def hand_control_p(robot: RoboticArm,position,target=ROH_FINGER_POS_TARGET0, delay=0, num=None, warpped = False):
    """move the hand to the position.

    Args:
        robot (RoboticArm): robotic arm to be moved.
        position (list[float]): position of the hand. percent(0-100)
        other params defines same as hand_control_b

    """
    position = [precent_to_short(x) for x in position]
    return register_write(robot,position,target,delay,num,warpped)


def hand_control_a(robot: RoboticArm,angle:list[float],angle_thumb=None,angle_side=None,angle_hand=None
                    ,target=ROH_FINGER_POS_TARGET0, delay=0, num=None, warpped = False):
    """move the hand to the position.

    Args:
        robot (RoboticArm): robotic arm to be moved.
        angles (list[float]): angles of the hand. degree(0-180) please note that only the first 5 params
            should be passed to this param if you need to use every degree of freedom.
        angle_thumb (float, optional): angle of thumb(0-90). Defaults to None.
        angle_side (float, optional): angle of side(31-0). Defaults to None.
        angle_hand (float, optional): angle of hand(0-50). Defaults to None.
        other params defines same as hand_control_b
        
        to convert different angles of thumb_turn:
        angle_thumb=0 == angle_side=31 = angle_hand=0
        angle_thumb=90 == angle_side=0 = angle_hand=50    
        priority of the angle is thumb > side > hand.
    
    """
    # use angle_turn to log the angle of the thumb turn.
    angle_turn = None
    if angle_thumb is not None:
        angle_turn = angle_thumb/ANGLE_THUMB_MAX*ANGLE_MAX
    elif angle_side is not None:
        angle_turn = (ANGLE_SIDE_MAX-angle_side)/ANGLE_SIDE_MAX*ANGLE_MAX
    elif angle_hand is not None:
        angle_turn = angle_hand/ANGLE_HAND_MAX*ANGLE_MAX
    if angle_turn is not None:
        angle.append(angle_turn)        
    
    position = [angle_to_short(x) for x in position]
    
    return register_write(robot,position,target=ROH_FINGER_POS_TARGET0, delay=delay, num=num, warpped=warpped)
    

if __name__ == "__main__":
    arm_r,arm_l = arms_init([R_ARM_IP, L_ARM_IP])    
    def Arm_move(Arm:RoboticArm):
        # move the arm to the position
        #Arm.rm_movec([0,0,0,0,0,0], 30, 0, True)
        # move the arm to the position
        Arm.rm_movej([0,20,30,0,0,0], 30, 0, True,False)
        # move the arm to the position
        # Arm.rm_movej_p([40,0,0,0,0,0], 30, 0, True)
    # use different thread to control the arms.
    # threading.Thread(target=Arm_move,args=(arm_r,)).start()
    # threading.Thread(target=Arm_move,args=(arm_l,)).start()
    arm_r.rm_movej([0,0,0,0,0,0], 30, 0, False,False)
    arm_l.rm_movej([0,0,0,0,0,0], 30, 0, False,False)
    arm_r.rm_movej([40,20,30,0,0,0], 30, 0, False,False)
    arm_l.rm_movej([40,20,30,0,0,0], 30, 0, False,False)

    

    # robot.rm_movej(position, speed, rad, wait,True)
