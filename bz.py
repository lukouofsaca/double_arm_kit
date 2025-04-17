from Robotic_Arm import *
from Robotic_Arm.rm_robot_interface import *
from roh_registers_v1 import *
import time
import logging
from roh_develop_addon import register_read_holding
# 这个自己改
R_ARM_IP = "192.168.1.19"

# 这些别动

COM_PORT = 1
ROH_ADDR = 2
RIGHT_HAND = 0
LEFT_HAND = 1
SHORT_MAX = 65535

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


# arm_r = arms_init(arm_ips=[R_ARM_IP])[0]    
init = [0,0,0,0,0,0]




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
hand_control_b = register_write

def disconnect(robots:list[RoboticArm]):
    """disconnect arms.

    Args:
        arms (list[RoboticArm]): list of robotic arms.
    """
    for arm in robots:
        arm.rm_delete_robot_arm()
        
        
# GESTURE0 = [0,0,0,0,0,65535]
# GESTURE1 = [65535,0,0,0,0,65535]
# # GESTURE1 = [0,65535,65535,65535,65535,65535]
# GESTURE2 = [65535,65535,65535,65535,65535,0]
# hand_control_b(arm_r,init)
# hand_control_b(arm_r,GESTURE0)
# input()
# hand_control_b(arm_r,GESTURE1)
# time.sleep(3)
# hand_control_b(arm_r,GESTURE2)

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

def protect(arm, initial = 0,count = 10, delay = 0.1,maxtoler = 100):
    data_old = None
    true = 0
    time.sleep(initial)
    data_old = []
    def tolerate(old:enumerate[enumerate[int]],new:enumerate[enumerate[int]]):
        for i in old:
            for old,k in zip(i,new):
                if abs(old-k) > maxtoler:
                    logging.log(level=logging.INFO,msg=f"{old} {k} is not in tolerance")
                    return False
        return True
    while True:
        time.sleep(delay)
        _,data = register_read_holding(arm,ROH_FINGER_POS0, 6)
        if data_old is []:
            true+=1
        else:    
            if tolerate(data_old,data):
                true+=1
            else:
                true = 0
                data_old.pop(0)
        data_old.append(data)
        if true >= count:
            break
    logging.log(level=logging.WARN,msg=f"protecting pos {data}")
    hand_control_b(arm,data)
# protect(arm_r)
    

# input()
# hand_control_b(arm_r,init)
