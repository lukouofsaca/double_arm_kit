from roh_develop_addon import *

# arm_r,arm_l = arms_init([R_ARM_IP, L_ARM_IP])    
def Arm_move(Arm:RoboticArm):

    Arm.rm_movej([60,20,30,0,0,0], 30, 0, False,True)
    ret = Arm.rm_movej([0,0,0,0,0,0], 30, 0, False,True)
    Arm.rm_movej([60,20,30,0,0,0], 30, 0, False,True)
    # time.sleep(5)
    Arm.rm_movej([40,20,30,0,0,0], 30, 0, False,True)
    Arm.rm_movej([0,20,30,30,0,0], 30, 0, False,True)
    ret = Arm.rm_movej([0,0,0,0,0,0], 30, 0, False,True)

    print(f"move done r{ret}")
    # we observed that the ret is returned once the move START (NOT the move done).
    
def Arm_move_l(Arm):
    Arm.rm_movej([60,20,30,0,0,0], 30, 0, False,True)
    ret = Arm.rm_movej([0,0,0,0,0,0], 30, 0, False,True)
    Arm.rm_movej([60,20,30,0,0,0], 30, 0, False,True)
    # time.sleep(5)
    print("stop")

    block = Arm.rm_movej([40,20,30,0,0,0], 30, 0, False,True)
    Arm.rm_movej([0,20,30,30,0,0], 30, 0, False,True)
    print("stop")
    if(block==0):
        print("stop")
        Arm.rm_movej([0,0,10,10,0,0], 30, 0, False,True)
    ret = Arm.rm_movej([0,0,0,0,0,0], 30, 0, False,True)


    print(f"move done l{ret}")
    # we observed that the ret is returned once the move START (NOT the move done).    




# use different thread to control the arms.

arm_l = RoboticArm(rm_thread_mode_e.RM_TRIPLE_MODE_E)
arm_r = RoboticArm()

handle_l = arm_l.rm_create_robot_arm(L_ARM_IP,8080)
handle_r = arm_r.rm_create_robot_arm(R_ARM_IP,8080)

arm_l.rm_close_modbustcp_mode()
arm_l.rm_set_modbus_mode(1,115200,1)
arm_r.rm_close_modbustcp_mode()
arm_r.rm_set_modbus_mode(1,115200,1)

arm_l.rm_movej([0,0,0,0,0,0], 30, 0, False,True)
arm_r.rm_movej([0,0,0,0,0,0], 30, 0, False,True)
time.sleep(10)
# test 1.
t = True
if t:
    tr = threading.Thread(target=Arm_move,args=(arm_r,))
tl = threading.Thread(target=Arm_move_l,args=(arm_l,))
if t:
    tr.start()
tl.start()
tl.join()
if t:
    tr.join()
    print("join tr")


