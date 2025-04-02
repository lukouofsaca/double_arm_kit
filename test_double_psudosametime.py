from roh_develop_addon import *

# arm_r,arm_l = arms_init([R_ARM_IP, L_ARM_IP])    
def Arm_move(Arm:RoboticArm):

    Arm.rm_movej([60,20,30,0,0,0], 30, 0, False,True)
    ret = Arm.rm_movej([0,0,0,0,0,0], 30, 0, False,True)
    print(f"move done{ret}")
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
time.sleep(2)
# test 1. one block
tr = threading.Thread(target=Arm_move,args=(arm_r,))
tl = threading.Thread(target=Arm_move,args=(arm_l,))
tr.start()
tl.start()
tl.join()
tr.join()
arm_l.rm_movej([40,20,30,0,0,0], 30, 0, False,False)
arm_r.rm_movej([40,20,30,0,0,0], 30, 0, False,False) 
arm_r.rm_movej([0,20,30,30,0,0], 30, 0, False,False) 
arm_l.rm_movej([0,20,30,30,0,0], 30, 0, False,False)
# init.
arm_r.rm_movej([0,0,0,0,0,0], 30, 0, False,True)
arm_l.rm_movej([0,0,0,0,0,0], 30, 0, False,True)

time.sleep(2)
# test 2. two block
arm_l.rm_movej([40,20,30,0,0,0], 30, 0, False,True)
arm_r.rm_movej([40,20,30,0,0,0], 30, 0, False,False) 
arm_r.rm_movej([0,20,30,30,0,0], 30, 0, False,False) 
arm_l.rm_movej([0,20,30,30,0,0], 30, 0, False,False)

time.sleep(5)
arm_r.rm_movej([0,0,0,0,0,0], 30, 0, False,True)
arm_l.rm_movej([0,0,0,0,0,0], 30, 0, False,True)
## Note we use a different strategy to move the arm to the position.
## we observed that different arm did not interfere with each other at Block/No Block.

# time.sleep(5)
threading.Thread(target=Arm_move,args=(arm_r,)).start()
threading.Thread(target=Arm_move,args=(arm_l,)).start()
