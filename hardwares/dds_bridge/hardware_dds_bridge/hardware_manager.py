import sys
import time
import threading
import traceback
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import Vector3Stamped

from Motor_Manager_origin import *

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.utils.crc import CRC
crc = CRC()

def mujoco_ang2real_ang(dof_pos):
    motor_order = ['frd', 'fld', 'rrd', 'rld',  # Lower legs
                    'fru', 'flu', 'rru', 'rlu',  # Upper legs
                    'frh', 'flh', 'rrh', 'rlh']  # Hips

    mujoco_order = ['frh', 'fru', 'frd', 
                    'flh', 'flu', 'fld',  
                    'rrh', 'rru', 'rrd',
                    'rlh', 'rlu', 'rld']
        
    index_map = [mujoco_order.index(name) for name in motor_order]

    reordered_dof_pos = [dof_pos[i] for i in index_map]

    reordered_dof_pos = [[-reordered_dof_pos[0], reordered_dof_pos[1], -reordered_dof_pos[2], reordered_dof_pos[3]],  
                            [-reordered_dof_pos[4], reordered_dof_pos[5],  -reordered_dof_pos[6], reordered_dof_pos[7]], 
                            [ reordered_dof_pos[8], reordered_dof_pos[9],  -reordered_dof_pos[10], -reordered_dof_pos[11]]]
    
    return reordered_dof_pos

def real_ang2mujoco_ang(dof_pos):
    motor_order = ['frd', 'fld', 'rrd', 'rld',  # Lower legs
                    'fru', 'flu', 'rru', 'rlu',  # Upper legs
                    'frh', 'flh', 'rrh', 'rlh']  # Hips

    mujoco_order = ['frh', 'fru', 'frd', 
                    'flh', 'flu', 'fld',  
                    'rrh', 'rru', 'rrd',
                    'rlh', 'rlu', 'rld']
        
    index_map = [motor_order.index(name) for name in mujoco_order]

    reordered_dof_pos = [dof_pos[i] for i in index_map]

    reordered_dof_pos = [reordered_dof_pos[0], -reordered_dof_pos[1], -reordered_dof_pos[2], 
                          reordered_dof_pos[3], reordered_dof_pos[4], reordered_dof_pos[5],  
                          -reordered_dof_pos[6], -reordered_dof_pos[7], -reordered_dof_pos[8], 
                        -reordered_dof_pos[9],  reordered_dof_pos[10], reordered_dof_pos[11]]
    
    return reordered_dof_pos

class DDSHandler:
    def __init__(self, ether_name: str=""):
        self.pub = None
        self.low_state = None
        self.pub = ChannelPublisher("rt/lowstate", LowState_)
        self.pub.Init()  

        self.sub = None
        self.low_cmd = None
        self.sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self.sub.Init(self.LowCmdMessageHandler, 10) 

        self.joint_state_msg = JointState()
        self.last_joint_state_msg = JointState()
        self.joint_state_msg.name = [
            'flh', 'frh', 'rlh', 'rrh',   # Hips
            'flu', 'fru', 'rlu', 'rru',  # Upper legs
            'fld', 'frd', 'rld', 'rrd'   # Lower legs
        ]

        self.pid_gain_msg = Float32MultiArray()
        self.pid_gain_msg.data = [float(0), float(0)]

        self.dof_pos = []

    def LowCmdMessageHandler(self, msg: LowCmd_):
        self.low_cmd = msg
        if self.low_cmd is not None:
            # print(self.low_cmd.motor_cmd)
            self.joint_state_msg.position = []
            dof_pos = []
            self.joint_state_msg.velocity = []
            self.joint_state_msg.effort = []
            i = 0
            for motor_cmd in self.low_cmd.motor_cmd:
                if i < 12:
                    dof_pos.append(motor_cmd.q)
                    self.pid_gain_msg.data[0] = motor_cmd.kp
                    self.pid_gain_msg.data[1] = motor_cmd.kd
                    i += 1
                else:
                    break

            # self.joint_state_msg.position = self.mujoco_ang2real_ang(dof_pos)
            self.dof_pos = mujoco_ang2real_ang(dof_pos)

            self.last_joint_state_msg = self.joint_state_msg
        else:
            self.joint_state_msg = self.last_joint_state_msg

class LinearASub(Node):
    def __init__(self):
        super().__init__('Imu_linear_acceleration_subscriber')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/acceleration_hr',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to linear acceleration topic.')
        self.linear_acceleration = None

    def joint_state_callback(self, msg):
        self.linear_acceleration = msg.vector
        # self.get_logger().info(f"Received linear acceleration: x={msg.vector.x}, y={msg.vector.y}, z={msg.vector.z}")

    def get_Imu_data(self):
        return self.linear_acceleration

class AngularVSub(Node):
    def __init__(self):
        super().__init__('Imu_angular_velocity_subscriber')
        self.subscription = self.create_subscription(
            Vector3Stamped,
            '/imu/angular_velocity_hr',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to angular velocity topic.')
        self.angular_velocity = None

    def joint_state_callback(self, msg):
        self.angular_velocity = msg.vector
        # self.get_logger().info(f"Received angular velocity: x={msg.vector.x}, y={msg.vector.y}, z={msg.vector.z}")

    def get_Imu_data(self):
        return self.angular_velocity

class QuaternionSub(Node):
    def __init__(self):
        super().__init__('Imu_quaternion_subscriber')
        self.subscription = self.create_subscription(
            QuaternionStamped,
            '/filter/quaternion',
            self.joint_state_callback,
            10
        )
        self.get_logger().info('Subscribed to quaternion topic.')
        self.quaternion = None

    def joint_state_callback(self, msg):
        self.quaternion = msg.quaternion

    def get_Imu_data(self):
        return self.quaternion

class HardwareManager:
    def __init__(self):
        if len(sys.argv) <2:
            ChannelFactoryInitialize(1, "lo")
            self.dds_handler = DDSHandler()
        else:
            ChannelFactoryInitialize(0, sys.argv[1])
            self.dds_handler = DDSHandler(sys.argv[1])

        self.motor_manager = MotorManager()
        self.motor_manager.control_cmd.reset()
        time.sleep(0.1)
        self.motor_manager.run()
        # self.dds_handler = dds_handler
        # self.motor_manager = motor_manager

        rclpy.init()
        self.linearASub = LinearASub()
        self.angularVSub = AngularVSub()
        self.quaternionSub = QuaternionSub()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.angularVSub)
        self.executor.add_node(self.linearASub)
        self.executor.add_node(self.quaternionSub)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start()

        self.is_running = False

    def run(self):
        self.is_running = True
        self.run_thread = threading.Thread(target=self._run, daemon=True)
        self.run_thread.start()

    def _run(self):
        while self.is_running:
            # start_time = time.time()
            if self.dds_handler.dof_pos != []:
                self.motor_manager.joint_angles = self.dds_handler.dof_pos
                print("joint_angle",self.motor_manager.joint_angles)
                # self.motor_manager.joint_angles = [0.0] * 12
            if self.dds_handler.pid_gain_msg.data is not None:
                self.motor_manager.kp_kd_list = list(self.dds_handler.pid_gain_msg.data)
            else:
                self.motor_manager.kp_kd_list = [13, 0.4]
            time.sleep(1/500)

            joint_dof_pos = self.motor_manager.joint_position

            if joint_dof_pos is None:
                joint_dof_pos = [0.0] * 12
            joint_dof_pos = real_ang2mujoco_ang(joint_dof_pos)

            joint_dof_vel = self.motor_manager.joint_velocity
            if joint_dof_vel is None:
                joint_dof_vel = [0.0] * 12
            joint_dof_vel = real_ang2mujoco_ang(joint_dof_vel)

            quaternion = self.quaternionSub.get_Imu_data()
            angular_velocity = self.angularVSub.get_Imu_data()
            linear_acceleration = self.linearASub.get_Imu_data()
            if quaternion is None or angular_velocity is None or linear_acceleration is None:
                print("Error: IMU data is None")
                continue

            low_state = unitree_go_msg_dds__LowState_()
            low_state.head[0] = 0x00
            low_state.head[1] = 0x00
            low_state.level_flag = 0x00
            
            low_state.imu_state.quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
            low_state.imu_state.gyroscope = [angular_velocity.x, angular_velocity.y, angular_velocity.z]
            low_state.imu_state.accelerometer = [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]

            for i in range(12):
                low_state.motor_state[i].q = joint_dof_pos[i]
                low_state.motor_state[i].dq = joint_dof_vel[i]
                low_state.motor_state[i].tau_est = 0.0

            self.dds_handler.pub.Write(low_state)

            # interval = start_time - time.time()
            # print("interval", 1/interval)


        
    def run_low_cmd2hardware(self):
        self.is_running = True
        self.low_cmd_thread = threading.Thread(target=self.low_cmd2hardware, daemon=True)
        self.low_cmd_thread.start()

    def low_cmd2hardware(self):
        while self.is_running:

            # print(self.dds_handler.dof_pos)
            # print(self.dds_handler.dof_pos != [])
            # print("pid_gain_msg: ", list(self.dds_handler.pid_gain_msg.data))
            # if self.dds_handler.dof_pos != []:
            #     self.motor_manager.joint_angles = self.dds_handler.dof_pos
            # else:
            # initial_angles: [0.0, 1.6, -2.99, -0.0, 1.6, -2.99, -0.0, -1.6, 2.99, 0.0, -1.6, 2.99]
            # sit_angles: [0.0, 2.54, -2.7, -0.0, 2.54, -2.7, -0.0, -2.54, 2.7, 0.0, -2.54, 2.7]
            # default_angles: [0.1, 0.785, -1.57, -0.1, 0.785, -1.57, -0.1, -0.785, 1.57, 0.1, -0.785, 1.57]
            self.motor_manager.joint_angles = mujoco_ang2real_ang([0.0, 1.6, -2.99, -0.0, 1.6, -2.99, -0.0, -1.6, 2.99, 0.7, -0.1, 0.1])
            # FR FL RR
            # if self.dds_handler.pid_gain_msg.data is not None:
            #     self.motor_manager.kp_kd_list = list(self.dds_handler.pid_gain_msg.data)
            # else:
            self.motor_manager.kp_kd_list = [13, 0.4]
            time.sleep(1/500)

        # [[0.04186054691672325, -0.04186054691672325, 0.04186054691672325, -0.04186054691672325], 
        #  [1.7239521741867065, -1.7239521741867065, -1.7239521741867065, 1.7239521741867065], 
        #  [-2.226975917816162, 2.226975917816162, 2.226975917816162, -2.226975917816162]]

    def run_hardware2low_state(self):
        self.is_running = True
        self.hardware2low_state_thread = threading.Thread(target=self.hardware2low_state, daemon=True)
        self.hardware2low_state_thread.start()

    def hardware2low_state(self):
        while self.is_running:
            joint_dof_pos = self.motor_manager.joint_position

            if joint_dof_pos is None:
                joint_dof_pos = [0.0] * 12
            joint_dof_pos = real_ang2mujoco_ang(joint_dof_pos)
            joint_dof_vel = self.motor_manager.joint_velocity
            if joint_dof_vel is None:
                joint_dof_vel = [0.0] * 12

            quaternion = self.quaternionSub.get_Imu_data()
            angular_velocity = self.angularVSub.get_Imu_data()
            linear_acceleration = self.linearASub.get_Imu_data()
            if quaternion is None or angular_velocity is None or linear_acceleration is None:
                print("Error: IMU data is None")
                continue

            low_state = unitree_go_msg_dds__LowState_()
            low_state.head[0] = 0x00
            low_state.head[1] = 0x00
            low_state.level_flag = 0x00
            
            low_state.imu_state.quaternion = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
            low_state.imu_state.gyroscope = [angular_velocity.x, angular_velocity.y, angular_velocity.z]
            low_state.imu_state.accelerometer = [linear_acceleration.x, linear_acceleration.y, linear_acceleration.z]

            for i in range(12):
                low_state.motor_state[i].q = joint_dof_pos[i]
                low_state.motor_state[i].dq = joint_dof_vel[i]
                low_state.motor_state[i].tau_est = 0.0

            # low_state.crc = crc.Crc(low_state)
            self.dds_handler.pub.Write(low_state)

            time.sleep(1/500)

    def stop(self):
        self.is_running = False
        self.executor.shutdown()
        rclpy.shutdown()

def main():
    hardware_manager = HardwareManager()

    command_dict = {
        'run': hardware_manager.run,
        'lc2h': hardware_manager.run_low_cmd2hardware,
        'h2ls': hardware_manager.run_hardware2low_state,
        'stop': hardware_manager.stop,
    }
    print("Commands: run, lc2h, h2ls, stop, exit")
    
    while True:
        try:
            cmd = input("CMD: ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                hardware_manager.stop()
                break
            else:
                print("Invalid command")
        except Exception as e:
            traceback.print_exc()
            break

    # dof = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
    # print(dof)
    # dof = mujoco_ang2real_ang(dof)
    # print(dof)
    # flat_dof_pos = [0] * 12
    # for i in range(3):
    #     for j in range(4):
    #         flat_dof_pos[4*i+j] = dof[i][j]
    # dof = real_ang2mujoco_ang(flat_dof_pos)
    # print(dof)


if __name__ == '__main__':
    main()

# [[-2.6510298252105713, 2.6510298252105713, 2.6510298252105713, -2.6510298252105713], 
# [2.3300116062164307, -2.3300116062164307, -2.3300116062164307, 2.3300116062164307], 
# [-0.00433365348726511, 0.00433365348726511, -0.00433365348726511, 0.00433365348726511]]

# [motor_manager-1] [[ 2.04952884 -2.04952884 -2.04952884  2.04952884]
# [motor_manager-1]  [-1.47034442  1.47034442  1.47034442 -1.47034442]
# [motor_manager-1]  [-0.05756381  0.05756381 -0.05756381  0.05756381]]

# [ros2_control_node-2] Current position: 
# -0.0532158 -1.4723 -3.1199 
# 0.0185016 -1.51999 -3.15766 
# -0.0631342 1.54936 -3.08137 
# 0.0516899 1.5692 3.07755 

# [ros2_control_node-2] Current position: 
# -0.0185016 1.51961 -3.15843 
# -0.0535973 1.47879 -3.12066 
# 0.0516899 -1.56996 3.07755 
# -0.0700008 -1.55356 3.0848 
