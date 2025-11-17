from gazebo_env import GazeboEnv
from gazebo_connection import GazeboConnection
import time
import random
import os
import subprocess
import signal
from std_msgs.msg import Float32, ColorRGBA
from gazebo_msgs.msg import ContactsState, ModelState, ModelStates
from gazebo_msgs.srv import SetModelConfiguration, SetModelState, SetLightProperties
import rospy
from utils import quaternion_from_euler
from std_msgs.msg import Bool, Int32

class GazeboSimulation:
    def __init__(self, launch_file):
        self.gz_env = GazeboEnv(launchfile=launch_file)
        self.gz_con = GazeboConnection(start_init_physics_parameters=True, reset_world_or_sim="SIMULATION")
        self.gazebo_model_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.gazebo_model_configuration_service = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        
        # 控制话题发布器
        self.control_pub = rospy.Publisher('/simulation_control', Int32, queue_size=1)
        self.label_pub = rospy.Publisher('/is_label_mode', Bool, queue_size=1)
        
        # 完成确认订阅器
        self.completion_sub = rospy.Subscriber('/save_completion', Int32, self.completion_callback)
        
        self.gazebo_light_service = rospy.ServiceProxy(
            '/gazebo/set_light_properties', SetLightProperties
        )
        
        # 状态变量
        self.pose_count = 0
        self.waiting_for_completion = False
        self.last_completed_index = -1
        self.current_pose_index = -1
        
        time.sleep(12)
        self.gz_con.unpauseSim()
        time.sleep(3)
        
        print("✅ Gazebo仿真启动完成")
    
    def completion_callback(self, msg):
        """保存完成确认回调"""
        completed_index = msg.data
        if self.waiting_for_completion and completed_index == self.current_pose_index:
            print(f"✅ 收到完成确认: 位姿 {completed_index}")
            self.waiting_for_completion = False
            self.last_completed_index = completed_index
    
    def wait_for_save_completion(self, pose_index, timeout=15.0):
        """等待指定位姿的保存完成确认"""
        start_time = time.time()
        self.waiting_for_completion = True
        self.current_pose_index = pose_index
        
        print(f"⏳ 等待位姿 {pose_index} 保存完成...")
        
        while (time.time() - start_time < timeout and 
               self.waiting_for_completion and 
               not rospy.is_shutdown()):
            time.sleep(0.1)
        
        if self.waiting_for_completion:
            print(f"❌ 位姿 {pose_index} 保存确认超时")
            self.waiting_for_completion = False
            return False
        else:
            print(f"✅ 位姿 {pose_index} 保存确认完成")
            return True
    
    def set_camera_pose_sync(self, camera_pose_list, is_label=False):
        '''同步设置相机位姿 - 等待每张照片保存完成'''
        print("!!!!!!!!开始同步仿真!!!!!!!!")
        
        # 设置标签模式
        label_msg = Bool()
        label_msg.data = is_label
        self.label_pub.publish(label_msg)
        print(f"设置标签模式: {is_label}")
        time.sleep(2.0)
        
        success_count = 0
        
        for i in range(len(camera_pose_list)):
            print(f"\n=== 处理位姿 {i+1}/{len(camera_pose_list)} ===")
            self.pose_count += 1
            
            # 每15个位姿后清理一次进程
            if self.pose_count % 15 == 0:
                print("🔄 定期系统恢复等待...")
                time.sleep(2.0)
                self.cleanup_gz_processes()
            
            # 步骤1: 暂停仿真，设置位姿
            self.pauseSim()
            time.sleep(0.3)
            
            rospy.wait_for_service('gazebo/set_model_state')
            camera_pose = camera_pose_list[i]
            q = quaternion_from_euler(0, camera_pose[3], camera_pose[4])
            
            # 设置相机位姿
            model_state = ModelState()
            model_state.pose.position.x = camera_pose[0]
            model_state.pose.position.y = camera_pose[1]
            model_state.pose.position.z = camera_pose[2]
            model_state.pose.orientation.x = q[0]
            model_state.pose.orientation.y = q[1]
            model_state.pose.orientation.z = q[2]
            model_state.pose.orientation.w = q[3]
            model_state.model_name = 'ai_camera'

            self.gazebo_model_state_service(model_state)
            print(f"设置相机位姿: ({camera_pose[0]:.2f}, {camera_pose[1]:.2f}, {camera_pose[2]:.2f})")
            
            # 步骤2: 恢复仿真，等待稳定
            self.unpauseSim()
            print("等待场景稳定...")
            time.sleep(2.5)
            
            # 步骤3: 发送保存命令
            control_msg = Int32()
            control_msg.data = i
            self.control_pub.publish(control_msg)
            print(f"发送保存命令，索引: {i}")
            
            # 步骤4: 等待保存完成确认
            if self.wait_for_save_completion(i):
                success_count += 1
                print(f"✅ 位姿 {i} 处理完成")
            else:
                print(f"❌ 位姿 {i} 处理失败")
                
                # 失败重试机制
                print("🔄 尝试重新保存...")
                self.control_pub.publish(control_msg)
                if self.wait_for_save_completion(i, timeout=10.0):
                    success_count += 1
                    print(f"✅ 位姿 {i} 重试成功")
                else:
                    print(f"❌ 位姿 {i} 重试失败")
            
            # 短暂暂停
            time.sleep(0.5)
        
        print(f"\n🎯 本阶段完成: 成功保存 {success_count}/{len(camera_pose_list)} 张图像")
        return success_count
    
    def cleanup_gz_processes(self):
        """清理可能残留的gz进程"""
        try:
            result = subprocess.run(
                "pgrep -f 'gz.*camera\\|gz.*screenshot'", 
                shell=True, 
                capture_output=True, 
                text=True
            )
            
            if result.stdout:
                pids = result.stdout.strip().split('\n')
                for pid in pids:
                    if pid:
                        try:
                            os.kill(int(pid), signal.SIGTERM)
                            print(f"🛑 杀死残留gz进程: {pid}")
                        except:
                            pass
            
            time.sleep(0.5)
            
        except Exception as e:
            print(f"清理gz进程时出错: {e}")
    
    def light_reset(self):
        """重置光照"""
        rospy.wait_for_service('gazebo/set_light_properties')
        light_name = 'sun'
        cast_shadows = True
        diffuse = ColorRGBA()
        dif = 0.15+random.random()*0.55
        diffuse.r = dif
        diffuse.g = dif
        diffuse.b = dif
        diffuse.a = 1.0
        
        self.gazebo_light_service(light_name, diffuse, None, None, None)
        time.sleep(0.1)
    
    def pauseSim(self):
        '''暂停仿真'''
        self.gz_con.pauseSim()
    
    def unpauseSim(self):
        '''继续仿真'''
        self.gz_con.unpauseSim()
        
    def close(self):
        '''关闭仿真'''
        self.cleanup_gz_processes()
        self.gz_env._close()
