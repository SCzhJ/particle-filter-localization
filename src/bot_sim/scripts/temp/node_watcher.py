#!/usr/bin/env python3
import subprocess
import time
def get_node_list():
    global node_list
    node_list=subprocess.check_output(['rosnode', 'list'])
def is_node_alive(node_name):
    global node_list
    if(node_name in str(node_list)):
        return True
    # print(output)
    return False
    # finally:
    #     return False

if __name__ == '__main__':
    while True:
        #decisions
        get_node_list()
        print("alive")
        get_node_list
        if is_node_alive('serial_test_receive_node'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'decision', 'decision_serial.launch'])
            time.sleep(1)

        if is_node_alive('decision_node'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'decision', 'decision_node.launch'])
            time.sleep(1)

        #real_robot
        if is_node_alive('real_robot_transform'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'real_robot_transform.launch'])
            time.sleep(1)
        
        #lidar
        if is_node_alive('livox_lidar_publisher2'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'livox_ros_driver2', 'msg_mixed.launch'])
            time.sleep(1)
        
        #lidar_filter
        if is_node_alive('threeD_lidar_filter'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'lidar_filter.launch'])
            time.sleep(1)
        
        #3D22D
        if is_node_alive('pointcloud_to_laserscan'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', '3D22D.launch'])
            time.sleep(1)
        
        if is_node_alive('lidar_monitor'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'LidarMonitor.launch'])
            time.sleep(1)

        #dbscan_bfs_3D
        if is_node_alive('dbscan_bfs_3D'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'dbscan_bfs_3D.launch'])
            time.sleep(1)
        
        #point_lio
        if is_node_alive('laserMapping'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'point_lio', 'mapping_avia.launch'])
            time.sleep(1)

        #amcl
        if is_node_alive('amcl'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'launch_amcl_real.launch'])
            time.sleep(1)

        #ser2msg
        if is_node_alive('ser2msg_write_only'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'ser2msg_tf_write.launch'])
            time.sleep(1)

        #rrtstar
        if is_node_alive('RRT_star_srv'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'rrtstar.launch'])
            time.sleep(1)
        
        #navigate_ctrl
        if is_node_alive('nav_ctrl'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'navigate_ctrl.launch'])
            time.sleep(1)
        
        
        
        time.sleep(10)