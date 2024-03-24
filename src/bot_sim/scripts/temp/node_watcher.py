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

        if is_node_alive('decision_node'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'decision', 'decision_node.launch'])

        #ser2msg
        if is_node_alive('ser2msg'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'ser2msg_write.launch'])
        
        #lidar
        if is_node_alive('livox_lidar_publisher2'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'livox_ros_driver2', 'msg_mixed.launch'])
        
        #lidar_filter
        if is_node_alive('threeD_lidar_filter'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'lidar_filter.launch'])
        
        #3D22D
        if is_node_alive('pointcloud_to_laserscan'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', '3D22D.launch'])

        #dbscan_bfs_3D
        if is_node_alive('dbscan_bfs_3D'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'dbscan_bfs_3D.launch'])
        
        #point_lio
        if is_node_alive('laserMapping'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'point_lio', 'mapping_avia.launch'])

        #amcl
        if is_node_alive('amcl'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'launch_amcl_real.launch'])
        #navigate_ctrl
        if is_node_alive('nav_ctrl'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'navigate_ctrl.launch'])
        
        #rrtstar
        if is_node_alive('RRT_star_srv'):
            pass
        else:
            subprocess.Popen(['roslaunch', 'bot_sim', 'rrtstar.launch'])
        time.sleep(0.01)