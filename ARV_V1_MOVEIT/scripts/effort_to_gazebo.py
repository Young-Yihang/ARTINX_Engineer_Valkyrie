#!/usr/bin/env python3
"""
力矩命令转换节点
功能：将 /effort_controller/commands (Float64MultiArray) 
     转换为 Gazebo 各个关节的力矩命令
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64

class EffortToGazebo(Node):
    def __init__(self):
        super().__init__('effort_to_gazebo')
        
        # 订阅力矩控制器的命令（6个关节的力矩数组）
        self.sub = self.create_subscription(
            Float64MultiArray,
            '/effort_controller/commands',
            self.callback,
            10
        )
        
        # 发布到 Gazebo 各个关节（需要单独的话题）
        self.pubs = []
        joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        for joint_name in joint_names:
            pub = self.create_publisher(
                Float64,
                f'/model/arv_v1/joint/{joint_name}/cmd_force',
                10
            )
            self.pubs.append(pub)
        
        self.get_logger().info('🔄 力矩转换节点启动')
        self.get_logger().info(f'   订阅: /effort_controller/commands')
        self.get_logger().info(f'   发布: /model/arv_v1/joint/*/cmd_force')
    
    def callback(self, msg: Float64MultiArray):
        """接收力矩数组，分发到各个关节"""
        if len(msg.data) != 6:
            self.get_logger().warn(f'力矩数组长度错误: {len(msg.data)} (应为6)')
            return
        
        # 分发到各个关节
        for i, (pub, torque) in enumerate(zip(self.pubs, msg.data)):
            torque_msg = Float64()
            torque_msg.data = torque
            pub.publish(torque_msg)
        
        # 限流打印
        self.get_logger().info(
            f'τ=[{msg.data[0]:.1f}, {msg.data[1]:.1f}, {msg.data[2]:.1f}, '
            f'{msg.data[3]:.1f}, {msg.data[4]:.1f}, {msg.data[5]:.1f}]',
            throttle_duration_sec=1.0
        )

def main(args=None):
    rclpy.init(args=args)
    node = EffortToGazebo()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
