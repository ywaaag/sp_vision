import rclpy
from rclpy.node import Node
from sp_msgs.msg import EnemyStatusMsg
from builtin_interfaces.msg import Time
import time

class EnemyStatusPublisher(Node):
    def __init__(self):
        super().__init__('enemy_status_publisher')
        self.publisher_ = self.create_publisher(EnemyStatusMsg, '/enemy_status', 10)
        self.start_time = time.time()  # 记录程序启动时间
        self.timer = self.create_timer(1.0, self.publish_status)  # 每秒发布一次

    def publish_status(self):
        msg = EnemyStatusMsg()
        now = time.time()
        msg.timestamp = Time(sec=int(now), nanosec=int((now - int(now)) * 1e9))

        # 运行 10 秒后 invincible_enemy_ids 变为空列表
        if now - self.start_time >= 10:
            msg.invincible_enemy_ids = []
        else:
            msg.invincible_enemy_ids = [1, 3]

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = EnemyStatusPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
