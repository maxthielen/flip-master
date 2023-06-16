import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class UR5MoveNode(Node):
    _instance = None

    def __new__(cls):
        # Singleton class to prevent duplicate node in flipper and scanner classes
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls.instance

    def __init__(self):
        if self._instance is not None:
            return

        super().__init__('ur5_move_node')

        # Create publishers and subscribers (queue size:10)
        self.move_it_pub = self.create_publisher(Bool, 'ur_driver/move_it', 10)
        
    def __del__(self):
        self.destroy_publisher(self.move_it_pub)

    def sweep_scan(self):
        msg = Bool()
        msg.data = True
        # trigger scan
        self.move_it_pub.publish(msg)
        # wait for scan to finish
        rclpy.wait_for_message('ur_driver/sweep_scan', 'done')


class UR5IONode(Node):
    _instance = None

    def __new__(cls):
        # Singleton class to prevent duplicate node in flipper and scanner classes
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls.instance

    def __init__(self, callback=None):
        if self._instance is not None:
            # todo:: enable user to subscribe to input callback without initializing
            if callback is not None:
                self.callbacks.append(callback)
            return

        super().__init__('ur5_io_node')

        self.callbacks = [callback]

        # Create publishers and subscribers (queue size:10)
        self.digital_output_pub = self.create_publisher(Bool, 'ur_driver/digital_output', 10)
        self.digital_input_sub = self.create_subscription(Bool, 'ur_driver/digital_input', self.digital_input_callback, 10)

    def __del__(self):
        self.destroy_publisher(self.digital_output_pub)
        self.destroy_subscription(self.digital_input_sub)

    b

    def digital_input_callback(self, msg):
        value = msg.data
        self.get_logger().info('Digital input value: {}'.format(value))

        for callback in self.callbacks:
            callback()