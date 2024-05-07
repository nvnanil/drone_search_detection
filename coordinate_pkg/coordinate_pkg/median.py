import rclpy
from rclpy.node import Node
from voxl_msgs.msg import Aidetection
from std_msgs.msg import Float32, Float32MultiArray
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class ImageData(Node):
    def __init__(self):
        super().__init__('coordinate_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10,)
        self.publisher_data = self.create_publisher(Float32MultiArray,
                                               '/object_center',10)
        self.subscription_data = self.create_subscription(Aidetection,
                            '/tflite_data', self.listener_cb, qos_profile)
        self.subscription_data
        self.label_boundary = Aidetection()
        
    def listener_cb(self,msg: Aidetection):
        """Callback function for the /tflite_data subscriber

        Args:
            msg (Aidetection): Containes frame_id and bounding box information
        """
        # self.label_boundary = msg
        center = Float32MultiArray()
        # y_center = Float32MultiArray()
        # Calculates the centerpoint of the bounding box
        x_center = msg.x_min + (msg.x_max - msg.x_min)/2
        y_center = msg.y_min + (msg.y_max - msg.y_min)/2
        center.data = [x_center, y_center]
        self.get_logger().info(f"The center point of the object is:({center.data[0]},{center.data[1]})")
        self.publisher_data.publish(center)
    
def main(args = None):
    rclpy.init(args=args)
    object = ImageData()
    try:
        rclpy.spin(object)
    except KeyboardInterrupt:
        object.get_logger().info("Keyboard interrupt activated")
    finally:
        object.destroy_node()
        rclpy.shutdown()
    
    # if __name__ == '__main__':
    #     main()
