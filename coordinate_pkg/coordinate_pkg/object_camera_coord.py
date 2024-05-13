import rclpy
import numpy as np
import math
from rclpy.node import Node
from voxl_msgs.msg import Aidetection
from std_msgs.msg import Float32, Float32MultiArray
from px4_msgs.msg import VehicleLocalPosition
from geometry_msgs.msg import PoseStamped
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
        self.coordinate_data = self.create_publisher(PoseStamped,
                                                     '/object/pose_inertial',10)
        self.local_data = self.create_subscription(VehicleLocalPosition,
                            '/fmu/out/vehicle_local_position', self.local_cb, qos_profile)
        self.subscription_data = self.create_subscription(Aidetection,
                            '/tflite_data', self.listener_cb, qos_profile)
        self.subscription_data
        self.label_boundary = Aidetection()
        self.x = 0.0
        self.y = 0.0
        self._z_inertial = 0.0
        self._scaling_factor = 0.0
        
    def listener_cb(self,msg: Aidetection):
        """Callback function for the /tflite_data subscriber

        Args:
            msg (Aidetection): Containes frame_id and bounding box information
        """
        # self.label_boundary = msg
        center = Float32MultiArray()
        fc = PoseStamped()
        
        # y_center = Float32MultiArray()
        # Calculates the centerpoint of the bounding box
        x_center = msg.x_min + (msg.x_max - msg.x_min)/2
        y_center = msg.y_min + (msg.y_max - msg.y_min)/2
        x_camera, y_camera = self.calculate_camera_coords(x_center, y_center)
        # fc.header.stamp= self.get_clock().now().nanoseconds
        fc.header.frame_id = msg.class_name
        fc.pose.position.x = x_camera
        fc.pose.position.y = y_camera
        fc.pose.position.z = self._scaling_factor
        self.x = x_center
        self.y = y_center
        center.data = [x_center, y_center]
        self.get_logger().info(f"The center point of the object is:({center.data[0]},{center.data[1]})")
        # self.publisher_data.publish(center)
        # Publish data when the object is detected
        if msg.class_name == 'handbag' or msg.class_name == 'backpack':
            self.coordinate_data.publish(fc)
        
    def local_cb (self, msg: VehicleLocalPosition):
        z_inertial = msg.z
        self._scaling_factor = z_inertial/(math.cos(45))
        
        
    def calculate_camera_coords (self, x, y):
        pixel_matrix = np.array([[x],
                                [y],
                                [1]])
        I_00 = 2.797206*1e2
        I_01 = 0.0
        I_02 = 3.545686*1e2
        # I_03 = 0.0
        I_10 = 0.0
        I_11 = 2.797774*1e2
        I_12 = 2.183704*1e2
        # I_13 = 0.0
        I_20 = 0.0
        I_21 = 0.0
        I_22 = 1.0
        # I_23 = 0.0
        
        intrinsic = np.array([[I_00, I_01, I_02],
                             [I_10, I_11, I_12],
                             [I_20, I_21, I_22]])  
        
        inv_intrinsic = np.linalg.inv(intrinsic)
        coord = np.dot(inv_intrinsic,pixel_matrix)
        scale = self._scaling_factor/coord[2][0]
        # scaled_matrix = self._scaling_factor*coord
        x_ = scale*coord[0][0]
        y_ = scale*coord[1][0]
        return x_, y_
        
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
