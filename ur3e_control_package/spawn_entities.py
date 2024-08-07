import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

class SpawnEntities(Node):

    def __init__(self):
        super().__init__('spawn_entities')
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')
        self.attach_publisher = self.create_publisher(AttachedCollisionObject, '/attached_collision_object', 10)
        self.clean_start()
        #self.spawn_marker()
        self.spawn_whiteboard()
        #self.attach_marker()

    def clean_start(self):
        self.delete_entity('marker')
        self.delete_entity('whiteboard')

    def delete_entity(self, name):
        request = DeleteEntity.Request()
        request.name = name

        while not self.delete_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /delete_entity not available, waiting again...')

        self.delete_client.call_async(request)

    def spawn_marker(self):
        marker_pose = Pose()
        marker_pose.position.x = 0.0  # Spawn away from the robot initially
        marker_pose.position.y = 0.25
        marker_pose.position.z = 0.7
        marker_pose.orientation = Quaternion(x=0.0, y=0.0, z=1.0, w=0.0)  # Rotate 180 degrees around Z-axis

        request = SpawnEntity.Request()
        request.name = 'marker'
        request.xml = open('/home/bigchungus/ros2_iron_ws/src/move_robot/models/marker.sdf', 'r').read()
        request.robot_namespace = ''
        request.initial_pose = marker_pose
        request.reference_frame = 'world'

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn_entity not available, waiting again...')

        self.spawn_client.call_async(request)

    def spawn_whiteboard(self):
        whiteboard_pose = Pose()
        whiteboard_pose.position.x = 0.43
        whiteboard_pose.position.y = 0.0
        whiteboard_pose.position.z = 0.2
        whiteboard_pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        request = SpawnEntity.Request()
        request.name = 'whiteboard'
        request.xml = open('/home/bigchungus/ros2_iron_ws/src/move_robot/models/whiteboard.sdf', 'r').read()
        request.robot_namespace = ''
        request.initial_pose = whiteboard_pose
        request.reference_frame = 'world'

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /spawn_entity not available, waiting again...')

        self.spawn_client.call_async(request)

    def attach_marker(self):
        attached_object = AttachedCollisionObject()
        attached_object.link_name = 'wrist_3_link'
        attached_object.object.header.frame_id = 'wrist_3_link'
        attached_object.object.id = 'marker'

        # Define the shape and dimensions of the marker
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.001, 0.001, 0.1]  # Example dimensions, adjust as necessary

        # Define the pose of the marker relative to the wrist_3_link
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        pose.position.z = 0.1
        pose.orientation.w = 1.0

        # Add the primitive and pose to the collision object
        attached_object.object.primitives.append(primitive)
        attached_object.object.primitive_poses.append(pose)
        attached_object.object.operation = CollisionObject.ADD

        # Publish the attached collision object
        self.attach_publisher.publish(attached_object)

def main(args=None):
    rclpy.init(args=args)
    spawn_entities = SpawnEntities()
    rclpy.spin(spawn_entities)
    spawn_entities.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
