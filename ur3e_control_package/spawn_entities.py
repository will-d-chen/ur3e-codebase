import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


class SpawnEntities(Node):
    def __init__(self):
        super().__init__('spawn_entities')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SpawnEntity.Request()

    def spawn_box(self):
        # Box size
        box_size = "0.1 0.1 0.05"  # width, length, height

        # Box pose
        box_pose = Pose()
        box_pose.position.x = 0.0
        box_pose.position.y = 0.3
        box_pose.position.z = 0.0
        q = [1.0, 0.0, 0.0 ,0.0]
        box_pose.orientation.x = q[0]
        box_pose.orientation.y = q[1]
        box_pose.orientation.z = q[2]
        box_pose.orientation.w = q[3]

        # Box model XML
        model_xml = f"""
        <?xml version="1.0"?>
        <sdf version="1.6">
          <model name="box">
            <pose>0 0 0 0 0 0</pose>
            <link name="box_link">
              <inertial>
                <mass>1.0</mass>
                <inertia>
                  <ixx>0.083</ixx>
                  <ixy>0.0</ixy>
                  <ixz>0.0</ixz>
                  <iyy>0.083</iyy>
                  <iyz>0.0</iyz>
                  <izz>0.083</izz>
                </inertia>
              </inertial>
              <collision name="box_collision">
                <geometry>
                  <box>
                    <size>{box_size}</size>
                  </box>
                </geometry>
              </collision>
              <visual name="box_visual">
                <geometry>
                  <box>
                    <size>{box_size}</size>
                  </box>
                </geometry>
                <material>
                  <ambient>1 0 0 1</ambient>
                  <diffuse>1 0 0 1</diffuse>
                  <specular>0.1 0.1 0.1 1</specular>
                  <emissive>0 0 0 0</emissive>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """

        self.req.name = "box"
        self.req.xml = model_xml
        self.req.initial_pose = box_pose

        future = self.client.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Box spawned successfully')
        else:
            self.get_logger().error('Failed to spawn box')

def main(args=None):
    rclpy.init(args=args)
    box_spawner = SpawnEntities()
    box_spawner.spawn_box()
    box_spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()