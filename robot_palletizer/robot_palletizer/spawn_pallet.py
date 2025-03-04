import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose

class DynamicSpawner(Node):
    def __init__(self):
        super().__init__('dynamic_spawner')

        # Declare and get parameters for pallet and object dimensions
        self.declare_parameter('pallet_length', 2.0)  # Default pallet length
        self.declare_parameter('pallet_width', 1.0)   # Default pallet width
        self.declare_parameter('object_size', 0.5)    # Default object size

        # Get parameter values
        pallet_length = self.get_parameter('pallet_length').get_parameter_value().double_value
        pallet_width = self.get_parameter('pallet_width').get_parameter_value().double_value
        object_size = self.get_parameter('object_size').get_parameter_value().double_value

        # Create client for the spawn service
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        # Spawn pallet and object with dynamic dimensions
        self.spawn_pallet_and_object(pallet_length, pallet_width, object_size)

    def spawn_pallet_and_object(self, pallet_length, pallet_width, object_size):
        # Create pallet model (dynamically generated from parameters)
        pallet_sdf = f"""
        <sdf version="1.6">
            <model name="pallet">
                <static>false</static>
                <link name="pallet_link">
                    <visual name="pallet_visual">
                        <geometry>
                            <box>
                                <size>{pallet_length} {pallet_width} 0.2</size>
                            </box>
                        </geometry>
                    </visual>
                    <collision name="pallet_collision">
                        <geometry>
                            <box>
                                <size>{pallet_length} {pallet_width} 0.2</size>
                            </box>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>
        """

        # Create object model
        object_sdf = f"""
        <sdf version="1.6">
            <model name="object">
                <static>false</static>
                <link name="object_link">
                    <visual name="object_visual">
                        <geometry>
                            <box>
                                <size>{object_size} {object_size} {object_size}</size>
                            </box>
                        </geometry>
                    </visual>
                    <collision name="object_collision">
                        <geometry>
                            <box>
                                <size>{object_size} {object_size} {object_size}</size>
                            </box>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>
        """
        
        # Define spawn poses
        pallet_pose = Pose()
        pallet_pose.position.x = 0.0
        pallet_pose.position.y = 0.0
        pallet_pose.position.z = 0.0
        
        object_pose = Pose()
        object_pose.position.x = 1.0  # Object on top of pallet
        object_pose.position.y = 0.0
        object_pose.position.z = 0.5  # Height above the pallet

        # Call the spawn function for pallet and object
        self.spawn_entity('pallet', pallet_sdf, pallet_pose)
        self.spawn_entity('object', object_sdf, object_pose)

    def spawn_entity(self, name, model_sdf, pose):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = model_sdf
        request.initial_pose = pose
        request.reference_frame = 'world'
        
        # Call service to spawn the entity
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Successfully spawned {name}!')
        else:
            self.get_logger().error(f'Failed to spawn {name}')


def main(args=None):
    rclpy.init(args=args)
    node = DynamicSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
