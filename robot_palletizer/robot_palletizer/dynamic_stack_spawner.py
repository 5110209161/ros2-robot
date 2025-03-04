import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from robot_palletizer_data.srv import SetStackDimension
from geometry_msgs.msg import Pose


class DynamicStackSpawner(Node):
    def __init__(self):
        super().__init__('dynamic_stack_spawner')
        # declare and get parameters for pallet and products
        self.declare_parameter('pallet_length', 0.8)
        self.declare_parameter('pallet_width', 1.2)
        self.declare_parameter('pallet_height', 0.1)

        self.declare_parameter('product_length', 0.4)
        self.declare_parameter('product_width', 0.3)
        self.declare_parameter('product_height', 0.2)

        # entity spawner
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # create service to set dimensions
        self.size_srv = self.create_service(SetStackDimension, '/set_stack_dimension', self.set_dimensions_callback)

        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service ...')

        # Spawn pallet and product with dynamic dimensions
        self.spawn_pallet_and_product()
    
    def set_dimensions_callback(self, request, response):
        # Update parameters with the service call values, but use default if missing
        pallet_length = request.pallet_length if request.pallet_length is not None else self.pallet_length
        pallet_width = request.pallet_width if request.pallet_width is not None else self.pallet_width
        pallet_height= request.pallet_height if request.pallet_height is not None else self.pallet_height

        product_length = request.product_length if request.product_length is not None else self.product_length
        product_width = request.product_width if request.product_width is not None else self.product_width
        product_height = request.product_height if request.product_height is not None else self.product_height
        
        # Update parameters with the service call values
        self.set_parameters([
            rclpy.parameter.Parameter('pallet_length', rclpy.Parameter.Type.DOUBLE, request.pallet_length),
            rclpy.parameter.Parameter('pallet_width', rclpy.Parameter.Type.DOUBLE, request.pallet_width),
            rclpy.parameter.Parameter('pallet_height', rclpy.Parameter.Type.DOUBLE, request.pallet_height),
            rclpy.parameter.Parameter('product_length', rclpy.Parameter.Type.DOUBLE, request.product_length),
            rclpy.parameter.Parameter('product_width', rclpy.Parameter.Type.DOUBLE, request.product_width),
            rclpy.parameter.Parameter('product_height', rclpy.Parameter.Type.DOUBLE, request.product_height),
        ])
        # update dimensions
        self.get_logger().info('Set dimensions of pallet and product')
        # remove old entities
        self.remove_entity('pallet')
        self.remove_entity('product')

        self.spawn_pallet_and_product()
        response.success = True
        return response

    def remove_entity(self, entity_name):
        # Create a request to delete the entity
        request = DeleteEntity.Request()
        request.name = entity_name
        
        # Call the delete service to remove the old entity
        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Successfully deleted {entity_name}!')
        else:
            self.get_logger().error(f'Failed to delete {entity_name}')

    def spawn_pallet_and_product(self):
        # get parameter values
        pallet_length = self.get_parameter('pallet_length').get_parameter_value().double_value
        pallet_width = self.get_parameter('pallet_width').get_parameter_value().double_value
        pallet_height = self.get_parameter('pallet_height').get_parameter_value().double_value

        product_length = self.get_parameter('product_length').get_parameter_value().double_value
        product_width = self.get_parameter('product_width').get_parameter_value().double_value
        product_height = self.get_parameter('product_height').get_parameter_value().double_value

        # Create pallet model (dynamically generated from parameters)
        pallet_sdf = f"""
        <sdf version="1.6">
            <model name="pallet">
                <static>false</static>
                <link name="pallet_link">
                    <visual name="pallet_visual">
                        <geometry>
                            <box>
                                <size>{pallet_length} {pallet_width} {pallet_height}</size>
                            </box>
                        </geometry>
                    </visual>
                    <collision name="pallet_collision">
                        <geometry>
                            <box>
                                <size>{pallet_length} {pallet_width} {pallet_height}</size>
                            </box>
                        </geometry>
                    </collision>
                </link>
            </model>
        </sdf>
        """

        # Create product model
        product_sdf = f"""
        <sdf version="1.6">
            <model name="product">
                <static>false</static>
                <link name="product_link">
                    <visual name="product_visual">
                        <geometry>
                            <box>
                                <size>{product_length} {product_width} {product_height}</size>
                            </box>
                        </geometry>
                    </visual>
                    <collision name="product_collision">
                        <geometry>
                            <box>
                                <size>{product_length} {product_width} {product_height}</size>
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
        
        product_pose = Pose()
        product_pose.position.x = 1.0  # product on top of pallet
        product_pose.position.y = 0.0
        product_pose.position.z = 0.5  # Height above the pallet

        # Call the spawn function for pallet and product
        self.spawn_entity('pallet', pallet_sdf, pallet_pose)
        self.spawn_entity('product', product_sdf, product_pose)
    
    def spawn_entity(self, name, model_sdf, pose):
        request = SpawnEntity.Request()
        request.name = name
        request.xml = model_sdf
        request.initial_pose = pose
        request.reference_frame = 'world'
        # call service to spawn entity
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Successfully spawned {name}!')
        else:
            self.get_logger().error(f'Failed to spawn {name}')

def main(args=None):
    rclpy.init(args=args)
    node = DynamicStackSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()