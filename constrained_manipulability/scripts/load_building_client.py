#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import Mesh
from constrained_manipulability_interfaces.srv import AddRemoveCollisionMesh

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose

from geometric_shapes import shape_operations as so
from shapes import ShapeMsg
import numpy as np

def load_mesh_msg(uri: str, scale=(1.0, 1.0, 1.0)) -> Mesh:
    shape = so.create_mesh_from_resource(uri, np.array(scale, dtype=float))
    if shape is None:
        raise RuntimeError(f"Failed to load mesh: {uri}")
    sm = ShapeMsg()
    so.construct_msg_from_shape(shape, sm)
    mesh_msg = sm.mesh  # ShapeMsg is a variant, here we expect a Mesh
    if mesh_msg is None:
        raise RuntimeError("Loaded shape is not a triangle mesh")
    return mesh_msg

def main():
    rclpy.init()
    node = Node('add_building_mesh_client_py')

    # --- USER INPUTS ---
    mesh_uri = "package://my_world/meshes/building.stl"  # or file:///abs/path/building.obj
    mesh_in_mm = True  # set True if STL is authored in millimeters
    world_frame = "world"
    target_frame = "base_link"  # current server expects base_link
    pose_world = PoseStamped()
    pose_world.header.frame_id = world_frame
    pose_world.pose.position.x = 12.34
    pose_world.pose.position.y = -7.89
    pose_world.pose.position.z = 0.0
    pose_world.pose.orientation.w = 1.0  # no rotation for test
    object_id = 151

    scale = (0.001, 0.001, 0.001) if mesh_in_mm else (1.0, 1.0, 1.0)
    mesh_msg = load_mesh_msg(mesh_uri, scale)

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node, spin_thread=True)

    # Wait for TF and transform pose world->base_link
    node.get_logger().info(f"Transforming pose {world_frame} -> {target_frame}")
    tf_buffer.can_transform(target_frame, world_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=2.0))
    T_w_to_bl = tf_buffer.lookup_transform(target_frame, world_frame, rclpy.time.Time())
    pose_in_bl = do_transform_pose(pose_world, T_w_to_bl)

    client = node.create_client(AddRemoveCollisionMesh, '/add_remove_collision_mesh')
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error("Service not available")
        return

    req = AddRemoveCollisionMesh.Request()
    req.mesh = mesh_msg
    req.pose = pose_in_bl.pose        # pose now is in base_link
    req.object_id = object_id
    req.remove = False

    fut = client.call_async(req)
    rclpy.spin_until_future_complete(node, fut)
    resp = fut.result()
    node.get_logger().info(f"Service result: {resp.result}")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
