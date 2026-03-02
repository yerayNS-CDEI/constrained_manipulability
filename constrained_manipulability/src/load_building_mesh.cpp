// #include <rclcpp/rclcpp.hpp>
// #include <geometric_shapes/shape_operations.h>
// #include <resource_retriever/retriever.hpp>
// #include <shape_msgs/msg/mesh.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <constrained_manipulability_interfaces/srv/add_remove_collision_mesh.hpp>
// #include <Eigen/Core>
// #include <Eigen/Geometry>

// // Helper: load mesh from URI into shape_msgs::msg::Mesh
// shape_msgs::msg::Mesh loadMeshMsgFromResource(const std::string& uri,
//                                               const Eigen::Vector3d& scale = Eigen::Vector3d::Ones())
// {
//   // createMeshFromResource supports package://, file://, http(s)://
//   shapes::ShapePtr shape_ptr(shapes::createMeshFromResource(uri, scale));
//   if (!shape_ptr) {
//     throw std::runtime_error("Failed to load mesh from: " + uri);
//   }

//   // Convert shapes::Shape -> shapes::ShapeMsg -> shape_msgs::msg::Mesh
//   shapes::ShapeMsg shape_msg;
//   shapes::constructMsgFromShape(shape_ptr.get(), shape_msg);

//   // shapes::ShapeMsg is a boost::variant; extract Mesh
//   auto mesh_msg = boost::get<shape_msgs::msg::Mesh>(&shape_msg);
//   if (!mesh_msg) {
//     throw std::runtime_error("Loaded shape is not a triangle mesh: " + uri);
//   }
//   return *mesh_msg;
// }

// int main(int argc, char** argv)
// {
//   rclcpp::init(argc, argv);
//   auto node = rclcpp::Node::make_shared("load_building_mesh_client");

//   // --- Example inputs ---
//   const std::string mesh_uri =  "file:///home/yeray/ws_collision/src/example_building.stl"; // "package://my_world/meshes/building.stl"; // or file:///abs/path/building.obj
//   const bool mesh_in_mm = true;  // set true if STL units are millimeters
//   const Eigen::Vector3d scale = mesh_in_mm  ? Eigen::Vector3d(0.001, 0.001, 0.001)
//                                             : Eigen::Vector3d(1.0, 1.0, 1.0);

//   // Pose in base_link frame
//   geometry_msgs::msg::Pose pose;
//   pose.position.x = 0.0; pose.position.y = 0.0; pose.position.z = 0.0;
//   pose.orientation.x = 0.0; pose.orientation.y = 0.0; pose.orientation.z = 0.0; pose.orientation.w = 1.0;

//   // Load mesh
//   shape_msgs::msg::Mesh mesh_msg = loadMeshMsgFromResource(mesh_uri, scale);

//   // Prepare service client
//   using AddRemove = constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh;
//   auto client = node->create_client<AddRemove>("/add_remove_collision_mesh");

//   if (!client->wait_for_service(std::chrono::seconds(5))) {
//     RCLCPP_ERROR(node->get_logger(), "Service not available");
//     return 1;
//   }

//   auto req = std::make_shared<AddRemove::Request>();
//   req->mesh = mesh_msg;
//   req->pose = pose;              // Pose must be in base_link frame (as per your server)
//   req->object_id = 999;          // Choose a stable ID for this building
//   req->remove = false;           // Add (set true to remove later)

//   auto future = client->async_send_request(req);
//   if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
//     RCLCPP_ERROR(node->get_logger(), "Service call failed");
//     return 1;
//   }
//   RCLCPP_INFO(node->get_logger(), "Result: %s", future.get()->result ? "OK" : "FAILED");

//   rclcpp::shutdown();
//   return 0;
// }



#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <constrained_manipulability_interfaces/srv/add_remove_collision_mesh.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <Eigen/Geometry>
#include <limits>

// --- Helpers ---

// Load a mesh from resource (package://, file://, http(s)://) and convert to shape_msgs::Mesh with scale
static shape_msgs::msg::Mesh loadMeshMsgFromResource(const std::string& uri,
                                                     const Eigen::Vector3d& scale)
{
  shapes::Mesh* raw_mesh = shapes::createMeshFromResource(uri, scale);
  if (!raw_mesh) {
    throw std::runtime_error("Failed to load mesh: " + uri);
  }
  shapes::ShapePtr shape_ptr(raw_mesh);  // take ownership
  shapes::ShapeMsg shape_variant;
  shapes::constructMsgFromShape(shape_ptr.get(), shape_variant);

  auto mesh_ptr = boost::get<shape_msgs::msg::Mesh>(&shape_variant);
  if (!mesh_ptr) {
    throw std::runtime_error("Loaded resource is not a triangle mesh: " + uri);
  }
  return *mesh_ptr;
}

// Compute axis-aligned bounding box of a shape_msgs::Mesh (in its local coordinates)
static void computeMeshBounds(const shape_msgs::msg::Mesh& mesh,
                              Eigen::Vector3d& min_xyz,
                              Eigen::Vector3d& max_xyz)
{
  double minx= std::numeric_limits<double>::infinity();
  double miny= std::numeric_limits<double>::infinity();
  double minz= std::numeric_limits<double>::infinity();
  double maxx=-std::numeric_limits<double>::infinity();
  double maxy=-std::numeric_limits<double>::infinity();
  double maxz=-std::numeric_limits<double>::infinity();

  for (const auto& v : mesh.vertices) {
    minx = std::min(minx, (double)v.x);
    miny = std::min(miny, (double)v.y);
    minz = std::min(minz, (double)v.z);
    maxx = std::max(maxx, (double)v.x);
    maxy = std::max(maxy, (double)v.y);
    maxz = std::max(maxz, (double)v.z);
  }
  min_xyz = Eigen::Vector3d(minx, miny, minz);
  max_xyz = Eigen::Vector3d(maxx, maxy, maxz);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("add_building_mesh_client_cpp");

  // --- USER INPUTS ---
  // 1) Mesh resource (STL/OBJ)
  const std::string mesh_uri = "file:///home/yeray/ws_collision/src/example_building.stl"; // o file:///abs/path/building.obj

  // 2) Units: set true if STL is authored in millimeters
  const bool mesh_in_mm = false;

  // 3) Pose in WORLD (we will transform to base_link because server expects base_link)
  const std::string source_frame = "world";
  const std::string target_frame = "base_link";  // servicio actual
  geometry_msgs::msg::PoseStamped pose_world;
  pose_world.header.frame_id = source_frame;
  pose_world.pose.position.x = 0.0;
  pose_world.pose.position.y = 0.0;
  pose_world.pose.position.z = 0.0;
  pose_world.pose.orientation.x = 0.0;
  pose_world.pose.orientation.y = 0.0;
  pose_world.pose.orientation.z = 0.0;
  pose_world.pose.orientation.w = 1.0;

  const int object_id = 150;
  // --- END USER INPUTS ---

  // TF buffer/listener to transform world -> base_link
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  auto tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

  // Load and scale mesh
  Eigen::Vector3d scale = mesh_in_mm ? Eigen::Vector3d(0.001, 0.001, 0.001)
                                     : Eigen::Vector3d(1.0, 1.0, 1.0);
  shape_msgs::msg::Mesh mesh_msg = loadMeshMsgFromResource(mesh_uri, scale);

  // Log triangles/vertices and bounds to catch size issues
  Eigen::Vector3d bb_min, bb_max;
  computeMeshBounds(mesh_msg, bb_min, bb_max);
  RCLCPP_INFO(node->get_logger(), "Mesh vertices: %zu, triangles: %zu",
              mesh_msg.vertices.size(), mesh_msg.triangles.size());
  RCLCPP_INFO(node->get_logger(), "Mesh AABB (m): min[%.3f %.3f %.3f], max[%.3f %.3f %.3f]",
              bb_min.x(), bb_min.y(), bb_min.z(), bb_max.x(), bb_max.y(), bb_max.z());

  // Transform pose world -> base_link
  pose_world.header.stamp = node->now();
  geometry_msgs::msg::PoseStamped pose_in_bl;  // solo UNA vez, fuera del try

  try {
    // Usa transform con timeout y evita canTransform + lookup separados
    pose_in_bl = tf_buffer->transform(pose_world, target_frame, tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_ERROR(node->get_logger(), "TF transform %s -> %s failed: %s",
                 source_frame.c_str(), target_frame.c_str(), ex.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(),
              "Pose in base_link: [%.3f, %.3f, %.3f] q=[%.3f, %.3f, %.3f, %.3f]",
              pose_in_bl.pose.position.x, pose_in_bl.pose.position.y, pose_in_bl.pose.position.z,
              pose_in_bl.pose.orientation.x, pose_in_bl.pose.orientation.y,
              pose_in_bl.pose.orientation.z, pose_in_bl.pose.orientation.w);

  // geometry_msgs::msg::PoseStamped pose_in_bl;
  // pose_in_bl.header.frame_id = "base_link";
  // pose_in_bl.pose.position.x = 1.0;  // 1 m delante
  // pose_in_bl.pose.position.y = 0.0;
  // pose_in_bl.pose.position.z = 0.0;  // para que el minZ (~-0.470) quede a ras de suelo
  // pose_in_bl.pose.orientation.w = 1.0;

  // Prepare service client
  using AddRemove = constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh;
  auto client = node->create_client<AddRemove>("/collision/add_remove_collision_mesh");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "Service /add_remove_collision_mesh not available");
    rclcpp::shutdown();
    return 1;
  }

  auto req = std::make_shared<AddRemove::Request>();
  req->mesh = mesh_msg;
  req->pose = pose_in_bl.pose;  // IMPORTANT: pose now in base_link
  req->object_id = object_id;
  req->remove = false;

  auto future = client->async_send_request(req);
  if (rclcpp::spin_until_future_complete(node, future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Service result: %s", future.get()->result ? "OK" : "FAILED");
  rclcpp::shutdown();
  return 0;
}