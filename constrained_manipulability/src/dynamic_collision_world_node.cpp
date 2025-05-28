// // File: dynamic_collision_world_node.cpp

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <geometry_msgs/msg/pose.hpp>
// #include <visualization_msgs/msg/marker_array.hpp>
// #include <geometric_shapes/shape_to_marker.h>
// #include <geometric_shapes/mesh_operations.h>
// #include <urdf/model.h>
// #include <kdl_parser/kdl_parser.hpp>
// #include <tf2_kdl/tf2_kdl.hpp>
// #include <tf2_eigen/tf2_eigen.hpp>
// #include <fcl/fcl.h>

// #include "robot_collision_checking/fcl_interface.hpp"
// #include "robot_collision_checking/fcl_interface_collision_world.hpp"
// #include "robot_collision_checking/fcl_interface_types.hpp"

// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/jntarray.hpp>

// #include <shape_msgs/msg/mesh.hpp>
// #include <shape_msgs/msg/mesh_triangle.hpp>
// #include <geometry_msgs/msg/point.hpp>

// #include <memory>
// #include <cassert>
// #include <boost/variant.hpp> 
// #include <geometric_shapes/shapes.h>
// #include <geometric_shapes/shape_operations.h>
// #include <geometric_shapes/mesh_operations.h>
// #include <geometric_shapes/shape_messages.h>

// #include <map>

// class DynamicCollisionWorldNode : public rclcpp::Node
// {
// public:
//     DynamicCollisionWorldNode() : Node("dynamic_collision_world_node")
//     {
//         auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");

//         if (!param_client->wait_for_service(std::chrono::seconds(2))) {
//         RCLCPP_ERROR(get_logger(), "Parameter service not available");
//         rclcpp::shutdown();
//         return;
//         }

//         auto params = param_client->get_parameters({"robot_description"});
//         if (params.empty()) {
//         RCLCPP_ERROR(get_logger(), "robot_description not found in robot_state_publisher");
//         rclcpp::shutdown();
//         return;
//         }

//         std::string robot_description = params[0].as_string();

//         if (!model_.initString(robot_description)) {
//         RCLCPP_ERROR(get_logger(), "Failed to parse URDF");
//         rclcpp::shutdown();
//         return;
//         }

//         if (!kdl_parser::treeFromUrdfModel(model_, kdl_tree_))
//         {
//             RCLCPP_ERROR(get_logger(), "Failed to parse KDL tree from URDF");
//             rclcpp::shutdown();
//             return;
//         }

//         if (!kdl_tree_.getChain("base_link", "tool0", kdl_chain_))
//         {
//             RCLCPP_ERROR(get_logger(), "Failed to extract KDL chain from base_link to tool0");
//             rclcpp::shutdown();
//             return;
//         }

//         // Suscripción a JointStates para actualizar poses
//         joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
//             "/joint_states", rclcpp::SensorDataQoS(),
//             std::bind(&DynamicCollisionWorldNode::jointStateCallback, this, std::placeholders::_1));

//         marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/visualization_marker_array", 10);

//         collision_world_ = std::make_shared<robot_collision_checking::FCLInterfaceCollisionWorld>("world");

//         addStaticBox();

//         // Cargar geometría de colisión del robot y crear objetos FCL
//         loadRobotCollisionGeometry();

//         timer_ = create_wall_timer(
//             std::chrono::milliseconds(500), std::bind(&DynamicCollisionWorldNode::timerCallback, this));
//     }

// private:
//     urdf::Model model_;
//     KDL::Tree kdl_tree_;
//     KDL::Chain kdl_chain_;

//     sensor_msgs::msg::JointState latest_joint_state_;
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     std::shared_ptr<robot_collision_checking::FCLInterfaceCollisionWorld> collision_world_;

//     // Map link_name -> objeto FCL para el robot
//     std::map<std::string, std::shared_ptr<robot_collision_checking::FCLObject>> robot_collision_objects_;

//     void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
//     {
//         latest_joint_state_ = *msg;

//         // Actualiza poses de colisión del robot con el estado actual
//         updateRobotCollisionPoses();
//     }

//     void updateRobotCollisionPoses()
//     {
//         if (latest_joint_state_.name.empty())
//             return;

//         std::map<std::string, double> joint_positions;
//         for (size_t i = 0; i < latest_joint_state_.name.size(); ++i)
//             joint_positions[latest_joint_state_.name[i]] = latest_joint_state_.position[i];

//         KDL::ChainFkSolverPos_recursive fk_solver(kdl_chain_);
//         KDL::JntArray joint_array(kdl_chain_.getNrOfJoints());

//         for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
//         {
//             const auto& segment = kdl_chain_.getSegment(i);
//             const std::string& joint_name = segment.getJoint().getName();

//             if (joint_positions.find(joint_name) != joint_positions.end())
//             {
//                 unsigned int joint_index = getJointIndexInChain(joint_name);
//                 if (joint_index < joint_array.rows())
//                 {
//                     joint_array(joint_index) = joint_positions[joint_name];
//                 }
//             }
//         }

//         for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
//         {
//             KDL::Frame frame;
//             fk_solver.JntToCart(joint_array, frame, i + 1); // i+1 porque 0 es base_link

//             std::string link_name = kdl_chain_.getSegment(i).getName();

//             auto it = robot_collision_objects_.find(link_name);
//             if (it != robot_collision_objects_.end())
//             {
//                 Eigen::Affine3d affine;
//                 geometry_msgs::msg::Pose pose_msg = tf2::toMsg(frame);
//                 tf2::fromMsg(pose_msg, affine);

//                 it->second->object_transform = affine;
//             }
//         }
//     }

//     // Devuelve índice del joint en la cadena KDL
//     unsigned int getJointIndexInChain(const std::string& joint_name)
//     {
//         for (unsigned int i = 0; i < kdl_chain_.getNrOfSegments(); ++i)
//         {
//             if (kdl_chain_.getSegment(i).getJoint().getName() == joint_name)
//                 return i;
//         }
//         return kdl_chain_.getNrOfSegments(); // valor inválido
//     }

//     void timerCallback()
//     {
//         auto marker_array = std::make_shared<visualization_msgs::msg::MarkerArray>();
//         auto objects = collision_world_->getCollisionObjects();

//         for (const auto& obj : objects)
//         {
//             visualization_msgs::msg::Marker marker;
//             marker.ns = "obstacles";
//             marker.header.frame_id = "world";
//             marker.action = visualization_msgs::msg::Marker::ADD;
//             marker.id = obj->collision_id;
//             marker.pose = tf2::toMsg(obj->object->object_transform);

//             std_msgs::msg::ColorRGBA color;
//             color.r = 1.0; color.g = 0.0; color.b = 0.0; color.a = 1.0;
//             marker.color = color;
//             marker.lifetime = rclcpp::Duration(0, 0);

//             std::string type = obj->object->getTypeString();
//             if (type == "SPHERE" || type == "BOX" || type == "CYLINDER" || type == "CONE")
//             {
//                 geometric_shapes::constructMarkerFromShape(*obj->object->ptr.solid, marker);
//                 marker_array->markers.push_back(marker);
//             }
//         }

//         int id_counter = 1000;
//         for (const auto& [link_name, fcl_obj] : robot_collision_objects_)
//         {
//             visualization_msgs::msg::Marker marker;
//             marker.ns = "robot_collision";
//             marker.header.frame_id = "world";
//             marker.action = visualization_msgs::msg::Marker::ADD;
//             marker.id = id_counter++;
//             marker.pose = tf2::toMsg(fcl_obj->object_transform);

//             std_msgs::msg::ColorRGBA color;
//             color.r = 0.0; color.g = 0.5; color.b = 1.0; color.a = 0.6;
//             marker.color = color;
//             marker.lifetime = rclcpp::Duration(0, 0);

//             geometric_shapes::constructMarkerFromShape(*fcl_obj->ptr.solid, marker);
//             marker_array->markers.push_back(marker);
//         }

//         marker_pub_->publish(*marker_array);
//     }

//     void addStaticBox()
//     {
//         shape_msgs::msg::SolidPrimitive box;
//         box.type = shape_msgs::msg::SolidPrimitive::BOX;
//         box.dimensions = {0.5, 0.5, 0.5};

//         Eigen::Affine3d box_pose = Eigen::Affine3d::Identity();
//         box_pose.translation() << 1.0, 0.0, 0.25;

//         auto box_obj = std::make_shared<robot_collision_checking::FCLObject>(box, box_pose);
//         int static_box_id = 10000;
//         collision_world_->addCollisionObject(box_obj, static_box_id);
//     }
//     void convertShapesMeshToMsg(const shapes::Mesh* shapes_mesh, shape_msgs::msg::Mesh &mesh_msg)
//     {
//         // Convertir vértices
//         mesh_msg.vertices.clear();
//         mesh_msg.vertices.reserve(shapes_mesh->vertex_count);
//         for (size_t i = 0; i < shapes_mesh->vertex_count; ++i)
//         {
//             geometry_msgs::msg::Point p;
//             p.x = shapes_mesh->vertices[3 * i];
//             p.y = shapes_mesh->vertices[3 * i + 1];
//             p.z = shapes_mesh->vertices[3 * i + 2];
//             mesh_msg.vertices.push_back(p);
//         }

//         // Convertir triángulos
//         mesh_msg.triangles.clear();
//         mesh_msg.triangles.reserve(shapes_mesh->triangle_count);
//         for (size_t i = 0; i < shapes_mesh->triangle_count; ++i)
//         {
//             shape_msgs::msg::MeshTriangle tri;
//             tri.vertex_indices[0] = shapes_mesh->triangles[3 * i];
//             tri.vertex_indices[1] = shapes_mesh->triangles[3 * i + 1];
//             tri.vertex_indices[2] = shapes_mesh->triangles[3 * i + 2];
//             mesh_msg.triangles.push_back(tri);
//         }
//     }

//     // Construye shapes::Shape desde urdf::Geometry (completo para SPHERE, BOX, CYLINDER, MESH)
//     std::shared_ptr<shapes::Shape> constructShape(const urdf::Geometry* geom) const
//     {
//         if (!geom)
//             return nullptr;

//         switch (geom->type)
//         {
//             case urdf::Geometry::SPHERE:
//                 return std::make_shared<shapes::Sphere>(dynamic_cast<const urdf::Sphere*>(geom)->radius);

//             case urdf::Geometry::BOX:
//             {
//                 urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(geom)->dim;
//                 return std::make_shared<shapes::Box>(dim.x, dim.y, dim.z);
//             }

//             case urdf::Geometry::CYLINDER:
//             {
//                 auto cyl = dynamic_cast<const urdf::Cylinder*>(geom);
//                 return std::make_shared<shapes::Cylinder>(cyl->radius, cyl->length);
//             }

//             case urdf::Geometry::MESH:
//             {
//                 auto mesh = dynamic_cast<const urdf::Mesh*>(geom);
//                 Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
//                 return std::shared_ptr<shapes::Shape>(shapes::createMeshFromResource(mesh->filename, scale));
//             }

//             default:
//                 RCLCPP_WARN(this->get_logger(), "Unsupported geometry type: %d", static_cast<int>(geom->type));
//                 return nullptr;
//         }
//     }

//     // Carga geometría de colisión del robot y crea objetos FCL usando geometric_shapes
//     void loadRobotCollisionGeometry()
//     {
//         int id_counter = 0;

//         for (const auto& link_pair : model_.links_)
//         {
//             const urdf::LinkSharedPtr& link = link_pair.second;
//             if (!link->collision)
//                 continue;

//             const urdf::CollisionSharedPtr& collision = link->collision;
//             Eigen::Affine3d collision_pose = Eigen::Affine3d::Identity();

//             // Convertir pose urdf::Pose a Eigen::Affine3d
//             collision_pose.translation() << collision->origin.position.x,
//                                             collision->origin.position.y,
//                                             collision->origin.position.z;

//             Eigen::Quaterniond q(
//                 collision->origin.rotation.w,
//                 collision->origin.rotation.x,
//                 collision->origin.rotation.y,
//                 collision->origin.rotation.z);
//             collision_pose.linear() = q.toRotationMatrix();

//             // Construir la forma usando geometric_shapes
//             std::shared_ptr<shapes::Shape> shape = constructShape(collision->geometry.get());

//             if (!shape)
//             {
//                 RCLCPP_WARN(get_logger(), "Failed to construct shape for link: %s", link->name.c_str());
//                 continue;
//             }

//             // Convertir geometric_shapes::Shape a shape_msgs::ShapeMsg para crear FCLObject
//             shapes::ShapeMsg shape_msg;
//             shapes::constructMsgFromShape(shape.get(), shape_msg);

//             // robot_collision_checking::ShapeType shape_type = robot_collision_checking::ShapeType::BOX;

//             // Determinar ShapeType para FCLObject según el tipo del mensaje
//             if (shape_msg.which() == 0)  // SolidPrimitive
//             {
//                 const auto* solid = boost::get<shape_msgs::msg::SolidPrimitive>(&shape_msg);
//                 if (!solid) {
//                     RCLCPP_WARN(get_logger(), "Failed to get SolidPrimitive");
//                     continue;
//                 }
//                 // Según el tipo dentro de shape_msg.solid, asigna uno de los tipos de enum
//                 switch (solid->type)
//                 {
//                     case shape_msgs::msg::SolidPrimitive::BOX:
//                         shape_type = robot_collision_checking::ShapeType::BOX;
//                         break;
//                     case shape_msgs::msg::SolidPrimitive::SPHERE:
//                         shape_type = robot_collision_checking::ShapeType::SPHERE;
//                         break;
//                     case shape_msgs::msg::SolidPrimitive::CYLINDER:
//                         shape_type = robot_collision_checking::ShapeType::CYLINDER;
//                         break;
//                     case shape_msgs::msg::SolidPrimitive::CONE:
//                         shape_type = robot_collision_checking::ShapeType::CONE;
//                         break;
//                     default:
//                         RCLCPP_WARN(get_logger(), "Unknown SolidPrimitive type for link: %s", link->name.c_str());
//                         continue;
//                 }
//                 auto fcl_obj = std::make_shared<robot_collision_checking::FCLObject>(*solid, collision_pose);
//                 robot_collision_objects_[link->name] = fcl_obj;
//                 collision_world_->addCollisionObject(fcl_obj, id_counter++);
//             }
//             else if (shape_msg.which() == 1) // Mesh
//             {
//                 const auto* mesh = boost::get<shape_msgs::msg::Mesh>(&shape_msg);
//                 if (!mesh) {
//                     RCLCPP_WARN(get_logger(), "Failed to get Mesh");
//                     continue;
//                 }
//                 shape_type = robot_collision_checking::ShapeType::MESH;

//                 auto fcl_obj = std::make_shared<robot_collision_checking::FCLObject>(*mesh, shape_type, collision_pose);
//                 robot_collision_objects_[link->name] = fcl_obj;
//                 collision_world_->addCollisionObject(fcl_obj, id_counter++);
//             }
//             else if (shape_msg.which() == 2) // Plane
//             {
//                 const auto* plane = boost::get<shape_msgs::msg::Plane>(&shape_msg);
//                 if (!plane) {
//                     RCLCPP_WARN(get_logger(), "Failed to get Plane");
//                     continue;
//                 }
//                 shape_type = robot_collision_checking::ShapeType::PLANE;
                
//                 // Crear el objeto FCL
//                 auto fcl_obj = std::make_shared<robot_collision_checking::FCLObject>(*plane, shape_type, collision_pose);
//                 robot_collision_objects_[link->name] = fcl_obj;
//                 collision_world_->addCollisionObject(fcl_obj, id_counter++);
//             }
//             else
//             {
//                 RCLCPP_WARN(get_logger(), "Unsupported shape message type for link: %s", link->name.c_str());
//                 continue;
//             }
//         }
//     }

// };

int main(int argc, char * argv[])
{
//   rclcpp::init(argc, argv);
//   auto node = std::make_shared<DynamicCollisionWorldNode>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();
//   return 0;
}
