// dynamic_scene_node.cpp
#include <memory>
#include <thread>
#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <shape_msgs/msg/plane.hpp>
#include <shape_msgs/msg/mesh.hpp>
#include <nav2_msgs/msg/voxel_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <geometric_shapes/shape_to_marker.h>

#include <Eigen/Eigen>
#include <fcl/fcl.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "robot_collision_checking/fcl_interface_types.hpp"
#include "robot_collision_checking/fcl_interface_collision_world.hpp"
#include "robot_collision_checking/fcl_interface.hpp"

#include "constrained_manipulability_interfaces/srv/add_remove_collision_solid.hpp"
#include "constrained_manipulability_interfaces/srv/add_remove_collision_mesh.hpp"

#include "constrained_manipulability_interfaces/srv/update_collision_pose.hpp"

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <rclcpp/parameter_client.hpp>
#include <geometric_shapes/shape_operations.h>
#include <Eigen/Geometry>
#include <tf2_kdl/tf2_kdl.hpp> 
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/jntarray.hpp>

#include <sensor_msgs/msg/joint_state.hpp>


struct GeometryInformation
{
    std::vector<shapes::ShapeConstPtr> shapes;
    std::vector<Eigen::Affine3d> geometry_transforms;
};
GeometryInformation extractCollisionGeometryFromURDF(const urdf::Model& urdf_model)
{
    GeometryInformation geometry_info;

    for (const auto& link_pair : urdf_model.links_)
    {
        const urdf::LinkSharedPtr& link = link_pair.second;

        if (link->collision)
        {
            urdf::CollisionSharedPtr collision = link->collision;
            urdf::GeometrySharedPtr geom = collision->geometry;

            if (!geom)
                continue;

            shapes::ShapeConstPtr shape_ptr;

            switch (geom->type)
            {
                case urdf::Geometry::BOX:
                {
                    urdf::Box* box = dynamic_cast<urdf::Box*>(geom.get());
                    shape_ptr = std::make_shared<shapes::Box>(box->dim.x, box->dim.y, box->dim.z);
                    break;
                }
                case urdf::Geometry::CYLINDER:
                {
                    urdf::Cylinder* cylinder = dynamic_cast<urdf::Cylinder*>(geom.get());
                    shape_ptr = std::make_shared<shapes::Cylinder>(cylinder->radius, cylinder->length);
                    break;
                }
                case urdf::Geometry::SPHERE:
                {
                    urdf::Sphere* sphere = dynamic_cast<urdf::Sphere*>(geom.get());
                    shape_ptr = std::make_shared<shapes::Sphere>(sphere->radius);
                    break;
                }
                case urdf::Geometry::MESH:
                {
                    urdf::Mesh* mesh = dynamic_cast<urdf::Mesh*>(geom.get());
                    shape_ptr = shapes::ShapePtr(shapes::createMeshFromResource(mesh->filename));
                    break;
                }
                default:
                    continue;
            }

            Eigen::Affine3d geom_tf = Eigen::Affine3d::Identity();

            // No chequeo porque estos no son punteros o booleanos
            geom_tf.translation() = Eigen::Vector3d(
                collision->origin.position.x,
                collision->origin.position.y,
                collision->origin.position.z);

            double roll, pitch, yaw;
            collision->origin.rotation.getRPY(roll, pitch, yaw);
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            geom_tf.linear() = (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

            geometry_info.shapes.push_back(shape_ptr);
            geometry_info.geometry_transforms.push_back(geom_tf);
        }
    }

    return geometry_info;
}

class DynamicSceneNode : public rclcpp::Node
{
public:
    DynamicSceneNode() : Node("dynamic_scene_node")
    {
        this->declare_parameter<std::string>("robot_description", "");  // Por si no está declarado

        if (!loadURDFAndKDL())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model. Exiting...");
            rclcpp::shutdown();
            return;
        }
        
        if (!kdl_tree_.getChain(root_link_, tip_link_, kdl_chain_)) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get KDL chain from %s to %s", root_link_.c_str(), tip_link_.c_str());
            rclcpp::shutdown();
            return;
        }
        fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

        collision_world_ = std::make_shared<robot_collision_checking::FCLInterfaceCollisionWorld>("world");
        auto geometry_info = extractCollisionGeometryFromURDF(urdf_model_);
        createRobotCollisionModel(geometry_info);

        mkr_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/fcl_world_markers", 3);
        
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DynamicSceneNode::jointStateCallback, this, std::placeholders::_1));


        srv_ = this->create_service<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid>(
            "add_remove_collision_solid",
            std::bind(&DynamicSceneNode::handle_command, this, std::placeholders::_1, std::placeholders::_2));
        srv_mesh_ = this->create_service<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh>(
            "add_remove_collision_mesh",
            std::bind(&DynamicSceneNode::handle_mesh_command, this, std::placeholders::_1, std::placeholders::_2));
        srv_update_pose_ = this->create_service<constrained_manipulability_interfaces::srv::UpdateCollisionPose>(
            "update_collision_pose",
            std::bind(&DynamicSceneNode::handle_update_pose, this, std::placeholders::_1, std::placeholders::_2));

        init_example_objects();
        RCLCPP_INFO(this->get_logger(), "Dynamic scene node started.");
    }

    void update()
    {
        auto marker_array_msg = std::make_shared<visualization_msgs::msg::MarkerArray>();
        std::vector<robot_collision_checking::FCLInterfaceCollisionObjectPtr> world_objects = collision_world_->getCollisionObjects();
        std::string world_frame = collision_world_->getWorldFrame();
        int num_objects = collision_world_->getNumObjects();

        for (int i = 0; i < num_objects; /*i++*/)
        {
            auto world_obj = world_objects[i];
            // Make a marker
            visualization_msgs::msg::Marker mkr;
            mkr.ns = "collision_objects";
            mkr.header.frame_id = world_frame;
            mkr.action = visualization_msgs::msg::Marker::ADD;
            mkr.lifetime = rclcpp::Duration(0, 0);
            std::string obj_type = world_obj->object->getTypeString();

            // Get object pose relative to world_frame
            Eigen::Affine3d object_eig_pose = world_obj->object->object_transform;
            geometry_msgs::msg::Pose object_geo_pose;
            robot_collision_checking::fcl_interface::convertEigenTransformGeometryPose(object_eig_pose, object_geo_pose);
            mkr.pose = object_geo_pose;

            if ((obj_type == "BOX" || obj_type == "CYLINDER" || obj_type == "SPHERE") &&
                (world_obj->object->ptr.solid->dimensions.empty() ||
                    std::any_of(world_obj->object->ptr.solid->dimensions.begin(), world_obj->object->ptr.solid->dimensions.end(), [](double d) { return d <= 0.0; })))
            {
                RCLCPP_WARN(this->get_logger(), "Invalid solid dimensions for object ID %d. Skipping marker.", world_obj->collision_id);
                continue;
            }

            if (obj_type == "MESH")
            {
                geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.mesh), mkr);
                
                // Blue mesh
                mkr.id = world_obj->collision_id;
                mkr.color.b = 1.0;
                mkr.color.a = 1.0;
                marker_array_msg->markers.push_back(mkr);
                i++;
            }
            else if (obj_type == "OCTOMAP")
            {
                RCLCPP_WARN(this->get_logger(), "Unable to display octomap");
                i++;
            }
            else if (obj_type == "SPHERE" || obj_type == "BOX" || obj_type == "CYLINDER" || obj_type == "CONE")
            {
                geometric_shapes::constructMarkerFromShape(*(world_obj->object->ptr.solid), mkr);

                // Green primitives
                mkr.id = world_obj->collision_id;
                mkr.color.g = 1.0;
                mkr.color.a = 1.0;
                marker_array_msg->markers.push_back(mkr);
                i++;
            }
            else if (obj_type == "PLANE")
            {
                mkr.scale.x = 10.0;
                mkr.scale.y = 10.0;
                mkr.scale.z = 0.001; // very thin

                // Red cuboid
                mkr.type = visualization_msgs::msg::Marker::CUBE;
                mkr.id = world_obj->collision_id;
                mkr.color.r = 1.0;
                mkr.color.a = 0.3;
                marker_array_msg->markers.push_back(mkr);
                i++;
            }
            else if (obj_type == "VOXEL_GRID")
            {
                mkr.id = world_obj->collision_id;
                // Treat voxel grid as a cube list
                mkr.type = visualization_msgs::msg::Marker::CUBE_LIST;
                auto grid = *(world_obj->object->ptr.voxel_grid);
                mkr.scale.x = grid.resolutions.x;
                mkr.scale.y = grid.resolutions.y;
                mkr.scale.z = grid.resolutions.z;
                // Voxel cells already account for position in world
                mkr.pose.position.x = mkr.pose.position.y = mkr.pose.position.z = 0.0;

                // Iterate over remaining cells until no more objects in world or a new collision object
                do
                {
                    // The collision object is really a voxel cell
                    auto voxel_cell = *(world_objects[i]->collision_object);
                    fcl::Vector3d cell_center = voxel_cell.getTranslation();
                    // Invert rotation to obtain cell position in original world frame
                    Eigen::Matrix3d rotation_matrix = voxel_cell.getRotation().matrix();
                    rotation_matrix.transposeInPlace();
                    cell_center = rotation_matrix * cell_center;
                    geometry_msgs::msg::Point point;
                    point.x = cell_center[0];
                    point.y = cell_center[1];
                    point.z = cell_center[2];
                    mkr.points.push_back(point);
                    i++;
                } while ((i < num_objects) && (world_objects[i-1]->collision_id == world_objects[i]->collision_id));

                // Purple voxel grid
                mkr.color.r = 1.0;
                mkr.color.b = 1.0;
                mkr.color.a = 1.0;
                marker_array_msg->markers.push_back(mkr);
            }
            
            std::vector<int> collision_object_ids;

            bool is_collision = collision_world_->checkCollisionObject(world_obj->collision_id, collision_object_ids);
            if (is_collision)
            {
                for (int obj_id : collision_object_ids)
                {
                    if (obj_id == world_obj->collision_id)
                        continue;

                    RCLCPP_INFO(this->get_logger(), "%s with ID %d in collision with object with ID %d", 
                                obj_type.c_str(), world_obj->collision_id, obj_id);
                }
            }
        }
        mkr_pub_->publish(*marker_array_msg);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    urdf::Model urdf_model_;
    KDL::Tree kdl_tree_;
    KDL::Chain kdl_chain_;
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::string root_link_ = "base_link";  // Cambia si necesitas otro root
    std::string tip_link_ = "tool0";       // Cambia si necesitas otro tip

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() < (size_t)kdl_chain_.getNrOfJoints()) {
            RCLCPP_WARN(this->get_logger(), "JointState message has insufficient joint positions");
            return;
        }

        updateRobotCollisionPoses(msg->position);
        update();  // Publicar markers y chequear colisiones
    }

    void updateRobotCollisionPoses(const std::vector<double> & joint_positions)
    {
        KDL::JntArray joint_array(kdl_chain_.getNrOfJoints());
        for (size_t i = 0; i < joint_array.rows(); i++) {
            joint_array(i) = joint_positions[i];
        }

        int collision_obj_index = 0;

        for (size_t i = 0; i < kdl_chain_.getNrOfSegments(); i++)
        {
            KDL::Frame segment_frame;
            if (fk_solver_->JntToCart(joint_array, segment_frame, i + 1) >= 0)
            {
                Eigen::Affine3d eigen_pose;
                geometry_msgs::msg::Pose pose_msg = tf2::toMsg(segment_frame);
                tf2::fromMsg(pose_msg, eigen_pose);

                collision_world_->updateCollisionObjectPose(collision_obj_index, eigen_pose);
                collision_obj_index++;
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to compute FK for segment %zu", i);
            }
        }
    }

    bool loadURDFAndKDL()
    {
        RCLCPP_INFO(this->get_logger(), "Trying to get robot_description from /robot_state_publisher");

        // Crear cliente para parámetros remotos del nodo robot_state_publisher
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(this, "robot_state_publisher");

        // Esperar a que el servicio de parámetros esté disponible (máx 5 segundos)
        if (!param_client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Parameter service /robot_state_publisher not available");
            return false;
        }

        std::string urdf_xml_string;

        // Esperar hasta obtener un string URDF no vacío
        while (rclcpp::ok()) {
            auto params = param_client->get_parameters({"robot_description"});
            if (!params.empty() && !params[0].as_string().empty()) {
                urdf_xml_string = params[0].as_string();
                break;
            }
            RCLCPP_WARN(this->get_logger(), "robot_description parameter empty, waiting...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // Parsear el URDF
        if (!urdf_model_.initString(urdf_xml_string)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF");
            return false;
        }

        // Crear KDL tree a partir del modelo URDF
        if (!kdl_parser::treeFromUrdfModel(urdf_model_, kdl_tree_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree");
            return false;
        }

        KDL::Chain kdl_chain;
        if (!kdl_tree_.getChain(root_link_, tip_link_, kdl_chain)) {
            RCLCPP_FATAL(this->get_logger(), "Robot has 0 joints, check if root and/or tip name is incorrect!");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Number of tree segments: %u", kdl_tree_.getNrOfSegments());
        RCLCPP_INFO(this->get_logger(), "Number of tree joints: %u", kdl_tree_.getNrOfJoints());
        RCLCPP_INFO(this->get_logger(), "Number of chain joints: %u", kdl_chain.getNrOfJoints());
        RCLCPP_INFO(this->get_logger(), "Number of chain segments: %u", kdl_chain.getNrOfSegments());

        return true;
    }

    void createRobotCollisionModel(const GeometryInformation& geometry_info)
    {
        int num_shapes = geometry_info.shapes.size();
        for (int i = 0; i < num_shapes; i++)
        {
            Eigen::Affine3d obj_pose = geometry_info.geometry_transforms[i];
            shapes::ShapeConstPtr shape = geometry_info.shapes[i];

            robot_collision_checking::FCLObjectPtr obj = nullptr;

            if (const shapes::Box* box = dynamic_cast<const shapes::Box*>(shape.get()))
            {
                shape_msgs::msg::SolidPrimitive solid;
                solid.type = shape_msgs::msg::SolidPrimitive::BOX;
                solid.dimensions.resize(3);
                solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_X] = box->size[0];
                solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Y] = box->size[1];
                solid.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] = box->size[2];
                obj = std::make_shared<robot_collision_checking::FCLObject>(solid, obj_pose);
            }
            else if (const shapes::Cylinder* cylinder = dynamic_cast<const shapes::Cylinder*>(shape.get()))
            {
                shape_msgs::msg::SolidPrimitive solid;
                solid.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
                solid.dimensions.resize(2);
                solid.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = cylinder->length;
                solid.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = cylinder->radius;
                obj = std::make_shared<robot_collision_checking::FCLObject>(solid, obj_pose);
            }
            else if (const shapes::Sphere* sphere = dynamic_cast<const shapes::Sphere*>(shape.get()))
            {
                shape_msgs::msg::SolidPrimitive solid;
                solid.type = shape_msgs::msg::SolidPrimitive::SPHERE;
                solid.dimensions.resize(1);
                solid.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = sphere->radius;
                obj = std::make_shared<robot_collision_checking::FCLObject>(solid, obj_pose);
            }
            else if (const shapes::Mesh* mesh = dynamic_cast<const shapes::Mesh*>(shape.get()))
            {
                shapes::ShapeMsg shape_msg;
                if (!shapes::constructMsgFromShape(mesh, shape_msg)) {
                    RCLCPP_WARN(this->get_logger(), "Failed to construct ShapeMsg from mesh");
                    continue;
                }

                shape_msgs::msg::Mesh mesh_msg;
                try {
                    mesh_msg = boost::get<shape_msgs::msg::Mesh>(shape_msg);
                }
                catch (const boost::bad_get &ex) {
                    RCLCPP_WARN(this->get_logger(), "ShapeMsg does not contain a mesh");
                    continue;
                }

                obj = std::make_shared<robot_collision_checking::FCLObject>(mesh_msg, robot_collision_checking::MESH, obj_pose);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Unsupported shape type in robot collision model");
                continue;
            }

            if (obj)
            {
                collision_world_->addCollisionObject(obj, i);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Robot collision model created with %d shapes", num_shapes);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mkr_pub_;
    rclcpp::Service<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid>::SharedPtr srv_;
    rclcpp::Service<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh>::SharedPtr srv_mesh_;
    rclcpp::Service<constrained_manipulability_interfaces::srv::UpdateCollisionPose>::SharedPtr srv_update_pose_;
    std::shared_ptr<robot_collision_checking::FCLInterfaceCollisionWorld> collision_world_;

    void handle_command(
        const std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid::Request> request,
        std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionSolid::Response> response)
    {
        Eigen::Affine3d pose_eigen;
        tf2::fromMsg(request->pose, pose_eigen);
        RCLCPP_INFO(this->get_logger(), "Received request: type=%d, dimensions=%zu", request->solid.type, request->solid.dimensions.size());
        auto object = std::make_shared<robot_collision_checking::FCLObject>(request->solid, pose_eigen);
        if (!object) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create FCLObject from request.");
            response->result = false;
            return;
        }

        if (request->remove)
        {
            collision_world_->removeCollisionObject(request->object_id);
            RCLCPP_INFO(this->get_logger(), "Removed object with ID %d", request->object_id);
            response->result = true;

            // Publish DELETE marker to RViz
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = collision_world_->getWorldFrame();
            delete_marker.header.stamp = this->now();
            delete_marker.ns = "collision_objects";
            delete_marker.id = request->object_id;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;

            visualization_msgs::msg::MarkerArray delete_array;
            delete_array.markers.push_back(delete_marker);
            mkr_pub_->publish(delete_array);
        }
        else
        {
            collision_world_->addCollisionObject(object, request->object_id);
            RCLCPP_INFO(this->get_logger(), "Added object with ID %d", request->object_id);
            response->result = true;
        }
    }

    void handle_mesh_command(
        const std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh::Request> req,
        std::shared_ptr<constrained_manipulability_interfaces::srv::AddRemoveCollisionMesh::Response> res)
    {
        res->result = false;

        if (req->remove)
        {
            collision_world_->removeCollisionObject(req->object_id);
            RCLCPP_INFO(this->get_logger(), "Removed mesh with ID %d", req->object_id);
        }
        else
        {
            Eigen::Affine3d mesh_T;
            tf2::fromMsg(req->pose, mesh_T);

            auto world_obj = std::make_shared<robot_collision_checking::FCLObject>(req->mesh, robot_collision_checking::MESH, mesh_T);
            if (!world_obj)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to create FCLObject for mesh ID %d", req->object_id);
                return;
            }
            collision_world_->addCollisionObject(world_obj, req->object_id);
            RCLCPP_INFO(this->get_logger(), "Added mesh with ID %d", req->object_id);
        }

        // Publish DELETE or ADD marker to RViz
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = collision_world_->getWorldFrame();
        marker.header.stamp = this->now();
        marker.ns = "collision_objects";
        marker.id = req->object_id;

        if (req->remove)
        {
            marker.action = visualization_msgs::msg::Marker::DELETE;
        }
        else
        {
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = req->pose;
            geometric_shapes::constructMarkerFromShape(req->mesh, marker);
            marker.color.b = 1.0;
            marker.color.a = 1.0;
        }

        visualization_msgs::msg::MarkerArray array;
        array.markers.push_back(marker);
        mkr_pub_->publish(array);

        res->result = true;
    }

    void handle_update_pose(
        const std::shared_ptr<constrained_manipulability_interfaces::srv::UpdateCollisionPose::Request> request,
        std::shared_ptr<constrained_manipulability_interfaces::srv::UpdateCollisionPose::Response> response)
    {
        Eigen::Affine3d new_pose;
        tf2::fromMsg(request->pose, new_pose);
        response->result = collision_world_->updateCollisionObjectPose(request->object_id, new_pose);
    }

    void init_example_objects()
    {
        // Create some Eigen transforms in the world frame
        Eigen::Vector3d eig_wps1(0.0, 0.0, 0.0), eig_wps2(-1.3, 2.0, 0.3), eig_wps4(-1.4, 1.9, 0.35), eig_wps5(1.2, -1.1, 0.2);
        Eigen::Matrix3d eig_identity = Eigen::Matrix3d::Identity();
        Eigen::Affine3d eig_wTs1, eig_wTs2, eig_wTs3, eig_wTs4, eig_wTs5;

        eig_wTs1.linear() = eig_identity;
        eig_wTs1.translation() = eig_wps1;
        eig_wTs2.linear() = eig_identity;
        eig_wTs2.translation() = eig_wps2;

        Eigen::Quaterniond q1(0.5, 0.5, 0.23, 0.43);
        q1.normalize();
        Eigen::Quaterniond q2(-0.5, 0.5, -1.23, 0.43);
        q2.normalize();

        eig_wTs4.linear() = q1.toRotationMatrix();
        eig_wTs4.translation() = eig_wps4;
        eig_wTs5.linear() = q2.toRotationMatrix();
        eig_wTs5.translation() = eig_wps5;

        // Add a collection of objects
        std::vector<robot_collision_checking::FCLObjectPtr> fcl_objects;
        std::vector<int> object_ids = {100, 101, 102, 103, 104};

        // Sphere
        shape_msgs::msg::SolidPrimitive sphere;
        sphere.dimensions.resize(1);
        sphere.dimensions[shape_msgs::msg::SolidPrimitive::SPHERE_RADIUS] = 0.3;
        sphere.type = shape_msgs::msg::SolidPrimitive::SPHERE;
        fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(sphere, eig_wTs1));

        // Cylinder
        shape_msgs::msg::SolidPrimitive cylinder;
        cylinder.dimensions.resize(2);
        cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_HEIGHT] = 1.0;
        cylinder.dimensions[shape_msgs::msg::SolidPrimitive::CYLINDER_RADIUS] = 0.1;
        cylinder.type = shape_msgs::msg::SolidPrimitive::CYLINDER;
        fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(cylinder, eig_wTs2));

        // Plane Ax+By+Cz+D=0
        shape_msgs::msg::Plane plane;
        plane.coef[0] = 1.0;
        plane.coef[1] = 2.0;
        plane.coef[2] = 0.0;
        // D takes this value to satisfy Ax+By+D=0
        plane.coef[3] = -(plane.coef[0] * plane.coef[0] + plane.coef[1] * plane.coef[1]);
        // The coefficients A, B, C give the normal to the plane
        Eigen::Vector3d n(plane.coef[0], plane.coef[1], plane.coef[2]);
        // Plane is centered at this point
        double distance = plane.coef[3] / n.norm();
        eig_wTs3.translation() = -distance * n.normalized();
        // Calculate the rotation matrix from the original normal z_0 = (0,0,1) to new normal n = (A,B,C)
        Eigen::Vector3d z_0 = Eigen::Vector3d::UnitZ();
        Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(z_0, n);
        eig_wTs3.linear() = q.toRotationMatrix();
        fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
            plane, robot_collision_checking::PLANE, eig_wTs3));

        // Mesh
        shape_msgs::msg::Mesh mesh;
        // Define vertices of the mesh
        geometry_msgs::msg::Point p1, p2, p3, p4, p5;
        p1.x = 0.0; p1.y = 0.0; p1.z = 1.0;   // Apex of the pyramid
        p2.x = 1.0; p2.y = 0.0; p2.z = 0.0;   // Base vertex 1
        p3.x = -1.0; p3.y = 0.0; p3.z = 0.0;  // Base vertex 2
        p4.x = 0.0; p4.y = 1.0; p4.z = 0.0;   // Base vertex 3
        p5.x = 0.0; p5.y = -1.0; p5.z = 0.0;  // Base vertex 4
        mesh.vertices.push_back(p1);
        mesh.vertices.push_back(p2);
        mesh.vertices.push_back(p3);
        mesh.vertices.push_back(p4);
        mesh.vertices.push_back(p5);
        // Define the faces of the mesh
        shape_msgs::msg::MeshTriangle t1, t2, t3, t4, t5, t6;
        t1.vertex_indices = {0, 1, 2};
        t2.vertex_indices = {0, 1, 3};
        t3.vertex_indices = {0, 2, 3};
        t4.vertex_indices = {0, 3, 4};
        t5.vertex_indices = {1, 2, 4}; 
        t6.vertex_indices = {1, 3, 4};  
        mesh.triangles.push_back(t1);
        mesh.triangles.push_back(t2);
        mesh.triangles.push_back(t3);
        mesh.triangles.push_back(t4);
        mesh.triangles.push_back(t5);
        mesh.triangles.push_back(t6);
        fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
            mesh, robot_collision_checking::MESH, eig_wTs4));

        // Voxel grid
        int size_x = 4, size_y = 2, size_z = 3;
        nav2_voxel_grid::VoxelGrid vg(size_x, size_y, size_z);
        // Mark all cells
        for (int x_grid = 0; x_grid < size_x; ++x_grid)
        {
            for (int y_grid = 0; y_grid < size_y; ++y_grid)
            {
                for (int z_grid = 0; z_grid < size_z; ++z_grid)
                {
                    vg.markVoxel(x_grid, y_grid, z_grid);
                }
            }
        }
        // Create the VoxelGrid msg
        nav2_msgs::msg::VoxelGrid grid_msg;
        grid_msg.origin.x = 0.0;
        grid_msg.origin.y = 0.0;
        grid_msg.origin.z = 0.0;
        grid_msg.size_x = size_x;
        grid_msg.size_y = size_y;
        grid_msg.size_z = size_z;
        grid_msg.resolutions.x = 0.5;
        grid_msg.resolutions.y = 0.5;
        grid_msg.resolutions.z = 0.5;
        grid_msg.data.resize(size_x * size_y);
        memcpy(&grid_msg.data[0], vg.getData(), size_x * size_y * sizeof(int));

        fcl_objects.push_back(std::make_shared<robot_collision_checking::FCLObject>(
            grid_msg, robot_collision_checking::VOXEL_GRID, eig_wTs5));
            
        collision_world_->addCollisionObjects(fcl_objects, object_ids);
    }    
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicSceneNode>();

    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
