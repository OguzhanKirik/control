#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[]){
    // Initialize ROS and create the node
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
);
    //Create a Ros logger
    auto const logger = rclcpp::get_logger("hello_moveit");

    // PLAN
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, "manipulator");

    // Set a target pose
    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.orientation.y = 0.8;
        msg.orientation.w = 0.6;
        msg.position.x = 0.1;
        msg.position.y = 0.4;
        msg.position.z =0.4;
        return msg;
    }();
    move_group_interface.setPoseTarget(target_pose);

    // Create collision object Object
    auto const collision_object = [frame_id = 
        move_group_interface.getPlanningFrame()]{
            moveit_msgs::msg::CollisionObject collision_object;
            collision_object.header.frame_id =frame_id;
            collision_object.id = "box1";
            shape_msgs::msg::SolidPrimitive primitive;
            
            // Define the size of the box in meter
            primitive.type = primitive.BOX;
            primitive.dimensions.resize(3);
            primitive.dimensions[primitive.BOX_X] = 0.5;
            primitive.dimensions[primitive.BOX_Y] = 0.1;
            primitive.dimensions[primitive.BOX_Z] = 0.5;

            //Define the pose of the box (relatvie to the frame_id)
            geometry_msgs::msg::Pose box_pose;
            box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
            box_pose.position.x = 0.2;
            box_pose.position.y = 0.2;
            box_pose.position.z = 0.25;
            
            collision_object.primitives.push_back(primitive);
            collision_object.primitive_poses.push_back(box_pose);
            collision_object.operation = collision_object.ADD;
            
            return collision_object;
        };
    //ADD the collision object to the Planning Scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_object);

    // Creta a plan to that taget pose
    auto const [success, plan] = [&move_group_interface]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(move_group_interface.plan(msg));
        return std::make_pair(ok,msg);
    }();

    // Execute 
    if(success) move_group_interface.execute(plan);
    else RCLCPP_ERROR(logger, "Planning failed!");
    rclcpp::shutdown();
    return 0;
}
