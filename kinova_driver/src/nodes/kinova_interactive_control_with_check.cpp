#include <ros/ros.h>

#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <kinova/KinovaTypes.h>
#include "kinova_driver/kinova_ros_types.h"
#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"

#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"


#include <actionlib/client/simple_action_client.h>

#include <algorithm>
#include <math.h>

using namespace visualization_msgs;
using namespace interactive_markers;

// Create actionlib client
typedef actionlib::SimpleActionClient<kinova_msgs::ArmJointAnglesAction> ArmJoint_actionlibClient;
typedef actionlib::SimpleActionClient<kinova_msgs::ArmPoseAction> ArmPose_actionlibClient;
typedef actionlib::SimpleActionClient<kinova_msgs::SetFingersPositionAction> Finger_actionlibClient;

// %Tag(vars)%
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> armPose_interMark_server;
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> armJoint_interMark_server;
interactive_markers::MenuHandler menu_handler;

kinova::KinovaAngles current_joint_command;
kinova::KinovaPose current_pose_command;

geometry_msgs::PoseStamped pose_target_point_marker;

std::string tf_prefix_;
std::string kinova_robotType_;
char robot_category_;
int robot_category_version_;
char wrist_type_;
int arm_joint_number_;
char robot_mode_;
int finger_number_;
int joint_total_number_;
float marker_scale_;

bool mouse_was_up = true;
bool getCurrentCommand = false;
// %EndTag(vars)%

// declare processFeedback(), action when interative marker is clicked.
void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

bool areValuesClose(float first, float second, float tolerance) {
    return ((first <= second + tolerance) && (first >= second - tolerance));
}

Marker makeBox(InteractiveMarker &msg) {
    Marker marker;

    marker.type = Marker::SPHERE;
    marker.scale.x = msg.scale * 0.3;
    marker.scale.y = msg.scale * 0.3;
    marker.scale.z = msg.scale * 0.3;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_scale_ = msg.scale;
    return marker;
}

InteractiveMarkerControl& makeBoxControl(InteractiveMarker &msg) {
    InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(makeBox(msg));
    msg.controls.push_back(control);

    return msg.controls.back();
}
// %EndTag(Box)%

// %Tag(1DOF)%

void make1DofMarker(const std::string& frame_id, const std::string& axis, unsigned int interaction_mode, const tf::Vector3& position, const std::string& description, const std::string& name) {

    InteractiveMarker int_marker;
    int_marker.header.frame_id = frame_id;
    tf::pointTFToMsg(position, int_marker.pose.position);
    int_marker.scale = 0.08;
    int_marker.name = name;
    int_marker.description = description;

    // insert a box
    makeBoxControl(int_marker);
    InteractiveMarkerControl control;

    if (interaction_mode == InteractiveMarkerControl::ROTATE_AXIS)
        control.name = "rotate";
    else if (interaction_mode == InteractiveMarkerControl::MOVE_AXIS)
        control.name = "move";
    else
        ROS_INFO("interactive mode should be eigher ROTATE_AXIS or MOVE_AXIS");

    if (axis == "x") {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name += "_x";
    } else if (axis == "y") {
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name += "_y";
    } else if (axis == "z") {
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name += "_z";
    } else
        ROS_INFO("\n The rotation axis must be x, y or z. \n");

    control.interaction_mode = interaction_mode;
    int_marker.controls.push_back(control);

    armJoint_interMark_server->insert(int_marker);
    //    armJoint_interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
    armJoint_interMark_server->setCallback(int_marker.name, &processFeedback);
}
// %EndTag(1DOF)%position


// %Tag(6DOF)%

void make6DofMarker(bool fixed, unsigned int interaction_mode, const tf::Pose pose, bool show_6dof) {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = tf_prefix_ + "_link_base";
    tf::poseTFToMsg(pose, int_marker.pose);
    int_marker.scale = 0.1;
    int_marker.name = "cartesian_6dof";
    int_marker.description = "6-DOF Cartesian Control";

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if (fixed) {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (show_6dof) {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    armPose_interMark_server->insert(int_marker);
    //  armPose_interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
    armPose_interMark_server->setCallback(int_marker.name, &processFeedback);

}

void make6DofMarkerTarget(bool fixed, unsigned int interaction_mode, const tf::Pose pose, bool show_6dof) {
    InteractiveMarker int_marker;
    int_marker.header.frame_id = tf_prefix_ + "_link_base";
    tf::poseTFToMsg(pose, int_marker.pose);
    int_marker.scale = 0.1;
    int_marker.name = "TEST";
    int_marker.description = "Marker 2";

    // insert a box
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = interaction_mode;

    InteractiveMarkerControl control;

    if (fixed) {
        int_marker.name += "_fixed";
        int_marker.description += "\n(fixed orientation)";
        control.orientation_mode = InteractiveMarkerControl::FIXED;
    }

    if (show_6dof) {
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "rotate_x";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_x";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "rotate_y";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_y";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);

        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "rotate_z";
        control.interaction_mode = InteractiveMarkerControl::ROTATE_AXIS;
        int_marker.controls.push_back(control);
        control.name = "move_z";
        control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
        int_marker.controls.push_back(control);
    }

    armPose_interMark_server->insert(int_marker);
    //  armPose_interMark_server->setCallback(int_marker.name, &processFeedback, visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP);
    armPose_interMark_server->setCallback(int_marker.name, &processFeedback);
}

// %EndTag(6DOF)%


// %Tag(send actionlib goals)%

void sendFingerGoal(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    Finger_actionlibClient client("/" + tf_prefix_ + "_driver/fingers/finger_positions", true);
    kinova_msgs::SetFingersPositionGoal goal;

    client.waitForServer();
    // limit the range of marker to [0 10] before mapping to finger position
    float markerPos;
    float maxMarkerPose = 2.0f;
    markerPos = std::min(maxMarkerPose, std::max(0.0f, float(feedback->pose.position.x)));
    // map marker position to gripper position
    goal.fingers.finger1 = markerPos / maxMarkerPose * 5000;
    goal.fingers.finger2 = markerPos / maxMarkerPose * 5000;
    goal.fingers.finger3 = 0.0;
    client.sendGoal(goal);

    ROS_INFO("client send goal to Finger actionlib: %f \n", goal.fingers.finger1);
}

void sendArmPoseGoal(geometry_msgs::PoseStamped &endeffector_pose) {
    ArmPose_actionlibClient client("/" + tf_prefix_ + "_driver/pose_action/tool_pose", true);
    kinova_msgs::ArmPoseGoal goal;
    client.waitForServer();
    //Tu wpisać pozycje 
    goal.pose = endeffector_pose;

    ROS_INFO_STREAM("Goal parent frame is : " << goal.pose.header.frame_id);

    ROS_INFO_STREAM("Goal to arm pose actionlib: \n"
            << "  x: " << goal.pose.pose.position.x
            << ", y: " << goal.pose.pose.position.y
            << ", z: " << goal.pose.pose.position.z
            << ", qx: " << goal.pose.pose.orientation.x
            << ", qy: " << goal.pose.pose.orientation.y
            << ", qz: " << goal.pose.pose.orientation.z
            << ", qw: " << goal.pose.pose.orientation.w << std::endl);

    client.sendGoal(goal);

}

void sendArmJointGoal(const std::string marker_name, double joint_offset) {
    ArmJoint_actionlibClient client("/" + tf_prefix_ + "_driver/joints_action/joint_angles", true);
    kinova_msgs::ArmJointAnglesGoal goal;

    goal.angles.joint1 = current_joint_command.Actuator1;
    goal.angles.joint2 = current_joint_command.Actuator2;
    goal.angles.joint3 = current_joint_command.Actuator3;
    goal.angles.joint4 = current_joint_command.Actuator4;
    if (arm_joint_number_ == 4) {
        goal.angles.joint5 = 0.0;
        goal.angles.joint6 = 0.0;
    } else if (arm_joint_number_ == 6) {
        goal.angles.joint5 = current_joint_command.Actuator5;
        goal.angles.joint6 = current_joint_command.Actuator6;
    }

    client.waitForServer();

    // map marker position to the position of arm joint.
    switch (*marker_name.rbegin() - '0') {
        case 1:
            goal.angles.joint1 += joint_offset;
            break;
        case 2:
            goal.angles.joint2 += joint_offset;
            break;
        case 3:
            goal.angles.joint3 += joint_offset;
            break;
        case 4:
            goal.angles.joint4 += joint_offset;
            break;
        case 5:
            if (arm_joint_number_ == 6) {
                goal.angles.joint5 += joint_offset;
            }
            break;
        case 6:
            if (arm_joint_number_ == 6) {
                goal.angles.joint6 += joint_offset;
            }
            break;
    }

    ROS_INFO(" joint goal is set as : %f, %f, %f, %f, %f, %f (degree)\n", goal.angles.joint1, goal.angles.joint2, goal.angles.joint3, goal.angles.joint4, goal.angles.joint5, goal.angles.joint6);

    client.sendGoal(goal);
}
// %EndTag(send actionlib goals)%


// %Tag(processFeedback)%

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback) {
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = tf_prefix_ + "_link_base";
    tf::Quaternion quaternion_mousedown, quaternion_mouseup;

    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
            << " / control '" << feedback->control_name << "'";

    double roll_mousedown, pitch_mousedown, yaw_mousedown;
    double roll_mouseup, pitch_mouseup, yaw_mouseup;
    switch (feedback->event_type) {
        case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
            break;

        case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
            // the moment mouse is clicked down, not the moment keeps down.
            if (mouse_was_up = true) {
                tf::quaternionMsgToTF(feedback->pose.orientation, quaternion_mousedown);
                tf::Matrix3x3(quaternion_mousedown).getRPY(roll_mousedown, pitch_mousedown, yaw_mousedown);
                if (feedback->marker_name == "cartesian_6dof") {
                    ROS_INFO_STREAM("cartesian_6dof control mode.");
                } else {
                    ROS_INFO_STREAM("Joint control is mode.");
                    //                ROS_INFO_STREAM( s.str() << ": mouse DOWN (refers to current status): "
                    //                                 << feedback->marker_name
                    //                                 << ": " << yaw_mousedown
                    //                                 << "\nframe: " << feedback->header.frame_id
                    //                                 << " time: " << feedback->header.stamp.sec << "sec, "
                    //                                 << feedback->header.stamp.nsec << " nsec" );
                }
            }

            mouse_was_up = false;
            break;

        case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
            tf::quaternionMsgToTF(feedback->pose.orientation, quaternion_mouseup);
            tf::Matrix3x3(quaternion_mouseup).getRPY(roll_mouseup, pitch_mouseup, yaw_mouseup);
            if (feedback->marker_name == "cartesian_6dof") {
                geometry_msgs::PoseStamped pose_endeffector;
                pose_endeffector.header.frame_id = tf_prefix_ + "_link_base";
                pose_endeffector.header.stamp = ros::Time::now();

                pose_endeffector.pose = feedback->pose;
                //            ROS_INFO_STREAM( s.str() << ": mouse UP (refers to command): "
                //                             << "\nposition = "
                //                             << feedback->pose.position.x
                //                             << ", " << feedback->pose.position.y
                //                             << ", " << feedback->pose.position.z
                //                             << "\norientation = "
                //                             << "w: " << feedback->pose.orientation.w
                //                             << ", x: " << feedback->pose.orientation.x
                //                             << ", y: " << feedback->pose.orientation.y
                //                             << ", z: " << feedback->pose.orientation.z
                //                             << "\nRollPitchYaw = "
                //                             << ": Roll: " << roll_mouseup
                //                             << ", Pitch: " << pitch_mouseup
                //                             << ", Yaw: " << yaw_mouseup
                //                             << "\nframe: " << feedback->header.frame_id);

                //                            ROS_INFO_STREAM("_______________Pose_endeffector parent frame is__________________ : " << pose_endeffector.header.frame_id);
                pose_target_point_marker = pose_endeffector; //copy pose end effector to point info hendler marker.
                sendArmPoseGoal(pose_endeffector);
            } else {
                //            ROS_INFO_STREAM( s.str() << ": mouse UP (refers to command): "
                //                             << feedback->marker_name
                //                             << ": " << yaw_mouseup
                //                             << "\nframe: " << feedback->header.frame_id
                //                             << " time: " << feedback->header.stamp.sec << "sec, "
                //                             << feedback->header.stamp.nsec << " nsec");
                sendArmJointGoal(feedback->marker_name, (yaw_mouseup - yaw_mousedown)*180 / M_PI);
            }
            //reset flag for MOUSE_DOWN
            mouse_was_up = true;

            // sendFingerGoal(feedback);

            break;
    }


}
// %EndTag(processFeedback)%


// %Tag(CurrentJoint)%

void currentJointsFeedback(const kinova_msgs::JointAnglesConstPtr joint_command) {
    current_joint_command.Actuator1 = joint_command->joint1;
    current_joint_command.Actuator2 = joint_command->joint2;
    current_joint_command.Actuator3 = joint_command->joint3;
    current_joint_command.Actuator4 = joint_command->joint4;
    current_joint_command.Actuator5 = joint_command->joint5;
    current_joint_command.Actuator6 = joint_command->joint6;
}
// %EndTag(CurrentJoint)%

// %Tag(CurrentPose)%

bool isEqual(const float a, const float b) {
    return a == b;
}

bool isIdentical(const kinova::KinovaPose &in, const kinova_msgs::KinovaPoseConstPtr &out) {

    bool success = false;
    success = (isEqual(in.ThetaX, out->ThetaX) &&
            isEqual(in.ThetaY, out->ThetaY) &&
            isEqual(in.ThetaZ, out->ThetaZ) &&
            isEqual(in.X, out->X) &&
            isEqual(in.Y, out->Y) &&
            isEqual(in.Z, out->Z));
    return success;
}

void currentPoseFeedback(const kinova_msgs::KinovaPoseConstPtr pose_command) {
    /* to jest wywoływane czały czas 10 x na sekunde */
    
    //to stop applyChanges(), every frame if not needed because flooding the web browswer
    if (!isIdentical(current_pose_command, pose_command)) { 
        current_pose_command.X = pose_command->X;
        current_pose_command.Y = pose_command->Y;
        current_pose_command.Z = pose_command->Z;
        current_pose_command.ThetaX = pose_command->ThetaX;
        current_pose_command.ThetaY = pose_command->ThetaY;
        current_pose_command.ThetaZ = pose_command->ThetaZ;

        /* odczytują pozycję końcówki manipulatora i przekazuję ją 1 raz do stworzenia markera, bo flaga jest false na górze pliku - dzięki temu przy 1 właczeniu
         * raz sie ustawia w centrum reki i pojawia*/



        if (getCurrentCommand == false) {
            /* wywoływane tylko 1*/
            tf::Pose pose; // home position for j2n6
            pose.setOrigin(tf::Vector3(current_pose_command.X, current_pose_command.Y, current_pose_command.Z));
            tf::Quaternion q = kinova::EulerXYZ2Quaternion(current_pose_command.ThetaX, current_pose_command.ThetaY, current_pose_command.ThetaZ);
            pose.setRotation(q);

            //        
            /*tworzy marker*/
            make6DofMarker(false, visualization_msgs::InteractiveMarkerControl::NONE, pose, true);
            make6DofMarkerTarget(false, visualization_msgs::InteractiveMarkerControl::NONE, pose, false); //kolejny marker z inna nazwa
            armPose_interMark_server->applyChanges();

            getCurrentCommand = true;
        }

        InteractiveMarker marker;
        if (armPose_interMark_server->get("TEST", marker)) {

            marker.pose.position.x = pose_command->X;
            marker.pose.position.y = pose_command->Y;
            marker.pose.position.z = pose_command->Z;

            if (areValuesClose(pose_target_point_marker.pose.position.x, marker.pose.position.x, 0.01) &&
                    areValuesClose(pose_target_point_marker.pose.position.y, marker.pose.position.y, 0.01) &&
                    areValuesClose(pose_target_point_marker.pose.position.z, marker.pose.position.z, 0.01)
                    ) {

                //change the sphere color if marker is near center of the end of arm.
                marker.controls.at(0).markers.at(0).color.g = 1.0;
                marker.controls.at(0).markers.at(0).color.r = 0.0;


                marker.controls.at(0).markers.at(0).scale.x = marker_scale_ * 0.5;
                marker.controls.at(0).markers.at(0).scale.y = marker_scale_ * 0.5;
                marker.controls.at(0).markers.at(0).scale.z = marker_scale_ * 0.5;
                armPose_interMark_server->insert(marker);



            } else {
                marker.controls.at(0).markers.at(0).color.g = 0.0;
                marker.controls.at(0).markers.at(0).color.r = 1.0;
                marker.controls.at(0).markers.at(0).scale.x = marker_scale_ * 0.3;
                marker.controls.at(0).markers.at(0).scale.y = marker_scale_ * 0.3;
                marker.controls.at(0).markers.at(0).scale.z = marker_scale_ * 0.3;
                armPose_interMark_server->insert(marker);



            }
            //        armPose_interMark_server->setCallback(marker.name, &pf);

            /* This is server change 
             * - can use here publish for some topic and send position for center marker of arm and subscribe it some marker
             * - for both can use callback server to clean code.
             * - webgl calling for data image 10 times per sec because server is called so many times.
             */
            //        marker.header.stamp = ros::Time::now(); //what for
            //        armPose_interMark_server->insert(marker);
            armPose_interMark_server->applyChanges();
        }
    }
}

// %EndTag(CurrentPose)%
////////////////////////////////////////////////////////////////////////////////////
/// \brief main
/// \param argc
/// \param argv
/// \return
/// 

void menuFeedback(const geometry_msgs::PoseStamped &menu) {

    std::cout << menu.pose.orientation.w << "\n";
    ArmPose_actionlibClient client("/" + tf_prefix_ + "_driver/pose_action/tool_pose", true);
    kinova_msgs::ArmPoseGoal goal;
    client.waitForServer();
    //Tu wpisać pozycje 
    goal.pose = menu;

    ROS_INFO_STREAM("Goal parent frame is : " << goal.pose.header.frame_id);

    ROS_INFO_STREAM("Goal to arm pose actionlib: \n"
            << "  x: " << goal.pose.pose.position.x
            << ", y: " << goal.pose.pose.position.y
            << ", z: " << goal.pose.pose.position.z
            << ", qx: " << goal.pose.pose.orientation.x
            << ", qy: " << goal.pose.pose.orientation.y
            << ", qz: " << goal.pose.pose.orientation.z
            << ", qw: " << goal.pose.pose.orientation.w << std::endl);

    client.sendGoal(goal);
}

int main(int argc, char** argv) {
    // Retrieve the (non-option) argument:
    if ((argc <= 1) || (argv[argc - 1] == NULL)) // there is NO input...
    {
        std::cerr << "No kinova_robotType provided in the argument! so I take a j2n6s300" << std::endl;
        kinova_robotType_ = "j2n6s300";
        ROS_INFO("kinova_robotType is %s.", kinova_robotType_.c_str());
        //return -1;
    } else // there is an input...
    {
        //        if (valid_kinovaRobotType(kinova_robotType_) == false)
        //        {
        //            ROS_WARN("Invalid kinova_robotType error! Obtained: %s.", kinova_robotType_.c_str());
        //            return -1;
        //        }
        kinova_robotType_ = argv[argc - 1];
        ROS_INFO("kinova_robotType is %s.", kinova_robotType_.c_str());
    }

    //    tf_prefix_ = kinova_robotType_ + "_" + boost::lexical_cast<string>(same_type_index); // in case of multiple same_type robots
    tf_prefix_ = kinova_robotType_;

    // Maximum number of joints on Kinova-like robots:
    robot_category_ = kinova_robotType_[0];
    robot_category_version_ = kinova_robotType_[1] - '0';
    wrist_type_ = kinova_robotType_[2];
    arm_joint_number_ = kinova_robotType_[3] - '0';
    robot_mode_ = kinova_robotType_[4];
    finger_number_ = kinova_robotType_[5] - '0';
    joint_total_number_ = arm_joint_number_ + finger_number_;

    ros::init(argc, argv, tf_prefix_ + "_interactive_control");
    ros::NodeHandle nh("~");
    ros::Subscriber armJoint_sub = nh.subscribe("/" + tf_prefix_ + "_driver/out/joint_command", 1, &currentJointsFeedback);
    ros::Subscriber armCartesian_sub = nh.subscribe("/" + tf_prefix_ + "_driver/out/cartesian_command", 1, &currentPoseFeedback);

    ros::Subscriber armPoseStumped_sub = nh.subscribe("/robot_menu", 10, &menuFeedback);

    armJoint_interMark_server.reset(new interactive_markers::InteractiveMarkerServer(tf_prefix_ + "_interactive_control_Joint", "", false));
    armPose_interMark_server.reset(new interactive_markers::InteractiveMarkerServer(tf_prefix_ + "_interactive_control_Cart", "", false));

    ros::Duration(0.1).sleep();


    // %Tag(CreatInteractiveMarkers)%
    // Cartesian Control Marker build based on current pose in currentPoseFeedback().

    // Joint Control
    tf::Vector3 position = tf::Vector3(0, 0, 0);
    make1DofMarker(tf_prefix_ + "_link_1", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "1st Axis", "marker_joint1");
    make1DofMarker(tf_prefix_ + "_link_2", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "2nd Axis", "marker_joint2");
    make1DofMarker(tf_prefix_ + "_link_3", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "3rd Axis", "marker_joint3");
    make1DofMarker(tf_prefix_ + "_link_4", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "4th Axis", "marker_joint4");
    if (arm_joint_number_ == 6) {
        make1DofMarker(tf_prefix_ + "_link_5", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "5th Axis", "marker_joint5");
        make1DofMarker(tf_prefix_ + "_link_6", "z", visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS, position, "6th Axis", "marker_joint6");
    }
    // %EndTag(CreatInteractiveMarkers)%

    armJoint_interMark_server->applyChanges();
    armPose_interMark_server->applyChanges();

    ros::spin();

    armJoint_interMark_server.reset();
    armPose_interMark_server.reset();

    return 0;

}


