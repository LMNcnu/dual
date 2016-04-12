#include <moveit/move_group_interface/move_group.h>
#include <math.h>
#include  <std_msgs/Float32MultiArray.h>//define joints
#include <iostream>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_msgs/PlanningScene.h>


#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <iterator>
#include <vector>


#include <tf/transform_broadcaster.h>

//引用传参比较好，不改变值的情况下生明为const安全。
void add_object(const moveit::planning_interface::MoveGroup &group)
{
      ros::NodeHandle node_handle;
    //添加物体

    // Advertise the required topic
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Note that this topic may need to be remapped in the launch file
      ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
      while(planning_scene_diff_publisher.getNumSubscribers() < 1)
      {
        ros::WallDuration sleep_t(0.5);
        sleep_t.sleep();
      }

    // Define the attached object message
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // We will use this message to add or
    // subtract the object from the world
    // and to attach the object to the robot
      moveit_msgs::AttachedCollisionObject attached_object;
      attached_object.link_name = "lLink7";
      /* The header must contain a valid TF frame*/
      attached_object.object.header.frame_id = group.getPlanningFrame();

      /* The id of the object */
      attached_object.object.id = "box1";

      /* A default left pose */
      geometry_msgs::Pose pose1;
      pose1.orientation.w = 1.0;
      pose1.position.x=0.4;
      pose1.position.y=0.64;
      pose1.position.z=0.6;
      /* Define a left box to be attached */
      shape_msgs::SolidPrimitive primitive1;
      primitive1.type = primitive1.BOX;
      primitive1.dimensions.resize(3);
      primitive1.dimensions[0] = 0.03;
      primitive1.dimensions[1] = 0.03;
      primitive1.dimensions[2] = 0.2;

      /* A default right pose */
      geometry_msgs::Pose pose2;
      pose2.orientation.w = 1.0;
      pose2.position.x=-0.4;
      pose2.position.y=0.7;
      pose2.position.z=0.4;
      /* Define a right box to be attached */
      shape_msgs::SolidPrimitive primitive2;
      primitive2.type = primitive2.BOX;
      primitive2.dimensions.resize(3);
      primitive2.dimensions[0] = 0.3;
      primitive2.dimensions[1] = 0.3;
      primitive2.dimensions[2] = 0.4;

      //容器使用push_back进行添加元素
      attached_object.object.primitives.push_back(primitive1);
      attached_object.object.primitive_poses.push_back(pose1);
      attached_object.object.primitives.push_back(primitive2);
      attached_object.object.primitive_poses.push_back(pose2);

    // Note that attaching an object to the robot requires
    // the corresponding operation to be specified as an ADD operation
      attached_object.object.operation = attached_object.object.ADD;


    // Add an object into the environment
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // Add the object into the environment by adding it to
    // the set of collision objects in the "world" part of the
    // planning scene. Note that we are using only the "object"
    // field of the attached_object message here.
      ROS_INFO("Adding the object into the world ");
      moveit_msgs::PlanningScene planning_scene;
      planning_scene.world.collision_objects.push_back(attached_object.object);
      planning_scene.is_diff = true;
      planning_scene_diff_publisher.publish(planning_scene);
      sleep(2);
}

void rpy_to_xyzw(float r, float p, float y, geometry_msgs::Pose &pose)
{
    pose.orientation.w = cos(r/2)*cos(p/2)*cos(y/2)+sin(r/2)*sin(p/2)*sin(y/2);
    pose.orientation.x = sin(r/2)*cos(p/2)*cos(y/2)-cos(r/2)*sin(p/2)*sin(y/2);
    pose.orientation.y = cos(r/2)*sin(p/2)*cos(y/2)+sin(r/2)*cos(p/2)*sin(y/2);
    pose.orientation.z = cos(r/2)*cos(p/2)*sin(y/2)-sin(r/2)*sin(p/2)*cos(y/2);
}
void xyzw_to_rpy(float x, float y, float z, float w, float &r, float &p, float &yy)
{
    r = atan2(2*(w*x+y*z),1-2*(y*y+x*x));
    p = asin(2*(w*y-x*z));
    yy = atan2(2*(w*z+y*x),1-2*(z*z+y*y));
    std::cout<<r<<","<<p<<","<<yy<<std::endl;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_interactive_self");

    ros::AsyncSpinner spinner(1);
    spinner.start();

    //sleep(20.0);//等rviz起来，如果没在一个lanuch中，可以省去这个

    moveit::planning_interface::MoveGroup group("dual_arms");

    // Getting Basic Information
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());//打印坐标系
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());//打印末端链节

    add_object(group);//添加左右两个物体

    geometry_msgs::PoseStamped current_pose = group.getCurrentPose("lLink7");
    ROS_INFO("current left pose:x=%f,y=%f,z=%f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
    ROS_INFO("current let qurternion:x=%f,y=%f,z=%f,w=%f",current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w);
    std::cout<<"左臂初始RPY=";
    float r,p,y;
    xyzw_to_rpy(-1,0,0,0,r,p,y);
    current_pose = group.getCurrentPose("rLink7");
    ROS_INFO("current right pose:x=%f,y=%f,z=%f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);
    ROS_INFO("current right quaternion:x=%f,y=%f,z=%f,w=%f",current_pose.pose.orientation.x,current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w);
    std::cout<<"右臂初始RPY=";
    xyzw_to_rpy(-1,0,0,0,r,p,y);

    //给左臂末端赋值
    geometry_msgs::Pose testPose1 = current_pose.pose;
    geometry_msgs::Pose pose1;//和上面添加物体函数中给水杯添加的位置一样
    pose1.orientation.w = 1.0;
    pose1.position.x=0.4;
    pose1.position.y=0.64;
    pose1.position.z=0.6;
    testPose1.position = pose1.position;
    testPose1.position.y = pose1.position.y - 0.14;//不能碰杯子否则规划失败，所以留出一些空间
    std::cout<<"左臂goal的欧拉角：";
    xyzw_to_rpy(0.699477,0.714655,0.000155,0.000255,r,p,y);//显示下左臂goal的欧拉角
    std::cout<<"水杯的欧拉角";
    xyzw_to_rpy(pose1.orientation.x,pose1.orientation.y,pose1.orientation.z,pose1.orientation.w,r,p,y);//得到水杯的欧拉角
    rpy_to_xyzw(3.14+r,0+p,1.6+y,testPose1);
    std::cout<<"左臂goal的四元数：";
    std::cout<<testPose1.orientation.x<<testPose1.orientation.y<<testPose1.orientation.z<<testPose1.orientation.w<<std::endl;
    //给右臂末端赋值
    geometry_msgs::Pose testPose2 = current_pose.pose;
    testPose2.position.x = -0.385743;
    testPose2.position.y = 0.467167;
    testPose2.position.z = 0.674107;
//    testPose2.orientation.x = 0.702235;
//    testPose2.orientation.y =-0.710211;
//    testPose2.orientation.z = -0.033619;
//    testPose2.orientation.w = 0.036564;
    xyzw_to_rpy(0.70223,-0.710211,-0.03361,0.036564,r,p,y);
    rpy_to_xyzw(3.14,0,-1.5,testPose2);
    std::cout<<testPose2.orientation.x<<testPose2.orientation.y<<testPose2.orientation.z<<testPose2.orientation.w<<std::endl;


    //设定home位置
    std::vector<double> group_variable_values;
    group_variable_values = group.getCurrentJointValues();
    //c++98不支持，c++11新特性
    //for(double d : group_variable_values){
    //std::cout << d << std::endl;
    //    }
    std::cout<<"打印初始位姿的各个关节角度值:"<<std::endl;
    std::vector<double>::iterator d = group_variable_values.begin();
    while(d != group_variable_values.end()) {
        std::cout << *(d++) << std::endl;

    }
    for(int i=0; i<14; i++){
        group_variable_values[i] = 0.0;
    }

//group.setMaxVelocityScalingFactor(0.1);


    //规划运动到目标位置
    group.setPoseTarget(testPose1,"lLink7");
    group.setPoseTarget(testPose2,"rLink7");
    //group.asyncMove();
    moveit::planning_interface::MoveGroup::Plan plan_goal;
    group.plan(plan_goal);
    group.asyncExecute(plan_goal);

    //通过命令去控制规划到home还是goal位置
    std::string s1 = "go home";
    std::string command;
    std::string s2 = "go to the goal";
    moveit::planning_interface::MoveGroup::Plan plan_home;   
    while(1){
        std::cout << "Please input command(Eg:go home,go to the goal):";
        std::getline(std::cin, command);
        if(command == s1){
            group.setJointValueTarget(group_variable_values);
            group.plan(plan_home);
            group.asyncExecute(plan_home);
        }else if(command == s2){
            group.setPoseTarget(testPose1,"lLink7");
            group.setPoseTarget(testPose2,"rLink7");
            group.asyncMove();
        }else{
            ROS_INFO("command is invalid, please again");
        }
    }


    ros::shutdown();
    return 0;
}

