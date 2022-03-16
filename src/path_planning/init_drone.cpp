//initially include all header files necessary for compiling the subscribers, publishers and services 
//which are going to be used in the code

 #include <ros/ros.h>

 #include <geometry_msgs/PoseStamped.h>
 #include <std_msgs/Bool.h>
 #include <mavros_msgs/CommandBool.h>
 #include <mavros_msgs/SetMode.h>
//  #include <mavros_msgs/StreamRate.h>
 #include <mavros_msgs/State.h>

 #include <mavros_msgs/CommandLong.h>
 #include <mavros_msgs/CommandTOL.h>
 #include <mavros_msgs/PositionTarget.h>

 //creating a callback 'state_cb' to store the current state of the copter in variable 'current_state' 
 //this will be called by subscriber 'state_sub' when a message arrives in the 'mavros/state' topic 
//which is of data type 'mavros_msgs/State'

mavros_msgs::State current_state;              //this sets the variable type for 'current_state' 
                                                                          //variable

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped initial_wp;
std_msgs::Bool took_off;

ros::Publisher local_pos_pub;
ros::Publisher took_off_pub;

double takeoff_height = 25.0;
double reach_threshold = 0.5;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void wp_cb (const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (ros::ok() && current_state.armed)
    {
        ROS_INFO("Setting new waypoint..\nX: %f, Y: %f, Z: %f\n", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        local_pos_pub.publish(*msg);
    }
}

void current_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pose = *msg;
    ROS_INFO("Current Pose:\nX: %f, Y: %f, Z: %f\n", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    if (took_off.data == false)
    {
        ROS_INFO("Distance to Takeoff Height: %f\n", abs(takeoff_height - current_pose.pose.position.z));
        if (abs(takeoff_height - current_pose.pose.position.z) < reach_threshold)
        {
            took_off.data = true;
            ROS_INFO("Reached altitude of %f\n", current_pose.pose.position.z);
        }
    }
    took_off_pub.publish(took_off);
}

bool reached_wp(geometry_msgs::PoseStamped wp)
{
    // if (initial_wp.pose.orientation.x == -1)
    //     return false;

    // double diff_x = current_pose.pose.position.x - wp.pose.position.x;
    // double diff_y = current_pose.pose.position.y - wp.pose.position.y;
    double diff_z = current_pose.pose.position.z - takeoff_height;

    // double dist = sqrt(pow(diff_x, 2) + pow(diff_y, 2) + pow(diff_z, 2));

    return (diff_z < reach_threshold);
}

//the main ros function with all subscribers, publishers and services

int main(int argc, char **argv)
{
    // initial_wp.pose.orientation.x = -1;

    ros::init(argc, argv, "init_drone");  //initialising ros, with node name 'offn_node'
    ros::NodeHandle nh;                  //main accesspoint for this node to communicate with the system 

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>      //subscriber 'state_sub' is subscribing to topic 'mavros/state of type mavros_msgs/State by calling state_cb
     ("mavros/state", 10, state_cb);
    ros::Subscriber current_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
     ("mavros/local_position/pose", 10, current_pose_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped> //publisher 'local_pos_pub' topic 'maros/.../local' type 'geo../PoseStamped'
     ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>  //serviceclient 'arming_client' type(header) 'mav../Com..Bool' service 'mavros/cmd/arming'
     ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>   //serviceclient 'set_mode_client' type(header) 'mav../SetMode' service 'mavros/set_mode'
     ("mavros/set_mode");
    ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL> 
     ("/mavros/cmd/takeoff");

    ros::Subscriber waypoint_sub = nh.subscribe<geometry_msgs::PoseStamped>
     ("/uav_wp", 10, wp_cb);

    took_off_pub = nh.advertise<std_msgs::Bool>
     ("/took_off", 10);
    took_off.data = false;
    

    ros::Rate rate(20.0);                               //here we give 20Hz

    ROS_INFO("Waiting for Connection...");
    while(ros::ok() && !current_state.connected){
        // ROS_INFO("%d", current_state.connected);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connection with FCU has been establised.");

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";    //OFFBOARD node since we are 
                                                                                                        //using px4 flight stack
    offb_set_mode.request.base_mode = 0.0;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;                       //true=> armig the copter
    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;                   //fals=> disarmig the copter

    ros::Time last_request = ros::Time::now();
 
    while(ros::ok() && !current_state.armed){

        if( current_state.mode != "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("GUIDED enabled");
            }
            last_request = ros::Time::now();

        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle Armed");
                }
                last_request = ros::Time::now();
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    last_request = ros::Time::now();
    
    mavros_msgs::CommandTOL srv_takeoff;
    srv_takeoff.request.altitude = takeoff_height;

    if(takeoff_cl.call(srv_takeoff)){
        ROS_INFO("Takeoff sent %d", srv_takeoff.response.success);
    }
    else{
        ROS_ERROR("Failed Takeoff");
        return -1;
    }

   //after 5sec giving command to drone to takeoff to an altitude of 1.5m
    // ROS_INFO("Initial Set to X: %f, Y: %f, Z: %f\n", initial_wp.pose.orientation.x, initial_wp.pose.position.y, initial_wp.pose.position.z);


    // while(ros::ok() && current_state.armed){

    //     if( ros::Time::now() - last_request > ros::Duration(1.0)){
    //         // ROS_INFO("Initiating Takeoff");  
            
            
    //         last_request = ros::Time::now();
    //         // if (initial_wp.pose.orientation.x != -1)
    //         //     local_pos_pub.publish(initial_wp);
    //         ros::spinOnce();
    //         rate.sleep();
    //     }
    // }

    ros::spin();


}