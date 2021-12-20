#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float32.h>
#include <math.h>


class WaypointFollow {
// The class that handles waypoint following
private:
    ros::NodeHandle n;
    float angle;
    float error1;
    float error2;
    float leftdist;
    float kp;
    float ki;
    float kd;
    float speed;
    ros::Subscriber centroidsub;
    ros::Publisher followpub;

public:
    WaypointFollow() {
        kp = 0.7;
        ki = 0;
        kd = 0.25;
        error2=0;
        centroidsub = n.subscribe("centroid",1,&WaypointFollow::centroid_callback,this);
        followpub=n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_1",1000);
    }

    void centroid_callback(const std_msgs::Float32 msg) {
        
        ROS_INFO("error %f", msg.data);
        ackermann_msgs::AckermannDriveStamped follow_command;
        follow_command.drive.steering_angle =1.2*msg.data;
        ROS_INFO("Steering %f", follow_command.drive.steering_angle);
        error2=msg.data;
        float speed=0;
        if(abs(msg.data)>=0.25){
            speed=0.8;  // Speed when turning
        }else{
            speed=0.8;  // Speed when going straight
        }
        follow_command.drive.speed = speed;
        followpub.publish(follow_command);
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "waypointfollow_node");
    WaypointFollow lf;
    ros::Rate r(100);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
