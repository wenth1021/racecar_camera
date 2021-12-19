#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float32.h>
#include <math.h>

// TODO: include ROS msg type headers and libraries

class LineFollow {
// The class that handles emergency braking
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
    // TODO: create ROS subscribers and publishers
    ros::Subscriber centroidsub;
    ros::Publisher followpub;

public:
    LineFollow() {
        //n = ros::NodeHandle();
        /*
        kp = 1.5;
        ki = 0;
        kd = 0.05;
        scanA_angle = 45*M_PI/180.0;
        leftdist = 0.7;
        error2 = 0;
        error1 = 0;
        */
        kp = 0.7;
        ki = 0;
        kd = 0.25;
        error2=0;
        /*
        One publisher should publish to the /brake topic with an
        ackermann_msgs/AckermannDriveStamped brake message.
        One publisher should publish to the /brake_bool topic with a
        std_msgs/Bool message.
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /odom topic to get
        the nav_msgs/Odometry messages
        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods
        NOTE that the x component of the linear velocity in odom is the speed
        */

        // TODO: create ROS subscribers and publishers
        centroidsub = n.subscribe("centroid",1,&LineFollow::scan_callback,this);
        followpub=n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_1",1000);
    }

    void scan_callback(const std_msgs::Float32 msg) {
        /*
        float angle_increment = scan_msg->angle_increment;
        int index_b = 3*(M_PI/2)/angle_increment;
        int index_a = index_b + scanA_angle/angle_increment;
        float scanA = scan_msg->ranges[index_a];
        float scanB = scan_msg->ranges[index_b];
        ROS_INFO("indexes %f, %f", scanB, scanA);
        followLeft(scanA, scanB);
        ackermann_msgs::AckermannDriveStamped follow_command;
        angle = -kp*error2-kd*(error2-error1)/0.01;
        follow_command.drive.steering_angle = angle;
        if(abs(angle)>=20){
            speed = 0.5;
        }else if(abs(angle)>=10){
            speed = 1.0;
        }else if (abs(angle)>=0){
            speed = 1.5;
        }*/
        
        ROS_INFO("error %f", msg.data);
        ackermann_msgs::AckermannDriveStamped follow_command;
        follow_command.drive.steering_angle =1.2*msg.data;
	//+2*(error2-msg.data);
        /*if (msg.data > 0.5) {
            follow_command.drive.steering_angle = 0.34;
        }
        else if (msg.data < -0.5) {
            follow_command.drive.steering_angle = -0.34;
        }*/
        ROS_INFO("Steering %f", follow_command.drive.steering_angle);
        error2=msg.data;
        float speed=0;
        if(abs(msg.data)>=0.25){
            speed=1.2;
        }else{
            speed=1.2;
        }
        follow_command.drive.speed = speed;
        //ROS_INFO("angle %f", angle);
        followpub.publish(follow_command);
    }
/*
    void followLeft(float scanA, float scanB){
        float alpha = -atan2(scanA*cos(scanA_angle)-scanB,scanA*sin(scanA_angle));
        ROS_INFO("Alpha %f", alpha);
        float Dt=scanB*cos(alpha);
        float lookahead = 1;
        float Dt1=Dt+lookahead*sin(alpha);
        ROS_INFO("DT1 %f", Dt1);
        error1 = error2;
        error2 = leftdist - Dt1;
        return;
    }*/

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "linefollow_node");
    LineFollow lf;
    ros::Rate r(100);
    while(ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    //ros::spin();
    return 0;
}
