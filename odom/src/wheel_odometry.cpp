#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

#define pi 3.14

class Encoder_Values {
    public:
        float left_enc_val=0.0;
        float right_enc_val=0.0;
        void left_enc_cb(const std_msgs::Float32::ConstPtr& left_msg);
        void right_enc_cb(const std_msgs::Float32::ConstPtr& right_msg);
        void reset(void);
};

void Encoder_Values::left_enc_cb(const std_msgs::Float32::ConstPtr& left_msg){
   left_enc_val = left_msg->data; //left_distance_per_rev
//    ROS_INFO("left_enc_value: %f", left_enc_val);
}
void Encoder_Values::right_enc_cb(const std_msgs::Float32::ConstPtr& right_msg){
    right_enc_val = right_msg->data; //right_distance_per_rev
    // ROS_INFO("right_enc_value: %f", right_enc_val);
}
void Encoder_Values::reset(void)
{
    left_enc_val = 0.0;
    right_enc_val = 0.0;
    return;
}

int main(int argc, char **argv){
    float wheel_sep = 0.173;
    float distance = 0.0;
    float x = 0.0;
    float y = 0.0;
    float d_theta = 0.0;
    float theta = 0.0;

    float vx = 0.0;
    float vy = 0.0;
    float vyaw = 0.0;
    
    ros::init(argc, argv, "odom_cal");
    ros::NodeHandle n;
    ros::Time current_time, previous_time;

    // ros variables used by logic
    tf::TransformBroadcaster odom_broadcaster;      //used to update odom tf
    nav_msgs::Odometry odom;                        //used to update odom values
    Encoder_Values encoder_values;                  //class instance used to retrieve encoder distance
    geometry_msgs::Quaternion odom_quat;            //used to get the quarternion/orientation for odom tf and odom
    geometry_msgs::TransformStamped odom_trans;     //used to send the updated geometry messages to update odom tf

    ros::Subscriber l_sub = n.subscribe("l_dist", 1000, &Encoder_Values::left_enc_cb, &encoder_values);
    ros::Subscriber r_sub = n.subscribe("r_dist", 1000, &Encoder_Values::right_enc_cb, &encoder_values);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Rate loop_rate(30);

    while(ros::ok()){
        ros::spinOnce();

        current_time = ros::Time::now();

        // ROS_INFO("left_enc_value: %f", encoder_values.left_enc_val);
        // ROS_INFO("right_enc_value: %f", encoder_values.right_enc_val);

        distance = (encoder_values.left_enc_val+encoder_values.right_enc_val)/2;
        d_theta = (encoder_values.right_enc_val-encoder_values.left_enc_val)/wheel_sep;
        // resets encoder dist values for each motor
        encoder_values.reset();
        
        odom_quat = tf::createQuaternionMsgFromYaw(theta);
        
        // theta = theta*(180/pi);
        x = x + (distance*cos(d_theta));
        y = y + (distance*sin(d_theta));
        theta = theta + d_theta;

        vx = distance*cos(d_theta)/(current_time - previous_time).toSec();
        vy = distance*sin(d_theta)/(current_time - previous_time).toSec();
        vyaw = d_theta/(current_time - previous_time).toSec();

        // ROS_INFO("distance_value is: %f", distance);
        // ROS_INFO("theta_value is: %f", theta);
        // ROS_INFO("x_value is: %f", x);
        // ROS_INFO("y_value is: %f", y);
        // ROS_INFO("vyaw is: %f", vyaw);
        // ROS_INFO("vx is: %f", vx);
        // ROS_INFO("vy is: %f", vy);

        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        
        odom_broadcaster.sendTransform(odom_trans);

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromROllPitchYaw(0,0,theta);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;
        odom.twist.twist.angular.z = vyaw;

        previous_time = current_time;

        odom_pub.publish(odom);
        loop_rate.sleep();
    }
    return 0;
}
