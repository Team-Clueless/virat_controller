#include <ros/ros.h>
#include <cmath>
#include <signal.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <virat_msgs/MPC_EstStates.h>

// prototypes
void sigint_handler(int);
double vel_mod(double, double);

void sigint_handler(int sig)
{
    ROS_INFO("SIGINT Received ..... Exiting\n");
    ros::shutdown();
}

// find speed from velocity
double vel_mod(double vx, double vy)
{
    return sqrt(pow(vx, 2) + pow(vy, 2));
}

class Feed
{

private:
    double prev_speed; // for throttle calculation
    double ODOM_FREQ;

    virat_msgs::MPC_EstStates estState; // message to publish

    ros::Publisher pub;  // estimated state publisher
    ros::Subscriber sub; // odometry subscriber

public:
    Feed(ros::NodeHandle *nh)
    {

        this->pub = nh->advertise<virat_msgs::MPC_EstStates>("/virat/controller/input/state", 10);

        this->sub = nh->subscribe("/virat/odom", 100, &Feed::odom_callback, this);

        this->prev_speed = 0;

        this->ODOM_FREQ = 30.0; // 30 Hz
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
    {

        double quatx = msg->pose.pose.orientation.x;
        double quaty = msg->pose.pose.orientation.y;
        double quatz = msg->pose.pose.orientation.z;
        double quatw = msg->pose.pose.orientation.w;

        double roll, pitch, yaw;

        double vel_x, vel_y;

        // get angles from quaternions
        tf::Quaternion q(quatx, quaty, quatz, quatw);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // Save relevant position info into message
        this->estState.state_vars.x = msg->pose.pose.position.x;
        this->estState.state_vars.y = msg->pose.pose.position.y;
        this->estState.state_vars.yaw = yaw;

        vel_x = msg->twist.twist.linear.x;
        vel_y = msg->twist.twist.linear.y;

        this->estState.actuator_vars.lin_speed = vel_mod(vel_x, vel_y);
        this->estState.actuator_vars.ang_speed = msg->twist.twist.angular.z;
        this->estState.actuator_vars.throttle = (this->estState.actuator_vars.lin_speed - this->prev_speed) * this->ODOM_FREQ;

        // save speed for throttle calculation in next cycle
        this->prev_speed = this->estState.actuator_vars.lin_speed;

        // publish state info
        this->pub.publish(this->estState);
        ROS_INFO("Successfully published robot state");
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_state_estimator");
    ros::NodeHandle nh;

    signal(SIGINT, sigint_handler);

    Feed feeder = Feed(&nh);

    ROS_INFO("\n\nPublishing Robot state in topic    : /virat/controller/input/state\n");

    ros::spin();
    return 0;
}
