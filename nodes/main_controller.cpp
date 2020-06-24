#include <iostream>
#include <mpc_lib/utils.h>
#include <mpc_lib/dDrive_MPC.h>
#include <ros/ros.h>
#include <virat_msgs/Path.h>
#include <virat_msgs/MPC_EstStates.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <Eigen-3.3/Eigen/QR>
#include <Eigen-3.3/Eigen/Core>

class Controller
{
private:
    // MPC parameters
    struct MPCparams mpcParams;

    ros::NodeHandle n;

    message_filters::Subscriber<virat_msgs::Path> path_sub;
    message_filters::Subscriber<virat_msgs::MPC_EstStates> state_sub;

    message_filters::TimeSynchronizer<virat_msgs::Path, virat_msgs::MPC_EstStates> sync2;

    // to send actuator variables to bot
    ros::Publisher actPub;
    geometry_msgs::Twist velMsg;

public:
    Controller() : sync2(path_sub, state_sub, 10)
    {
        this->loadParams();

        path_sub.subscribe(n, "/virat/fake/path", 1);
        state_sub.subscribe(n, "/virat/controller/input/state", 1);

        sync2.registerCallback(boost::bind(&Controller::callback, this, _1, _2));

        this->actPub = n.advertise<geometry_msgs::Twist>("/virat/cmd_vel", 10);
    }

    void loadParams(void)
    {
        // load MPC parameters from parameter server
        n.getParam("/MPCcontroller/timeSteps", mpcParams.N);
        n.getParam("/MPCcontroller/sampleTime", mpcParams.dt);

        n.getParam("/MPCcontroller/reference/velocity", mpcParams.ref_v);
        n.getParam("/MPCcontroller/reference/crossTrackError", mpcParams.ref_cte);
        n.getParam("/MPCcontroller/reference/orientationError", mpcParams.ref_etheta);

        n.getParam("/MPCcontroller/maxBounds/maxOmega", mpcParams.MAX_OMEGA);
        n.getParam("/MPCcontroller/maxBounds/maxThrottle", mpcParams.MAX_THROTTLE);

        n.getParam("/MPCcontroller/weights/w_cte", mpcParams.W_CTE);
        n.getParam("/MPCcontroller/weights/w_etheta", mpcParams.W_ETHETA);
        n.getParam("/MPCcontroller/weights/w_vel", mpcParams.W_VEL);
        n.getParam("/MPCcontroller/weights/w_omega", mpcParams.W_OMEGA);
        n.getParam("/MPCcontroller/weights/w_acc", mpcParams.W_ACC);
        n.getParam("/MPCcontroller/weights/w_omega_d", mpcParams.W_OMEGA_D);
        n.getParam("/MPCcontroller/weights/w_acc_d", mpcParams.W_ACC_D);
    }

    void callback(const virat_msgs::Path::ConstPtr &TarPath, const virat_msgs::MPC_EstStates::ConstPtr &estState)
    {
        // instantiate mpc
        MPC mpc;

        boost::array<double, 10ul> ptsx = TarPath->x_vals;
        boost::array<double, 10ul> ptsy = TarPath->y_vals;

        double px = estState->state_vars.x;
        double py = estState->state_vars.y;
        double theta = estState->state_vars.yaw;
        double v = estState->actuator_vars.lin_speed;
        double omega = estState->actuator_vars.ang_speed;
        double throttle = estState->actuator_vars.throttle;

        // convert coordintes to vehicle frame from global frame
        for (int i = 0; i < ptsx.size(); i++)
        {
            double shift_x = ptsx[i] - px;
            double shift_y = ptsy[i] - py;
            ptsx[i] = shift_x * cos(-theta) - shift_y * sin(-theta);
            ptsy[i] = shift_x * sin(-theta) + shift_y * cos(-theta);
        }

        double *ptrx = &ptsx[0];
        Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);

        double *ptry = &ptsy[0];
        Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

        Eigen::VectorXd coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

        double cte = polyeval(coeffs, 0);
        double etheta = -atan(coeffs[1]);

        double dt = mpcParams.dt;
        double current_px = 0.0 + v * dt;
        double current_py = 0.0;
        double current_theta = 0.0 + omega * dt;
        double current_v = v + throttle * dt;
        double current_cte = cte + v * sin(etheta) * dt;
        double current_etheta = etheta - current_theta;

        Eigen::VectorXd model_state(6);
        model_state << current_px, current_py, current_theta, current_v, current_cte, current_etheta;

        // time to solve !

        std::vector<double> mpc_solns = mpc.Solve(model_state, coeffs, mpcParams);

        omega = mpc_solns[0];
        throttle = mpc_solns[1];
        double speed = current_v + throttle * dt;

        velMsg.linear.x = speed;
        velMsg.angular.z = omega;

        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout.precision(4);
        std::cout << "Speed : " << speed << "\t"
                  << "Omega : " << omega << std::endl;
        std::cout << "Throttle : " << throttle << std::endl;
        std::cout << "Cross track error : " << current_cte << std::endl;
        std::cout << "Orientation error : " << current_etheta << std::endl;
        std::cout << std::endl;

        this->actPub.publish(velMsg);
        ros::Duration(mpcParams.dt).sleep();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virat_controller");

    Controller controller;

    ros::spin();
    return 0;
}
