#include <math.h>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <mpc_lib/MPC.h>
#include <Eigen-3.3/Eigen/QR>
#include <Eigen-3.3/Eigen/Core>
#include <virat_msgs/Path.h>
#include <virat_msgs/MPC_EstStates.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

double pi()
{
    return M_PI;
}

// For converting back and forth between radians and degrees.
double deg2rad(double x)
{
    return x * pi() / 180;
}
double rad2deg(double x)
{
    return x * 180 / pi();
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x)
{

    double result = 0.0;

    for (int i = 0; i < coeffs.size(); i++)
    {
        result += coeffs[i] * pow(x, i);
    }

    return result;
}

// Fit a polynomial.
// Adapted from https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order)
{
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++)
    {
        A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++)
    {
        for (int i = 0; i < order; i++)
        {
            A(j, i + 1) = A(j, i) * xvals(j);
        }
    }

    Eigen::VectorXd result = A.householderQr().solve(yvals);

    return result;
}

class Controller
{
private:
    ros::NodeHandle n;

    message_filters::Subscriber<virat_msgs::Path> path_sub;
    message_filters::Subscriber<virat_msgs::MPC_EstStates> state_sub;

    message_filters::TimeSynchronizer<virat_msgs::Path, virat_msgs::MPC_EstStates> sync2;

    ros::Publisher actPub;
    geometry_msgs::Twist velMsg;

public:
    Controller() : sync2(path_sub, state_sub, 10)
    {
        path_sub.subscribe(n, "/virat/fake/path", 1);
        state_sub.subscribe(n, "/virat/controller/input/state", 1);

        sync2.registerCallback(boost::bind(&Controller::callback, this, _1, _2));

        this->actPub = n.advertise<geometry_msgs::Twist>("/virat/cmd_vel", 10);
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

        double dt = 0.1;
        double current_px = 0.0 + v * dt;
        double current_py = 0.0;
        double current_theta = 0.0 + omega * dt;
        double current_v = v + throttle * dt;
        double current_cte = cte + v * sin(etheta) * dt;
        double current_etheta = etheta - current_theta;

        Eigen::VectorXd model_state(6);
        model_state << current_px, current_py, current_theta, current_v, current_cte, current_etheta;

        // time to solve !

        std::vector<double> mpc_solns = mpc.Solve(model_state, coeffs);

        omega = mpc_solns[0];
        throttle = mpc_solns[1];
        double speed = current_v + throttle * dt;

        velMsg.linear.x = speed;
        velMsg.angular.z = omega;

        std::cout.setf(std::ios::fixed, std::ios::floatfield);
        std::cout.precision(4);
        std::cout << "Speed : " << speed << "    "
                  << "Omega : " << omega << std::endl;
        std::cout << "Throttle : " << throttle << std::endl;
        std::cout << "Cross track error : " << current_cte << std::endl;
        std::cout << "Orientation error : " << current_etheta << std::endl;
        std::cout << std::endl;

        this->actPub.publish(velMsg);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "virat_controller");

    Controller controller;

    ros::spin();
    return 0;
}
