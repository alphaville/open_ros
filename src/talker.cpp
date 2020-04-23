#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "mpc_dummy/OptimisationResult.h"
#include "mpc_dummy/OptimisationParameters.h"
#include <iostream>
#include "rosenbrock_bindings.hpp"

#define MPC_DUMMY_NODE_NAME "mpc_dummy_talker"
#define MPC_DUMMY_BASE_TOPIC "/mpc"
#define MPC_DUMMY_SOLUTION_TOPIC "open_solution"
#define MPC_DUMMY_PARAMS_TOPIC "open_parameters"
#define MPC_DUMMY_RATE 10

/**
 * Parameters received on topic /mpc/mpc_parameters
 */
static mpc_dummy::OptimisationParameters params;
static mpc_dummy::OptimisationResult results;


void mpcReceiveRequestCallback(const mpc_dummy::OptimisationParameters::ConstPtr& msg)
{
  std::cout << *msg;
  params = *msg;
}


int main(int argc, char **argv)
{

  double p[ROSENBROCK_NUM_PARAMETERS] = {2.0, 0.0};
  double u[ROSENBROCK_NUM_DECISION_VARIABLES] = {0};
  double init_penalty = 15.0;
  rosenbrockCache *cache = rosenbrock_new();

  ros::init(argc, argv, MPC_DUMMY_NODE_NAME);
  ros::NodeHandle n(MPC_DUMMY_BASE_TOPIC);

  ros::Publisher mpc_pub = n.advertise<mpc_dummy::OptimisationResult>(MPC_DUMMY_SOLUTION_TOPIC, 1000);
  ros::Subscriber sub = n.subscribe(MPC_DUMMY_PARAMS_TOPIC, 1000, mpcReceiveRequestCallback);
  ros::Rate loop_rate(MPC_DUMMY_RATE);
  
 
  int count = 0;
  while (ros::ok())
  {

    rosenbrockSolverStatus status = rosenbrock_solve(cache, u, p, 0, &init_penalty);  // solve!

    /* pack results into `results` */
    std::vector<double> sol(u, u+ROSENBROCK_NUM_DECISION_VARIABLES); results.solution = sol;
    std::vector<double> y(status.lagrange, status.lagrange+ROSENBROCK_N1); results.lagrange_multipliers = y;
    results.inner_iterations = status.num_inner_iterations;
    results.outer_iterations = status.num_outer_iterations;
    results.norm_fpr = status.last_problem_norm_fpr;
    results.penalty = status.penalty;
    results.status = (int) status.exit_status;

    mpc_pub.publish(results);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  rosenbrock_free(cache);


  return 0;
}
