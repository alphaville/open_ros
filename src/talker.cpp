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


class TalkerManager {
    public:
        mpc_dummy::OptimisationParameters params;
        mpc_dummy::OptimisationResult results;
        double p[ROSENBROCK_NUM_PARAMETERS] = {0};
        double u[ROSENBROCK_NUM_DECISION_VARIABLES] = {0};

    TalkerManager() {
    }
};


static TalkerManager mng;


void mpcReceiveRequestCallback(const mpc_dummy::OptimisationParameters::ConstPtr& msg)
{
  mng.params = *msg;
}


/**
 * updateResults 
 */
void updateResults(rosenbrockSolverStatus& status) 
{
    std::vector<double> sol(mng.u, mng.u+ROSENBROCK_NUM_DECISION_VARIABLES); 
    mng.results.solution = sol;
    std::vector<double> y(status.lagrange, status.lagrange+ROSENBROCK_N1);
    mng.results.lagrange_multipliers = y;
    mng.results.inner_iterations = status.num_inner_iterations;
    mng.results.outer_iterations = status.num_outer_iterations;
    mng.results.norm_fpr = status.last_problem_norm_fpr;
    mng.results.penalty = status.penalty;
    mng.results.status = (int) status.exit_status;
}


/**
 * Main method
 */
int main(int argc, char **argv)
{
  double init_penalty = 15.0;
  rosenbrockCache *cache = rosenbrock_new();

  ros::init(argc, argv, MPC_DUMMY_NODE_NAME);
  ros::NodeHandle n(MPC_DUMMY_BASE_TOPIC);

  ros::Publisher mpc_pub 
      = n.advertise<mpc_dummy::OptimisationResult>(MPC_DUMMY_SOLUTION_TOPIC, 1000);
  ros::Subscriber sub 
      = n.subscribe(MPC_DUMMY_PARAMS_TOPIC, 1000, mpcReceiveRequestCallback);
  ros::Rate loop_rate(MPC_DUMMY_RATE);
  
 
  int count = 0;
  while (ros::ok())
  {

    if (mng.params.parameter.size() > 0) 
    {
	for (int i=0; i<ROSENBROCK_NUM_PARAMETERS; ++i) mng.p[i] = mng.params.parameter[i];
    }

    if (mng.params.initial_guess.size() == ROSENBROCK_NUM_DECISION_VARIABLES) 
    {
	for (int i=0; i<ROSENBROCK_NUM_DECISION_VARIABLES; ++i) 
            mng.u[i] = mng.params.initial_guess[i];
    }

    rosenbrockSolverStatus status 
       = rosenbrock_solve(cache, mng.u, mng.p, 0, &init_penalty);  /* solve!  */

    updateResults(status); /* pack results into `results` */
    mpc_pub.publish(mng.results); /* publish results (on /mpc/open_solution )*/

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }

  rosenbrock_free(cache);


  return 0;
}
