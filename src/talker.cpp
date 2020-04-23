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


class OptimizationEngineManager {
    private:
        mpc_dummy::OptimisationParameters params;
        mpc_dummy::OptimisationResult results;
        double p[ROSENBROCK_NUM_PARAMETERS] = {0};
        double u[ROSENBROCK_NUM_DECISION_VARIABLES] = {0};
        rosenbrockCache *cache;
        double init_penalty = 15.0;

    public:
    OptimizationEngineManager() {
        cache = rosenbrock_new();
    }

    ~OptimizationEngineManager() {
        rosenbrock_free(cache);
    }

    /**
     * updateResults 
     */
    void updateResults(rosenbrockSolverStatus& status) 
    {
        std::vector<double> sol(u, u+ROSENBROCK_NUM_DECISION_VARIABLES); 
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange+ROSENBROCK_N1);
        results.lagrange_multipliers = y;
        results.inner_iterations = status.num_inner_iterations;
        results.outer_iterations = status.num_outer_iterations;
        results.norm_fpr = status.last_problem_norm_fpr;
        results.penalty = status.penalty;
        results.status = (int) status.exit_status;
    }

    void mpcReceiveRequestCallback(
           const mpc_dummy::OptimisationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void publishToTopic(ros::Publisher &publisher) 
    {
        publisher.publish(results);
    }


    void updateInputData() {
        if (params.parameter.size() > 0) 
        {
	    for (int i=0; i<ROSENBROCK_NUM_PARAMETERS; ++i) 
              p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == ROSENBROCK_NUM_DECISION_VARIABLES) 
        {
	    for (int i=0; i<ROSENBROCK_NUM_DECISION_VARIABLES; ++i) 
                u[i] = params.initial_guess[i];
        }
    }

    rosenbrockSolverStatus solve() {
        return rosenbrock_solve(cache, u, p, 0, &init_penalty);
    }
};




/**
 * Main method
 */
int main(int argc, char **argv)
{  
  OptimizationEngineManager mng;
  ros::init(argc, argv, MPC_DUMMY_NODE_NAME);
  ros::NodeHandle n(MPC_DUMMY_BASE_TOPIC);

  ros::Publisher mpc_pub 
      = n.advertise<mpc_dummy::OptimisationResult>(
                  MPC_DUMMY_SOLUTION_TOPIC, 1000);
  ros::Subscriber sub 
      = n.subscribe(
                  MPC_DUMMY_PARAMS_TOPIC, 1000, 
                  &OptimizationEngineManager::mpcReceiveRequestCallback,
                  &mng);
  ros::Rate loop_rate(MPC_DUMMY_RATE);
  
 
  int count = 0;
  while (ros::ok())
  {

    mng.updateInputData();
    rosenbrockSolverStatus status = mng.solve();  /* solve!  */
    mng.updateResults(status); /* pack results into `results` */
    mng.publishToTopic(mpc_pub);

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }



  return 0;
}
