#include "ros/ros.h"
#include "parametric_optimizer/OptimizationResult.h"
#include "parametric_optimizer/OptimizationParameters.h"
#include "rosenbrock_bindings.hpp"
#include "open_optimizer.hpp"

namespace parametric_optimizer {
/**
 * Class parametric_optimizer::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {

private:
    parametric_optimizer::OptimizationParameters params;
    parametric_optimizer::OptimizationResult results;
    double p[ROSENBROCK_NUM_PARAMETERS] = { 0 };
    double u[ROSENBROCK_NUM_DECISION_VARIABLES] = { 0 };
		double *y = NULL;

    rosenbrockCache* cache;
    double init_penalty = ROS_NODE_ROSENBROCK_DEFAULT_INITIAL_PENALTY;

    /**
     * Publish obtained results to output topic
     */
    void publishToTopic(ros::Publisher& publisher)
    {
        publisher.publish(results);
    }

    /**
     * Updates the input data based on the data that are posted
     * on /mpc/open_parameters (copies value from topic data to
     * local variables). This method is responsible for parsing
     * the data announced on the input topic.
     */
    void updateInputData()
    {
        init_penalty = (params.initial_penalty > 1.0)
            ? params.initial_penalty
            : ROS_NODE_ROSENBROCK_DEFAULT_INITIAL_PENALTY;

        if (params.parameter.size() > 0) {
            for (size_t i = 0; i < ROSENBROCK_NUM_PARAMETERS; ++i)
                p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == ROSENBROCK_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < ROSENBROCK_NUM_DECISION_VARIABLES; ++i)
                u[i] = params.initial_guess[i];
        }

		if (params.initial_y.size() == ROSENBROCK_N1) {
            for (size_t i = 0; i < ROSENBROCK_N1; ++i)
                y[i] = params.initial_y[i];
		}

    }

    /**
     * Call OpEn to solve the problem
     */
    rosenbrockSolverStatus solve()
    {
        return rosenbrock_solve(cache, u, p, y, &init_penalty);
    }


public:
    /**
     * Constructor of OptimizationEngineManager
     */
    OptimizationEngineManager()
    {
			  y = new double[ROSENBROCK_N1];
        cache = rosenbrock_new();
    }

    /**
     * Destructor of OptimizationEngineManager
     */
    ~OptimizationEngineManager()
    {
			  if (y!=NULL) delete[] y;
        rosenbrock_free(cache);
    }

    /**
     * Copies results from `status` to the local field `results`
     */
    void updateResults(rosenbrockSolverStatus& status)
    {
        std::vector<double> sol(u, u + ROSENBROCK_NUM_DECISION_VARIABLES);
        results.solution = sol;
        std::vector<double> y(status.lagrange, status.lagrange + ROSENBROCK_N1);
        results.lagrange_multipliers = y;
        results.inner_iterations = status.num_inner_iterations;
        results.outer_iterations = status.num_outer_iterations;
        results.norm_fpr = status.last_problem_norm_fpr;
        results.penalty = status.penalty;
        results.status = (int)status.exit_status;
        results.solve_time_ms = (double)status.solve_time_ns / 1000000.0;
        results.infeasibility_f2 = status.f2_norm;
        results.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    /**
     * Callback that obtains data from topic `/parametric_optimizer/open_params`
     */
    void mpcReceiveRequestCallback(
        const parametric_optimizer::OptimizationParameters::ConstPtr& msg)
    {
        params = *msg;
    }

    void solveAndPublish(ros::Publisher& publisher)
    {
        updateInputData(); /* get input data */
        rosenbrockSolverStatus status = solve(); /* solve!  */
        updateResults(status); /* pack results into `results` */
        publishToTopic(publisher);
    }
}; /* end of class OptimizationEngineManager */

} /* end of namespace parametric_optimizer */

/**
 * Main method
 *
 * This advertises a new (private) topic to which the optimizer
 * announces its solution and solution status and details. The
 * publisher topic is 'parametric_optimizer/result'.
 *
 * It obtains inputs from 'parametric_optimizer/parameters'.
 *
 */
int main(int argc, char** argv)
{

    std::string solution_topic, params_topic;  /* parameter and solution topics */
    double rate; /* rate of node (specified by parameter) */

    parametric_optimizer::OptimizationEngineManager mng;
    ros::init(argc, argv, ROS_NODE_ROSENBROCK_NODE_NAME);
    ros::NodeHandle private_nh("~");

    private_nh.param("solution_topic", solution_topic, std::string("result"));
    private_nh.param("params_topic", params_topic, std::string(ROS_NODE_ROSENBROCK_PARAMS_TOPIC));
    private_nh.param("rate", rate, double(ROS_NODE_ROSENBROCK_RATE));

    ros::Publisher mpc_pub
        = private_nh.advertise<parametric_optimizer::OptimizationResult>(
            ROS_NODE_ROSENBROCK_SOLUTION_TOPIC,
            ROS_NODE_ROSENBROCK_SOLUTION_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = private_nh.subscribe(
            ROS_NODE_ROSENBROCK_PARAMS_TOPIC,
            ROS_NODE_ROSENBROCK_PARAMS_TOPIC_QUEUE_SIZE,
            &parametric_optimizer::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);
    ros::Rate loop_rate(ROS_NODE_ROSENBROCK_RATE);

    while (ros::ok()) {
        mng.solveAndPublish(mpc_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}