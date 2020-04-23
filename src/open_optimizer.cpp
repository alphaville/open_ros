#include "ros/ros.h"
#include "mpc_dummy/OptimizationResult.h"
#include "mpc_dummy/OptimizationParameters.h"
#include "rosenbrock_bindings.hpp"
#include "open_optimizer.hpp"

namespace mpc_dummy {
/**
 * Class mpc_dummy::OptimizationEngineManager manages the
 * exchange of data between the input and output topics
 * of this node
 */
class OptimizationEngineManager {

private:
    mpc_dummy::OptimizationParameters params;
    mpc_dummy::OptimizationResult results;
    double p[ROSENBROCK_NUM_PARAMETERS] = { 0 };
    double u[ROSENBROCK_NUM_DECISION_VARIABLES] = { 0 };
#if ROSENBROCK_N1 > 0
		double y[ROSENBROCK_N1] = { 0 };
#endif
    rosenbrockCache* cache;
    double init_penalty = MPC_DUMMY_DEFAULT_INITIAL_PENALTY;

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
            : MPC_DUMMY_DEFAULT_INITIAL_PENALTY;

        if (params.parameter.size() > 0) {
            for (size_t i = 0; i < ROSENBROCK_NUM_PARAMETERS; ++i)
                p[i] = params.parameter[i];
        }

        if (params.initial_guess.size() == ROSENBROCK_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < ROSENBROCK_NUM_DECISION_VARIABLES; ++i)
                u[i] = params.initial_guess[i];
        }

#if ROSENBROCK_N1 > 0
				if (params.initial_y.size() == ROSENBROCK_N1) {
            for (size_t i = 0; i < ROSENBROCK_N1; ++i)
                y[i] = params.initial_y[i];
				}
#endif

    }

    /**
     * Call OpEn to solve the problem
	   */
    rosenbrockSolverStatus solve()
    {
#if ROSENBROCK_N1 > 0
            return rosenbrock_solve(cache, u, p, y, &init_penalty);
#else
            return rosenbrock_solve(cache, u, p, 0, &init_penalty);
#endif
    }

public:
    /**
	   * Constructor of OptimizationEngineManager
	   */
    OptimizationEngineManager()
    {
        cache = rosenbrock_new();
    }

    /**
	   * Destructor of OptimizationEngineManager
	   */
    ~OptimizationEngineManager()
    {
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
		 * Callback that obtains data from topic `/mpc_dummy/open_params`
		 */
    void mpcReceiveRequestCallback(
        const mpc_dummy::OptimizationParameters::ConstPtr& msg)
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

} /* end of namespace mpc_dummy */

/**
 * Main method
 */
int main(int argc, char** argv)
{
    mpc_dummy::OptimizationEngineManager mng;
    ros::init(argc, argv, MPC_DUMMY_NODE_NAME);
    ros::NodeHandle n(MPC_DUMMY_BASE_TOPIC);

    ros::Publisher mpc_pub
        = n.advertise<mpc_dummy::OptimizationResult>(
            MPC_DUMMY_SOLUTION_TOPIC,
            MPC_DUMMY_SOLUTION_TOPIC_QUEUE_SIZE);
    ros::Subscriber sub
        = n.subscribe(
            MPC_DUMMY_PARAMS_TOPIC,
            MPC_DUMMY_PARAMS_TOPIC_QUEUE_SIZE,
            &mpc_dummy::OptimizationEngineManager::mpcReceiveRequestCallback,
            &mng);
    ros::Rate loop_rate(MPC_DUMMY_RATE);

    while (ros::ok()) {
        mng.solveAndPublish(mpc_pub);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
