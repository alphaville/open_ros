/**
 * This is an auto-generated file by Optimization Engine (OpEn)
 * OpEn is a free open-source software - see doc.optimization-engine.xyz
 * dually licensed under the MIT and Apache v2 licences.
 *
 */
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "parametric_optimizer_ros2/msg/optimization_parameters.hpp"
#include "parametric_optimizer_ros2/msg/optimization_result.hpp"
#include "rosenbrock_ros2_bindings.hpp"
#include "open_optimizer.hpp"

namespace parametric_optimizer_ros2 {
class OptimizationEngineNode : public rclcpp::Node {
private:
    using OptimizationParametersMsg = parametric_optimizer_ros2::msg::OptimizationParameters;
    using OptimizationResultMsg = parametric_optimizer_ros2::msg::OptimizationResult;

    OptimizationParametersMsg params_;
    OptimizationResultMsg results_;
    bool has_received_request_ = false;
    double p_[ROSENBROCK_ROS2_NUM_PARAMETERS] = { 0 };
    double u_[ROSENBROCK_ROS2_NUM_DECISION_VARIABLES] = { 0 };
    double* y_ = nullptr;
    rosenbrock_ros2Cache* cache_ = nullptr;
    double init_penalty_ = ROS2_NODE_ROSENBROCK_ROS2_DEFAULT_INITIAL_PENALTY;

    rclcpp::Publisher<OptimizationResultMsg>::SharedPtr publisher_;
    rclcpp::Subscription<OptimizationParametersMsg>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    static std::chrono::milliseconds rateToPeriod(double rate)
    {
        if (rate <= 0.0) {
            return std::chrono::milliseconds(100);
        }
        int period_ms = static_cast<int>(1000.0 / rate);
        if (period_ms < 1) {
            period_ms = 1;
        }
        return std::chrono::milliseconds(period_ms);
    }

    void updateInputData()
    {
        init_penalty_ = (params_.initial_penalty > 1.0)
            ? params_.initial_penalty
            : ROS2_NODE_ROSENBROCK_ROS2_DEFAULT_INITIAL_PENALTY;

        if (params_.parameter.size() == ROSENBROCK_ROS2_NUM_PARAMETERS) {
            for (size_t i = 0; i < ROSENBROCK_ROS2_NUM_PARAMETERS; ++i) {
                p_[i] = params_.parameter[i];
            }
        }

        if (params_.initial_guess.size() == ROSENBROCK_ROS2_NUM_DECISION_VARIABLES) {
            for (size_t i = 0; i < ROSENBROCK_ROS2_NUM_DECISION_VARIABLES; ++i) {
                u_[i] = params_.initial_guess[i];
            }
        }

        if (params_.initial_y.size() == ROSENBROCK_ROS2_N1) {
            for (size_t i = 0; i < ROSENBROCK_ROS2_N1; ++i) {
                y_[i] = params_.initial_y[i];
            }
        }
    }

    rosenbrock_ros2SolverStatus solve()
    {
        return rosenbrock_ros2_solve(cache_, u_, p_, y_, &init_penalty_);
    }

    void initializeSolverIfNeeded()
    {
        if (y_ == nullptr) {
            y_ = new double[ROSENBROCK_ROS2_N1]();
        }
        if (cache_ == nullptr) {
            cache_ = rosenbrock_ros2_new();
        }
    }

    void updateResults(rosenbrock_ros2SolverStatus& status)
    {
        results_.solution.clear();
        for (size_t i = 0; i < ROSENBROCK_ROS2_NUM_DECISION_VARIABLES; ++i) {
            results_.solution.push_back(u_[i]);
        }

        results_.lagrange_multipliers.clear();
        for (size_t i = 0; i < ROSENBROCK_ROS2_N1; ++i) {
            results_.lagrange_multipliers.push_back(status.lagrange[i]);
        }

        results_.inner_iterations = status.num_inner_iterations;
        results_.outer_iterations = status.num_outer_iterations;
        results_.norm_fpr = status.last_problem_norm_fpr;
        results_.cost = status.cost;
        results_.penalty = status.penalty;
        results_.status = static_cast<uint8_t>(status.exit_status);
        results_.solve_time_ms = static_cast<double>(status.solve_time_ns) / 1000000.0;
        results_.infeasibility_f2 = status.f2_norm;
        results_.infeasibility_f1 = status.delta_y_norm_over_c;
    }

    void receiveRequestCallback(const OptimizationParametersMsg::ConstSharedPtr msg)
    {
        params_ = *msg;
        has_received_request_ = true;
    }

    void solveAndPublish()
    {
        if (!has_received_request_) {
            return;
        }
        initializeSolverIfNeeded();
        updateInputData();
        rosenbrock_ros2SolverStatus status = solve();
        updateResults(status);
        publisher_->publish(results_);
    }

public:
    OptimizationEngineNode()
        : Node(ROS2_NODE_ROSENBROCK_ROS2_NODE_NAME)
    {
        this->declare_parameter<std::string>(
            "result_topic",
            std::string(ROS2_NODE_ROSENBROCK_ROS2_RESULT_TOPIC));
        this->declare_parameter<std::string>(
            "params_topic",
            std::string(ROS2_NODE_ROSENBROCK_ROS2_PARAMS_TOPIC));
        this->declare_parameter<double>(
            "rate",
            double(ROS2_NODE_ROSENBROCK_ROS2_RATE));

        std::string result_topic = this->get_parameter("result_topic").as_string();
        std::string params_topic = this->get_parameter("params_topic").as_string();
        double rate = this->get_parameter("rate").as_double();

        publisher_ = this->create_publisher<OptimizationResultMsg>(
            result_topic,
            ROS2_NODE_ROSENBROCK_ROS2_RESULT_TOPIC_QUEUE_SIZE);
        subscriber_ = this->create_subscription<OptimizationParametersMsg>(
            params_topic,
            ROS2_NODE_ROSENBROCK_ROS2_PARAMS_TOPIC_QUEUE_SIZE,
            std::bind(&OptimizationEngineNode::receiveRequestCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            rateToPeriod(rate),
            std::bind(&OptimizationEngineNode::solveAndPublish, this));
    }

    ~OptimizationEngineNode() override
    {
        if (y_ != nullptr) {
            delete[] y_;
        }
        if (cache_ != nullptr) {
            rosenbrock_ros2_free(cache_);
        }
    }
};
} /* end of namespace parametric_optimizer_ros2 */

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<parametric_optimizer_ros2::OptimizationEngineNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}