# ROS2 Package: parametric_optimizer_ros2

## Generation

This is an auto-generated ROS2 package, created using the following Python code
using [OpEn](https://alphaville.github.io/optimization-engine/docs/python-ros2)

```python
import opengen as og
import casadi.casadi as cs

u = cs.SX.sym("u", 5)
p = cs.SX.sym("p", 2)
phi = og.functions.rosenbrock(u, p)

problem = og.builder.Problem(u, p, phi) \
    .with_constraints(og.constraints.Ball2(None, 1.5))

meta = og.config.OptimizerMeta() \
    .with_optimizer_name("rosenbrock_ros2")

ros2_config = og.config.RosConfiguration() \
    .with_package_name("parametric_optimizer_ros2") \
    .with_node_name("open_node_ros2") \
    .with_rate(10)

build_config = og.config.BuildConfiguration() \
    .with_build_directory("my_optimizers") \
    .with_ros2(ros2_config)

builder = og.builder.OpEnOptimizerBuilder(problem, meta, build_config)
builder.build()
```

## Installation and Setup

Move or link the auto-generated ROS2 package (folder `parametric_optimizer_ros2`) to your workspace source tree (typically `~/ros2_ws/src/`).

From within the folder `parametric_optimizer_ros2`, compile with:

```bash
colcon build --packages-select parametric_optimizer_ros2
source install/setup.bash 
# or source install/setup.zsh on MacOS
```

If you want to activate logging (recommended), do

```bash
mkdir -p .ros_log
export ROS_LOG_DIR="$PWD/.ros_log"
```


## Launch and Use

Start the optimizer in one terminal. The process stays in the foreground while
the node is running.

```bash
# Terminal 1
source install/setup.bash 
# or: source install/setup.zsh
ros2 run parametric_optimizer_ros2 open_node_ros2
```

In a second terminal, source the same environment and verify discovery:

```bash
# Terminal 2
source install/setup.bash   
# or: source install/setup.zsh
ros2 node list --no-daemon --spin-time 5
ros2 topic list --no-daemon --spin-time 5
```

You should see the node `/open_node_ros2`, the input topic
`/parameters`, and the output topic
`/result`.

Then publish a request to the configured parameters topic
(default: `/parameters`):

```bash
ros2 topic pub --once /parameters parametric_optimizer_ros2/msg/OptimizationParameters "{parameter: [YOUR_PARAMETER_VECTOR], initial_guess: [INITIAL_GUESS_OPTIONAL], initial_y: [], initial_penalty: 15.0}"
```

The result will be announced on the configured result topic
(default: `/result`):

```bash
ros2 topic echo /result
```

To get the optimal solution you can do:

```bash
ros2 topic echo /result --field solution
```


## Messages

This package involves two messages: `OptimizationParameters`
and `OptimizationResult`, which are used to define the input
and output values to the node. `OptimizationParameters` specifies
the parameter vector, the initial guess (optional), the initial
guess for the vector of Lagrange multipliers and the initial value
of the penalty value. `OptimizationResult` is a message containing
all information related to the solution of the optimization
problem, including the optimal solution, the solver status,
solution time, Lagrange multiplier vector and more.

The message structures are defined in the following msg files:

- [`OptimizationParameters.msg`](msg/OptimizationParameters.msg)
- [`OptimizationResult.msg`](msg/OptimizationResult.msg)


## Configure

You can configure the rate and topic names by editing
[`config/open_params.yaml`](config/open_params.yaml).


## Directory structure and contents

The following auto-generated files are included in your ROS2 package:

```txt
├── CMakeLists.txt
├── config
│   └── open_params.yaml
├── extern_lib
│   └── librosenbrock.a
├── include
│   ├── open_optimizer.hpp
│   └── rosenbrock_bindings.hpp
├── launch
│   └── open_optimizer.launch.py
├── msg
│   ├── OptimizationParameters.msg
│   └── OptimizationResult.msg
├── package.xml
├── README.md
└── src
    └── open_optimizer.cpp
```
