# ROS Package parametric_optimizer


## Installation and Setup

Move the auto-generated ROS package (folder `parametric_optimizer`) to your catkin workspace (typically `~/catkin_ws/`):

Compile with:

```console
cd ~/catkin_ws/
catkin_make
``` 


## Launch and Use

Run with:

```
roslaunch parametric_optimizer open_optimizer.launch
```

Once you run the node, you can post your request to `parametric_optimizer/parameters` as follows:

```
rostopic pub /parametric_optimizer/parameters  \
  parametric_optimizer/OptimizationParameters  \
  "{'parameter':[YOUR_PARAMETER_VECTOR],       \
   'initial_guess':[INITIAL_GUESS (OPTIONAL)]}" --once
```

The result will be announced on `/parametric_optimizer/result`:

```
rostopic echo /parametric_optimizer/result
```

To get the optimal solution you can do

```
rostopic echo /parametric_optimizer/result/solution
```

## Messages

This package involves two messages: `OptimizationParameters` 
and `OptimizationResult`, which are used to define the input 
and output values to the node. `OptimizationParameters` specifies
the parameter vector, the initial guess (optional), the initial
guess for the vector of Lagrange vector and the initial value
of the penalty value. `OptimizationResult` is a message containing
all information related to the solution of the optimization 
problem, including the optimal solution, the solver status, 
solution time, Lagrangian vector and more. 

The message structures are defined in the following msg files:

- [`OptimizationParameters.msg`](msg/OptimizationParameters.msg)
- [`OptimizationResult.msg`](msg/OptimizationResult.msg)


## Configure

You can configure the rate and topic names by editing 
[`config/open_params.yaml`](config/open_params.yaml).


## Directory structure and contents

The following auto-generated files are included in your ROS package:

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
│   └── open_optimizer.launch
├── msg
│   ├── OptimizationParameters.msg
│   └── OptimizationResult.msg
├── package.xml
├── README.md
└── src
    └── open_optimizer.cpp
```