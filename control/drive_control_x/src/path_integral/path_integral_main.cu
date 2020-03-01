#define __CUDACC_VER__ __CUDACC_VER_MAJOR__ * 10000 + __CUDACC_VER_MINOR__ * 100 + __CUDACC_VER_BUILD__

#include "drive_control_x/path_integral/meta_math.h"
#include "drive_control_x/path_integral/param_getter.h"
#include "drive_control_x/path_integral/adam_plant.h"
#include "drive_control_x/PathIntegralParamsConfig.h"
#include "drive_control_x/path_integral/costs.cuh"

//Including neural net model
#ifdef MPPI_NNET_USING_CONSTANT_MEM__
__device__ __constant__ float NNET_PARAMS[param_counter(6,32,32,4)];
#endif

#include "drive_control_x/path_integral/neural_net_model.cuh"
#include "drive_control_x/path_integral/car_bfs.cuh"
#include "drive_control_x/path_integral/car_kinematics.cuh"
#include "drive_control_x/path_integral/generalized_linear.cuh"
#include "drive_control_x/path_integral/mppi_controller.cuh"
#include "drive_control_x/path_integral/run_control_loop.cuh"

#include <ros/ros.h>
#include <atomic>
#include <math.h> 

#include <stdlib.h>
#include <stdio.h>

using namespace drive_control;

#ifdef USE_NEURAL_NETWORK_MODEL__ /*Use neural network dynamics model*/
const int MPPI_NUM_ROLLOUTS__ = 1920;
const int BLOCKSIZE_X = 8;
const int BLOCKSIZE_Y = 16;
typedef NeuralNetModel<7,2,3,6,32,32,4> DynamicsModel;
#elif USE_BASIS_FUNC_MODEL__ /*Use the basis function model* */
const int MPPI_NUM_ROLLOUTS__ = 2560;
const int BLOCKSIZE_X = 16;
const int BLOCKSIZE_Y = 4;
typedef GeneralizedLinear<CarBasisFuncs, 7, 2, 25, CarKinematics, 3> DynamicsModel;
#endif

//Convenience typedef for the MPPI Controller.
typedef MPPIController<DynamicsModel, MPPICosts, MPPI_NUM_ROLLOUTS__, BLOCKSIZE_X, BLOCKSIZE_Y> Controller;

int main(int argc, char** argv){
    //Ros node initialization
    ros::init(argc, argv, "mppi_controller");
    
    ros::NodeHandle mppi_node("~");

    //Load setup parameters
    SystemParams params;
    loadParams(&params, mppi_node);

    //Define the mppi costs
    MPPICosts* costs = new MPPICosts(mppi_node);

    //Define the internal dynamics model for mppi
    float2 control_constraints[2] = {make_float2(-.99, .99), make_float2(-.99, params.max_throttle)};
    DynamicsModel* model = new DynamicsModel(1.0/params.hz, control_constraints);
    model->loadParams(params.model_path); //Load the model parameters from the launch file specified path//Define the internal dynamics model for mppi


    int optimization_stride = getRosParam<int>("optimization_stride", mppi_node);

    //Define the controller
    float init_u[2] = {(float)params.init_steering, (float)params.init_throttle};
    float exploration_std[2] = {(float)params.steering_std, (float)params.throttle_std};
    Controller* mppi = new Controller(model, costs, params.num_timesteps, params.hz, params.gamma, exploration_std, 
                                        init_u, params.num_iters, optimization_stride);

    AdamPlant* robot = new AdamPlant(mppi_node, mppi_node, params.debug_mode, params.hz, false);

    //Setup dynamic reconfigure callback
    dynamic_reconfigure::Server<PathIntegralParamsConfig> server;
    dynamic_reconfigure::Server<PathIntegralParamsConfig>::CallbackType callback_f;
    callback_f = boost::bind(&AdamPlant::dynRcfgCall, robot, _1, _2);
    server.setCallback(callback_f);

    boost::thread optimizer;

    std::atomic<bool> is_alive(true);
    optimizer = boost::thread(&runControlLoop<Controller>, mppi, robot, &params, &mppi_node, &is_alive);

    ros::spin();

    //Shutdown procedure
    is_alive.store(false);
    optimizer.join();
    robot->shutdown();
    mppi->deallocateCudaMem();
    delete robot;
    delete mppi;
    delete costs;
    delete model;
}