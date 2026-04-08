#ifndef THRUST_ALLOCATION_SOLVER_HPP
#define THRUST_ALLOCATION_SOLVER_HPP


#include <OsqpEigen/OsqpEigen.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Sparse>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>

class ThrustAllocationSolver
{
public:
    ThrustAllocationSolver() : num_dof(6), is_initialized(false) {};
    void addThruster(const Eigen::Vector3d &position,
                     const Eigen::Vector3d &axis,
                     double max_thrust,
                     double k_direction=1, // 推力方向系数 (正负表示推力方向)
                     double k_torque=1,    // 扭矩产生系数 (旋转的反作用力矩)
                     double k_force=500      // 推力产生系数 (根据轴向计算)
    );
    bool initialize();
    bool solve(const Eigen::VectorXd &T_desired, Eigen::VectorXd &k_solution);

private:
    struct ThrusterConfig
    {
        Eigen::Vector3d position;
        Eigen::Vector3d axis;
        double k_direction; // 推力方向系数 (正负表示推力方向)
        double k_torque;    // 扭矩产生系数 (旋转的反作用力矩)
        double k_force;     // 推力产生系数 (根据轴向计算)
        double max_thrust;
    };
    int num_thrusters;
    int num_dof;
    int num_constraints;
    std::vector<ThrusterConfig> thrusters;
    Eigen::MatrixXd B;        // 推力矩阵
    OsqpEigen::Solver solver; // OSQP 求解器实例
    bool is_initialized;      // 初始化标志位
};















#endif // THRUST_ALLOCATION_SOLVER_HPP
