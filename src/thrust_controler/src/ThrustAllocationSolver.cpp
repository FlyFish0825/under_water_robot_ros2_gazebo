#include "thrust_controler/ThrustAllocationSolver.hpp"

void ThrustAllocationSolver::addThruster(
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &axis,
    double max_thrust,
    double k_direction, // 推力方向系数 (正负表示推力方向)
    double k_torque ,    // 扭矩产生系数 (旋转的反作用力矩)
    double k_force      // 推力产生系数 (根据轴向计算)
)
{

    if (is_initialized)
    {
        std::cerr << "[错误] 求解器已初始化，无法再添加推进器！" << std::endl;
        return;
    }
    ThrusterConfig thruster_config;
    thruster_config.position = position;
    thruster_config.axis = axis.normalized(); // 确保轴向是单位向量
    thruster_config.k_force = k_force;        // 推力产生系数
    thruster_config.k_torque = k_torque;      // 扭矩产生系数
    thruster_config.max_thrust = max_thrust;
    thruster_config.k_direction = k_direction; // 推力方向系数
    thrusters.push_back(thruster_config);
}

bool ThrustAllocationSolver::initialize()
{
    num_thrusters = thrusters.size();
    if (num_thrusters == 0)
        return false;

    num_constraints = num_dof + num_thrusters;
    B.resize(num_dof, num_thrusters);

    // 1. 利用叉乘自动计算 B 矩阵的每一列
    for (int i = 0; i < num_thrusters; ++i)
    {
        // 计算力向量 F (三维)
        Eigen::Vector3d force = thrusters[i].k_force * thrusters[i].axis;
        // 计算力矩向量 Tau = Position x Force (三维叉乘)
        Eigen::Vector3d torque = thrusters[i].position.cross(force) + thrusters[i].k_torque * thrusters[i].axis; // 加上扭矩产生系数的贡献

        // 将力和力矩填入 B 矩阵的第 i 列
        B.block<3, 1>(0, i) = force;  // 前三行: Fx, Fy, Fz
        B.block<3, 1>(3, i) = torque; // 后三行: Roll, Pitch, Yaw
    }

    

    // 2. 构造 OSQP 的 P, q, A 矩阵
    Eigen::SparseMatrix<double> P(num_thrusters, num_thrusters);
    P.setIdentity();
    P *= 2.0;

    Eigen::VectorXd q = Eigen::VectorXd::Zero(num_thrusters);

    Eigen::SparseMatrix<double> A_osqp(num_constraints, num_thrusters);
    std::vector<Eigen::Triplet<double>> tripletList;
    tripletList.reserve(B.size() + num_thrusters);

    // 填入 B 矩阵
    for (int i = 0; i < num_dof; ++i)
    {
        for (int j = 0; j < num_thrusters; ++j)
        {
            if (B(i, j) != 0.0)
            {
                tripletList.push_back(Eigen::Triplet<double>(i, j, B(i, j)));
            }
        }
    }
    // 填入单位矩阵 I (用于电机限幅)
    for (int j = 0; j < num_thrusters; ++j)
    {
        tripletList.push_back(Eigen::Triplet<double>(num_dof + j, j, 1.0));
    }
    A_osqp.setFromTriplets(tripletList.begin(), tripletList.end());

    // 3. 配置求解器 (初始边界全部设为 0)
    Eigen::VectorXd l = Eigen::VectorXd::Zero(num_constraints);
    Eigen::VectorXd u = Eigen::VectorXd::Zero(num_constraints);

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.settings()->setAbsoluteTolerance(1e-4);
    solver.settings()->setRelativeTolerance(1e-4);

    solver.data()->setNumberOfVariables(num_thrusters);
    solver.data()->setNumberOfConstraints(num_constraints);

    if (!solver.data()->setHessianMatrix(P))
        return false;
    if (!solver.data()->setGradient(q))
        return false;
    if (!solver.data()->setLinearConstraintsMatrix(A_osqp))
        return false;
    if (!solver.data()->setLowerBound(l))
        return false;
    if (!solver.data()->setUpperBound(u))
        return false;

    if (!solver.initSolver())
        return false;

    is_initialized = true;
    return true;
}

bool ThrustAllocationSolver::solve(const Eigen::VectorXd &T_desired, Eigen::VectorXd &k_solution)
{

    if (!is_initialized)
        return false;

    Eigen::VectorXd l(num_constraints);
    Eigen::VectorXd u(num_constraints);

    // 更新目标推力约束 (等式约束)
    l.head(num_dof) = T_desired;
    u.head(num_dof) = T_desired;

    // 更新电机输出比例限制 [-1.0, 1.0]
    l.tail(num_thrusters).setConstant(-1.0);
    u.tail(num_thrusters).setConstant(1.0);

    // **核心优化**: 运行时只更新 bounds，不需要重新初始化矩阵！
    solver.updateBounds(l, u);

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    {
        return false;
    }

    k_solution = solver.getSolution();
    return true;
}