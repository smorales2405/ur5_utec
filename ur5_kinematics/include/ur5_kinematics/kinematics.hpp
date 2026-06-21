#ifndef UR5_KINEMATICS_HPP
#define UR5_KINEMATICS_HPP

#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/spatial/explog.hpp> // Necesario para pinocchio::log6
#include <cmath>
#include <algorithm>

#include <Eigen/Dense>
#include <string>
#include <memory>

#define PI 3.14159265358979323846


class UR5Kinematics {
public:
    // El constructor ahora toma la ruta al archivo URDF.
    explicit UR5Kinematics(const std::string& urdf_path);

    Eigen::VectorXd computeVelocityControlStep(
        const Eigen::VectorXd& q_real,
        const Eigen::Vector3d& desired_pos,
        const Eigen::Matrix3d& desired_orient,
        double Kp_pos,
        double Kp_orient,
        double dt); 
    
    Eigen::VectorXd inverseKinematicsQP2(
        const Eigen::VectorXd& q_initial,
        const Eigen::Vector3d& desired_pos,
        const Eigen::Matrix3d& desired_orient,
        int max_iterations,
        double alpha,
        double weight_pos = 1.0,
        double weight_orient = 0.9
    );

    // Control de Impedancia
    // Cinemática Directa (útil para obtener la pose actual)
    pinocchio::SE3 forwardKinematics(const Eigen::VectorXd& q);

    // Registra un frame operacional fijo a 'offset_from_tool0' desde tool0.
    // Permite añadir gripper_tcp u otros frames sin modificar el URDF.
    void registerFixedFrame(const std::string& name,
                            const pinocchio::SE3& offset_from_tool0);

    // Cambia el frame destino del IK / FK (debe existir en el modelo).
    void setTargetFrame(const std::string& frame_name);

private:
    std::unique_ptr<pinocchio::Model> model_;
    std::unique_ptr<pinocchio::Data> data_;
    pinocchio::FrameIndex tool_frame_id_;

    // NUEVO: Métodos privados para el solver QP
    Eigen::Matrix<double, 6, 1> computePoseError(const pinocchio::SE3& desired_pose);
    Eigen::Matrix<double, 6, 1> computePoseError2(const pinocchio::SE3& desired_pose);

    Eigen::VectorXd solveQPIK(const Eigen::MatrixXd& J, const Eigen::Matrix<double, 6, 1>& error, const Eigen::MatrixXd& W_p, const Eigen::MatrixXd& W_o);
    Eigen::VectorXd solveQPIK_Velocity(  const Eigen::MatrixXd& J, const Eigen::VectorXd& x_dot_des, const Eigen::VectorXd& current_q, double dt) ;
};

#endif // UR5_KINEMATICS_HPP