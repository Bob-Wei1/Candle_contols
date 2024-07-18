#include "candle.hpp"
#include <eigen-3.4.0/Eigen/Eigen>
#include <eigen-3.4.0/Eigen/Dense>
#include <iostream>



// Robot parameters
const double m1 = 0.2186+0.465, m2 = 0.08503;  // masses of the links
const double l1 = 0.21, l2 = 0.194;  // lengths of the links
const double lc1 = 0.05, lc2 = 0.1;  // center of mass of the links
const double I1 = 0.00, I2 = 0.00;  // moments of inertia of the links
const double g = 9.81;  // gravity

using namespace Eigen;

Vector2d forward_kinematics(const Vector2d& q) {
    double q1 = q(0);
    double q2 = q(1);

    // Position of the end-effector
    double x = l1 * cos(q1) + l2 * cos(q1 + q2);
    double y = l1 * sin(q1) + l2 * sin(q1 + q2);

    return Vector2d(x, y);
}

// Function to compute the inertia matrix
Matrix2d inertia_matrix(const Vector2d& q) {
    double q1 = q(0), q2 = q(1);
    Matrix2d M;
    M(0, 0) = m1 * lc1 * lc1 + m2 * (l1 * l1 + lc2 * lc2 + 2 * l1 * lc2 * cos(q2)) + I1 + I2;
    M(0, 1) = m2 * (lc2 * lc2 + l1 * lc2 * cos(q2)) + I2;
    M(1, 0) = M(0, 1);
    M(1, 1) = m2 * lc2 * lc2 + I2;
    return M;
}

// Function to compute the Coriolis and centrifugal forces matrix
Matrix2d coriolis_matrix(const Vector2d& q, const Vector2d& q_dot) {
    double q1 = q(0), q2 = q(1);
    double q1_dot = q_dot(0), q2_dot = q_dot(1);
    Matrix2d C;
    C(0, 0) = -m2 * l1 * lc2 * sin(q2) * q2_dot;
    C(0, 1) = -m2 * l1 * lc2 * sin(q2) * (q1_dot + q2_dot);
    C(1, 0) = m2 * l1 * lc2 * sin(q2) * q1_dot;
    C(1, 1) = 0;
    return C;
}

// Function to compute the gravity vector
Vector2d gravity_vector(const Vector2d& q) {
    double q1 = q(0), q2 = q(1);
    Vector2d G;
    G(0) = (m1 * lc1 + m2 * l1) * g * cos(q1) + m2 * lc2 * g * cos(q1 + q2);
    G(1) = m2 * lc2 * g * cos(q1 + q2);
    return G;
}

// Function to compute the Jacobian matrix
Matrix2d jacobian_matrix(const Vector2d& q) {
    double q1 = q(0), q2 = q(1);
    Matrix2d J;
    J(0, 0) = -l1 * sin(q1) - l2 * sin(q1 + q2);
    J(0, 1) = -l2 * sin(q1 + q2);
    J(1, 0) = l1 * cos(q1) + l2 * cos(q1 + q2);
    J(1, 1) = l2 * cos(q1 + q2);
    return J;
}

// Function to compute the impedance control force
Vector2d impedance_control(const Vector2d& x, const Vector2d& x_dot, const Vector2d& x_d, const Vector2d& x_dot_d, const Vector2d& x_ddot_d, const Matrix2d& M_d, const Matrix2d& D_d, const Matrix2d& K_d) {
    Vector2d x_error = x_d - x;
    Vector2d x_dot_error = x_dot_d - x_dot;
    Vector2d F = M_d * x_ddot_d + D_d * x_dot_error + K_d * x_error;
    return F;
}

// Function to compute the control input (torques) using inverse dynamics and impedance control
Vector2d combined_control(const Vector2d& q, const Vector2d& q_dot, const Vector2d& q_d, const Vector2d& q_dot_d, const Vector2d& q_ddot_d, const Vector2d& x_d, const Vector2d& x_dot_d, const Vector2d& x_ddot_d, const Matrix2d& Kp, const Matrix2d& Kd, const Matrix2d& M_d, const Matrix2d& D_d, const Matrix2d& K_d) {
    // Compute the current end-effector position and velocity
    Matrix2d J = jacobian_matrix(q);
    Vector2d x = J * q;
    Vector2d x_dot = J * q_dot;

    // Compute the desired force using impedance control
    Vector2d F = impedance_control(x, x_dot, x_d, x_dot_d, x_ddot_d, M_d, D_d, K_d);

    // Transform the desired force to joint torques using the Jacobian transpose
    Vector2d tau_impedance = J.transpose() * F;

    // Control input for stability (PD term)
    Vector2d u = Kp * (q_d - q) + Kd * (q_dot_d - q_dot);

    // Calculate required torques using inverse dynamics
    //Vector2d tau = inertia_matrix(q) * q_ddot_d + coriolis_matrix(q, q_dot) * q_dot + gravity_vector(q) + u + tau_impedance;
    Vector2d tau = tau_impedance;
    return tau;
}

int main() {
    // Create CANdle object and set FDCAN baudrate to 1Mbps
    mab::Candle candle(mab::CAN_BAUD_1M, true);

    // Ping FDCAN bus in search of drives
    auto ids = candle.ping();

    if (ids.size() == 0)  // If no drives found -> quit
        return EXIT_FAILURE;

    // Add all found to the update list
    for (auto& id : ids)
        candle.addMd80(id);


    // calibration(candle,ids);
    // Begin update loop (it starts in the background)

    float t = 0.0f;
    float dt = 0.04f;
    //calibration(candle,ids);


    candle.controlMd80Mode(ids[1], mab::Md80Mode_E::RAW_TORQUE);
    candle.controlMd80Mode(ids[2], mab::Md80Mode_E::RAW_TORQUE);	 // Set mode to impedance control
    candle.controlMd80Mode(ids[0], mab::Md80Mode_E::POSITION_PID);
    //candle.controlMd80Mode(ids[2], mab::Md80Mode_E::IMPEDANCE);
    candle.controlMd80Enable(ids[0], true);
    candle.controlMd80Enable(ids[1], true);
    candle.controlMd80Enable(ids[2], true);

    candle.controlMd80SetEncoderZero(ids[0]);
    candle.controlMd80SetEncoderZero(ids[1]);
    candle.controlMd80SetEncoderZero(ids[2]);
    candle.begin();
    Vector2d last_q_ddot(0,0);

    // PD control gains
    //Matrix2d Kp = Matrix2d::Identity() * 6;
    Matrix2d Kp(2,2);
    Kp<<2,0,
    0,2;
    Matrix2d Kd = Matrix2d::Identity() * 0.3;
    //std::cout<<Kp(1,1);
    // Impedance control gains
    Matrix2d M_d = Matrix2d::Identity() * 0.1;
    Matrix2d D_d = Matrix2d::Identity() * 0.1;
    Matrix2d K_d = Matrix2d::Identity() * 0.1;

    for(int i = 0; i<100000;i++) {
        // Actual joint positions and velocities
        Vector2d q(candle.md80s[1].getPosition(), candle.md80s[2].getPosition());
        Vector2d q_dot(candle.md80s[1].getVelocity(), candle.md80s[2].getVelocity());


        // Desired joint positions, velocities, and accelerations
        Vector2d q_d(0,0);
        //Vector2d q_d(-1.6408,(-3.65));
        //Vector2d q_d(0,0);
        Vector2d q_dot_d(0,0);
        Vector2d q_ddot_d(0, 0);


        // Desired end-effector positions, velocities, and accelerations

        Vector2d x_d = forward_kinematics(q_d);
        Vector2d x_dot_d(0, 0);
        Vector2d x_ddot_d(0, 0);
        std::cout<<x_d(0)<<std::endl;
        // Control input for stability (PD term)
        Vector2d u = Kp * (q_d - q) + Kd * (q_dot_d - q_dot);

        // Calculate required torques
        Vector2d tau = inertia_matrix(q) * q_ddot_d + coriolis_matrix(q, q_dot) * q_dot_d - gravity_vector(q)+u;
        Vector2d tau_impedance = combined_control(q,q_dot,q_d,q_dot_d,q_ddot_d,x_d,x_dot_d,x_ddot_d,Kp,Kd,M_d,D_d,K_d);
        std::cout << "combined joint torques: " << tau_impedance.transpose() << std::endl;
        std::cout << "current joint torques: " << candle.md80s[1].getTorque() <<" "<<candle.md80s[2].getTorque()  << std::endl;
        //std::cout << "joint position " << candle.md80s[1].getPosition() <<" "<<candle.md80s[2].getPosition()  << std::endl;

        candle.md80s[1].setTargetTorque(tau(0));
        candle.md80s[2].setTargetTorque(tau(1));

        candle.md80s[0].setTargetPosition(0);
        usleep(1000);
    }
    return 0;
}
