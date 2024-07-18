#include <unistd.h>

#include <iostream>

#include "candle.hpp"

#include <eigen-3.4.0/Eigen/Eigen>
#include <eigen-3.4.0/Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
double kp = 20;
double kd = 0.1;
double k = 0.5;

VectorXd teleop(mab::Candle &candle){
    double q1 = candle.md80s[0].getPosition();
    double q2 = candle.md80s[3].getPosition();
    double q_d1 = candle.md80s[0].getVelocity();
    double q_d2 = candle.md80s[3].getVelocity();

    double tau1 = kp*(q2-q1)+kd*(q_d2-q_d1)-k*(q_d1);
    double tau2 = kp*(q1-q2)+kd*(q_d1-q_d2)-k*(q_d2);

    //candle.md80s[0].setTargetTorque(tau1);
    //candle.md80s[3].setTargetTorque(tau2);
}


int main()
{
	// Create CANdle object and set FDCAN baudrate to 1Mbps
	mab::Candle candle(mab::CAN_BAUD_8M, true, mab::BusType_E::SPI);

	// Ping FDCAN bus in search of drives
	auto ids = candle.ping();

	if (ids.size() == 0)  // If no drives found -> quit
		return EXIT_FAILURE;

	// Add all found to the update list
	for (auto& id : ids)
		candle.addMd80(id);

    for (auto& id : ids){
        candle.controlMd80Mode(id, mab::Md80Mode_E::RAW_TORQUE);
        candle.controlMd80Enable(id, true);
    }
    for (auto& id : ids){
        candle.controlMd80SetEncoderZero(id);
    }

	// Begin update loop (it starts in the background)
	candle.begin();



	for(int i = 0; i<10000000;i++){

        //hip
        double q1 = candle.md80s[0].getPosition();
        double q2 = candle.md80s[3].getPosition();
        double q_d1 = candle.md80s[0].getVelocity();
        double q_d2 = candle.md80s[3].getVelocity();
        //thigh
        double q1_t = candle.md80s[1].getPosition();
        double q2_t = candle.md80s[4].getPosition();
        double q_d1_t = candle.md80s[1].getVelocity();
        double q_d2_t = candle.md80s[4].getVelocity();
        //knee
        double q1_k = candle.md80s[2].getPosition();
        double q2_k = candle.md80s[5].getPosition();
        double q_d1_k = candle.md80s[2].getVelocity();
        double q_d2_k = candle.md80s[5].getVelocity();

        double tau1_h = kp*(q2-q1)+kd*(q_d2-q_d1)-k*(q_d1);
        double tau2_h = kp*(q1-q2)+kd*(q_d1-q_d2)-k*(q_d2);

        double tau1_t = kp*(q2_t-q1_t)+kd*(q_d2_t-q_d1_t)-k*(q_d1_t);
        double tau2_t = kp*(q1_t-q2_t)+kd*(q_d1_t-q_d2_t)-k*(q_d2_t);

        double tau1_k = kp*(q2_k-q1_k)+kd*(q_d2_k-q_d1_k)-k*(q_d1_k);
        double tau2_k = kp*(q1_k-q2_k)+kd*(q_d1_k-q_d2_k)*(q_d2_k);

        candle.md80s[0].setTargetTorque(tau1_h);
        candle.md80s[3].setTargetTorque(tau2_h);
        candle.md80s[1].setTargetTorque(tau1_t);
        candle.md80s[4].setTargetTorque(tau2_t);
        candle.md80s[2].setTargetTorque(tau1_k);
        candle.md80s[5].setTargetTorque(tau2_k);

        usleep(1);
    }

	// Close the update loop
	candle.end();

	return EXIT_SUCCESS;
}