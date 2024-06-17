#include <iostream>
#include <random>

#include "candle.hpp"
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <vector>
#include "modern_robotics.h"
using Eigen::VectorXd;

int main()
{
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


    candle.controlMd80Mode(ids[0], mab::Md80Mode_E::RAW_TORQUE);
    candle.controlMd80Mode(ids[1], mab::Md80Mode_E::RAW_TORQUE);	 // Set mode to impedance control
    candle.controlMd80Mode(ids[2], mab::Md80Mode_E::RAW_TORQUE);
    candle.controlMd80Enable(ids[0], true);
    candle.controlMd80Enable(ids[1], true);
    candle.controlMd80Enable(ids[2], true);
    candle.controlMd80SetEncoderZero(ids[0]);
    candle.controlMd80SetEncoderZero(ids[1]);
    candle.controlMd80SetEncoderZero(ids[2]);
    candle.begin();
    VectorXd gravity(3);
    gravity<<0,0,-9.8;

    for (int i = 0; i < 10; i++)
    {
        VectorXd thetalist(2);
        thetalist<<candle.md80s[0].getPosition(),candle.md80s[1].getPosition();
        VectorXd dthetalist(2);
        dthetalist<<candle.md80s[0].getVelocity(),candle.md80s[1].getVelocity();
        VectorXd ddthetalist(2);
        ddthetalist<<candle.md80s[0].get,candle.md80s[1].getVelocity();


        VectorXd joint_torque(3);
        joint_torque = mr::InverseDynamics();


}