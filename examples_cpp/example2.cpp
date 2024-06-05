#include <unistd.h>

#include <iostream>

#include "candle.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;

MatrixXd jacobianUpdater(mab::Candle &candle){
    VectorXd q;
    q<<candle.md80s[0].getPosition(),candle.md80s[0].getPosition(),candle.md80s[0].getPosition();

    VectorXd qdot;
    qdot<<candle.md80s[0].getVelocity(),candle.md80s[0].getVelocity(),candle.md80s[0].getVelocity();

    int theta = 0;
    double alpha = 1.5708;
    double d = alpha;
    MatrixXd homgen_0_1;
    homgen_0_1 << cos(q[theta]),-sin(q[theta])*cos(alpha),sin(q[theta])*sin(alpha),0,
                  sin(q[theta]),cos(q[theta])*cos(alpha),-cos(q[theta])*sin(alpha),0,
                  0,sin(alpha),cos(alpha),d,
                  0,0,0,1; //d1?
    MatrixXd rot_mat_0_1;
    rot_mat_0_1 << cos(q[theta]),0,sin(q[theta]),
                   sin(q[theta]),0,-cos(q[theta]),
                   0,1,0;

    MatrixXd homgen_1_2;
    theta = 1;
    alpha = 0;
    d = 0;
    homgen_1_2 << cos(q[theta]),-sin(q[theta])*cos(alpha),sin(q[theta])*sin(alpha),0,
            sin(q[theta]),cos(q[theta])*cos(alpha),-cos(q[theta])*sin(alpha),0,
            0,sin(alpha),cos(alpha),d,
            0,0,0,1; //d1?
    MatrixXd rot_mat_1_2;
    rot_mat_1_2 << cos(q[theta]),-sin(q[theta]),0,
                    sin(q[theta]),cos(q[theta]),0,
                    0,0,1;
    theta = 2;
    d = 0;
    MatrixXd homgen_2_3;
    homgen_2_3 << cos(q[theta]),-sin(q[theta])*cos(alpha),sin(q[theta])*sin(alpha),0,
            sin(q[theta]),cos(q[theta])*cos(alpha),-cos(q[theta])*sin(alpha),0,
            0,sin(alpha),cos(alpha),d,
            0,0,0,1; //d1?
    MatrixXd rot_mat_2_3;
    rot_mat_2_3 << cos(q[theta]),-sin(q[theta]),0,
                   sin(q[theta]),cos(q[theta]),0,
                    0,0,1;

    MatrixXd homgen_0_3 = homgen_0_1*homgen_1_2*homgen_2_3;
    MatrixXd homgen_0_2 = homgen_0_1*homgen_1_2;


    MatrixXd rot_mat_0_2 = rot_mat_0_1*rot_mat_1_2;

    Vector3d d01;
    d01 << homgen_0_1(homgen_0_1.size()-1,0),homgen_0_1(homgen_0_1.size()-1,1),homgen_0_1(homgen_0_1.size()-1,2);
    Vector3d d02;
    d02 << homgen_0_2(homgen_0_2.size()-1,0),homgen_0_2(homgen_0_2.size()-1,1),homgen_0_2(homgen_0_2.size()-1,2);
    Vector3d d03;
    d03 << homgen_0_3(homgen_0_3.size()-1,0),homgen_0_3(homgen_0_3.size()-1,1),homgen_0_3(homgen_0_3.size()-1,2);

    Vector3d idenityMatrix;
    idenityMatrix<<0,0,1;

    Vector3d topleft = idenityMatrix.cross(d03);
    Vector3d topmid = idenityMatrix.cross(d03-d01);
    Vector3d topright = idenityMatrix.cross(d03-d02);
    Vector3d bottomleft = idenityMatrix;
    Vector3d bottommid = rot_mat_0_1*idenityMatrix;
    VectorXd bottomright = rot_mat_0_2*idenityMatrix;

    MatrixXd J_6xN;
    J_6xN << topleft[0],topmid[0],topright[0],
            topleft[1],topmid[1],topright[1],
            topleft[2],topmid[2],topright[2],
            bottomleft[0],bottommid[0],bottomright[0],
            bottomleft[1],bottommid[1],bottomright[1],
            bottomleft[2],bottommid[2],bottomright[2];
    std::cout<<J_6xN;
    return J_6xN;
}










int main()
{
	// Create CANdle object and set FDCAN baudrate to 1Mbps
	mab::Candle candle(mab::CAN_BAUD_1M, true);

	// Ping FDCAN bus in search of drives
	auto ids = candle.ping();

	// Add all found to the update list
	for (auto& id : ids)
		candle.addMd80(id);

	// Begin update loop (it starts in the background)
	candle.begin();

	// Auto update loop is running in the background updating data in candle.md80s vector. Each md80 object can be
	// called for data at any time
	for (int i = 0; i < 1000; i++)
	{
        jacobianUpdater(candle);

		usleep(10000000);
	}

	// Close the update loop
	candle.end();

	return EXIT_SUCCESS;
}