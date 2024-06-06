#include <unistd.h>

#include <iostream>

#include "candle.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
float l1 = 0.249926;
float l2 = 0.273;
double pi = 3.141592653589793238462643383279502884197;
float max = l1+l2;

MatrixXd jacobianUpdater(mab::Candle &candle){
    VectorXd q(3);
    q<<candle.md80s[0].getPosition(),candle.md80s[2].getPosition(),candle.md80s[1].getPosition();

    VectorXd qdot(3);
    qdot<<candle.md80s[0].getVelocity(),candle.md80s[2].getVelocity(),candle.md80s[1].getVelocity();

    int theta = 0;
    double alpha = 1.5708;
    double d = 0.06;
    double r = 0;
    MatrixXd homgen_0_1(4,4);
    homgen_0_1 << cos(q[theta]),-sin(q[theta])*cos(alpha),sin(q[theta])*sin(alpha),0,
                  sin(q[theta]),cos(q[theta])*cos(alpha),-cos(q[theta])*sin(alpha),0,
                  0,sin(alpha),cos(alpha),d,
                  0,0,0,1; //d1?


    MatrixXd rot_mat_0_1(3,3);
    rot_mat_0_1 << cos(q[theta]),0,sin(q[theta]),
                   sin(q[theta]),0,-cos(q[theta]),
                   0,1,0;

    MatrixXd homgen_1_2(4,4);
    theta = 1;
    alpha = 0;
    d = 0;
    r = 0.249926; //link 2
    homgen_1_2 << cos(q[theta]),-sin(q[theta])*cos(alpha),sin(q[theta])*sin(alpha),r*cos(q[theta]),
            sin(q[theta]),cos(q[theta])*cos(alpha),-cos(q[theta])*sin(alpha),r*sin(q[theta]),
            0,sin(alpha),cos(alpha),d,
            0,0,0,1; //d1?
    MatrixXd rot_mat_1_2(3,3);
    rot_mat_1_2 << cos(q[theta]),-sin(q[theta]),0,
                    sin(q[theta]),cos(q[theta]),0,
                    0,0,1;

    theta = 2;
    d = 0;
    r = 0.273; //link 3
    MatrixXd homgen_2_3(4,4);
    homgen_2_3 << cos(q[theta]),-sin(q[theta])*cos(alpha),sin(q[theta])*sin(alpha),r*cos(q[theta]),
            sin(q[theta]),cos(q[theta])*cos(alpha),-cos(q[theta])*sin(alpha),r*sin(q[theta]),
            0,sin(alpha),cos(alpha),d,
            0,0,0,1; //d1?

    MatrixXd rot_mat_2_3(3,3);
    rot_mat_2_3 << cos(q[theta]),-sin(q[theta]),0,
                   sin(q[theta]),cos(q[theta]),0,
                    0,0,1;

    MatrixXd homgen_0_3 = homgen_0_1*homgen_1_2*homgen_2_3;
    MatrixXd homgen_0_2 = homgen_0_1*homgen_1_2;


    MatrixXd rot_mat_0_2 = rot_mat_0_1*rot_mat_1_2;

    Vector3d d01(3);
    d01 << homgen_0_1(homgen_0_1.size()-1,0),homgen_0_1(homgen_0_1.size()-1,1),homgen_0_1(homgen_0_1.size()-1,2);
    Vector3d d02(3);
    d02 << homgen_0_2(homgen_0_2.size()-1,0),homgen_0_2(homgen_0_2.size()-1,1),homgen_0_2(homgen_0_2.size()-1,2);
    Vector3d d03(3);
    d03 << homgen_0_3(homgen_0_3.size()-1,0),homgen_0_3(homgen_0_3.size()-1,1),homgen_0_3(homgen_0_3.size()-1,2);

    Vector3d idenityMatrix(3);
    idenityMatrix<<0,0,1;

    Vector3d topleft = idenityMatrix.cross(d03);
    Vector3d topmid = idenityMatrix.cross(d03-d01);
    Vector3d topright = idenityMatrix.cross(d03-d02);
    Vector3d bottomleft = idenityMatrix;
    Vector3d bottommid = rot_mat_0_1*idenityMatrix;
    VectorXd bottomright = rot_mat_0_2*idenityMatrix;

    MatrixXd J_6xN(6,3);
    J_6xN << topleft[0],topmid[0],topright[0],
            topleft[1],topmid[1],topright[1],
            topleft[2],topmid[2],topright[2],
            bottomleft[0],bottommid[0],bottomright[0],
            bottomleft[1],bottommid[1],bottomright[1],
            bottomleft[2],bottommid[2],bottomright[2];
    MatrixXd j = J_6xN *qdot;
    //std::cout<<j<<'\n'<<'\n';
    return j;
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

    std::vector<double> current_xyz = {0,0,max};
    MatrixXd j(6,1);
    for (int i = 0; i < 100000; i++)
	{

        j = jacobianUpdater(candle);
        current_xyz[0] = current_xyz[0] + (j(0,0)*0.01);
        current_xyz[1] = current_xyz[1] + (j(1,0)*0.01);
        current_xyz [2] = current_xyz[2] + (j(2,0)*0.01);

        std::cout<<current_xyz[0] <<" "<<current_xyz[1]<<" "<<current_xyz[2] <<'\n'<<'\n';
		usleep(10000);
	}

	// Close the update loop
	candle.end();

	return EXIT_SUCCESS;
}