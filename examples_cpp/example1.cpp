#include "candle.hpp"

#include <unistd.h>
#include <vector>
#include <iostream>
#include <cmath>

float l1 = 0.249926;
float l2 = 0.273;
double pi = 3.141592653589793238462643383279502884197;
float max = l1+l2;



std::pair<double,double> iksolver(std::pair<double,double> endpos){
    double gamma =  atan2(endpos.second,endpos.first);
    double beta = acos((pow(l1,2)+pow(l2,2)-pow(endpos.first,2)-pow(endpos.second,2))/(2*l1*l2));
    double alpha = acos((pow(l1,2)-pow(l2,2)+pow(endpos.first,2)+pow(endpos.second,2))/(2*l1*sqrt(pow(endpos.first,2)+pow(endpos.second,2))));
    double theta1 = gamma - alpha;
    double theta2 = pi - beta;
    std::pair<float,float> joint_Ang = {theta1,theta2};
    //std::cout<<joint_Ang.first<<" "<<joint_Ang.second;
    return joint_Ang;
}

std::pair<double,double> referenceTraj(double t){
    std::pair<double,double> currentpos;

    currentpos.first = max - 0.125 - t/10000;
    currentpos.second = 0.1*sin((((t/10000)*2*pi)-1.4));




    return currentpos;
}
void calibration(mab::Candle &candle, std::vector<unsigned short> &ids){
    int hip = 1;
    int knee = 2;
    candle.controlMd80Mode(ids[knee], mab::Md80Mode_E::VELOCITY_PID);	 // Set mode to impedance control
    candle.controlMd80Enable(ids[knee], true);						 // Enable the drive
    candle.controlMd80Mode(ids[hip], mab::Md80Mode_E::IDLE);	 // Set mode to impedance control
    candle.controlMd80Enable(ids[hip], true);						 // Enable the drive




    candle.begin();
    candle.md80s[knee].setTargetVelocity(1);
    candle.md80s[hip].setTargetVelocity(1);
    usleep(50000);

    while(candle.md80s[hip].getVelocity()>0.03||candle.md80s[knee].getVelocity()>0.05){
        if(candle.md80s[knee].getVelocity()<0.05){
            candle.md80s[knee].setTargetVelocity(0);
        }
        if(candle.md80s[hip].getVelocity()<0.03){
            candle.md80s[hip].setTargetVelocity(0);
        }
        usleep(50);
    }
    candle.md80s[hip].setTargetVelocity(-1);
    candle.md80s[knee].setTargetVelocity(-1);
    usleep(50000);
    candle.md80s[hip].setTargetVelocity(0.3);
    candle.md80s[knee].setTargetVelocity(0.3);
    usleep(50000);
    while(candle.md80s[hip].getVelocity()>0.03||candle.md80s[knee].getVelocity()>0.03){
        if(candle.md80s[knee].getVelocity()<0.03){
            candle.md80s[knee].setTargetVelocity(0);
        }
        if(candle.md80s[hip].getVelocity()<0.03){
            candle.md80s[hip].setTargetVelocity(0);
        }
        usleep(50);
    }
    usleep(100000);
    candle.end();

    candle.controlMd80SetEncoderZero(ids[hip]);
    candle.controlMd80SetEncoderZero(ids[knee]);

    candle.controlMd80Mode(ids[knee], mab::Md80Mode_E::IMPEDANCE);	 // Set mode to impedance control
    candle.controlMd80Mode(ids[hip], mab::Md80Mode_E::IMPEDANCE);
    candle.controlMd80Enable(ids[knee], true);
    candle.controlMd80Enable(ids[hip], true);	// Set mode to impedance control
    candle.begin();

    for(int i = 0;i <2000;i++) {
        candle.md80s[hip].setTargetPosition(-0.53 * pi);  //hip motor
        candle.md80s[knee].setTargetPosition(-1.07 * pi); //knee motor
        usleep(100);
    }
    candle.end();
    candle.controlMd80SetEncoderZero(ids[knee]);
    candle.controlMd80SetEncoderZero(ids[hip]);
};

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


    candle.controlMd80Mode(ids[1], mab::Md80Mode_E::IMPEDANCE);
    candle.controlMd80Mode(ids[2], mab::Md80Mode_E::IMPEDANCE);	 // Set mode to impedance control
    //candle.controlMd80Mode(ids[2], mab::Md80Mode_E::IMPEDANCE);
    candle.controlMd80Enable(ids[1], true);
    candle.controlMd80Enable(ids[2], true);
    //candle.controlMd80Enable(ids[2], true);
    candle.controlMd80SetEncoderZero(ids[1]);
    candle.controlMd80SetEncoderZero(ids[2]);
    //candle.controlMd80SetEncoderZero(ids[2]);
    candle.begin();

    /*for(int i = 0;i<10000;i++){
        //float timevariant = max-0.05-((0.1/1000)*i);
        std::vector<float> reference = {0.1,0.2,0.2};
        std::pair<float,float> commandpos = iksolver({sqrt(pow(reference[0],2)+pow(reference[1],2)),reference[2]});
        //std::cout<<timevariant<<"\n";
        std::cout<<commandpos.first*((float)i/1000.0)<<" "<<commandpos.second*(float)1.375*(float)(i/1000.0)<<"\n";
        candle.md80s[0].setTargetPosition((-1.0*atan(reference[1]/reference[0])));
        candle.md80s[2].setTargetPosition(commandpos.first*(i/1000.0));
        candle.md80s[1].setTargetPosition(commandpos.second*1.375*(i/1000.0));

        usleep(5000);
    }*/
    std::pair<float,float> commandpos = iksolver({max-0.1,-0.05});

    candle.md80s[1].setTargetPosition(-1*commandpos.first);  //hip motor
    candle.md80s[2].setTargetPosition(commandpos.second*1.375); //knee motor
    usleep(1000);

    for (int i = 0; i < 10; i++)
    {
        t += dt;
        // After powerup the drive will load set of default parameters for every controller.
        // To make the drive move all we got to do is set mode (.controlMd80Mode) and set target
        // (.setTargetPosition, .setTargetVelocity, .setTargetTorque)
        for (int j = 0; j<10000;j++) { // upper gait
            std::pair<float, float> reference = {max -0.1-(0.15* sin((2*pi*j)/20000)),-0.2+(j*0.35/10000)};
            commandpos = iksolver(reference);
            candle.md80s[1].setTargetPosition(-1 * commandpos.first);  //hip motor
            candle.md80s[2].setTargetPosition(commandpos.second * 1.375); //knee motor
            //std::cout <<"commandpos:" <<commandpos.first<< " "<<commandpos.second << '\n'<<'\n';
            usleep(50);    // Add some delay
        }



        for (int j = 0; j<9000; j++){
            std::pair<float, float> reference = {max -0.1,0.15-((0.35/9000)*j)};
            commandpos = iksolver(reference);

            candle.md80s[1].setTargetPosition(-1*commandpos.first);  //hip motor
            candle.md80s[2].setTargetPosition(commandpos.second*1.375); //knee motor
            usleep(50);
        }
    }



    // Close the update loop
    candle.end();
    return EXIT_SUCCESS;
}