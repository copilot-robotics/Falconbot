/*
控制三个舵机(ID:22, 23, 24)同时运动演示程序
舵机出厂速度单位是0.0146rpm，速度改为V=2400
*/

#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
	if(argc<2){
        std::cout<<"argc error!"<<std::endl;
        return 0;
	}
	std::cout<<"serial:"<<argv[1]<<std::endl;
    if(!sm_st.begin(1000000, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }

	u8 servoIDs[] = {22, 23, 24};
	u8 numServos = 3;
	s16 positions[3];
	u16 speeds[3];
	u8 accs[3];

	while(1){
		positions[0] = 4095;
		positions[1] = 3072;
		positions[2] = 2048;
		speeds[0] = 2400;
		speeds[1] = 2400;
		speeds[2] = 2400;
		accs[0] = 50;
		accs[1] = 50;
		accs[2] = 50;

		sm_st.SyncWritePosEx(servoIDs, numServos, positions, speeds, accs);
		std::cout<<"Servo 22 pos = "<<positions[0]<<std::endl;
		std::cout<<"Servo 23 pos = "<<positions[1]<<std::endl;
		std::cout<<"Servo 24 pos = "<<positions[2]<<std::endl;
		usleep(2187*1000);

		positions[0] = 0;
		positions[1] = 1024;
		positions[2] = 2048;

		sm_st.SyncWritePosEx(servoIDs, numServos, positions, speeds, accs);
		std::cout<<"Servo 22 pos = "<<positions[0]<<std::endl;
		std::cout<<"Servo 23 pos = "<<positions[1]<<std::endl;
		std::cout<<"Servo 24 pos = "<<positions[2]<<std::endl;
		usleep(2187*1000);

		positions[0] = 2048;
		positions[1] = 2048;
		positions[2] = 2048;

		sm_st.SyncWritePosEx(servoIDs, numServos, positions, speeds, accs);
		std::cout<<"Servo 22 pos = "<<positions[0]<<std::endl;
		std::cout<<"Servo 23 pos = "<<positions[1]<<std::endl;
		std::cout<<"Servo 24 pos = "<<positions[2]<<std::endl;
		usleep(2187*1000);
	}
	sm_st.end();
	return 1;
}
