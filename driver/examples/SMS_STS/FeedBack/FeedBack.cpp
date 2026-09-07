#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

int main(int argc, char **argv)
{
	int servo_id = 22;
	int baud = 1000000;
	if(argc<2){
        std::cout<<"Usage: "<<argv[0]<<" <serial_port> [servo_id] [baud]"<<std::endl;
        std::cout<<"Default: servo_id=22, baud=1000000"<<std::endl;
        return 0;
	}
	if(argc>=3) servo_id = atoi(argv[2]);
	if(argc>=4) baud = atoi(argv[3]);

	std::cout<<"serial:"<<argv[1]<<" servo_id:"<<servo_id<<" baud:"<<baud<<std::endl;
    if(!sm_st.begin(baud, argv[1])){
        std::cout<<"Failed to init sms/sts motor!"<<std::endl;
        return 0;
    }

	int Pos, Speed, Load, Voltage, Temper, Move, Current;

	// Method 1: FeedBack (bulk read all status at once)
	std::cout<<"\n=== FeedBack (bulk read) ==="<<std::endl;
	if(sm_st.FeedBack(servo_id)!=-1){
		Pos = sm_st.ReadPos(-1);
		Speed = sm_st.ReadSpeed(-1);
		Load = sm_st.ReadLoad(-1);
		Voltage = sm_st.ReadVoltage(-1);
		Temper = sm_st.ReadTemper(-1);
		Move = sm_st.ReadMove(-1);
		Current = sm_st.ReadCurrent(-1);
		std::cout<<"Position    = "<<Pos<<std::endl;
		std::cout<<"Speed       = "<<Speed<<std::endl;
		std::cout<<"Load        = "<<Load<<std::endl;
		std::cout<<"Voltage     = "<<Voltage<<" (x10 = "<<Voltage/10.0<<"V)"<<std::endl;
		std::cout<<"Temperature = "<<Temper<<"C"<<std::endl;
		std::cout<<"Move        = "<<Move<<std::endl;
		std::cout<<"Current     = "<<Current<<std::endl;
	}else{
		std::cout<<"FeedBack failed - no response from servo "<<servo_id<<std::endl;
	}

	// Method 2: Individual reads
	std::cout<<"\n=== Individual Reads ==="<<std::endl;

	Pos = sm_st.ReadPos(servo_id);
	std::cout<<"Position:    "<<(Pos!=-1 ? std::to_string(Pos) : "READ ERROR")<<std::endl;

	Voltage = sm_st.ReadVoltage(servo_id);
	std::cout<<"Voltage:     "<<(Voltage!=-1 ? std::to_string(Voltage) : "READ ERROR")<<std::endl;

	Temper = sm_st.ReadTemper(servo_id);
	std::cout<<"Temperature: "<<(Temper!=-1 ? std::to_string(Temper) : "READ ERROR")<<std::endl;

	Speed = sm_st.ReadSpeed(servo_id);
	std::cout<<"Speed:       "<<(Speed!=-1 ? std::to_string(Speed) : "READ ERROR")<<std::endl;

	Load = sm_st.ReadLoad(servo_id);
	std::cout<<"Load:        "<<(Load!=-1 ? std::to_string(Load) : "READ ERROR")<<std::endl;

	Current = sm_st.ReadCurrent(servo_id);
	std::cout<<"Current:     "<<(Current!=-1 ? std::to_string(Current) : "READ ERROR")<<std::endl;

	Move = sm_st.ReadMove(servo_id);
	std::cout<<"Move:        "<<(Move!=-1 ? std::to_string(Move) : "READ ERROR")<<std::endl;

	// Ping test
	std::cout<<"\n=== Ping Test ==="<<std::endl;
	int pingResult = sm_st.Ping(servo_id);
	std::cout<<"Ping ID "<<servo_id<<": "<<(pingResult!=-1 ? "SUCCESS" : "FAILED - no response")<<std::endl;

	sm_st.end();
	return 1;
}
