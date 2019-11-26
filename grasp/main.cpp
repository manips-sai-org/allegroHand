//
// 20141209: kcchang: changed window version to linux 

// myAllegroHand.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>  //_getch
#include <string.h>
#include <pthread.h>
#include "canAPI.h"
#include "rDeviceAllegroHandCANDef.h"
// #include "RockScissorsPaper.h"
#include <BHand/BHand.h>
#include <time.h>

// ============= For REDIS =================

#include <hiredis/hiredis.h>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <json/json.h>

// =========================================

#define CHECK(X) if ( !X || X->type == REDIS_REPLY_ERROR ) { printf("Error\n"); exit(-1); }

typedef char    TCHAR;
#define _T(X)   X
#define _tcsicmp(x, y)   strcmp(x, y)

using namespace std;

// redis keys
// read
const string ALLGERO_CONTROL_MODE = "allegroHand::controller::control_mode";
const string ALLGERO_TORQUE_COMMANDED = "allegroHand::controller::joint_torques_commanded";
const string ALLGERO_POSITION_COMMANDED = "allegroHand::controller::joint_positions_commanded";
const string ALLGERO_PALM_ORIENTATION = "allegroHand::controller::palm_orientation";

// write
const string ALLEGRO_CURRENT_POSITIONS = "allegroHand::sensors::joint_positions";
const string ALLEGRO_CURRENT_VELOCITIES = "allegroHand::sensors::joint_velocities";

/** Global constants for REDIS host and port. */
static const string REDIS_HOST = "127.0.0.1";
static const int REDIS_PORT = 6379;

/** Global REDIS interface variables */
redisContext *GLOBAL_Redis_Context;
redisReply *GLOBAL_Redis_Reply;
bool redis_initialized = false;


/////////////////////////////////////////////////////////////////////////////////////////
// for CAN communication
const double delT = 0.003;
int CAN_Ch = 0;
bool ioThreadRun = false;
pthread_t        hThread;
int recvNum = 0;
int sendNum = 0;
double statTime = -1.0;
AllegroHand_DeviceMemory_t vars;

double curTime = 0.0;

/////////////////////////////////////////////////////////////////////////////////////////

// hand mode
string hand_control_mode = "t";    // t for torque, p for position

// palm rotation to read from redis
double R_palm[] = {
	1, 0, 0,
	0, 1, 0,
	0, 0, 1
};

// for default pd controller
double kp_default[] = {
	1.8, 1.8, 1.8, 1.8,
	1.8, 1.8, 1.8, 1.8,
	1.8, 1.8, 1.8, 1.8,
	1.8, 1.8, 1.8, 1.8
};
double kv_default[] = {
	0.07, 0.07, 0.07, 0.07,
	0.07, 0.07, 0.07, 0.07,
	0.07, 0.07, 0.07, 0.07,
	0.07, 0.07, 0.07, 0.07
};

// velocity filters
// bool use_butterworth_filter = false;
// bool use_maverage_filter = false;

// moving average filter
const bool use_maverage_filter = true;
const int filter_buffer_size = 5;
double dq_filtered[MAX_DOF] = {0};
double dq_buffer[filter_buffer_size][MAX_DOF] = {0};
int filter_counter = 0;


// // butterworth filter filter
// double dq_filter_input[MAX_DOF];
// double dq_prev_filter_input[MAX_DOF];
// double dq_prev_prev_filter_input[MAX_DOF];
// double dq_filtered[MAX_DOF];
// double dq_prev_filtered[MAX_DOF];
// double dq_prev_prev_filtered[MAX_DOF];
// double gain = 2.419823131e+01;                              // 25 Hz
// double filter_coeffs[] = {-0.5136414053, 1.3483400678};
// // double gain = 4.020427297e+00;                                 // 75 Hz
// // double filter_coeffs[] = {-0.1774700802, 0.1825509574};

// moving average filter

// unsigned long long counter = 0;

/////////////////////////////////////////////////////////////////////////////////////////
// for BHand library
BHand* pBHand = NULL;
double q[MAX_DOF];
double dq[MAX_DOF];
double q_prev[MAX_DOF];
double q_des[MAX_DOF];
double tau_des[MAX_DOF];
double cur_des[MAX_DOF];
double gravity_torque[MAX_DOF];
double control_torque[MAX_DOF];

// USER HAND CONFIGURATION
const bool	RIGHT_HAND = true;
const int	HAND_VERSION = 3;

const double tau_cov_const_v2 = 800.0; // 800.0 for SAH020xxxxx
const double tau_cov_const_v3 = 1200.0; // 1200.0 for SAH030xxxxx

//Added from another .cpp https://github.com/simlabrobotics/allegro_hand_windows/blob/master/myAllegroHand.cpp
//they are used in ioThreadProc for setting a max level for the pwm
const short pwm_max_DC8V = 800; // 1200 is max
const short pwm_max_DC24V = 500;
const bool  DC_24V = false;

//const double enc_dir[MAX_DOF] = { // SAH020xxxxx
//	1.0, -1.0, 1.0, 1.0,
//	1.0, -1.0, 1.0, 1.0,
//	1.0, -1.0, 1.0, 1.0,
//	1.0, 1.0, -1.0, -1.0
//};
//const double motor_dir[MAX_DOF] = { // SAH020xxxxx
//	1.0, 1.0, 1.0, 1.0,
//	1.0, -1.0, -1.0, 1.0,
//	-1.0, 1.0, 1.0, 1.0,
//	1.0, 1.0, 1.0, 1.0
//};
//const int enc_offset[MAX_DOF] = { // SAH020CR020
//	-611, -66016, 1161, 1377,
//	-342, -66033, -481, 303,
//	30, -65620, 446, 387,
//	-3942, -626, -65508, -66768
//};
//const int enc_offset[MAX_DOF] = { // SAH020BR013
//	-391,	-64387,	-129,	 532,
//	 178,	-66030,	-142,	 547,
//	-234,	-64916,	 7317,	 1923,
//	 1124,	-1319,	-65983, -65566
//};

const double enc_dir[MAX_DOF] = { // SAH030xxxxx
	1.0, 1.0, 1.0, 1.0,
	1.0, 1.0, 1.0, 1.0,
	1.0, 1.0, 1.0, 1.0,
	1.0, 1.0, 1.0, 1.0
};
const double motor_dir[MAX_DOF] = { // SAH030xxxxx
	1.0, 1.0, 1.0, 1.0,
	1.0, 1.0, 1.0, 1.0,
	1.0, 1.0, 1.0, 1.0,
	1.0, 1.0, 1.0, 1.0
};
//const int enc_offset[MAX_DOF] = { // SAH030AR023
//	-1700, -568, -3064, -36,
//	-2015, -1687, 188, -772,
//	-3763, 782, -3402, 368,
//	1059, -2547, -692, 2411
//};
//const int enc_offset[MAX_DOF] = { // SAH030AL026
//	699, 1654, 5, -464,
//	-47, 1640, 325, 687,
//	-361, 1161, -259, -510,
//	-1563, 569, 470, -812
//};
// const int enc_offset[MAX_DOF] = { // SAH030C033R
//   -1591, -277, 545, 168,
//   -904, 53, -233, -1476,
//   2, -987, -230, -106,
//   -1203, 361, 327, 565
// };

int enc_offset[MAX_DOF] = { // SAH030BR027. Found in AllegroHand wiki, Stanford customer page
	697, 243, 722, -1075,
	508, 215, -900, 682,
	-804, -575, -1696, 1920,
	1254, -2384, 623, -135 
};

///////////////////////////////////////////////////////////

bool initializeRedis()
{
	GLOBAL_Redis_Reply = NULL;
	GLOBAL_Redis_Context = redisConnect(REDIS_HOST.c_str(), REDIS_PORT);
	if (GLOBAL_Redis_Context->err) {
		cerr << "Error: " <<  GLOBAL_Redis_Context->errstr << endl;
		return false;
	} else {
		cout << "REDIS Connection Successful.\n" << endl;
		redisCommand(GLOBAL_Redis_Context, "SET %s t", ALLGERO_CONTROL_MODE.c_str());
		redisCommand(GLOBAL_Redis_Context, "SET %s [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]", ALLGERO_TORQUE_COMMANDED.c_str());
		redisCommand(GLOBAL_Redis_Context, "SET %s [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]", ALLGERO_POSITION_COMMANDED.c_str());
		redisCommand(GLOBAL_Redis_Context, "SET %s [[1,0,0],[0,1,0],[0,0,1]]", ALLGERO_PALM_ORIENTATION.c_str());
		return true;
	}
}

string convertHandPosOrVelToStringArrayJSON(double* double_array)
{
	stringstream ss;
	string return_string_array = "[";
	for(int i=0; i<16; ++i)
	{
		if(i>0) 
		{
			return_string_array.append(",");
		}
		ss << double_array[i];
		return_string_array.append(ss.str());
		ss.str(std::string());
	}
	return_string_array.append("]");
	return return_string_array;
}

void convertTorqueOrPosCommandToDoubleArray(const std::string &string_array, double* return_double_array)
{
	// std::array<double, 16> return_double_array = {0};
	Json::Value jval;
	Json::Reader json_reader;
	if(!json_reader.parse(string_array,jval))
	{ 
		throw runtime_error("Could not parse redis value to JSON in convertTorqueOrPosCommandToDoubleArray\n"); 
	}

	if(!jval.isArray())
	{
		throw runtime_error("Parsed value is not array in convertTorqueOrPosCommandToDoubleArray\n"); 
	}
	unsigned int nrows = jval.size();
	if(nrows < 1) 
	{
		throw runtime_error("Parsed value is empty in convertTorqueOrPosCommandToDoubleArray\n"); 
	}

	bool is_matrix = jval[0].isArray();
	if(!is_matrix)
	{
		if(nrows != 16)
		{
			throw runtime_error("Parsed value does not have 16 elements in convertTorqueOrPosCommandToDoubleArray\n"); 
		}
		for(int i=0; i<16; ++i) 
		{
			return_double_array[i] = jval[i].asDouble();
		}
	}
	else
	{
		throw runtime_error("Parsed value is a Matrix in convertTorqueOrPosCommandToDoubleArray\n"); 
	}
	// return return_double_array;
}

void convertRPalmToDoubleArray(const std::string &string_array, double* return_double_array)
{
	// std::array<double, 9> return_double_array = {1,0,0,0,1,0,0,0,1};
	Json::Value jval;
	Json::Reader json_reader;
	if(!json_reader.parse(string_array,jval))
	{ 
		throw runtime_error("Could not parse redis value to JSON in convertRPalmToDoubleArray\n"); 
	}

	if(!jval.isArray())
	{
		throw runtime_error("Parsed value is not array in convertRPalmToDoubleArray\n"); 
	}
	unsigned int nrows = jval.size();
	if(nrows < 1) 
	{
		throw runtime_error("Parsed value is empty in convertRPalmToDoubleArray\n"); 
	}
	if(nrows !=3) 
	{
		throw runtime_error("Parsed value does not have 3 rows in convertRPalmToDoubleArray\n"); 
	}
	for(int i=0 ; i<3 ; i++)
	{
		if(!jval[i].isArray())
		{
			throw runtime_error("non array line in convertRPalmToDoubleArray\n"); 
		}
		if(jval[i].size() != 3)
		{
			throw runtime_error("incorrect line size (not 3) in convertRPalmToDoubleArray\n"); 
		}
	}

	for(int i=0 ; i<3 ; i++)
	{
		for(int j=0 ; j<3 ; j++)
		{
			return_double_array[3*i + j] = jval[i][j].asDouble();
		}
	}
	// return return_double_array;
}

void getSetRedisCommands()
{ 
	// get commands
	redisAppendCommand(GLOBAL_Redis_Context,"GET %s", ALLGERO_CONTROL_MODE.c_str());
	redisAppendCommand(GLOBAL_Redis_Context,"GET %s", ALLGERO_TORQUE_COMMANDED.c_str());
	redisAppendCommand(GLOBAL_Redis_Context,"GET %s", ALLGERO_POSITION_COMMANDED.c_str());
	redisAppendCommand(GLOBAL_Redis_Context,"GET %s", ALLGERO_PALM_ORIENTATION.c_str());

	// set commands
	redisAppendCommand(GLOBAL_Redis_Context,"SET %s %s", ALLEGRO_CURRENT_POSITIONS.c_str(),convertHandPosOrVelToStringArrayJSON(q).c_str());
	redisAppendCommand(GLOBAL_Redis_Context,"SET %s %s", ALLEGRO_CURRENT_VELOCITIES.c_str(),convertHandPosOrVelToStringArrayJSON(dq).c_str());

	// // Read (and process) the repliy to the 4 get command 
	int r;
	// control mode
	r = redisGetReply(GLOBAL_Redis_Context, (void **) &GLOBAL_Redis_Reply );
	if ( r == REDIS_ERR ) { printf("redis Error\n"); exit(-1); }
	CHECK(GLOBAL_Redis_Reply);   
	hand_control_mode = GLOBAL_Redis_Reply->str;
	// torque commanded
	r = redisGetReply(GLOBAL_Redis_Context, (void **) &GLOBAL_Redis_Reply );
	if ( r == REDIS_ERR ) { printf("redis Error\n"); exit(-1); }
	CHECK(GLOBAL_Redis_Reply);   
	convertTorqueOrPosCommandToDoubleArray(GLOBAL_Redis_Reply->str, control_torque);
	// position commanded
	r = redisGetReply(GLOBAL_Redis_Context, (void **) &GLOBAL_Redis_Reply );
	if ( r == REDIS_ERR ) { printf("redis Error\n"); exit(-1); }
	CHECK(GLOBAL_Redis_Reply);   
	convertTorqueOrPosCommandToDoubleArray(GLOBAL_Redis_Reply->str, q_des);
	// palm orientation
	r = redisGetReply(GLOBAL_Redis_Context, (void **) &GLOBAL_Redis_Reply );
	if ( r == REDIS_ERR ) { printf("redis Error\n"); exit(-1); }
	CHECK(GLOBAL_Redis_Reply);   
	convertRPalmToDoubleArray(GLOBAL_Redis_Reply->str, R_palm);

	// Read (and ignore) the replies to the set commands
	for(int i=0 ; i<2 ; i++)
	{
		r = redisGetReply(GLOBAL_Redis_Context, (void **) &GLOBAL_Redis_Reply );
		if ( r == REDIS_ERR ) { printf("Error\n"); exit(-1); }
		CHECK(GLOBAL_Redis_Reply);        
		// freeReplyObject(GLOBAL_Redis_Reply);
	}  
}


/////////////////////////////////////////////////////////////////////////////////////////
// functions declarations
char Getch();
void PrintInstruction();
void MainLoop();
bool OpenCAN();
void CloseCAN();
int GetCANChannelIndex(const TCHAR* cname);
bool CreateBHandAlgorithm();
void DestroyBHandAlgorithm();
void ComputeTorqueCustom();
void ComputeTorque();

/////////////////////////////////////////////////////////////////////////////////////////
// Read keyboard input (one char) from stdin
char Getch()
{
	/*#include <unistd.h>   //_getch*/
	/*#include <termios.h>  //_getch*/
	char buf=0;
	struct termios old={0};
	fflush(stdout);
	if(tcgetattr(0, &old)<0)
		perror("tcsetattr()");
	old.c_lflag&=~ICANON;
	old.c_lflag&=~ECHO;
	old.c_cc[VMIN]=1;
	old.c_cc[VTIME]=0;
	if(tcsetattr(0, TCSANOW, &old)<0)
		perror("tcsetattr ICANON");
	if(read(0,&buf,1)<0)
		perror("read()");
	old.c_lflag|=ICANON;
	old.c_lflag|=ECHO;
	if(tcsetattr(0, TCSADRAIN, &old)<0)
		perror ("tcsetattr ~ICANON");
	printf("%c\n",buf);
	return buf;
}

/////////////////////////////////////////////////////////////////////////////////////////
// CAN communication thread
static void* ioThreadProc(void* inst)
{
	char id_des;
	char id_cmd;
	char id_src;
	int len;
	unsigned char data[8];
	unsigned char data_return = 0;
	// int i;

	struct timespec start, end;
	double time_spent = 0;

	while (ioThreadRun)
	{
		// wait for next loop
		struct timespec t1, t2;
		t1.tv_sec = 0;
		t1.tv_nsec = (int) (1e9 * (delT - time_spent));
		nanosleep(&t1, &t2);

		clock_gettime(CLOCK_REALTIME, &start);

		/* wait for the event */
		while (0 == get_message(CAN_Ch, &id_cmd, &id_src, &id_des, &len, data, FALSE))
		{
			switch (id_cmd)
			{
				case ID_CMD_QUERY_ID:
				{
					printf(">CAN(%d): AllegroHand revision info: 0x%02x%02x\n", CAN_Ch, data[3], data[2]);
					printf("                      firmware info: 0x%02x%02x\n", data[5], data[4]);
					printf("                      hardware type: 0x%02x\n", data[7]);
				}
				break;

				case ID_CMD_AHRS_POSE:
				{
					printf(">CAN(%d): AHRS Roll : 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
					printf("               Pitch: 0x%02x%02x\n", data[2], data[3]);
					printf("               Yaw  : 0x%02x%02x\n", data[4], data[5]);
				}
				break;

				case ID_CMD_AHRS_ACC:
				{
					printf(">CAN(%d): AHRS Acc(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
					printf("               Acc(y): 0x%02x%02x\n", data[2], data[3]);
					printf("               Acc(z): 0x%02x%02x\n", data[4], data[5]);
				}
				break;

				case ID_CMD_AHRS_GYRO:
				{
					printf(">CAN(%d): AHRS Angular Vel(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
					printf("               Angular Vel(y): 0x%02x%02x\n", data[2], data[3]);
					printf("               Angular Vel(z): 0x%02x%02x\n", data[4], data[5]);
				}
				break;

				case ID_CMD_AHRS_MAG:
				{
					printf(">CAN(%d): AHRS Magnetic Field(x): 0x%02x%02x\n", CAN_Ch, data[0], data[1]);
					printf("               Magnetic Field(y): 0x%02x%02x\n", data[2], data[3]);
					printf("               Magnetic Field(z): 0x%02x%02x\n", data[4], data[5]);
				}
				break;

				case ID_CMD_QUERY_CONTROL_DATA:
				{
					if (id_src >= ID_DEVICE_SUB_01 && id_src <= ID_DEVICE_SUB_04)
					{
						vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 0] = (int)(data[0] | (data[1] << 8));
						vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 1] = (int)(data[2] | (data[3] << 8));
						vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 2] = (int)(data[4] | (data[5] << 8));
						vars.enc_actual[(id_src-ID_DEVICE_SUB_01)*4 + 3] = (int)(data[6] | (data[7] << 8));
						data_return |= (0x01 << (id_src-ID_DEVICE_SUB_01));
						recvNum++;
					}
					if (data_return == (0x01 | 0x02 | 0x04 | 0x08))
					{
						// convert encoder count to joint angle
						for (int i=0; i<MAX_DOF; i++)
						{
							q_prev[i] = q[i];
							q[i] = (double)(vars.enc_actual[i]*enc_dir[i]-32768-enc_offset[i])*(333.3/65536.0)*(3.141592/180.0);
						}

						// compute velocity
						for (int i = 0; i < MAX_DOF; i++)    
						{
							dq[i] = (q[i] - q_prev[i]) / delT;
						}

						// filter velocity
						if(use_maverage_filter)
						{

							for(int i=0 ; i<MAX_DOF ; i++)
							{
								dq_buffer[filter_counter][i] = dq[i];
								double dqi = 0;
								for(int j=0 ; j<filter_buffer_size ; j++)
								{
									dqi += dq_buffer[j][i];
								}
								dq_filtered[i] = dqi / filter_buffer_size;
							}
							filter_counter = (filter_counter + 1) % filter_buffer_size;

						}
						else
						{
							for(int i=0 ; i<MAX_DOF ; i++)
							{
								dq_filtered[i] = dq[i];
							}
						}

						// read and write to redis
						if(redis_initialized)
						{
							// cout << "update_redis" << endl;
							getSetRedisCommands();
						}

						// compute gravity compensation
						if(!pBHand) 
						{
							void* dummy = NULL;
							return dummy;
						}
						pBHand->SetOrientation(R_palm);
						pBHand->SetJointPosition(q); // tell BHand library the current joint positions
						pBHand->UpdateControl(0);
						pBHand->GetJointTorque(gravity_torque);
						gravity_torque[12] = 0;

						// compute joint torque
						for (int i = 0; i < MAX_DOF; i++)  
						{
							if(hand_control_mode == "t")
							{
								tau_des[i] = control_torque[i] + gravity_torque[i];
							}
							else if(hand_control_mode == "p")
							{
								control_torque[i] = -kp_default[i]*(q[i]-q_des[i]) - kv_default[i]*dq_filtered[i];
								tau_des[i] = control_torque[i] + gravity_torque[i];
							}
							else
							{
								tau_des[i] = gravity_torque[i];
							}
						}  

						// convert desired torque to desired current and PWM count
						for (int i=0; i<MAX_DOF; i++)
						{
							cur_des[i] = tau_des[i] * motor_dir[i];
							if (cur_des[i] > 1.0) cur_des[i] = 1.0;
							else if (cur_des[i] < -1.0) cur_des[i] = -1.0;
						}

						// send torques
						for (int i=0; i<4;i++)
						{
							// the index order for motors is different from that of encoders
							switch (HAND_VERSION)
							{
								case 1:
								case 2:
								vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v2);
								vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v2);
								vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v2);
								vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v2);
								break;

								case 3:
								default:
								vars.pwm_demand[i*4+3] = (short)(cur_des[i*4+0]*tau_cov_const_v3);
								vars.pwm_demand[i*4+2] = (short)(cur_des[i*4+1]*tau_cov_const_v3);
								vars.pwm_demand[i*4+1] = (short)(cur_des[i*4+2]*tau_cov_const_v3);
								vars.pwm_demand[i*4+0] = (short)(cur_des[i*4+3]*tau_cov_const_v3);
								break;
							}

							// Added from https://github.com/simlabrobotics/allegro_hand_windows/blob/master/myAllegroHand.cpp
							if (DC_24V) 
							{
								for (int j=0; j<4; j++) 
								{
									if (vars.pwm_demand[i*4+j] > pwm_max_DC24V) vars.pwm_demand[i*4+j] = pwm_max_DC24V;
									else if (vars.pwm_demand[i*4+j] < -pwm_max_DC24V) vars.pwm_demand[i*4+j] = -pwm_max_DC24V;
								}
							} 
							else 
							{
								for (int j=0; j<4; j++) 
								{
									if (vars.pwm_demand[i*4+j] > pwm_max_DC8V) vars.pwm_demand[i*4+j] = pwm_max_DC8V;
									else if (vars.pwm_demand[i*4+j] < -pwm_max_DC8V) vars.pwm_demand[i*4+j] = -pwm_max_DC8V;
								}
							}

							write_current(CAN_Ch, i, &vars.pwm_demand[4*i]);
							usleep(5);
						}
						sendNum++;
						curTime += delT;

						data_return = 0;
					}
				}
				break;
			}
		}
		clock_gettime(CLOCK_REALTIME, &end);

		// time_spent = end - start
		time_spent = (end.tv_sec - start.tv_sec) +
		          (end.tv_nsec - start.tv_nsec) / 1000000000.0;

	}

	return NULL;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Application main-loop. just a while true for now
void MainLoop()
{

	bool bRun = true;
	pBHand->SetMotionType(eMotionType_GRAVITY_COMP);

	if(!initializeRedis()) {cout<<"Redis initialization failed";exit(1);}
	redis_initialized = true;
	// int prev_c = 0;
	while (bRun)
	{

	}
}


/////////////////////////////////////////////////////////////////////////////////////////
// Open a CAN data channel
bool OpenCAN()
{
#if defined(PEAKCAN)
	CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
#elif defined(IXXATCAN)
	CAN_Ch = 1;
#elif defined(SOFTINGCAN)
	CAN_Ch = 1;
#else
	CAN_Ch = 1;
#endif
	CAN_Ch = GetCANChannelIndex(_T("USBBUS1"));
	printf(">CAN(%d): open\n", CAN_Ch);

	int ret = command_can_open(CAN_Ch);
	if(ret < 0)
	{
		printf("ERROR command_canopen !!! \n");
		return false;
	}

	ioThreadRun = true;

	/* initialize condition variable */
	int ioThread_error = pthread_create(&hThread, NULL, ioThreadProc, 0);
	printf(">CAN: starts listening CAN frames\n");

	printf(">CAN: query system id\n");
	ret = command_can_query_id(CAN_Ch);
	if(ret < 0)
	{
		printf("ERROR command_can_query_id !!! \n");
		command_can_close(CAN_Ch);
		return false;
	}

	printf(">CAN: AHRS set\n");
	ret = command_can_AHRS_set(CAN_Ch, AHRS_RATE_100Hz, AHRS_MASK_POSE | AHRS_MASK_ACC);
	if(ret < 0)
	{
		printf("ERROR command_can_AHRS_set !!! \n");
		command_can_close(CAN_Ch);
		return false;
	}

	printf(">CAN: system init\n");
	ret = command_can_sys_init(CAN_Ch, 3/*msec*/);
	if(ret < 0)
	{
		printf("ERROR command_can_sys_init !!! \n");
		command_can_close(CAN_Ch);
		return false;
	}

	printf(">CAN: start periodic communication\n");
	ret = command_can_start(CAN_Ch);

	if(ret < 0)
	{
		printf("ERROR command_can_start !!! \n");
		command_can_stop(CAN_Ch);
		command_can_close(CAN_Ch);
		return false;
	}

	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Close CAN data channel
void CloseCAN()
{
	printf(">CAN: stop periodic communication\n");
	int ret = command_can_stop(CAN_Ch);
	if(ret < 0)
	{
		printf("ERROR command_can_stop !!! \n");
	}

	if (ioThreadRun)
	{
		printf(">CAN: stoped listening CAN frames\n");
		ioThreadRun = false;
		int status;
		pthread_join(hThread, (void **)&status);
		hThread = 0;
	}

	printf(">CAN(%d): close\n", CAN_Ch);
	ret = command_can_close(CAN_Ch);
	if(ret < 0) printf("ERROR command_can_close !!! \n");
}

/////////////////////////////////////////////////////////////////////////////////////////
// Load and create grasping algorithm
bool CreateBHandAlgorithm()
{
	if (RIGHT_HAND)
		pBHand = bhCreateRightHand();
	else
		pBHand = bhCreateLeftHand();

	if (!pBHand) return false;
	pBHand->SetTimeInterval(delT);
	return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
// Destroy grasping algorithm
void DestroyBHandAlgorithm()
{
	if (pBHand)
	{
#ifndef _DEBUG
		delete pBHand;
#endif
		pBHand = NULL;
	}
}

////////////////////////////////////////////////////////////////////////////////////////
// Print program information and keyboard instructions
void PrintInstruction()
{
	printf("--------------------------------------------------\n");
	printf("myAllegroHand: ");
	if (RIGHT_HAND) printf("Right Hand, v%i.x\n\n", HAND_VERSION); else printf("Left Hand, v%i.x\n\n", HAND_VERSION);

	printf("Keyboard Commands:\n");
	printf("H: Home Position (PD control)\n");
	printf("R: Ready Position (used before grasping)\n");
	printf("G: Three-Finger Grasp\n");
	printf("K: Four-Finger Grasp\n");
	printf("P: Two-finger pinch (index-thumb)\n");
	printf("M: Two-finger pinch (middle-thumb)\n");
	printf("E: Envelop Grasp (all fingers)\n");
	printf("A: Gravity Compensation\n\n");

	printf("O: Servos OFF (any grasp cmd turns them back on)\n");
	printf("Q: Quit this program\n");

	printf("--------------------------------------------------\n\n");
}
/////////////////////////////////////////////////////////////////////////////////////////
// Get channel index for Peak CAN interface
int GetCANChannelIndex(const TCHAR* cname)
{
	if (!cname) return 0;

	if (!_tcsicmp(cname, _T("0")) || !_tcsicmp(cname, _T("PCAN_NONEBUS")) || !_tcsicmp(cname, _T("NONEBUS")))
		return 0;
	else if (!_tcsicmp(cname, _T("1")) || !_tcsicmp(cname, _T("PCAN_ISABUS1")) || !_tcsicmp(cname, _T("ISABUS1")))
		return 1;
	else if (!_tcsicmp(cname, _T("2")) || !_tcsicmp(cname, _T("PCAN_ISABUS2")) || !_tcsicmp(cname, _T("ISABUS2")))
		return 2;
	else if (!_tcsicmp(cname, _T("3")) || !_tcsicmp(cname, _T("PCAN_ISABUS3")) || !_tcsicmp(cname, _T("ISABUS3")))
		return 3;
	else if (!_tcsicmp(cname, _T("4")) || !_tcsicmp(cname, _T("PCAN_ISABUS4")) || !_tcsicmp(cname, _T("ISABUS4")))
		return 4;
	else if (!_tcsicmp(cname, _T("5")) || !_tcsicmp(cname, _T("PCAN_ISABUS5")) || !_tcsicmp(cname, _T("ISABUS5")))
		return 5;
	else if (!_tcsicmp(cname, _T("7")) || !_tcsicmp(cname, _T("PCAN_ISABUS6")) || !_tcsicmp(cname, _T("ISABUS6")))
		return 6;
	else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS7")) || !_tcsicmp(cname, _T("ISABUS7")))
		return 7;
	else if (!_tcsicmp(cname, _T("8")) || !_tcsicmp(cname, _T("PCAN_ISABUS8")) || !_tcsicmp(cname, _T("ISABUS8")))
		return 8;
	else if (!_tcsicmp(cname, _T("9")) || !_tcsicmp(cname, _T("PCAN_DNGBUS1")) || !_tcsicmp(cname, _T("DNGBUS1")))
		return 9;
	else if (!_tcsicmp(cname, _T("10")) || !_tcsicmp(cname, _T("PCAN_PCIBUS1")) || !_tcsicmp(cname, _T("PCIBUS1")))
		return 10;
	else if (!_tcsicmp(cname, _T("11")) || !_tcsicmp(cname, _T("PCAN_PCIBUS2")) || !_tcsicmp(cname, _T("PCIBUS2")))
		return 11;
	else if (!_tcsicmp(cname, _T("12")) || !_tcsicmp(cname, _T("PCAN_PCIBUS3")) || !_tcsicmp(cname, _T("PCIBUS3")))
		return 12;
	else if (!_tcsicmp(cname, _T("13")) || !_tcsicmp(cname, _T("PCAN_PCIBUS4")) || !_tcsicmp(cname, _T("PCIBUS4")))
		return 13;
	else if (!_tcsicmp(cname, _T("14")) || !_tcsicmp(cname, _T("PCAN_PCIBUS5")) || !_tcsicmp(cname, _T("PCIBUS5")))
		return 14;
	else if (!_tcsicmp(cname, _T("15")) || !_tcsicmp(cname, _T("PCAN_PCIBUS6")) || !_tcsicmp(cname, _T("PCIBUS6")))
		return 15;
	else if (!_tcsicmp(cname, _T("16")) || !_tcsicmp(cname, _T("PCAN_PCIBUS7")) || !_tcsicmp(cname, _T("PCIBUS7")))
		return 16;
	else if (!_tcsicmp(cname, _T("17")) || !_tcsicmp(cname, _T("PCAN_PCIBUS8")) || !_tcsicmp(cname, _T("PCIBUS8")))
		return 17;
	else if (!_tcsicmp(cname, _T("18")) || !_tcsicmp(cname, _T("PCAN_USBBUS1")) || !_tcsicmp(cname, _T("USBBUS1")))
		return 18;
	else if (!_tcsicmp(cname, _T("19")) || !_tcsicmp(cname, _T("PCAN_USBBUS2")) || !_tcsicmp(cname, _T("USBBUS2")))
		return 19;
	else if (!_tcsicmp(cname, _T("20")) || !_tcsicmp(cname, _T("PCAN_USBBUS3")) || !_tcsicmp(cname, _T("USBBUS3")))
		return 20;
	else if (!_tcsicmp(cname, _T("21")) || !_tcsicmp(cname, _T("PCAN_USBBUS4")) || !_tcsicmp(cname, _T("USBBUS4")))
		return 21;
	else if (!_tcsicmp(cname, _T("22")) || !_tcsicmp(cname, _T("PCAN_USBBUS5")) || !_tcsicmp(cname, _T("USBBUS5")))
		return 22;
	else if (!_tcsicmp(cname, _T("23")) || !_tcsicmp(cname, _T("PCAN_USBBUS6")) || !_tcsicmp(cname, _T("USBBUS6")))
		return 23;
	else if (!_tcsicmp(cname, _T("24")) || !_tcsicmp(cname, _T("PCAN_USBBUS7")) || !_tcsicmp(cname, _T("USBBUS7")))
		return 24;
	else if (!_tcsicmp(cname, _T("25")) || !_tcsicmp(cname, _T("PCAN_USBBUS8")) || !_tcsicmp(cname, _T("USBBUS8")))
		return 25;
	else if (!_tcsicmp(cname, _T("26")) || !_tcsicmp(cname, _T("PCAN_PCCBUS1")) || !_tcsicmp(cname, _T("PCCBUS1")))
		return 26;
	else if (!_tcsicmp(cname, _T("27")) || !_tcsicmp(cname, _T("PCAN_PCCBUS2")) || !_tcsicmp(cname, _T("PCCBUS2")))
		return 271;
	else
		return 0;
}


/////////////////////////////////////////////////////////////////////////////////////////
// Program main
int main(int argc, TCHAR* argv[])
{
	PrintInstruction();

	memset(&vars, 0, sizeof(vars));
	memset(q, 0, sizeof(q));
	memset(q_des, 0, sizeof(q_des));
	memset(tau_des, 0, sizeof(tau_des));
	memset(cur_des, 0, sizeof(cur_des));
	curTime = 0.0;

	if (CreateBHandAlgorithm() && OpenCAN())
		MainLoop();

	CloseCAN();
	DestroyBHandAlgorithm();

	return 0;
}

