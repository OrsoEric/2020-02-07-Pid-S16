/****************************************************************
**	OrangeBot Project
*****************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************
**	Project
*****************************************************************
**   Compiler Flage:
**	-fkeep-inline-functions
****************************************************************/

/****************************************************************
**	DESCRIPTION
****************************************************************
**
****************************************************************/

/****************************************************************
**	HISTORY VERSION
****************************************************************
**
****************************************************************/

/****************************************************************
**	KNOWN BUGS
****************************************************************
**
****************************************************************/

/****************************************************************
**	TODO
****************************************************************
**
****************************************************************/

/****************************************************************
**	INCLUDES
****************************************************************/

//Standard C Libraries
#include <cstdio>
#include <cstdlib>

//Standard C++ libraries
#include <iostream>
//#include <array>
//#include <vector>
//#include <queue>
//#include <string>
//#include <fstream>
//#include <chrono>
//#include <thread>

//OS Libraries
//#define _WIN32_WINNT 0x0500	//Enable GetConsoleWindow
//#include <windows.h>

//User Libraries


//Include user log trace
#include "at_utils.h"
//Debug
#define ENABLE_DEBUG
#include "debug.h"
//Control system library
#include "Pid_s16.h"

/****************************************************************
**	NAMESPACES
****************************************************************/

//Never use a whole namespace. Use only what you need from it.
using std::cout;
using std::endl;

/****************************************************************
**	DEFINES
****************************************************************/

//Number of steps to simulate
#define NUM_STEPS				1024

	//!TESTS - Activate only one

//Test the natural response of the system under test
//#define TEST_SYSTEM_RESPONSE

//Use input as reference and use a PID to drive the system under test
#define TEST_PID
//32b reference and feedback. Error and command limited to 16b
//#define TEST_PID32

//Limit the output command of the PID
//#define TEST_PID_SATCMD

//Test PID unlock error callback
//#define TEST_PID_SATCMD_ERR

/****************************************************************
**	MACROS
****************************************************************/

/****************************************************************
**	PROTOTYPES
****************************************************************/

extern bool pid_s16_test_bench( void );

//System under test is a simple low-pass filter
extern int16_t test_system( int16_t input, int16_t max_delta, int16_t min_delta );
//Test motor with encoder. 16b command. 32b feedback
extern int32_t test_system32( int16_t input );

//Return true when output does not change for a given threshold of samples
extern bool detect_stability( int16_t in, int stability_th );

//Error handler automatically called by the PID in case of error
extern void my_error_handler( void );

/****************************************************************
**	GLOBAL VARIABILES
****************************************************************/

//User::Dummy my_class;

bool g_flag = false;

/****************************************************************
**	FUNCTIONS
****************************************************************/

/****************************************************************************
**	Function
**	main |
****************************************************************************/
//! @return bool |
//! @brief dummy method to copy the code
//! @details verbose description
/***************************************************************************/

int main()
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Start Debugging. Show function nesting level 0 and above
	DSTART( 0 );
	//Trace Enter main
	DENTER();

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------

	cout << "OrangeBot Projects\n";
	//print in the 'debug.log' file. works just like a fully featured printf
	DPRINT("OrangeBot Projects\n");

	pid_s16_test_bench();

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return from main
	DRETURN();
	//Stop Debugging
	DSTOP();

    return 0;
}	//end function: main

/****************************************************************************
**	Function
**	pid_s16_test_bench | bool
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief dummy method to copy the code
//! @details verbose description
/***************************************************************************/

bool pid_s16_test_bench( void )
{
	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", 0);

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	int t;
	//int16_t in;
	//int16_t out;

	int stability_th = 500;
	bool f_stability;

	FILE *g_csv = nullptr;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	g_csv = fopen("trace.csv","w+");

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:



		//----------------------------------------------------------------
		//	TEST_SYSTEM_RESPONSE
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_SYSTEM_RESPONSE

	in = 256;
	DPRINT("Test natural response on a constant input: %d", in);

	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false))
	{
		out = test_system( in, +100, -100 );
		f_stability = detect_stability( out, stability_th );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}


	#endif // TEST_SYSTEM_RESPONSE


		//----------------------------------------------------------------
		//	TEST_PID
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_PID

	//Construct a PID instance
	Orangebot::Pid_s16 my_pid = Orangebot::Pid_s16();

	//Fetch initial reference position

	int16_t feedback = test_system(0, +127, -127);
	int16_t reference = 100;

	int16_t cmd;

	my_pid.gain_kp() = +0;
	my_pid.gain_ki() = +1000;
	my_pid.gain_kd() = +100;
	my_pid.set_limit_cmd( -127, +127 );

	fprintf( g_csv, "KP;KI;KD;PID_FP;IN_TH;OUT_TH;SLOPE\n");
	fprintf( g_csv, "%d;%d;%d;%d;%d;%d;%d\n", my_pid.gain_kp(), my_pid.gain_ki(), my_pid.gain_kd(), Orangebot::Pid_s16_config::GAIN_FP, Orangebot::Pid_s16_config::TWO_SLOPE_INPUT_TH, Orangebot::Pid_s16_config::TWO_SLOPE_OUTPUT_TH, Orangebot::Pid_s16_config::TWO_SLOPE_GAIN_FP );
	fprintf( g_csv, "KP;KI;KD\n");
	fprintf( g_csv, "%f;%f;%f\n", 1.0f *my_pid.gain_kp() /(1<<Orangebot::Pid_s16_config::GAIN_FP), 1.0f *my_pid.gain_ki() /(1<<Orangebot::Pid_s16_config::GAIN_FP), 1.0f *my_pid.gain_kd() /(1<<Orangebot::Pid_s16_config::GAIN_FP) );
	fprintf( g_csv, "step;reference;feedback;err;cmd;slew_rate\n");

	DPRINT("Test natural response on a constant input: %d\n", reference);

	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false))
	{
		fprintf( g_csv, "%d;%d;%d;", t, reference,feedback);
		cmd = my_pid.exe( reference, feedback );
		fprintf( g_csv, "%d;", my_pid.get_err());
		fprintf( g_csv, "%d;", cmd);
		fprintf( g_csv, "%d;", my_pid.get_slew_rate());
		fprintf( g_csv, "\n");


		feedback = test_system(cmd, +127, -127);
		f_stability = detect_stability( feedback, stability_th );

		DPRINT(" step: %5d | reference: %5d | command: %5d | output: %5d\n", t, reference, cmd, feedback );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}


	#endif // TEST_PID

		//----------------------------------------------------------------
		//	TEST_PID32
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_PID32

	//Construct a PID instance
	Orangebot::Pid_s16 my_pid = Orangebot::Pid_s16();

	//Fetch initial reference position

	int32_t feedback = test_system32(0);
	int32_t reference = feedback+1000;

	int16_t cmd;

	const int16_t reference_change_th = 500;

	my_pid.gain_kp() = +5000;
	my_pid.gain_ki() = +100;
	my_pid.gain_kd() = +3000;
	my_pid.set_limit_cmd( -127, +127 );

	fprintf( g_csv, "KP;KI;KD;PID_FP;IN_TH;OUT_TH;SLOPE\n");
	fprintf( g_csv, "%d;%d;%d;%d;%d;%d;%d\n", my_pid.gain_kp(), my_pid.gain_ki(), my_pid.gain_kd(), Orangebot::Pid_s16_config::GAIN_FP, Orangebot::Pid_s16_config::TWO_SLOPE_INPUT_TH, Orangebot::Pid_s16_config::TWO_SLOPE_OUTPUT_TH, Orangebot::Pid_s16_config::TWO_SLOPE_GAIN_FP );
	fprintf( g_csv, "KP;KI;KD\n");
	fprintf( g_csv, "%f;%f;%f\n", 1.0f *my_pid.gain_kp() /(1<<Orangebot::Pid_s16_config::GAIN_FP), 1.0f *my_pid.gain_ki() /(1<<Orangebot::Pid_s16_config::GAIN_FP), 1.0f *my_pid.gain_kd() /(1<<Orangebot::Pid_s16_config::GAIN_FP) );
	fprintf( g_csv, "step;reference;feedback;err;cmd;slew_rate\n");

	DPRINT("Test natural response on a constant input: %d\n", reference);

	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false))
	{
		if (t == reference_change_th)
		{
			reference += -2000;
		}
		fprintf( g_csv, "%d;%d;%d;", t, reference,feedback);
		cmd = my_pid.exe( reference, feedback );
		fprintf( g_csv, "%d;", my_pid.get_err());
		fprintf( g_csv, "%d;", cmd);
		fprintf( g_csv, "%d;", my_pid.get_slew_rate());
		fprintf( g_csv, "\n");

		feedback = test_system32( cmd );
		f_stability = detect_stability( feedback, stability_th );

		DPRINT(" step: %5d | reference: %5d | command: %5d | output: %5d\n", t, reference, cmd, feedback );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}


	#endif // TEST_PID32

		//----------------------------------------------------------------
		//	TEST_PID_SATCMD
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_PID_SATCMD

	//Construct a PID instance
	Orangebot::Pid_s16 my_pid = Orangebot::Pid_s16();

	int16_t reference = 128;
	int16_t cmd;

	my_pid.gain_kp()		= 1024;
	my_pid.gain_kd()		= -32;
	my_pid.gain_ki()		= 64;
	my_pid.limit_cmd_max()	= +256;
	my_pid.limit_cmd_min()	= -256;


	DPRINT("Test natural response on a constant input: %d\n", reference);

	out = test_system( 0, +100, -100 );;
	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false))
	{
		cmd = my_pid.exe( reference, out );

		out = test_system( cmd, +100, -100 );
		f_stability = detect_stability( out, stability_th );

		DPRINT(" step: %5d | reference: %5d | command: %5d | output: %5d\n", t, reference, cmd, out );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}

	#endif // TEST_PID_SATCMD

		//----------------------------------------------------------------
		//	TEST_PID_SATCMD_ERR
		//----------------------------------------------------------------
		//Test the natural response of the system under test

	#ifdef TEST_PID_SATCMD_ERR

	//Construct a PID instance
	Orangebot::Pid_s16 my_pid = Orangebot::Pid_s16();

	int16_t reference = 256;
	int16_t cmd;

	my_pid.gain_kp() = -1024;
	my_pid.gain_kd() = 0;
	my_pid.gain_ki() = 0;
	my_pid.limit_cmd_max() = +256;
	my_pid.limit_cmd_min() = -256;

	my_pid.limit_sat_th() = 16;

	DPRINT("Test natural response on a constant input: %d\n", reference);

	out = test_system( 0, +100, -100 );;
	t = 0;
	f_stability = false;

	while ((t < NUM_STEPS) && (f_stability == false) && (g_flag == false))
	{
		//Execute Control system
		cmd = my_pid.exe( reference, out );
		//Check if protection kicked in
		if (my_pid.get_pid_status() == true)
		{
			my_pid.reset();
			DPRINT("ERR: Saturation detected! Reset!\n");
		}

		out = test_system( cmd, +100, -100 );
		f_stability = detect_stability( out, stability_th );

		DPRINT(" step: %5d | reference: %5d | command: %5d | output: %5d\n", t, reference, cmd, out );
		t++;
	}
	if (f_stability == true)
	{
		DPRINT("stability reached after %d steps\n", t -stability_th);
	}


	#endif // TEST_PID_SATCMD_ERR

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	fclose(g_csv);
	g_csv = nullptr;

	//Trace Return vith return value
	DRETURN_ARG("out: %d\n", 0);

	return true; //OK
}	//end function: pid_s16_test_bench | bool

/****************************************************************************
**	Function
**	test_system |
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief System under test is a simple low-pass filter
//! @details
/***************************************************************************/

int16_t test_system( int16_t input, int16_t max_delta, int16_t min_delta )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//internal accumulator
	static int16_t acc = 0;
	//Residual for the rounding
	static int16_t res = 0;

	int16_t delta_in, delta_out;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", input);

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	//two samples derivative
	delta_in = input -acc +res;
	//Reset residual
	res = 0;
	//Time constant
	delta_out = SSHR_RTO( delta_in, 4 );
	//Accumulate residual of operation
	res += delta_in -delta_out *(1<<4);
	DPRINT("res: %d\n", res);


	if (delta_out > max_delta)
	{
		delta_out = max_delta;

	}
	else if (delta_out < min_delta)
	{
		delta_out = min_delta;
	}

	acc += delta_out;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN_ARG("out: %d\n", acc);

	return acc; //OK
}	//end function: Dummy | test_system

/****************************************************************************
**	@brief Function
**	test_system32 |
****************************************************************************/
//! @param f bool
//! @return bool |
//! @details
//! It's a motor with inertia
//! Command is speed
//! Speed takes time to change
/***************************************************************************/

int32_t test_system32( int16_t input )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//Initial position
	static int32_t pos = 1000000;
	//Inertial speed
	static int16_t spd = 0;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", input);

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:


	//Low pass filter
	spd = spd *7 +input * 1;

	spd = SSHR_RTO( spd, 3 );
	//spd = spd /8;
	//Integrate position
	pos = pos +spd;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN_ARG("out: %d\n", pos);

	return pos; //OK
}	//end function: Dummy | test_system


/****************************************************************************
**	Function
**	detect_stability | int16_t, int
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief
//! @details Return true when output does not change for a given threshold of samples
/***************************************************************************/

//
bool detect_stability( int16_t in, int stability_th )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	static int16_t old = in;

	static int cnt = 0;

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", in);

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	if (in == old)
	{
		cnt++;
		if (cnt >= stability_th)
		{
			DRETURN_ARG("Stability Detected!\n");
			//System is stable
			return true;
		}

	}
	else
	{
		cnt=0;
	}

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	old = in;

	//Trace Return vith return value
	DRETURN();

	return false; //OK
}	//end function: detect_stability | int16_t, int

/****************************************************************************
**	Function
**	Dummy | bool
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief Error handler automatically called by the PID in case of error
//! @details verbose description
/***************************************************************************/

void my_error_handler( void )
{
	//Trace Enter with arguments
	DENTER();

	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------



	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	DPRINT("PID Unlock error detected!!! Manually stop PID\n");

	g_flag = true;

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN();

	return; //OK
}	//end function: Dummy | bool

/****************************************************************************
**	Function
**	Dummy | bool
****************************************************************************/
//! @param f bool
//! @return bool |
//! @brief dummy method to copy the code
//! @details verbose description
/***************************************************************************/

bool f( bool f )
{
	//----------------------------------------------------------------
	//	VARS
	//----------------------------------------------------------------

	//----------------------------------------------------------------
	//	INIT
	//----------------------------------------------------------------

	//Trace Enter with arguments
	DENTER_ARG("in: %d\n", 0);

	//----------------------------------------------------------------
	//	BODY
	//----------------------------------------------------------------
	//! @details algorithm:

	//----------------------------------------------------------------
	//	RETURN
	//----------------------------------------------------------------

	//Trace Return vith return value
	DRETURN_ARG("out: %d\n", 0);

	return false; //OK
}	//end function: Dummy | bool
