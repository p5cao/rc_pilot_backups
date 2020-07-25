/**
 * @file printf_manager.c
 */

#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <time.h>
#include <stdlib.h>
#include <rc/start_stop.h>
#include <rc/time.h>
#include <rc/pthread.h>

#include <printf_manager.h>
#include <input_manager.h>
#include <setpoint_manager.h>
#include <feedback.h>
#include <thread_defs.h>
#include <settings.h>




static pthread_t printf_manager_thread;
static int initialized = 0;

// terminal emulator control sequences
#define WRAP_DISABLE	"\033[?7l"
#define WRAP_ENABLE	"\033[?7h"
#define KNRM		"\x1B[0m"	// "normal" to return to default after colour

#define KRED		"\x1B[31m"
#define KGRN		"\x1B[32m"
#define KYEL		"\x1B[33m"
#define KBLU		"\x1B[34m"
#define KMAG		"\x1B[35m"
#define KCYN		"\x1B[36m"
#define KWHT		"\x1B[37m"

FILE *flight_log;


const char* const colours[] = {KYEL, KCYN, KGRN};
const int num_colours = 3; // length of above array
int current_colour = 0;
//print time in log


/**
 * @brief      { function_description }
 *
 * @return     string with ascii colour code
 */
static const char* __next_colour()
{
	// if reached the end of the colour list, loop around
	if(current_colour>=(num_colours-1)){
		current_colour=0;
		return colours[num_colours-1];
	}
	// else increment counter and return
	current_colour++;
	return colours[current_colour-1];
}

static void __reset_colour()
{
	current_colour = 0;
}


static int __print_header()
{
	int i;

	printf("\n");
	__reset_colour();
	if(settings.printf_arm){
		printf("  arm   |");
		//log flight data into log file only when the drone is armed
		if(settings.printf_altitude){
		fprintf(flight_log, "%s alt(m)|altdot| acc_x |acc_y | acc_z|");
	}
	if(settings.printf_rpy){
		fprintf(flight_log, "%s roll|pitch| yaw |");
	}
	if(settings.printf_sticks){
		fprintf(flight_log, "%s  kill  | thr |roll |pitch| yaw | ");
	}
	if(settings.printf_setpoint){
		fprintf(flight_log, "%s sp_a| sp_r| sp_p| sp_y|");
	}
	if(settings.printf_u){
		fprintf(flight_log, "%s U0X | U1Y | U2Z | U3r | U4p | U5y |");
	}
	if(settings.printf_motors){
		printf("%s", __next_colour());
		for(i=0;i<settings.num_rotors;i++){
			fprintf(flight_log, "  M%d |", i+1);
		}
	}
	}
	if(settings.printf_altitude){
		printf("%s alt(m)|altdot|", __next_colour());
		fprintf(flight_log, "%s alt(m)|altdot| acc_x |acc_y | acc_z|");
	}
	if(settings.printf_rpy){
		printf("%s roll|pitch| yaw |", __next_colour());
		fprintf(flight_log, "%s roll|pitch| yaw |");
	}
	if(settings.printf_sticks){
		printf("%s  kill  | thr |roll |pitch| yaw |", __next_colour());
		//fprintf(flight_log, "%s  kill  | thr |roll |pitch| yaw |");
	}
	if(settings.printf_setpoint){
		printf("%s sp_a| sp_r| sp_p| sp_y|", __next_colour());
		fprintf(flight_log, "%s sp_a| sp_r| sp_p| sp_y|");
	}
	if(settings.printf_u){
		printf("%s U0X | U1Y | U2Z | U3r | U4p | U5y |", __next_colour());
		fprintf(flight_log, "%s U0X | U1Y | U2Z | U3r | U4p | U5y |");
	}
	if(settings.printf_motors){
		printf("%s", __next_colour());
		for(i=0;i<settings.num_rotors;i++){
			printf("  M%d |", i+1);
			fprintf(flight_log, "  M%d |", i+1);
		}
	}
	printf(KNRM);
	if(settings.printf_mode){
		printf("   MODE ");
		//fprintf(flight_log, "\n");
	}

	printf("\n");
	fprintf(flight_log,"\n");
	fflush(stdout);
	return 0;
}


static void* __printf_manager_func(__attribute__ ((unused)) void* ptr)
{	
	
	
	arm_state_t prev_arm_state;
	int i;
	initialized = 1;
	printf("\nTurn your transmitter kill switch to arm.\n");
	printf("Then move throttle UP then DOWN to arm controller\n\n");

	// turn off linewrap to avoid runaway prints
	printf(WRAP_DISABLE);
	flight_log = fopen("flight_log.txt","w");
	// log the date
	time_t now = time(NULL);
    time (&now);
	fprintf(flight_log, "This flight happened on:%s\n",ctime(&now));
	// print the header
	__print_header();
	prev_arm_state = fstate.arm_state;


	
	while(rc_get_state()!=EXITING){
		// re-print header on disarming
		//if(fstate.arm_state==DISARMED && prev_arm_state==ARMED){
		//	__print_header();
		//}

		printf("\r");
		if(settings.printf_arm){
			if(fstate.arm_state==ARMED){
				printf("%s ARMED %s |",KRED,KNRM);
				//fprintf(flight_log,"%s ARMED %s |",KRED,KNRM);
				if(settings.printf_altitude){
			fprintf(flight_log, "%+5.2f |%+5.2f | %+5.2f |%+5.2f |%+5.2f |",\
							fstate.altitude_kf,\
							fstate.alt_kf_vel,\
						    fstate.x_accel,\
							fstate.y_accel,\
							fstate.alt_kf_accel);
		}
		if(settings.printf_rpy){
			fprintf(flight_log, "%+5.2f|%+5.2f|%+5.2f|",\
							fstate.roll,\
							fstate.pitch,\
							fstate.yaw);
		}
		if(settings.printf_sticks){
			fprintf(flight_log,"|%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							user_input.thr_stick,\
							user_input.roll_stick,\
							user_input.pitch_stick,\
							user_input.yaw_stick);

		}
		if(settings.printf_setpoint){
			fprintf(flight_log, "%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							setpoint.altitude,\
							setpoint.roll,\
							setpoint.pitch,\
							setpoint.yaw);
		}
		if(settings.printf_u){
			fprintf(flight_log, "%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							fstate.u[0],\
							fstate.u[1],\
							fstate.u[2],\
							fstate.u[3],\
							fstate.u[4],\
							fstate.u[5]);
		}
		if(settings.printf_motors){
			for(i=0;i<settings.num_rotors;i++){
				fprintf(flight_log, "%+5.2f|", fstate.m[i]);
			}
			}
		fprintf(flight_log,"\n");
			//else			    printf("%sDISARMED%s|",KGRN,KNRM);
			}
		}
		if(settings.printf_altitude){
			printf("%s|%+5.2f |%+5.2f | %+5.2f |%+5.2f |%+5.2f |",\
							__next_colour(),\
							fstate.altitude_kf,\
							fstate.alt_kf_vel,\
						    fstate.x_accel,\
							fstate.y_accel,\
							fstate.alt_kf_accel);
		}
		if(settings.printf_rpy){
			printf("%s|%+5.2f|%+5.2f|%+5.2f|",\
							__next_colour(),\
							fstate.roll,\
							fstate.pitch,\
							fstate.yaw);
		}
		if(settings.printf_sticks){
			if(user_input.requested_arm_mode==ARMED)
				printf("%s ARMED  ",KRED);
			else	printf("%sDISARMED",KGRN);
			printf(KGRN);
			printf("%s|%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							__next_colour(),\
							user_input.thr_stick,\
							user_input.roll_stick,\
							user_input.pitch_stick,\
							user_input.yaw_stick);
			// fprintf(flight_log,"|%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
			// 				user_input.thr_stick,\
			// 				user_input.roll_stick,\
			// 				user_input.pitch_stick,\
			// 				user_input.yaw_stick);

		}
		if(settings.printf_setpoint){
			printf("%s%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							__next_colour(),\
							setpoint.altitude,\
							setpoint.roll,\
							setpoint.pitch,\
							setpoint.yaw);
			// fprintf(flight_log, "%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
			// 				setpoint.altitude,\
			// 				setpoint.roll,\
			// 				setpoint.pitch,\
			// 				setpoint.yaw);
		}
		if(settings.printf_u){
			printf("%s%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
							__next_colour(),\
							fstate.u[0],\
							fstate.u[1],\
							fstate.u[2],\
							fstate.u[3],\
							fstate.u[4],\
							fstate.u[5]);
			// fprintf(flight_log, "%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|%+5.2f|",\
			// 				fstate.u[0],\
			// 				fstate.u[1],\
			// 				fstate.u[2],\
			// 				fstate.u[3],\
			// 				fstate.u[4],\
			// 				fstate.u[5]);
		}
		if(settings.printf_motors){
			printf("%s",__next_colour());
			for(i=0;i<settings.num_rotors;i++){
				printf("%+5.2f|", fstate.m[i]);
				//fprintf(flight_log, "%+5.2f|", fstate.m[i]);
			}
		}
		printf(KNRM);
		// fprintf(flight_log,KNRM);
		if(settings.printf_mode){
			print_flight_mode(user_input.flight_mode);
		}

		fflush(stdout);
		prev_arm_state = fstate.arm_state;
		rc_usleep(1000000/PRINTF_MANAGER_HZ);
	}

	// put linewrap back on
	printf(WRAP_ENABLE);
	// fprintf(flight_log, WRAP_ENABLE);
	return NULL;
}



int printf_init()
{
	if(rc_pthread_create(&printf_manager_thread, __printf_manager_func, NULL,
				SCHED_FIFO, PRINTF_MANAGER_PRI)==-1){
		fprintf(stderr,"ERROR in start_printf_manager, failed to start thread\n");
		return -1;
	}
	rc_usleep(50000);
	return 0;
}


int printf_cleanup()
{
	int ret = 0;
	if(initialized){
		// wait for the thread to exit
		ret = rc_pthread_timed_join(printf_manager_thread,NULL,PRINTF_MANAGER_TOUT);
		if(ret==1) fprintf(stderr,"WARNING: printf_manager_thread exit timeout\n");
		else if(ret==-1) fprintf(stderr,"ERROR: failed to join printf_manager thread\n");
	}
	initialized = 0;
	return ret;
}


int print_flight_mode(flight_mode_t mode){
	switch(mode){
	case TEST_BENCH_4DOF:
		printf("%sTEST_BENCH_4DOF%s",KYEL,KNRM);
		return 0;
	case TEST_BENCH_6DOF:
		printf("%sTEST_BENCH_6DOF%s",KYEL,KNRM);
		return 0;
	case DIRECT_THROTTLE_4DOF:
		printf("%sDIR_THRTLE_4DOF%s",KCYN,KNRM);
		return 0;
	case DIRECT_THROTTLE_6DOF:
		printf("%sDIR_THRTLE_6DOF%s",KCYN,KNRM);
		return 0;
	case ALT_HOLD_4DOF:
		printf("%sALT_HOLD_4DOF  %s",KBLU,KNRM);
		return 0;
	case ALT_HOLD_6DOF:
		printf("%sALT_HOLD_6DOF  %s",KBLU,KNRM);
		return 0;
	default:
		fprintf(stderr,"ERROR in print_flight_mode, unknown flight mode\n");
		return -1;
	}
}
