/**
 * Script used to 
 * 
 * 1. test vicon/mavlink connection 
 * 2. integrate VICON altitude with IMU data 
 * 3. bmp mode and VICON mode
 * 4. when VICON is not activated, simulate VICON altitude by a sine function 
 * 
 * Pengcheng Cao 10/3/2019
 */


#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <rc/math/quaternion.h>
#include <rc/math/filter.h>
#include <rc/math/kalman.h>
#include <rc/math/other.h>
#include <rc/math/vector.h>
#include <rc/start_stop.h>
#include <rc/led.h>
#include <rc/mpu.h>
#include <rc/servo.h>
#include <rc/adc.h>
#include <rc/time.h>
#include <rc/bmp.h>


//#include <feedback.h>
//#include <rc_pilot_defs.h>
//#include <setpoint_manager.h>
#include <rc/mavlink_udp.h>
#include <roboticscape.h>
#include <rc/mavlink_udp_helpers.h>
//#include <log_manager.h>
#include "mavlink_manager.h"
#include "settings.h"
//#include <mix.h>
//#include <thrust_map.h>

#define TWO_PI (M_PI*2.0)
#define Nx 3
#define Ny 1
#define Nu 1
#define SAMPLE_RATE     200     // hz
#define DT              (1.0/SAMPLE_RATE)
#define ACCEL_LP_TC     20*DT   // fast LP filter for accel
#define PRINT_HZ        10
#define VICON_RATE_DIV    0 //10      // optionally sample VICON less frequently than mpu
#define BMP_RATE_DIV      10      // optionally sample BMP less frequently than mpu
#define SIN_FREQ          2       // simulating a VICON signal

#define LOCALHOST_IP	"192.168.8.1"
#define DEFAULT_SYS_ID	1
uint8_t my_sys_id;
uint16_t mav_port;
double d;

static int running = 0;
// static rc_filter_t D_roll, D_pitch, D_yaw, D_batt, D_altitude, altitude_lp;
static rc_mpu_data_t mpu_data;
//static rc_bmp_data_t bmp_data;
static rc_kalman_t kf = RC_KALMAN_INITIALIZER;
static rc_vector_t u = RC_VECTOR_INITIALIZER;
static rc_vector_t y = RC_VECTOR_INITIALIZER;
static rc_filter_t acc_lp = RC_FILTER_INITIALIZER;
// altitude kalman filer elements


// log_entry_t new_log;

// local functions
static void __feedback_isr(void);
static int __send_motor_stop_pulse();
static double __batt_voltage();
static int __estimate_altitude();
static int __feedback_control();
static int __feedback_state_estimate();


mocap_state_t mocap_state; // primary mocap_state struct declared as extern in header is defined ONCE here
//feedback_state_t fstate; // extern variable in feedback.h


static void __callback_func_connection_lost(void)
{
        fprintf(stderr,"CONNECTION LOST\n");
        return;
}
// interrupt handler to catch ctrl-c
static void __signal_handler(__attribute__ ((unused)) int dummy)
{
        running=0;
        return;
}

void callback_func_mocap()
{
        
	int i;
	static int mocap_sample_count = 0;
	mavlink_att_pos_mocap_t data;
        double current_time = 0;
        current_time = clock()/CLOCKS_PER_SEC;
        //deactivate the following until actual VICON mocap is online       
        rc_mav_get_att_pos_mocap(&data);

	if(rc_mav_get_att_pos_mocap(&data)<0){
	        fprintf(stderr, "ERROR in mavlink manager, problem fetching att_pos_mocal packet\n");
	        return;
	}

	//check if position is 0 0 0 which indicates mocap system is alive but
	//has lost visual contact on the object
// 	if(fabs(data.x)<0.001 && fabs(data.y)<0.001 && fabs(data.z)<0.001){
// 		if(mocap_state.is_active==1){
// 			mocap_state.is_active=0;
// 			fprintf(stderr,"WARNING, MOCAP LOST VISUAL\n");
// 			}
// 		else{
// 			mocap_state.is_active=0;
// 		}
// 	return;
// 	}

	//copy data
	for(i=0;i<4;i++) mocap_state.q[i]=(double)data.q[i];
	//normalize quaternion because we don't trust the mocap system
	rc_quaternion_norm_array(mocap_state.q);
	//calculate tait bryan angles too
	rc_quaternion_to_tb_array(mocap_state.q, mocap_state.tait_bryan);
        
        mocap_state.position[0] = (double)data.x/1000.0;
        mocap_state.position[1] = (double)data.y/1000.0;
        mocap_state.position[2] = (double)data.z/1000.0;
        
        
	//sample using mocap sample rate

        // mocap_sample_count ++;
        // if (mocap_sample_count >= VICON_RATE_DIV) {
        //         mocap_state.position[0] = (double)data.x;
        //         mocap_state.position[1] = (double)data.y;
        //         mocap_state.position[2] = (double)10.0*sin(current_time);
        //         mocap_sample_count = 0;
        // }
	mocap_state.timestamp_ns=rc_nanos_since_boot();
	mocap_state.is_active=1;

	return;
}


int mavlink_manager_init()
{
	// set default options before checking options
	const char* dest_ip=LOCALHOST_IP;
	uint8_t my_sys_id=DEFAULT_SYS_ID;
	uint16_t mav_port=RC_MAV_DEFAULT_UDP_PORT;

	// initialize the UDP port and listening thread with the rc_mav lib
	if(rc_mav_init(my_sys_id, dest_ip, mav_port,RC_MAV_DEFAULT_CONNECTION_TIMEOUT_US)==0){
		rc_mav_set_callback_all(callback_func_mocap);
		//callback_func_mocap();
		printf("Mavlink initialized\n");
	}
	else{
		return -1;
	}
}

int cleanup_mavlink_manager()
{
	return rc_mav_cleanup();
}

static void __dmp_handler(void)
{
        int i;
        double accel_vec[3];
        // make copy of acceleration reading before rotating
        for(i=0;i<3;i++) accel_vec[i]=mpu_data.accel[i];
        // rotate accel vector
        rc_quaternion_rotate_vector_array(accel_vec,mpu_data.dmp_quat);
        // do first-run filter setup
        // adjust the sample rate here and do the interpolation
        if(kf.step==0){
                kf.x_est.d[0] = mocap_state.position[2];
                rc_filter_prefill_inputs(&acc_lp, accel_vec[2]-9.80665);
                rc_filter_prefill_outputs(&acc_lp, accel_vec[2]-9.80665);
        }
        // calculate acceleration and smooth it just a tad
        rc_filter_march(&acc_lp, accel_vec[2]-9.80665);
        u.d[0] = acc_lp.newest_output;
        // don't bother filtering Barometer, kalman will deal with that
        y.d[0] = mocap_state.position[2];
        if(rc_kalman_update_lin(&kf, u, y)) running=0;
        // now check if we need to sample BMP this loop

}

int main(void)
{
        const char* dest_ip=LOCALHOST_IP;
	uint8_t my_sys_id=DEFAULT_SYS_ID;
	uint16_t mav_port=RC_MAV_DEFAULT_UDP_PORT;
        
        
    	rc_mpu_config_t mpu_conf;
    	rc_matrix_t F = RC_MATRIX_INITIALIZER;
        rc_matrix_t G = RC_MATRIX_INITIALIZER;
        rc_matrix_t H = RC_MATRIX_INITIALIZER;
        rc_matrix_t Q = RC_MATRIX_INITIALIZER;
        rc_matrix_t R = RC_MATRIX_INITIALIZER;
        rc_matrix_t Pi = RC_MATRIX_INITIALIZER;
        // allocate appropirate memory for system
        rc_matrix_zeros(&F, Nx, Nx);
        rc_matrix_zeros(&G, Nx, Nu);
        rc_matrix_zeros(&H, Ny, Nx);
        rc_matrix_zeros(&Q, Nx, Nx);
        rc_matrix_zeros(&R, Ny, Ny);
        rc_matrix_zeros(&Pi, Nx, Nx);
        rc_vector_zeros(&u, Nu);
        rc_vector_zeros(&y, Ny);
        // define system -DT; // accel bias
        F.d[0][0] = 1.0;
        F.d[0][1] = DT;
        F.d[0][2] = 0.0;
        F.d[1][0] = 0.0;
        F.d[1][1] = 1.0;
        F.d[1][2] = -DT; // subtract accel bias
        F.d[2][0] = 0.0;
        F.d[2][1] = 0.0;
        F.d[2][2] = 1.0; // accel bias state
        G.d[0][0] = 0.5*DT*DT;
        G.d[0][1] = DT;
        G.d[0][2] = 0.0;
        H.d[0][0] = 1.0;
        H.d[0][1] = 0.0;
        H.d[0][2] = 0.0;
        // covariance matrices
        Q.d[0][0] = 0.000000001;
        Q.d[1][1] = 0.000000001;
        Q.d[2][2] = 0.0001; // don't want bias to change too quickly
        R.d[0][0] = 1000000.0;
        // initial P, cloned from converged P while running
        Pi.d[0][0] = 1258.69;
        Pi.d[0][1] = 158.6114;
        Pi.d[0][2] = -9.9937;
        Pi.d[1][0] = 158.6114;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            
        Pi.d[1][1] = 29.9870;
        Pi.d[1][2] = -2.5191;
        Pi.d[2][0] = -9.9937;
        Pi.d[2][1] = -2.5191;
        Pi.d[2][2] = 0.3174;

        // initialize the kalman filter
        if(rc_kalman_alloc_lin(&kf,F,G,H,Q,R,Pi)==-1) return -1;
        // initialize the little LP filter to take out accel noise
        if(rc_filter_first_order_lowpass(&acc_lp, DT, ACCEL_LP_TC)) return -1;
        // set signal handler so the loop can exit cleanly
    	signal(SIGINT, __signal_handler);
    	running = 1;
    	// init mavlink
    	printf("initializing mavlink\n");
    	
    	if(mavlink_manager_init() <0){
    	        fprintf(stderr,"ERROR: failed to initialize mavlink manager\n");
		return -1;
    	}
    	// init DMP
	dest_ip = LOCALHOST_IP;
	my_sys_id = DEFAULT_SYS_ID;
	mav_port = RC_MAV_DEFAULT_UDP_PORT;

	
    	printf("initializing DMP\n");
    	mpu_conf = rc_mpu_default_config();
    	mpu_conf.dmp_sample_rate = SAMPLE_RATE;
    	mpu_conf.dmp_fetch_accel_gyro = 1;
    	if(rc_mpu_initialize_dmp(&mpu_data, mpu_conf)) return -1;
    	
    	// init VICON and read in first data
        printf("initializing VICON\n");
        
        mpu_conf = rc_mpu_default_config();
        mpu_conf.dmp_sample_rate = SAMPLE_RATE;
        mpu_conf.dmp_fetch_accel_gyro = 1;
        // wait for dmp to settle then start filter callback
        printf("waiting for sensors to settle");
        fflush(stdout);
        rc_usleep(3000000);
        rc_mpu_set_dmp_callback(__dmp_handler);
        
        
        // print a header
        printf("\r\n");
        printf(" altitude |");
        printf("  velocity |");
        printf(" accel_bias |");
        printf(" X(VICON) |");
        printf(" Y(VICON) |");
        printf(" alt (VICON)|");
        printf(" vert_accel |");
        printf("\n");

        //now just wait, print_data will run
        while(running){
                // For testing
                //callback_func_mocap();
                rc_usleep(1000000/PRINT_HZ);
                printf("\r");
                printf("%8.2fm |", kf.x_est.d[0]);
                printf("%7.2fm/s |", kf.x_est.d[1]);
                printf("%7.2fm/s^2|", kf.x_est.d[2]);
                printf(" %9.2fm |", mocap_state.position[0]);
                printf(" %9.2fm |", mocap_state.position[1]);
                printf(" %9.2fm |", mocap_state.position[2]);
                printf("%7.2fm/s^2|", acc_lp.newest_output);
                fflush(stdout);
        }
        printf("\n");
        rc_mpu_power_off();
        //rc_bmp_power_off();
        return 0;

}