/**
 * @file mavlink_manager.c
 *
 *
 */


#include <stdio.h>
#include <rc/mavlink_udp.h>
#include <rc/math/quaternion.h>
#include <rc/time.h>
#include <mavlink_manager.h>
#include <settings.h>

#define LOCALHOST_IP	"192.168.8.1"
#define DEFAULT_SYS_ID	1
uint8_t my_sys_id;
uint16_t mav_port;
double d;

// primary mocap_state struct declared as extern in header is defined ONCE here
mocap_state_t mocap_state;


void callback_func_mocap()
{
	int i;
	mavlink_att_pos_mocap_t data;
	static int mocap_sample_count = 0;
    double current_time = 0;
    current_time = clock()/CLOCKS_PER_SEC;
    //deactivate the following until actual VICON mocap is online       
    rc_mav_get_att_pos_mocap(&data);

	
	if(rc_mav_get_att_pos_mocap(&data)<0){
		fprintf(stderr, "ERROR in mavlink manager, problem fetching att_pos_mocal packet\n");
		return;
	}

	// check if position is 0 0 0 which indicates mocap system is alive but
	// has lost visual contact on the object
	// if(fabs(data.x)<0.001 && fabs(data.y)<0.001 && fabs(data.z)<0.001){
	// 	if(mocap_state.is_active==1){
	// 		mocap_state.is_active=0;
	// 		fprintf(stderr,"WARNING, MOCAP LOST VISUAL\n");
	// 	}
	// 	else{
	// 		mocap_state.is_active=0;
	// 	}
	// 	return;
	// }

	// copy data
	for(i=0;i<4;i++) mocap_state.q[i]=(double)data.q[i];
	// normalize quaternion because we don't trust the mocap system
	rc_quaternion_norm_array(mocap_state.q);
	// calculate tait bryan angles too
	rc_quaternion_to_tb_array(mocap_state.q, mocap_state.tait_bryan);
	// position
	mocap_state.position[0]=(double)data.x/1000.0;
	mocap_state.position[1]=(double)data.y/1000.0;
	mocap_state.position[2]=(double)data.z/1000.0;
	// mark timestamp
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

int mavlink_manager_cleanup()
{
	return rc_mav_cleanup();
}