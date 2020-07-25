/**
* @file settings.c
*
* contains all the functions for io to the settings file, including default
* values that can be written to disk if no file is present.
 **/

#include <stdio.h>
#include <string.h>	// FOR str_cmp()
#include <fcntl.h>	// for F_OK
#include <unistd.h>	// for access()

#include <json-c/json.h>
#include <rc/math/filter.h>

#include <settings.h>
#include <rc_pilot_defs.h>


// json object respresentation of the whole settings file
static json_object* jobj;

// primary settings struct declared as extern in header is defined ONCE here
settings_t settings;

// filters pulled from settings file
static rc_filter_t roll_controller;
static rc_filter_t pitch_controller;
static rc_filter_t yaw_controller;
static rc_filter_t altitude_controller;

// if anything goes wrong set this flag back to 0
static int was_load_successful = 0;


// macro for reading a boolean
#define PARSE_BOOL(name) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){\
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_boolean)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be a boolean\n");\
	return -1;\
}\
settings.name = json_object_get_boolean(tmp);\

// macro for reading an integer
#define PARSE_INT(name) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){ \
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_int)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be an int\n");\
	return -1;\
}\
settings.name = json_object_get_int(tmp);\

// macro for reading a bound integer
#define PARSE_INT_MIN_MAX(name,min,max) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){ \
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_int)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be an int\n");\
	return -1;\
}\
settings.name = json_object_get_int(tmp);\
if(settings.name<min || settings.name>max){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be between min and max\n");\
	return -1;\
}\

// macro for reading a polarity which should be +-1
#define PARSE_POLARITY(name) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){ \
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_int)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be an int\n");\
	return -1;\
}\
settings.name = json_object_get_int(tmp);\
if(settings.name!=-1 && settings.name!=1){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be -1 or 1\n");\
	return -1;\
}\

// macro for reading a floating point number
#define PARSE_DOUBLE_MIN_MAX(name,min,max)\
if(json_object_object_get_ex(jobj, #name, &tmp)==0){\
	fprintf(stderr,"ERROR can't find " #name " in settings file\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_double)==0){\
	fprintf(stderr,"ERROR " #name " should be a double\n");\
	return -1;\
}\
settings.name = json_object_get_double(tmp);\
if(settings.name<min || settings.name>max){\
	fprintf(stderr,"ERROR " #name " should be between min and max\n");\
	return -1;\
}\

// macro for reading a string
#define PARSE_STRING(name) \
if(json_object_object_get_ex(jobj, #name, &tmp)==0){ \
	fprintf(stderr,"ERROR parsing settings file, can't find " #name "\n");\
	return -1;\
}\
if(json_object_is_type(tmp, json_type_string)==0){\
	fprintf(stderr,"ERROR parsing settings file, " #name " should be a string\n");\
	return -1;\
}\
strcpy(settings.name, json_object_get_string(tmp));



/**
 * @brief      pulls rotor layout out of json object into settings struct
 *
 * @return     0 on success, -1 on failure
 */
int __parse_layout()
{
	struct json_object *tmp = NULL;
	char* tmp_str = NULL;
	if(json_object_object_get_ex(jobj, "layout", &tmp)==0){
		fprintf(stderr,"ERROR: can't find layout in settings file\n");
		return -1;
	}
	if(json_object_is_type(tmp, json_type_string)==0){
		fprintf(stderr,"ERROR: layout should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);
	if(strcmp(tmp_str, "LAYOUT_6DOF_ROTORBITS")==0){
		settings.num_rotors = 6;
		settings.layout = LAYOUT_6DOF_ROTORBITS;
	}
	else if(strcmp(tmp_str, "LAYOUT_6DOF_5INCH_MONOCOQUE")==0){
		settings.num_rotors = 6;
		settings.layout = LAYOUT_6DOF_5INCH_MONOCOQUE;
		printf("\nsetting = LAYOUT_6DOF_5INCH_MONOCOQUE\n");
	}
	else if(strcmp(tmp_str, "LAYOUT_4X")==0){
		settings.num_rotors = 4;
		settings.layout = LAYOUT_4X;
	}
	else if(strcmp(tmp_str, "LAYOUT_4PLUS")==0){
		settings.num_rotors = 4;
		settings.layout = LAYOUT_4PLUS;
	}
	else if(strcmp(tmp_str, "LAYOUT_6X")==0){
		settings.num_rotors = 6;
		settings.layout = LAYOUT_6X;
	}
	else if(strcmp(tmp_str, "LAYOUT_8X")==0){
		settings.num_rotors = 8;
		settings.layout = LAYOUT_8X;
	}
	else{
		fprintf(stderr,"ERROR: invalid layout string\n");
		return -1;
	}
}


int __parse_thrust_map()
{
	struct json_object *tmp = NULL;
	char* tmp_str = NULL;
	if(json_object_object_get_ex(jobj, "thrust_map", &tmp)==0){
		fprintf(stderr,"ERROR: can't find thrust_map in settings file\n");
		return -1;
	}
	if(json_object_is_type(tmp, json_type_string)==0){
		fprintf(stderr,"ERROR: thrust map should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);
	if(strcmp(tmp_str, "LINEAR_MAP")==0){
		settings.thrust_map = LINEAR_MAP;
	}
	else if(strcmp(tmp_str, "MN1806_1400KV_4S")==0){
		settings.thrust_map = MN1806_1400KV_4S;
	}
	else if(strcmp(tmp_str, "F20_2300KV_2S")==0){
		settings.thrust_map = F20_2300KV_2S;
	}
	else if(strcmp(tmp_str, "RX2206_4S")==0){
		settings.thrust_map = RX2206_4S;
	}
	else{
		fprintf(stderr,"ERROR: invalid thrust_map string\n");
		return -1;
	}
}


/**
 * @brief      parses a json_object and fills in the flight mode.
 *
 * @param      jobj  The jobj to parse
 * @param      mode  pointer to write mode out to
 *
 * @return     returns 0 on success or -1 on failure
 */
int __parse_flight_mode(json_object* jobj_str, flight_mode_t* mode)
{
	char* tmp_str = NULL;
	struct json_object *tmp = NULL;
	if(json_object_is_type(jobj_str, json_type_string)==0){
		fprintf(stderr,"ERROR: flight_mode should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(jobj_str);
	if(strcmp(tmp_str, "TEST_BENCH_4DOF")==0){
		*mode = TEST_BENCH_4DOF;
	}
	else if(strcmp(tmp_str, "TEST_BENCH_6DOF")==0){
		*mode = TEST_BENCH_6DOF;
	}
	else if(strcmp(tmp_str, "DIRECT_THROTTLE_4DOF")==0){
		*mode = DIRECT_THROTTLE_4DOF;
	}
	else if(strcmp(tmp_str, "DIRECT_THROTTLE_6DOF")==0){
		*mode = DIRECT_THROTTLE_6DOF;
	}
	else if(strcmp(tmp_str, "ALT_HOLD_4DOF")==0){
		*mode = ALT_HOLD_4DOF;
	}
	else if(strcmp(tmp_str, "ALT_HOLD_6DOF")==0){
		*mode = ALT_HOLD_6DOF;
	}
	else{
		fprintf(stderr,"ERROR: invalid flight mode\n");
		return -1;
	}
	return 0;
}



int __parse_kill_mode()
{
	struct json_object *tmp = NULL;
	char* tmp_str = NULL;
	if(json_object_object_get_ex(jobj, "dsm_kill_mode", &tmp)==0){
		fprintf(stderr,"ERROR: can't find dsm_kill_mode in settings file\n");
		return -1;
	}
	if(json_object_is_type(tmp, json_type_string)==0){
		fprintf(stderr,"ERROR: dsm_kill_mode should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);
	if(strcmp(tmp_str, "DSM_KILL_DEDICATED_SWITCH")==0){
		settings.dsm_kill_mode = DSM_KILL_DEDICATED_SWITCH;
	}
	else if(strcmp(tmp_str, "DSM_KILL_NEGATIVE_THROTTLE")==0){
		settings.dsm_kill_mode = DSM_KILL_NEGATIVE_THROTTLE;
	}
	else{
		fprintf(stderr,"ERROR: invalid dsm_kill_mode string\n");
		return -1;
	}
}


/**
 * @ brief     parses a json_object and sets up a new controller
 *
 * @param      jobj         The jobj to parse
 * @param      filter       pointer to write the new filter to
 * @param[in]  feedback_hz  frequency of the dt controller
 *
 * @return     0 on success, -1 on failure
 */
int __parse_controller(json_object* jobj_ctl, rc_filter_t* filter, int feedback_hz){
	struct json_object *array = NULL;	// to hold num & den arrays
	struct json_object *tmp = NULL;		// temp object
	char* tmp_str = NULL;
	double tmp_flt, tmp_kp, tmp_ki, tmp_kd;
	int i, num_len, den_len;
	rc_vector_t num_vec = rc_vector_empty();
	rc_vector_t den_vec = rc_vector_empty();
	double dt = 1.0/feedback_hz;

	// destroy old memory in case the order changes
	rc_filter_free(filter);

	// pull out gain
	if(json_object_object_get_ex(jobj_ctl, "gain", &tmp)==0){
		fprintf(stderr,"ERROR: can't find controller gain in settings file\n");
		return -1;
	}
	if(json_object_is_type(tmp, json_type_double)==0){
		fprintf(stderr,"ERROR: controller gain should be a double\n");
		return -1;
	}
	tmp_flt = json_object_get_double(tmp);

	// check if PID gains or transfer function coefficients
	if(json_object_object_get_ex(jobj_ctl, "TF_or_PID", &tmp)==0){
		fprintf(stderr,"ERROR: can't find TF_or_PID in settings file\n");
		return -1;
	}
	if(json_object_is_type(tmp, json_type_string)==0){
		fprintf(stderr,"ERROR: TF_or_PID should be a string\n");
		return -1;
	}
	tmp_str = (char*)json_object_get_string(tmp);

	if(strcmp(tmp_str, "TF")==0){

		// pull out numerator
		if(json_object_object_get_ex(jobj_ctl, "numerator", &array)==0){
			fprintf(stderr,"ERROR: can't find controller numerator in settings file\n");
			return -1;
		}
		if(json_object_is_type(array, json_type_array)==0){
			fprintf(stderr,"ERROR: controller numerator should be an array\n");
			return -1;
		}
		num_len = json_object_array_length(array);
		if(num_len<1){
			fprintf(stderr,"ERROR, numerator must have at least 1 entry\n");
			return -1;
		}
		rc_vector_alloc(&num_vec,num_len);
		for(i=0;i<num_len;i++){
			tmp = json_object_array_get_idx(array,i);
			if(json_object_is_type(tmp, json_type_double)==0){
				fprintf(stderr,"ERROR: numerator array entries should be a doubles\n");
				return -1;
			}
			tmp_flt = json_object_get_double(tmp);
			num_vec.d[i]=tmp_flt;
		}


		// pull out denominator
		if(json_object_object_get_ex(jobj_ctl, "denominator", &array)==0){
			fprintf(stderr,"ERROR: can't find controller denominator in settings file\n");
			return -1;
		}
		if(json_object_is_type(array, json_type_array)==0){
			fprintf(stderr,"ERROR: controller denominator should be an array\n");
			return -1;
		}
		den_len = json_object_array_length(array);
		if(den_len<1){
			fprintf(stderr,"ERROR, denominator must have at least 1 entry\n");
			return -1;
		}
		rc_vector_alloc(&den_vec,den_len);
		for(i=0;i<den_len;i++){
			tmp = json_object_array_get_idx(array,i);
			if(json_object_is_type(tmp, json_type_double)==0){
				fprintf(stderr,"ERROR: denominator array entries should be a doubles\n");
				return -1;
			}
			tmp_flt = json_object_get_double(tmp);
			den_vec.d[i]=tmp_flt;
		}

		// check for improper TF
		if(num_len>den_len){
			fprintf(stderr,"ERROR: improper transfer function\n");
			rc_vector_free(&num_vec);
			rc_vector_free(&den_vec);
			return -1;
		}

		// check CT continuous time or DT discrete time
		if(json_object_object_get_ex(jobj_ctl, "CT_or_DT", &tmp)==0){
			fprintf(stderr,"ERROR: can't find CT_or_DT in settings file\n");
			return -1;
		}
		if(json_object_is_type(tmp, json_type_string)==0){
			fprintf(stderr,"ERROR: CT_or_DT should be a string\n");
			return -1;
		}
		tmp_str = (char*)json_object_get_string(tmp);


		// if CT, use tustin's approx to get to DT
		if(strcmp(tmp_str, "CT")==0){
			// get the crossover frequency
			if(json_object_object_get_ex(jobj_ctl, "crossover_freq_rad_per_sec", &tmp)==0){
				fprintf(stderr,"ERROR: can't find crossover frequency in settings file\n");
				return -1;
			}
			if(json_object_is_type(tmp, json_type_double)==0){
				fprintf(stderr,"ERROR: crossover frequency should be a double\n");
				return -1;
			}
			tmp_flt = json_object_get_double(tmp);
			if(rc_filter_c2d_tustin(filter,dt, num_vec, den_vec, tmp_flt)){
				fprintf(stderr,"ERROR: failed to c2dtustin while parsing json\n");
				return -1;
			}
		}

		// if DT, much easier, just construct filter
		else if(strcmp(tmp_str, "DT")==0){
			if(rc_filter_alloc(filter,num_vec, den_vec, dt)){
				fprintf(stderr,"ERROR: failed to alloc filter in __parse_controller()");
				return -1;
			}
		}

		// wrong value for CT_or_DT
		else{
			fprintf(stderr,"ERROR: CT_or_DT must be 'CT' or 'DT'\n");
			printf("instead got :%s\n", tmp_str);
			return -1;
		}
	}

	else if(strcmp(tmp_str, "PID")==0){
		// pull out gains
		if(json_object_object_get_ex(jobj_ctl, "kp", &tmp)==0){
			fprintf(stderr,"ERROR: can't find kp in settings file\n");
			return -1;
		}
		tmp_kp = json_object_get_double(tmp);
		if(json_object_object_get_ex(jobj_ctl, "ki", &tmp)==0){
			fprintf(stderr,"ERROR: can't find ki in settings file\n");
			return -1;
		}
		tmp_ki = json_object_get_double(tmp);
		if(json_object_object_get_ex(jobj_ctl, "kd", &tmp)==0){
			fprintf(stderr,"ERROR: can't find kd in settings file\n");
			return -1;
		}
		tmp_kd = json_object_get_double(tmp);
		// get the crossover frequency
		if(json_object_object_get_ex(jobj_ctl, "crossover_freq_rad_per_sec", &tmp)==0){
			fprintf(stderr,"ERROR: can't find crossover frequency in settings file\n");
			return -1;
		}
		if(json_object_is_type(tmp, json_type_double)==0){
			fprintf(stderr,"ERROR: crossover frequency should be a double\n");
			return -1;
		}
		tmp_flt = json_object_get_double(tmp);
		if(rc_filter_pid(filter,tmp_kp,tmp_ki,tmp_kd,1.0/tmp_flt,dt)){
				fprintf(stderr,"ERROR: failed to alloc pid filter in __parse_controller()");
				return -1;
			}


	}

	return 0;
}




int settings_load_from_file(char* path)
{
	struct json_object *tmp = NULL; // temp object
	char* tmp_str = NULL; // temp string poitner
	double tmp_flt;
	int tmp_int;

	was_load_successful = 0;

	#ifdef DEBUG
	fprintf(stderr,"beginning of load_settings_from_file\n");
	fprintf(stderr,"about to check access of fly settings file\n");
	#endif

	// read in file contents
	if(access(path, F_OK)!=0){
		fprintf(stderr,"ERROR: settings file missing\n");
		return -1;
	}
	else{
		#ifdef DEBUG
		printf("about to read json from file\n");
		#endif
		jobj = json_object_from_file(path);
		if(jobj==NULL){
			fprintf(stderr,"ERROR, failed to read settings from disk\n");
			return -1;
		}
	}

	#ifdef DEBUG
	settings_print();
	#endif

	/*
	 * START PARSING
	 */
	PARSE_STRING(name)

	/*
	 * PHYSICAL PARAMETERS
	 * layout populates num_rotors, layout, and dof
	 * feedback_hz is also here because I don't know where else to put it
	 */
	if(__parse_layout()==-1) return -1; // parse_layout also fill in num_rotors and dof
	if(__parse_thrust_map()==-1) return -1;
	PARSE_DOUBLE_MIN_MAX(v_nominal,7.0,18.0)
	PARSE_INT(feedback_hz)
	if(settings.feedback_hz!=50 && settings.feedback_hz!=100 && settings.feedback_hz!=200){
		fprintf(stderr,"ERROR: feedback_hz must be 50,100,or 200\n");
		return -1;
	}

	/*
	 * FEATURES
	 * not many yet..
	 */
	PARSE_BOOL(enable_logging)
	PARSE_BOOL(enable_magnetometer)

	/*
	 * FLIGHT MODES
	 */
	PARSE_INT_MIN_MAX(num_dsm_modes,1,3)
	if(json_object_object_get_ex(jobj, "flight_mode_1", &tmp)==0){
		fprintf(stderr,"ERROR: can't find flight_mode_1 in settings file\n");
		return -1;
	}
	if(__parse_flight_mode(tmp, &settings.flight_mode_1)) return -1;
	if(json_object_object_get_ex(jobj, "flight_mode_2", &tmp)==0){
		fprintf(stderr,"ERROR: can't find flight_mode_2 in settings file\n");
		return -1;
	}
	if(__parse_flight_mode(tmp, &settings.flight_mode_2)) return -1;
	if(json_object_object_get_ex(jobj, "flight_mode_3", &tmp)==0){
		fprintf(stderr,"ERROR: can't find flight_mode_3 in settings file\n");
		return -1;
	}
	if(__parse_flight_mode(tmp, &settings.flight_mode_3)) return -1;

	/*
	 * DSM RADIO CONFIG
	 */
	PARSE_INT_MIN_MAX(dsm_thr_ch,1,9)
	PARSE_POLARITY(dsm_thr_pol)
	PARSE_INT_MIN_MAX(dsm_roll_ch,1,9)
	PARSE_POLARITY(dsm_roll_pol)
	PARSE_INT_MIN_MAX(dsm_pitch_ch,1,9)
	PARSE_POLARITY(dsm_pitch_pol)
	PARSE_INT_MIN_MAX(dsm_yaw_ch,1,9)
	PARSE_POLARITY(dsm_yaw_pol)
	PARSE_INT_MIN_MAX(dsm_mode_ch,1,9)
	PARSE_POLARITY(dsm_mode_pol)
	if(__parse_kill_mode()==-1) return -1;
	PARSE_INT_MIN_MAX(dsm_kill_ch,1,9)
	PARSE_POLARITY(dsm_kill_pol)


	/*
	 * PRINTF OPTIONS
	 */
	PARSE_BOOL(printf_arm)
	PARSE_BOOL(printf_altitude)
	PARSE_BOOL(printf_rpy)
	PARSE_BOOL(printf_sticks)
	PARSE_BOOL(printf_setpoint)
	PARSE_BOOL(printf_u)
	PARSE_BOOL(printf_motors)
	PARSE_BOOL(printf_mode)

	/*
	 * MAVLINK
	 */
	PARSE_STRING(dest_ip)
	PARSE_INT(my_sys_id)
	PARSE_INT(mav_port)

	/*
	 * FEEDBACK CONTROLLERS
	 */

	// parse roll controller
	if(json_object_object_get_ex(jobj, "roll_controller", &tmp)==0){
		fprintf(stderr,"ERROR: can't find roll_controller in settings file\n");
		return -1;
	}
	if(__parse_controller(tmp, &roll_controller, settings.feedback_hz)){
		fprintf(stderr,"ERROR: could not parse roll_controller\n");
		return -1;
	}

	// parse pitch controller
	if(json_object_object_get_ex(jobj, "pitch_controller", &tmp)==0){
		fprintf(stderr,"ERROR: can't find pitch_controller in settings file\n");
		return -1;
	}
	if(__parse_controller(tmp, &pitch_controller, settings.feedback_hz)){
		fprintf(stderr,"ERROR: could not parse pitch_controller\n");
		return -1;
	}

	// parse yaw controller
	if(json_object_object_get_ex(jobj, "yaw_controller", &tmp)==0){
		fprintf(stderr,"ERROR: can't find yaw_controller in settings file\n");
		return -1;
	}
	if(__parse_controller(tmp, &yaw_controller, settings.feedback_hz)){
		fprintf(stderr,"ERROR: could not parse yaw_controller\n");
		return -1;
	}

	// parse altitude controller
	if(json_object_object_get_ex(jobj, "altitude_controller", &tmp)==0){
		fprintf(stderr,"ERROR: can't find altitude_controller in settings file\n");
		return -1;
	}
	if(__parse_controller(tmp, &altitude_controller, settings.feedback_hz)){
		fprintf(stderr,"ERROR: could not parse altitude_controller\n");
		return -1;
	}

	json_object_put(jobj);	// free memory
	was_load_successful = 1;
	return 0;
}




int settings_print()
{
	if(jobj == NULL){
		fprintf(stderr,"ERROR: NULL object passed to settings_print\n");
		return -1;
	}
	printf("settings:\n\n");
	printf("%s", json_object_to_json_string_ext(jobj, \
			JSON_C_TO_STRING_SPACED | JSON_C_TO_STRING_PRETTY));
	printf("\n");
	return 0;
}



int settings_get_roll_controller(rc_filter_t* ctrl)
{
	if(was_load_successful == 0){
		fprintf(stderr,"ERROR: can't get json controller, last read failed\n");
		return -1;
	}
	*ctrl = roll_controller;
	return 0;
}


int settings_get_pitch_controller(rc_filter_t* ctrl)
{
	if(was_load_successful == 0){
		fprintf(stderr,"ERROR: can't get json controller, last read failed\n");
		return -1;
	}
	*ctrl = pitch_controller;
	return 0;
}


int settings_get_yaw_controller(rc_filter_t* ctrl)
{
	if(was_load_successful == 0){
		fprintf(stderr,"ERROR: can't get json controller, last read failed\n");
		return -1;
	}
	*ctrl = yaw_controller;
	return 0;
}


int settings_get_altitude_controller(rc_filter_t* ctrl)
{
	if(was_load_successful == 0){
		fprintf(stderr,"ERROR: can't get json controller, last read failed\n");
		return -1;
	}
	*ctrl = altitude_controller;
	return 0;
}

