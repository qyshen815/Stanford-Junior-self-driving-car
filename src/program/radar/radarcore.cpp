#include <roadrunner.h>
#include <serial.h>
#include "radarcore.h"
#include <canlib.h>
#include <stdint.h>

namespace dgc{

dgc_bosch_lrr3_radar_p dgc_bosch_radar_connect(int radar_num)
{
  dgc_bosch_lrr3_radar_p radar;

  radar = (dgc_bosch_lrr3_radar_p)calloc(1, sizeof(dgc_bosch_lrr3_radar_t));
  dgc_test_alloc(radar);

  radar->can_device = new KvaserCan(radar_num, BAUD_500K);
  if(!radar->can_device->Open()) {
    fprintf(stderr, "Error: could not open connection to canbus\n");
    delete radar->can_device; 
    free(radar);
    return NULL;
  }
  radar->targets_ready = 0;
  return radar;
}

int bit_string(unsigned char buffer[8], int start, int num_bits)
{
  int i, value = 0, factor = 1;
  int byte, bit;

  for(i = start; i < start + num_bits; i++) {
    byte = i / 8;
    bit = i % 8;
    if(buffer[byte] & (1 << bit))
      value += factor;
    factor *= 2;
  }
  return value;
}

void print_bit_string(unsigned char buffer[8], int start, int num_bits, 
		      char *str)
{
  int i;
  int byte, bit;

  fprintf(stderr, "%s : %d - %d : ", str, start, start + num_bits);
  for(i = start + num_bits - 1; i >= start; i--) {
    byte = i / 8;
    bit = i % 8;
    if(buffer[byte] & (1 << bit)) 
      fprintf(stderr, "1");
    else
      fprintf(stderr, "0");
  }
  fprintf(stderr, "\n");
}

int twos_complement(int x, int n)
{
  int mask, i;

  mask = 1;
  for(i = 0; i < n; i++)
    mask *= 2;
  mask -= 1;

  if(x & (1 << (n - 1))) {
    x = ~x;
    x += 1;
    x &= mask;
    x *= -1;
  }
  return x;
}

void dgc_bosch_radar_send_motion_data(dgc_bosch_lrr3_radar_p radar,
                                      int received_vehicle_state,
                                      dgc_bosch_lrr3_pose_p pose_to_radar)
{
  unsigned char message[8];
  short int i;
  double v;

  if (!received_vehicle_state){
    fprintf(stderr, "X");
    return;
  }

  /* vehicle state has been received, wheel_speeds and yaw_rate are valid */
  //fprintf(stderr, "*");

  /* VEH_VELOCITY Message */
  i = (short int)rint(pose_to_radar->vWheelFL * 16.0); 	/* in km/h, per LRR3 spec */
  printf("FL wheel velocity is: %d 1/16th km/h\n", i);
  message[0] = i & 0xFF;
  message[1] = (i >> 8) & 0xFF;
  i = (short int)rint(pose_to_radar->vWheelFR * 16.0);
  message[2] = i & 0xFF;
  message[3] = (i >> 8) & 0xFF;
  i = (short int)rint(pose_to_radar->vWheelRL * 16.0);
  message[4] = i & 0xFF;
  message[5] = (i >> 8) & 0xFF;
  i = (short int)rint(pose_to_radar->vWheelRR * 16.0);
  message[6] = i & 0xFF;
  message[7] = (i >> 8) & 0xFF;
  radar->can_device->SendMessage((long)BOSCH_LRR3_RADAR_VELOCITY_CAN_ID, (void*)message, 8);

  /* VEH_YAWRATE Message */
  i = (int)rint(pose_to_radar->yaw_rate * 5.0);		/* in deg/s, per LRR3 spec */
  printf("yaw rate is: %f\n", dgc_d2r(i / 5));
  message[0] = 0;
  message[1] = 0;
  message[2] = 0;
  message[3] = 0;
  message[4] = (i << 4) & 0xF0;          // Mask lower 4 bits and retain upper
  message[5] = (i >> 4) & 0xFF;   // 12 bits of message
  message[6] = 0;
  message[7] = 0;
  radar->can_device->SendMessage((long)BOSCH_LRR3_RADAR_YAWRATE_CAN_ID, (void*)message, 8);

  /* VEH_STATUS Message */
  v = 0.5 * (pose_to_radar->vWheelRL + pose_to_radar->vWheelRR);
  if (v < 0.05 && v > -0.05)
    i = 0x00;
  else if (v > 0.0)
    i = 0x10;
  else if (v < 0.0)
    i = 0x20;
  else{
    fprintf(stderr, "\tWARNING: Vehicle velocity cannot be determined \n");
    i = 0x30;
  }
  printf("vehicle status: %d\n", i >> 4);
  message[0] = 0;
  message[1] = i;	// Mask lower 4 bits and retain upper 4 bits
  message[2] = 0;
  message[3] = 0;
  message[4] = 0;
  message[5] = 0;
  message[6] = 0;
  message[7] = 0;
  radar->can_device->SendMessage((long)BOSCH_LRR3_RADAR_DRIVING_STATUS_CAN_ID, (void*)message, 8);
}

void dgc_bosch_radar_process(dgc_bosch_lrr3_radar_p radar)
{
  unsigned char message[100];
  unsigned int can_id;
  int i, message_length, message_b_index = -1, match_message_a_message_b = 0;
  static int counter_message_a = 0, counter_targets_message_a = 0, counter_targets_message_b = 0;
  static int data_consistency_bit = 0, first = 1, valid_msg = 0;
  static int measurement_number = 0;    /* Used to stay compatible with LRR2 code */
  static int message_a_can_id[MAX_TARGETS] = {0};
  static int valid_message_a_index[MAX_TARGETS] = {-1};

  /* Read the CAN buffer until empty */
  while((message_length = radar->can_device->ReadMessage((long*)&can_id, message))>0) { 
    /* STARTER Message */
    /* Note: Contrary to popular belief, the STARTER message does NOT actually start the frame! 
     * It has been observed that the SGU message (can_id = 0x0B9) commonly starts the frame. 
     * Therefore, the resetting of various control variables has been moved to the ENDER message
     * as that message has been observed to ALWAYS be the last message in any given frame */
    if(can_id == BOSCH_LRR3_RADAR_OBJECT_STARTER_CAN_ID) {
      radar->vehicle_yaw_rate = twos_complement(bit_string(message, 0, 16), 16) / 16384.0;
      radar->vehicle_velocity = twos_complement(bit_string(message, 16, 12), 12) / 16.0;
      radar->vehicle_acceleration = twos_complement(bit_string(message, 28, 10), 10) / 32.0;
      radar->vehicle_slip_angle = twos_complement(bit_string(message, 38, 14), 14) / 32768.0;
      if (DEBUG){
        print_bit_string(message, 0, 64, "STARTER  ");
        fprintf(stderr, "(%X)\t\t slip_angle = %.3f \n\t\t veh_accel = %.3f \n\t\t veh_vel = %.3f \n\t\t veh_yaw = %.3f \n",
                can_id, radar->vehicle_slip_angle, radar->vehicle_acceleration, radar->vehicle_velocity, radar->vehicle_yaw_rate);
      }
      if ( (data_consistency_bit != bit_string(message, 63, 1)) && !first ){
        //fprintf(stderr, "\tWARNING: (OBJECT_STARTER message) Data consistency bit invalid... possible dropped packet.\n");
      }
    }
    
    /* ENDER Message */
    else if(can_id == BOSCH_LRR3_RADAR_OBJECT_ENDER_CAN_ID) {
      radar->sync_time = bit_string(message, 0, 26) / 1024.0;
      radar->curvature = twos_complement(bit_string(message, 26, 16), 16) / 131072.0;
      /*
      if ((data_consistency_bit != bit_string(message, 63, 1)) && !first)
        printf(stderr, "\tWARNING: (OBJECT_ENDER message) Data consistency bit invalid... possible dropped packet.\n");
      */
      radar->num_objects = counter_targets_message_a;
      radar->measurement_number = measurement_number;
      if (DEBUG){
        print_bit_string(message, 0, 64, "ENDER    ");
        fprintf(stderr, "(%X)\t\t curvature = %.3f \n\t\t sync_time = %.3f \n\n\t\t num_objects = %i \n\t\t measurement_num = %i \n",
                can_id, radar->curvature, radar->sync_time, radar->num_objects, radar->measurement_number);
      }
      if (counter_targets_message_a == counter_targets_message_b){
        radar->targets_ready = 1;
        if (DEBUG)
          fprintf(stderr, "TARGETS READY!...\n");
      }
      else {
        fprintf(stderr, "\tWARNING: Number of valid MESSAGE A (%i) does not equal number of valid MESSAGE B (%i)\n", 
                          counter_targets_message_a, counter_targets_message_b);
        radar->targets_ready = 0;
      }
      
      /* Check data consistency bit... should toggle (0->1 and 1->0) on new frame */
      if (first) {
        /* State of data_consistency_bit previously unknown, so after this iteration it will be valid... */
        first = 0;
      }
      /* Want to invert this frame's data consistency bit in preparation for the next frame */
      data_consistency_bit = ~bit_string(message, 63, 1);
      
      /* Clear or reset all array tracking variables */
      for (i=0; i<MAX_TARGETS; i++) {
	//radar->target[i].id = -1;
        message_a_can_id[i] = 0;
        valid_message_a_index[i] = -1;
      }
      counter_message_a = 0;
      counter_targets_message_a = 0;
      counter_targets_message_b = 0;
      measurement_number++;
    }

    /* MESSAGE A */
    else if(can_id >= BOSCH_LRR3_RADAR_FIRST_A_TARGET_CAN_ID &&
	    can_id <= BOSCH_LRR3_RADAR_LAST_A_TARGET_CAN_ID &&
            (can_id % 2) == 1) { /* Ensure can_id is within MESSAGE A range and odd */
      if (counter_message_a < MAX_TARGETS) {
        message_a_can_id[counter_message_a] = can_id;
        valid_msg = bit_string(message, 62, 1);
        if (valid_msg){
          radar->target[counter_targets_message_a].long_distance = bit_string(message, 0, 12) / 16.0;
	  radar->target[counter_targets_message_a].long_velocity = 
	    twos_complement(bit_string(message, 12, 12), 12) / 16.0;
	  radar->target[counter_targets_message_a].lateral_distance = 
	    twos_complement(bit_string(message, 24, 14), 14) / 64.0;
	  radar->target[counter_targets_message_a].long_acceleration = 
	    twos_complement(bit_string(message, 38, 10), 10) / 32.0;
          radar->target[counter_targets_message_a].prob_exist = bit_string(message, 48, 6) / 64.0;
          radar->target[counter_targets_message_a].lateral_distance_std = bit_string(message, 54, 6) / 16.0;
          radar->target[counter_targets_message_a].historical = bit_string(message, 60, 1);
          radar->target[counter_targets_message_a].measured = bit_string(message, 61, 1);
          radar->target[counter_targets_message_a].valid = bit_string(message, 62, 1);
          /*if (data_consistency_bit != bit_string(message, 63, 1)){
            fprintf(stderr, "\tWARNING: (OBJECT_A message) Data consistency bit invalid... possible dropped packet.\n");
          }*/
          if (DEBUG){
            print_bit_string(message, 0, 64, "MESSAGE_A");
            fprintf(stderr, "(%X = %i)\t valid = %i \n\t\t measured = %i \n\t\t historical = %i \n\t\t lat_dist_std = %.3f \n\t\t prob_exist = %.3f \n\t\t long_accel = %.3f \n\t\t lat_dist = %.3f \n\t\t long_vel = %.3f \n\t\t long_dist = %.3f \n", 
                    can_id, can_id - BOSCH_LRR3_RADAR_FIRST_A_TARGET_CAN_ID,
                    radar->target[counter_targets_message_a].valid, radar->target[counter_targets_message_a].measured, 
                    radar->target[counter_targets_message_a].historical, radar->target[counter_targets_message_a].lateral_distance_std, 
                    radar->target[counter_targets_message_a].prob_exist, radar->target[counter_targets_message_a].long_acceleration, 
                    radar->target[counter_targets_message_a].lateral_distance, radar->target[counter_targets_message_a].long_velocity, 
                    radar->target[counter_targets_message_a].long_distance);
          }
          valid_message_a_index[counter_message_a] = counter_targets_message_a;
          counter_targets_message_a++;
        }
        counter_message_a++;
      }
      else
        fprintf(stderr, "\tWARNING: Reported targets exceed maximum (%i) targets.\n", MAX_TARGETS);
    }

    /* MESSAGE B */
    else if(can_id >= BOSCH_LRR3_RADAR_FIRST_B_TARGET_CAN_ID &&
            can_id <= BOSCH_LRR3_RADAR_LAST_B_TARGET_CAN_ID &&
            (can_id % 2) == 0) { /* Ensure can_id is within MESSAGE A range and even */
      /* Ensure MESSAGE B has an appropriate MESSAGE A mate */
      for (i = 0; i < counter_message_a; i++){
        if ( (can_id - message_a_can_id[i]) == 0x01 ){
          match_message_a_message_b = 1;
          message_b_index = valid_message_a_index[i];	// This does not ensure message_b_index is valid, still need to check if it is != -1
	  break;
        }
      }
      if ( !match_message_a_message_b ){
        fprintf(stderr, "\tWARNING: Radar MESSAGE B (id = %X) has no matching MESSAGE A (id = %X) \n", can_id, can_id - 0x20);
      }

      if (message_b_index >= 0){
        radar->target[message_b_index].lateral_velocity = twos_complement(bit_string(message, 0, 12), 12) / 16.0;
        radar->target[message_b_index].id = bit_string(message, 12, 7);
        radar->target[message_b_index].prob_obstacle = bit_string(message, 19, 5) / 32.0;
        radar->target[message_b_index].moving_state = bit_string(message, 24, 3);
        radar->target[message_b_index].long_acceleration_std = bit_string(message, 27, 6) / 16.0;
        radar->target[message_b_index].long_velocity_std = bit_string(message, 33, 6) / 16.0;
        radar->target[message_b_index].long_distance_std = bit_string(message, 39, 6) / 16.0;
        if (DEBUG){
          print_bit_string(message, 0, 64, "MESSAGE_B");
          fprintf(stderr, "(%X = %i)\t long_dist_std = %.3f \n\t\t long_vel_std = %.3f \n\t\t long_accel_std = %.3f \n\t\t moving_state = %i \n\t\t prob_obstacle = %.3f \n\t\t id = %i \n\t\t lat_vel = %.3f \n",
                  can_id, can_id - BOSCH_LRR3_RADAR_FIRST_B_TARGET_CAN_ID, radar->target[message_b_index].long_distance_std, radar->target[message_b_index].long_velocity_std, 
                  radar->target[message_b_index].long_acceleration_std, radar->target[message_b_index].moving_state,
                  radar->target[message_b_index].prob_obstacle, radar->target[message_b_index].id, 
                  radar->target[message_b_index].lateral_velocity);
        }
        counter_targets_message_b++;
      }
      /*if (data_consistency_bit != bit_string(message, 63, 1)){
        fprintf(stderr, "\tWARNING: (OBJECT_B message) Data consistency bit invalid... possible dropped packet.\n");
      }*/
    }

    /* ACC TARGET message */
    else if(can_id == BOSCH_LRR3_RADAR_ACC_TARGET_CAN_ID){
      radar->acc_distance = bit_string(message, 0, 12) / 16.0;
      radar->acc_velocity = twos_complement(bit_string(message, 12, 12), 12) / 16.0;
      radar->acc_course_offset = twos_complement(bit_string(message, 24, 14), 14) / 64.0;
      radar->acc_acceleration = twos_complement(bit_string(message, 38, 10), 10) / 32.0;
      radar->acc_target_id = bit_string(message, 48, 7);
      radar->acc_stationary_id = bit_string(message, 55, 7);
      if (DEBUG){
        print_bit_string(message, 0, 64, "ACC      ");
        fprintf(stderr, "(%X)\t\t stationary_id = %i \n\t\t target_id = %i \n\t\t accel = %.3f \n\t\t course_offset = %.3f \n\t\t vel = %.3f \n\t\t dist = %.3f \n", 
                can_id, radar->acc_stationary_id, radar->acc_target_id, radar->acc_acceleration, radar->acc_course_offset, 
                radar->acc_velocity, radar->acc_distance);
      }
      /*if (data_consistency_bit != bit_string(message, 63, 1)){
        fprintf(stderr, "\tWARNING: (ACC_TARGET message) Data consistency bit invalid... possible dropped packet.\n");
      }*/
    }
    
    /* PSS COLLISION message */
    else if(can_id == BOSCH_LRR3_RADAR_PSS_COLLISION_CAN_ID){
      radar->pss_moving_distance = bit_string(message, 0, 12) / 16.0;
      radar->pss_moving_lateral_offset = twos_complement(bit_string(message, 12, 12), 12) / 128.0;
      radar->pss_moving_id = bit_string(message, 24, 7);
      radar->pss_stationary_distance = bit_string(message, 31, 12) / 16.0;
      radar->pss_stationary_lateral_offset = twos_complement(bit_string(message, 43, 12), 12) / 128.0;
      radar->pss_stationary_id = bit_string(message, 55, 7);
      if (DEBUG){
        print_bit_string(message, 0, 64, "PSS      ");
        fprintf(stderr, "(%X)\t\t stationary_id = %i \n\t\t stationary_lat_offset = %f \n\t\t stationary_lat_dist = %.3f \n\t\t moving_id = %i \n\t\t moving_lat_offset = %f \n\t\t moving_lat_dist = %.3f \n", 
                can_id, radar->pss_stationary_id, radar->pss_stationary_lateral_offset, radar->pss_stationary_distance, 
                radar->pss_moving_id, radar->pss_moving_lateral_offset, radar->pss_moving_distance);
      }
      /*if (data_consistency_bit != bit_string(message, 63, 1)){
        fprintf(stderr, "\tWARNING: (PSS_COLLISION message) Data consistency bit invalid... possible dropped packet.\n");
      }*/
    }

    /* COLLISION UNAVOIDABLE message */
    else if(can_id == BOSCH_LRR3_RADAR_COLLISION_CAN_ID){
      radar->cu_request = bit_string(message, 0, 1); 
      radar->cu_handle = bit_string(message, 1, 7); 
      if (DEBUG){
        print_bit_string(message, 0, 64, "CU       ");
        fprintf(stderr, "(%X)\t\t cu_request = %i \n\t\t cu_handle = %i \n", can_id, radar->cu_request, radar->cu_handle); 
      }
      /*if (data_consistency_bit != bit_string(message, 63, 1)){
        fprintf(stderr, "\tWARNING: (COLLISION message) Data consistency bit invalid... possible dropped packet.\n");
      }*/
    }
   
    /* SGU INFO message */
    else if(can_id == BOSCH_LRR3_RADAR_ACC_SGU_INFO_CAN_ID){
      radar->sensor_dirty = bit_string(message, 0, 1); 
      radar->hw_failure = bit_string(message, 1, 1); 
      radar->cycle_time = bit_string(message, 2, 16) / 2048.0; 
      radar->scu_temperature = twos_complement(bit_string(message, 18, 8), 8); 
      radar->horz_missalign_angle = twos_complement(bit_string(message, 26, 16), 16) / 8192.0; 
      radar->sgu_failure = bit_string(message, 42, 1); 
      if (DEBUG){
        print_bit_string(message, 0, 64, "SGU INFO ");
        fprintf(stderr, "(%X)\t\t sgu_failure = %i \n\t\t horz_missalign_angle = %.3f \n\t\t scu_temp = %i \n\t\t cycle_time = %.3f \n\t\t hw_failure = %i \n\t\t sensor_dirty = %i \n",
                can_id, radar->sgu_failure, radar->horz_missalign_angle, radar->scu_temperature, radar->cycle_time, 
                radar->hw_failure, radar->sensor_dirty);
      }
      /*if (data_consistency_bit != bit_string(message, 63, 1)){
        fprintf(stderr, "\tWARNING: (SGU_INFO message) Data consistency bit invalid... possible dropped packet.\n");
      }*/
    }
  }
}

void dgc_bosch_radar_disconnect(dgc_bosch_lrr3_radar_p *radar)
{
  delete (*radar)->can_device;
  free(*radar);
  *radar = NULL;
}

}
