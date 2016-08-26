/*
 * Copyright (C) 2005-2013 The Paparazzi Team
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#include "high_speed_logger_spi_link.h"

#include "subsystems/imu.h"
#include "subsystems/commands.h"
#include "mcu_periph/spi.h"
#include "state.h"
#include "firmwares/fixedwing/stabilization/stabilization_indi.h"
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/guidance/guidance_v.h"
#include "firmwares/fixedwing/nav.h"
#include "modules/sensors/turbulence_adc.h"
#include "modules/sensors/potentiometer_adc.h"
#include "modules/servo_controller/servo_controller.h"
#include "arch/stm32/subsystems/actuators/actuators_pwm_arch.h"
#include "subsystems/radio_control.h"
#include "subsystems/gps/gps_datalink.h"
#include "modules/sensors/turbulence_adc.h"


struct high_speed_logger_spi_link_data high_speed_logger_spi_link_data;
struct spi_transaction high_speed_logger_spi_link_transaction;

static volatile bool high_speed_logger_spi_link_ready = true;

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans);

void high_speed_logger_spi_link_init(void)
{
  high_speed_logger_spi_link_data.id = 0;

  high_speed_logger_spi_link_transaction.select        = SPISelectUnselect;
  high_speed_logger_spi_link_transaction.cpol          = SPICpolIdleHigh;
  high_speed_logger_spi_link_transaction.cpha          = SPICphaEdge2;
  high_speed_logger_spi_link_transaction.dss           = SPIDss8bit;
  high_speed_logger_spi_link_transaction.bitorder      = SPIMSBFirst;
  high_speed_logger_spi_link_transaction.cdiv          = SPIDiv64;
  high_speed_logger_spi_link_transaction.slave_idx     = HIGH_SPEED_LOGGER_SPI_LINK_SLAVE_NUMBER;
  high_speed_logger_spi_link_transaction.output_length = sizeof(high_speed_logger_spi_link_data);
  high_speed_logger_spi_link_transaction.output_buf    = (uint8_t *) &high_speed_logger_spi_link_data;
  high_speed_logger_spi_link_transaction.input_length  = 0;
  high_speed_logger_spi_link_transaction.input_buf     = NULL;
  high_speed_logger_spi_link_transaction.after_cb      = high_speed_logger_spi_link_trans_cb;
}


void high_speed_logger_spi_link_periodic(void)
{
  static int32_t counter = 0;
  counter++;
  if (high_speed_logger_spi_link_ready) {

    high_speed_logger_spi_link_data.id = counter;

    high_speed_logger_spi_link_ready = false;

    /* Data which will be logged for servo model */
    /*high_speed_logger_spi_link_data.cmd_cw_l      = ANGLE_BFP_OF_REAL(left_wing.pwm_cw);
    high_speed_logger_spi_link_data.cmd_ccw_l     = ANGLE_BFP_OF_REAL(left_wing.pwm_ccw);
    high_speed_logger_spi_link_data.motor_dyn     = ANGLE_BFP_OF_REAL(left_wing.motor_dyn);
    high_speed_logger_spi_link_data.filt_speed    = ANGLE_BFP_OF_REAL(left_wing.filtered_rate);

    //high_speed_logger_spi_link_data.cmd_cw_r      = ANGLE_BFP_OF_REAL(right_wing.pwm_cw);
    //high_speed_logger_spi_link_data.cmd_ccw_r     = ANGLE_BFP_OF_REAL(right_wing.pwm_ccw);

    high_speed_logger_spi_link_data.potscaled_l   = ANGLE_BFP_OF_REAL(pot_left_wing_scaled);
    high_speed_logger_spi_link_data.potscaled_r   = ANGLE_BFP_OF_REAL(pot_right_wing_scaled);

    high_speed_logger_spi_link_data.potraw_l      = potentiometer_adc_raw_left;
    high_speed_logger_spi_link_data.potraw_r      = potentiometer_adc_raw_right;

    high_speed_logger_spi_link_data.servoerr_l    = ANGLE_BFP_OF_REAL(left_wing.err);
    high_speed_logger_spi_link_data.servoerr_r    = ANGLE_BFP_OF_REAL(right_wing.err);

    high_speed_logger_spi_link_data.pwm_cw_l      = actuators_pwm_values[PWM_SERVO_2];
    high_speed_logger_spi_link_data.pwm_ccw_l     = actuators_pwm_values[PWM_SERVO_3];
    high_speed_logger_spi_link_data.pwm_cw_r      = actuators_pwm_values[PWM_SERVO_4];
    high_speed_logger_spi_link_data.pwm_ccw_r     = actuators_pwm_values[PWM_SERVO_5];

    high_speed_logger_spi_link_data.cmd_roll      = commands[1];*/


    /* Data which will be logged always */

    /*high_speed_logger_spi_link_data.phi             = stateGetNedToBodyEulers_i()->phi;
    high_speed_logger_spi_link_data.theta           = stateGetNedToBodyEulers_i()->theta;
    //high_speed_logger_spi_link_data.psi             = stateGetNedToBodyEulers_i()->psi;*/
    
    /* Test 1 determine G matrix */
    // rates
    /*high_speed_logger_spi_link_data.p               = state.body_rates_i.p;
    high_speed_logger_spi_link_data.q               = state.body_rates_i.q;
    high_speed_logger_spi_link_data.r               = state.body_rates_i.r;
    // reference angles
    high_speed_logger_spi_link_data.roll_setpoint   = ANGLE_BFP_OF_REAL(h_ctl_roll_setpoint);
    high_speed_logger_spi_link_data.pitch_setpoint  = ANGLE_BFP_OF_REAL(h_ctl_pitch_loop_setpoint);
    // commands
    high_speed_logger_spi_link_data.cmd_throttle    = commands[0];
    high_speed_logger_spi_link_data.cmd_roll        = commands[1];
    high_speed_logger_spi_link_data.cmd_pitch       = commands[2];
    high_speed_logger_spi_link_data.cmd_yaw         = commands[3];
    // position x,y,z
    high_speed_logger_spi_link_data.x               = POS_BFP_OF_REAL(stateGetPositionEnu_f()->x);
    high_speed_logger_spi_link_data.y               = POS_BFP_OF_REAL(stateGetPositionEnu_f()->y);
    high_speed_logger_spi_link_data.z               = POS_BFP_OF_REAL(stateGetPositionEnu_f()->z);*/

    /* Test 2 INDI and PID reference tracking */
    // rates
    /*high_speed_logger_spi_link_data.p               = state.body_rates_i.p;
    high_speed_logger_spi_link_data.q               = state.body_rates_i.q;
    //high_speed_logger_spi_link_data.r               = state.body_rates_i.r;
    // reference angles
    high_speed_logger_spi_link_data.roll_setpoint   = ANGLE_BFP_OF_REAL(h_ctl_roll_setpoint);
    high_speed_logger_spi_link_data.pitch_setpoint  = ANGLE_BFP_OF_REAL(h_ctl_pitch_loop_setpoint);
    // commands
    high_speed_logger_spi_link_data.cmd_indi        = radio_control.values[6];
    high_speed_logger_spi_link_data.cmd_roll        = commands[1];
    high_speed_logger_spi_link_data.cmd_pitch       = commands[2];
    // accelerations
    high_speed_logger_spi_link_data.ref_acc_pdot    = ANGLE_BFP_OF_REAL(indi.angular_accel_ref.p);
    high_speed_logger_spi_link_data.filt_acc_pdot   = ANGLE_BFP_OF_REAL(indi.filtered_rate_deriv.p);
    high_speed_logger_spi_link_data.ref_acc_qdot    = ANGLE_BFP_OF_REAL(indi.angular_accel_ref.q);
    high_speed_logger_spi_link_data.filt_acc_qdot   = ANGLE_BFP_OF_REAL(indi.filtered_rate_deriv.q);
    high_speed_logger_spi_link_data.probe_cmd_l     = cmd_trimmed_left;
    high_speed_logger_spi_link_data.probe_cmd_r     = cmd_trimmed_right;*/

    /* Test 3 OUTER LOOP reference tracking */
    // altitude, course and inner loop setpoints
    /*high_speed_logger_spi_link_data.cmd_indi            = radio_control.values[6];
    high_speed_logger_spi_link_data.probe_cmd_l         = cmd_trimmed_left;
    high_speed_logger_spi_link_data.probe_cmd_r         = cmd_trimmed_right;

    high_speed_logger_spi_link_data.altitude_setpoint    = POS_BFP_OF_REAL(v_ctl_altitude_setpoint);  
    high_speed_logger_spi_link_data.altitude    	 = POS_BFP_OF_REAL(enu_posf.z);
    high_speed_logger_spi_link_data.climb_setpoint   	 = SPEED_BFP_OF_REAL(v_ctl_climb_setpoint);
    high_speed_logger_spi_link_data.EnuSpeedZ    	 = SPEED_BFP_OF_REAL(enu_speedf.z);
    high_speed_logger_spi_link_data.throttle_controlled  = ANGLE_BFP_OF_REAL(v_ctl_throttle_setpoint);
    high_speed_logger_spi_link_data.pitch_setpoint       = ANGLE_BFP_OF_REAL(h_ctl_pitch_loop_setpoint);
    high_speed_logger_spi_link_data.course_setpoint      = ANGLE_BFP_OF_REAL(h_ctl_course_setpoint);
    high_speed_logger_spi_link_data.des_x    		 = POS_BFP_OF_REAL(desired_x);
    high_speed_logger_spi_link_data.des_y    		 = POS_BFP_OF_REAL(desired_y);
    high_speed_logger_spi_link_data.x    		 = POS_BFP_OF_REAL(enu_posf.x);
    high_speed_logger_spi_link_data.y    		 = POS_BFP_OF_REAL(enu_posf.y);
    high_speed_logger_spi_link_data.roll_setpoint        = ANGLE_BFP_OF_REAL(h_ctl_roll_setpoint);*/

    /* Test 3 OUTER LOOP reference tracking */
    // altitude, course and inner loop setpoints

    high_speed_logger_spi_link_data.airspeed_left_adc_scaled       = ANGLE_BFP_OF_REAL(airspeed_left_adc.scaled);
    high_speed_logger_spi_link_data.pitch_left_adc_scaled          = ANGLE_BFP_OF_REAL(pitch_left_adc.scaled);
    high_speed_logger_spi_link_data.airspeed_right_adc_scaled      = ANGLE_BFP_OF_REAL(airspeed_right_adc.scaled);  
    high_speed_logger_spi_link_data.pitch_right_adc_scaled         = ANGLE_BFP_OF_REAL(pitch_right_adc.scaled);

    high_speed_logger_spi_link_data.airspeed_left_adc_raw       = airspeed_left_adc.raw;
    high_speed_logger_spi_link_data.pitch_left_adc_raw          = pitch_left_adc.raw;
    high_speed_logger_spi_link_data.airspeed_right_adc_raw      = airspeed_right_adc.raw;  
    high_speed_logger_spi_link_data.pitch_right_adc_raw         = pitch_right_adc.raw;

    // high_speed_logger_spi_link_data.cmd_indi            = radio_control.values[6];
    high_speed_logger_spi_link_data.theta           	 = stateGetNedToBodyEulers_i()->theta;
    high_speed_logger_spi_link_data.course_setpoint      = ANGLE_BFP_OF_REAL(h_ctl_course_setpoint);
    high_speed_logger_spi_link_data.des_x    		 = POS_BFP_OF_REAL(desired_x);
    high_speed_logger_spi_link_data.des_y    		 = POS_BFP_OF_REAL(desired_y);
    high_speed_logger_spi_link_data.x    		 = POS_BFP_OF_REAL(enu_posf.x);

    high_speed_logger_spi_link_data.y    		 = POS_BFP_OF_REAL(enu_posf.y);
    high_speed_logger_spi_link_data.roll_setpoint        = ANGLE_BFP_OF_REAL(h_ctl_roll_setpoint);


/*
    // probe calibration and gain
    high_speed_logger_spi_link_data.offset_pl  = pitch_left_adc.offset;
    high_speed_logger_spi_link_data.offset_al  = airspeed_left_adc.offset;
    high_speed_logger_spi_link_data.offset_pr  = pitch_right_adc.offset;
    high_speed_logger_spi_link_data.offset_ar  = airspeed_right_adc.offset;
    high_speed_logger_spi_link_data.pprobes    = ANGLE_BFP_OF_REAL(pgain);
    // probe pressures differentials in millipascals
    high_speed_logger_spi_link_data.probe_press_l = ANGLE_BFP_OF_REAL(pitch_left_adc.scaled);
    high_speed_logger_spi_link_data.probe_press_r = ANGLE_BFP_OF_REAL(pitch_right_adc.scaled);

*/
    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);
  }

}

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  high_speed_logger_spi_link_ready = true;
}


