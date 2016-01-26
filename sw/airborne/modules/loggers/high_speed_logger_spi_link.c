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
#include "modules/sensors/turbulence_adc.h"
#include "modules/sensors/potentiometer_adc.h"

struct high_speed_logger_spi_link_data high_speed_logger_spi_link_data;
struct spi_transaction high_speed_logger_spi_link_transaction;

static volatile bool_t high_speed_logger_spi_link_ready = TRUE;

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
  if (high_speed_logger_spi_link_ready) {
    high_speed_logger_spi_link_ready = FALSE;

    /* Data which will be logged for servo model */
    high_speed_logger_spi_link_data.uin        = ANGLE_BFP_OF_REAL(indi.u_in.p);
    high_speed_logger_spi_link_data.uact       = ANGLE_BFP_OF_REAL(indi.u_act_dyn.p);
    high_speed_logger_spi_link_data.servo      = ANGLE_BFP_OF_REAL(servo_delayed_input);
    // probe calibration and gain
    high_speed_logger_spi_link_data.offset_pl  = potentiometer_adc_raw;
    high_speed_logger_spi_link_data.offset_al  = potentiometer_adc_raw;
    high_speed_logger_spi_link_data.offset_pr  = potentiometer_adc_raw;
    high_speed_logger_spi_link_data.offset_ar  = potentiometer_adc_raw;
    high_speed_logger_spi_link_data.pprobes    = potentiometer_adc_raw;
    // gains
    high_speed_logger_spi_link_data.pgain      = potentiometer_adc_raw;
    high_speed_logger_spi_link_data.dgain      = potentiometer_adc_raw;
    // probe pressures differentials in millipascals
    high_speed_logger_spi_link_data.probe_press_l = potentiometer_adc_raw;
    high_speed_logger_spi_link_data.probe_press_r = potentiometer_adc_raw;
    // commands
    high_speed_logger_spi_link_data.command_roll   = potentiometer_adc_raw;
    high_speed_logger_spi_link_data.command_turb_l = potentiometer_adc_raw;
    high_speed_logger_spi_link_data.command_turb_r = potentiometer_adc_raw;

    /* Data which will be logged for both pid and indi */
/*
    high_speed_logger_spi_link_data.phi        	    = stateGetNedToBodyEulers_i()->phi;
    high_speed_logger_spi_link_data.p               = state.body_rates_i.p;
    high_speed_logger_spi_link_data.roll_setpoint   = ANGLE_BFP_OF_REAL(h_ctl_roll_setpoint);
*/

    /* Indi parameters */
/*
    // outer loop gains
    high_speed_logger_spi_link_data.pgain      = ANGLE_BFP_OF_REAL(reference_acceleration.err_p);
    high_speed_logger_spi_link_data.dgain      = ANGLE_BFP_OF_REAL(reference_acceleration.rate_p);
    // accelerations
    high_speed_logger_spi_link_data.ref_acc    = ANGLE_BFP_OF_REAL(indi.angular_accel_ref.p);
    high_speed_logger_spi_link_data.filt_acc   = ANGLE_BFP_OF_REAL(indi.filtered_rate_deriv.p);
    // indi paramters
    high_speed_logger_spi_link_data.act_dyn_p  = ANGLE_BFP_OF_REAL(tau_act_dyn_p);
    high_speed_logger_spi_link_data.g          = ANGLE_BFP_OF_REAL(G);
    // indi commands
    high_speed_logger_spi_link_data.du_p         = ANGLE_BFP_OF_REAL(indi.du.p);
    high_speed_logger_spi_link_data.u_p          = ANGLE_BFP_OF_REAL(indi.u.p);
    high_speed_logger_spi_link_data.u_in_p       = ANGLE_BFP_OF_REAL(indi.u_in.p);
    high_speed_logger_spi_link_data.u_act_dyn    = ANGLE_BFP_OF_REAL(indi.u_act_dyn.p);
    high_speed_logger_spi_link_data.probes_acc   = ANGLE_BFP_OF_REAL(probes_ang_acc);
    high_speed_logger_spi_link_data.command_roll = commands[1];
*/
    /* Pid parameters */
/*
    // probe calibration and gain
    high_speed_logger_spi_link_data.offset_pl  = pitch_left_adc.offset;
    high_speed_logger_spi_link_data.offset_al  = airspeed_left_adc.offset;
    high_speed_logger_spi_link_data.offset_pr  = pitch_right_adc.offset;
    high_speed_logger_spi_link_data.offset_ar  = airspeed_right_adc.offset;
    high_speed_logger_spi_link_data.pprobes    = ANGLE_BFP_OF_REAL(pgain);
    // gains
    high_speed_logger_spi_link_data.pgain      = ANGLE_BFP_OF_REAL(h_ctl_roll_attitude_gain);
    high_speed_logger_spi_link_data.dgain      = ANGLE_BFP_OF_REAL(h_ctl_roll_rate_gain);
    // probe pressures differentials in millipascals
    high_speed_logger_spi_link_data.probe_press_l = ANGLE_BFP_OF_REAL(pitch_left_adc.scaled);
    high_speed_logger_spi_link_data.probe_press_r = ANGLE_BFP_OF_REAL(pitch_right_adc.scaled);
    // commands
    high_speed_logger_spi_link_data.command_roll   = commands[1];
    high_speed_logger_spi_link_data.command_turb_l = commands[4];
    high_speed_logger_spi_link_data.command_turb_r = commands[5];
*/

    spi_submit(&(HIGH_SPEED_LOGGER_SPI_LINK_DEVICE), &high_speed_logger_spi_link_transaction);
  }

  high_speed_logger_spi_link_data.id++;
}

static void high_speed_logger_spi_link_trans_cb(struct spi_transaction *trans __attribute__((unused)))
{
  high_speed_logger_spi_link_ready = TRUE;
}


