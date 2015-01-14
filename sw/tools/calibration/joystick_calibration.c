/*
 * Basic joystick calibration
 *
 * Copyright (C) 2015 Elisabeth van der Sman
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING. If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */



#include "../../ground_segment/joystick/sdl_stick.h"
#include "../../include/std.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#define AXIS_COUNT        32


struct axis_properties{
  char name[10];
  int number;
  int16_t min, center, max;
  int reverse;
};

struct axis_properties axis[7];
int8_t already_read[AXIS_COUNT];

char answer;
char y;

volatile int done;
int current_axis;
int roll;
int pitch;
int yaw;
int throttle;
int mode;

char *r[AXIS_COUNT];


void *find_max_min(void* data)
{
    int n;
    while (done == 0)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        if (stick_axis_values[n] > axis[n].max){
          axis[n].max = stick_axis_values[n];
        }
        if (axis[n].min > stick_axis_values[n]){
          axis[n].min = stick_axis_values[n];
        }
      }
    }
    //printf("Finished setting max and min values\n");
}

void *find_center(void* data)
{
    int n;
    while (done == 0)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        axis[n].center = stick_axis_values[n];
      }
    }
    //printf("Finished setting axis center\n");
}

void *calibrate_axis(void* data)
{
    int n;
    while (done == 0)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        if (n != already_read[n]){
          if (stick_axis_values[n] > 0.8*axis[n].max && stick_axis_values[n] > 0){
            already_read[n] = n;
            current_axis = n;
            //printf("axis %i has already been read: %d \n", n, already_read[n]);
            axis[n].number = n;
            axis[n].reverse = 0;
          }
          if (-stick_axis_values[n] > -0.8*axis[n].min && 0 > stick_axis_values[n]){
            already_read[n] = n;
            current_axis = n;
            //printf("axis %i has already been read: %d \n", n, already_read[n]);
            axis[n].number = n;
            axis[n].reverse = 1;
          }
        }
      }
    }
    //printf("Finished finding the corresponding axis\n");
}

void axis_sign(void)
{
  if (axis[current_axis].reverse == 1){r[current_axis] = "-";}
  if (axis[current_axis].reverse == 0){r[current_axis] = '\0';}
}

void start_thread(void)
{
  done = 0;
  pthread_t thread_axis;
  //start thread
  pthread_create( &thread_axis, NULL, calibrate_axis, NULL);
  //stop thread
  printf("Done? Press any key to continue\n");
  scanf("%s", &answer);
  done = 1;
}

int main(void)
{  
  int cnt;
  printf("Would you like to start the joystick calibration? Enter (Y/n) \n");
  scanf("%s", &answer);

  if (answer == 'y'){
    
    int device_index = 0;
    int opened = stick_init(device_index);

    printf("The number of axis is %i\n", stick_axis_count);
    printf("\n");

    //setting max and min and center to 0  and already_read to -1 (for all axis)
    for (cnt = 0; cnt < stick_axis_count; cnt++)
    {
      axis[cnt].center = 0;
      axis[cnt].min = 0;
      axis[cnt].max = 0;
      axis[cnt].reverse = 0;
      already_read[cnt] = -1;
    }

    //find max and min position
    printf("Move all axis from max to min.\n");
    done = 0;
    //start thread find_max_min()
    pthread_t thread_max_min;
    pthread_create( &thread_max_min, NULL, find_max_min, NULL);
    //stop thread find_max_min
    printf("Done? Press any key to continue\n");
    scanf("%s", &answer);
    done = 1;

    //find center position
    printf("Move all axis to the center position.\n");
    done = 0;
    //start thread find_center()
    pthread_t thread_center;
    pthread_create( &thread_center, NULL, find_center, NULL);
    //stop thread find_max_min
    printf("Done? Press any key to continue\n");
    scanf("%s", &answer);
    done = 1;

    /*for (cnt = 0; cnt < stick_axis_count; cnt++)
    {
      //print max and min and center
      printf("(min, center, max) (%d, %d, %d)\n", axis[cnt].min, axis[cnt].center, axis[cnt].max);
    }*/

    for (cnt = 0; cnt < 5; cnt++)
    {
      //calibrate one axis at the time
      switch(cnt)
      {
      case 0 :
        printf("Start calibrating the roll axis. Move the roll axis to the right only. \n");
        start_thread();
        roll = current_axis;
        axis_sign();
        break;
      case 1 :
        printf("Start calibrating the pitch axis.  Move the pitch axis towards you (pitch up). \n");
        start_thread();
        pitch = current_axis;
        axis_sign();
        break;
      case 2 :
        printf("Start calibrating the yaw axis.  Move the yaw axis to the right only.\n");
        start_thread();
        yaw = current_axis;
        axis_sign();
        break;
      case 3 :
        printf("Start calibrating the throttle axis.  Move the throttle axis to up only (away from you). \n");
        start_thread();
        throttle = current_axis;
        axis_sign();
        break;
      case 4 :
        printf("Start calibrating the mode axis. switch to manual mode only. \n");
        start_thread();
        mode = current_axis;
        axis_sign();
        break;
      }
    }

    strcpy( axis[roll].name, "roll");
    strcpy( axis[pitch].name, "pitch");
    strcpy( axis[yaw].name, "yaw");
    strcpy( axis[throttle].name, "throttle");
    strcpy( axis[mode].name, "mode");

    // write the calibration values to a joystick configuration file
    FILE * fp;
    fp = fopen ("../../../conf/joystick/joystick_calibration.xml", "w+");
    fprintf(fp, "<joystick>\n");
    fprintf(fp, "  <input>\n");
    fprintf(fp, "    <axis index=\"%i\" name=\"roll\"/>\n", axis[roll].number);
    fprintf(fp, "    <axis index=\"%i\" name=\"pitch\"/>\n", axis[pitch].number);
    fprintf(fp, "    <axis index=\"%i\" name=\"yaw\"/>\n", axis[yaw].number);
    fprintf(fp, "    <axis index=\"%i\" name=\"throttle\"/>\n", axis[throttle].number);
    fprintf(fp, "    <axis index=\"%i\" name=\"mode\"/>\n", axis[mode].number);
    fprintf(fp, "\n");
    fprintf(fp, "  <input/>\n");
    fprintf(fp, "  <messages period=\"0.025\">\n");
    fprintf(fp, "    <message class=\"datalink\" name=\"RC_4CH\" send_always=\"true\">\n");
    fprintf(fp, "      <field name=\"roll\" value=\"Fit(%sroll,%d,%d,-127,127)\"/>\n", r[roll], axis[roll].min, axis[roll].max);
    fprintf(fp, "      <field name=\"pitch\" value=\"Fit(%spitch,%d,%d,-127,127)\"/>\n", r[pitch], axis[pitch].min, axis[pitch].max);
    fprintf(fp, "      <field name=\"yaw\" value=\"Fit(%syaw,%d,%d,-127,127)\"/>\n", r[yaw], axis[yaw].min, axis[yaw].max);
    fprintf(fp, "      <field name=\"throttle\" value=\"Fit(%sthrottle,%d,%d,0,127)\"/>\n", r[throttle], axis[throttle].min, axis[throttle].max);
    fprintf(fp, "      <field name=\"mode\" value=\"Fit(%smode,%d,%d,0,2)\"/>\n", r[mode], axis[mode].min, axis[mode].max);
    fprintf(fp, "    </message>\n");
    fprintf(fp, "  </messages>\n");
    fprintf(fp, "<joystick/>\n");
    printf("The results of the calibration have been saved in joystick_calibration.xml\n");
    fclose(fp);
  }   
  return 0;
}
