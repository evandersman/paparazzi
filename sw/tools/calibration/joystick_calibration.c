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


struct joystick_axis{
  char name[10];
  int number;
  int16_t min, center, max;
  int reverse;
  bool used;
};

struct output_axis{
  char name[10];
  int index;
  char convention[164];
};


struct joystick_axis axis[AXIS_COUNT];
struct output_axis axis_output[5];

volatile int done;
int current_axis;
char answer;

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
}

void *calibrate_axis(void* data)
{
    int n;
    while (done == 0)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        if (axis[n].used == 0){
          if (stick_axis_values[n] > 0.8*axis[n].max && stick_axis_values[n] > 0){
            axis[n].used = 1;
            current_axis = n;
            axis[n].number = n;
            axis[n].reverse = 0;
          }
          if (-stick_axis_values[n] > -0.8*axis[n].min && 0 > stick_axis_values[n]){
            axis[n].used = 1;
            current_axis = n;
            axis[n].number = n;
            axis[n].reverse = 1;
          }
        }
      }
    }
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

  strcpy( axis_output[0].name, "roll");
  strcpy( axis_output[1].name, "pitch");
  strcpy( axis_output[2].name, "yaw");
  strcpy( axis_output[3].name, "throttle");
  strcpy( axis_output[4].name, "mode");

  strcpy( axis_output[0].convention, "Move ROLL axis to the right");
  strcpy( axis_output[1].convention, "Move PITCH axis towards you (pitch up)");
  strcpy( axis_output[2].convention, "Move YAW axis to the right");
  strcpy( axis_output[3].convention, "Move THROTTLE axis up");
  strcpy( axis_output[4].convention, "Move MODE axis to manual");

  printf("Would you like to start the joystick calibration? Enter (Y/n) \n");
  scanf("%s", &answer);

  if (answer == 'y'){
    
    int device_index = 0;
    int opened = stick_init(device_index);

    printf("The number of axis is %i\n", stick_axis_count);
    printf("\n");

    //setting max and min and center and used to 0 (for all axis)
    for (cnt = 0; cnt < stick_axis_count; cnt++)
    {
      axis[cnt].center = 0;
      axis[cnt].min = 0;
      axis[cnt].max = 0;
      axis[cnt].reverse = 0;
      axis[cnt].used = 0;
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

    // write the calibration values to a joystick configuration file
    FILE * fp;
    fp = fopen ("../../../conf/joystick/joystick_calibration.xml", "w+");
    fprintf(fp, "<joystick>\n");
    fprintf(fp, "  <input>\n");

    printf("Start calibrating the axis \n");
    for (cnt = 0; cnt < 5; cnt++)
    {
      printf("%s\n", axis_output[cnt].convention);
      start_thread();
      axis_output[cnt].index = current_axis;
      strcpy( axis[axis_output[cnt].index].name, axis_output[cnt].name);
      fprintf(fp, "    <axis index=\"%i\" name=\"%s\"/>\n", axis[axis_output[cnt].index].number, axis_output[cnt].name);
    }
    fprintf(fp, "\n");
    fprintf(fp, "  </input>\n");
    fprintf(fp, "  <messages period=\"0.025\">\n");
    fprintf(fp, "    <message class=\"datalink\" name=\"RC_4CH\" send_always=\"true\">\n");
    for (cnt = 0; cnt < 5; cnt++)
    {
      if(axis[axis_output[cnt].index].reverse == 0)
      {
        fprintf(fp, "      <field name=\"%s\" value=\"Fit(%s,%d,%d,-127,127)\"/>\n", axis_output[cnt].name, axis_output[cnt].name, axis[axis_output[cnt].index].min, axis[axis_output[cnt].index].max);
      }
      if(axis[axis_output[cnt].index].reverse == 1)
      {
        fprintf(fp, "      <field name=\"%s\" value=\"Fit(-%s,%d,%d,-127,127)\"/>\n", axis_output[cnt].name, axis_output[cnt].name, axis[axis_output[cnt].index].min, axis[axis_output[cnt].index].max);
      }
    }
    fprintf(fp, "    </message>\n");
    fprintf(fp, "  </messages>\n");
    fprintf(fp, "</joystick>\n");
    printf("The results of the calibration have been saved in joystick_calibration.xml\n");
    fclose(fp);
  }  
  return 0;
}
