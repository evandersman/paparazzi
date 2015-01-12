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


int32_t axis_min[AXIS_COUNT], axis_max[AXIS_COUNT];
int32_t axis_center[AXIS_COUNT];
int axis;
char answer;
char y;
int done;
int roll;
int pitch;
int yaw;
int throttle;
int mode;
int rev;
char *r0;
char *r1;
char *r2;
char *r3;
char *r4;


void *find_max_min(void* data)
{
    int n;
    while (1)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        if (stick_axis_values[n] > axis_max[n]){
          axis_max[n] = stick_axis_values[n];
        }
        if (axis_min[n] > stick_axis_values[n]){
          axis_min[n] = stick_axis_values[n];
        }
        if(done)
          return;
      }
    }
}

void *find_center(void* data)
{
    int n;
    while (1)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        axis_center[n] = stick_axis_values[n];
        if(done)
          return;
      }
    }
}

void *calibrate_axis(void* data)
{
    int n;
    while (1)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        if (stick_axis_values[n] > 0.8*axis_max[n] && stick_axis_values[n] > 0){
          axis = n;
          rev = 0;
        }
        if (-stick_axis_values[n] > -0.8*axis_min[n] && 0 > stick_axis_values[n]){
          axis = n;
          rev = 1;
        }
        if(done)
          return;
      }
    }
}

int main(void)
{  
  int cnt;
  printf("Would you like to start the joystick calibration? Enter (y/n) \n");
  scanf("%s", &answer);

  if (answer == 'y'){
    
    int device_index = 0;
    int opened = stick_init(device_index);

    FILE * fp;
    fp = fopen ("joystick_calibration.xml", "w+");

    printf("The number of axis is %i\n", stick_axis_count);
    printf("\n");

    for (cnt = 0; cnt < stick_axis_count; cnt++)
    {
      //setting max and min and center to zero initially
      axis_center[cnt] = 0;
      axis_min[cnt] = 0;
      axis_max[cnt] = 0;
    }

    //find center position
    printf("Move all axis to the center position.\n");
    done = 0;
    //start thread find_center()
    pthread_t thread1;
    pthread_create( &thread1, NULL, find_center, NULL);
    //stop thread find_max_min
    printf("Done? Press any key to continue\n");
    scanf("%s", &answer);
    done = 1;

    //find max and min position
    printf("Move all axis from max to min.\n");
    done = 0;
    //start thread find_max_min()
    pthread_t thread2;
    pthread_create( &thread2, NULL, find_max_min, NULL);
    //stop thread find_max_min
    printf("Done? Press any key to continue\n");
    scanf("%s", &answer);
    done = 1;

    fprintf(fp, "<joystick>\n");
    fprintf(fp, "  <input>\n");
    for (cnt = 0; cnt < stick_axis_count; cnt++)
    {
      //calibrate one axis at the time
      switch(cnt)
      {
      case 0 :
        printf("Start calibrating the roll axis. Move the roll axis to the right only. \n");
        done = 0;
        rev = 0;
        //start thread
        pthread_t thread3;
        pthread_create( &thread3, NULL, calibrate_axis, NULL);
        //stop thread
        printf("Done? Press any key to continue\n");
        scanf("%s", &answer);
        done = 1;
        roll = axis;
        if (rev == 1){r0 = "-";}
        if (rev == 0){r0 = " ";}
        fprintf(fp, "    <axis index=\"%i\" name=\"roll\"/>\n", axis);
        break;
      case 1 :
        printf("Start calibrating the pitch axis.  Move the pitch axis towards you (pitch up). \n");
        done = 0;
        rev = 0;
        //start thread
        pthread_t thread4;
        pthread_create( &thread4, NULL, calibrate_axis, NULL);
        //stop thread
        printf("Done? Press any key to continue\n");
        scanf("%s", &answer);
        done = 1;
        pitch = axis;
        if (rev == 1){r1 = "-";}
        if (rev == 0){r1 = " ";}
        fprintf(fp, "    <axis index=\"%i\" name=\"pitch\"/>\n", axis);
        break;
      case 2 :
        printf("Start calibrating the yaw axis.  Move the yaw axis to the right only.\n");
        done = 0;
        rev = 0;
        //start thread
        pthread_t thread5;
        pthread_create( &thread5, NULL, calibrate_axis, NULL);
        //stop thread
        printf("Done? Press any key to continue\n");
        scanf("%s", &answer);
        done = 1;
        if (rev == 1){r2 = "-";}
        if (rev == 0){r2 = " ";}
        yaw = axis;
        fprintf(fp, "    <axis index=\"%i\" name=\"yaw\"/>\n", axis);
        break;
      case 3 :
        printf("Start calibrating the throttle axis.  Move the throttle axis to up only (away from you). \n");
        done = 0;
        rev = 0;
        //start thread
        pthread_t thread6;
        pthread_create( &thread6, NULL, calibrate_axis, NULL);
        //stop thread
        printf("Done? Press any key to continue\n");
        scanf("%s", &answer);
        done = 1;
        if (rev == 1){r3 = "-";}
        if (rev == 0){r3 = " ";}
        throttle = axis;
        fprintf(fp, "    <axis index=\"%i\" name=\"throttle\"/>\n", axis);
        break;
      case 4 :
        printf("Start calibrating the mode axis. switch to manual mode only. \n");
        done = 0;
        rev = 0;
        //start thread
        pthread_t thread7;
        pthread_create( &thread7, NULL, calibrate_axis, NULL);
        //stop thread
        printf("Done? Press any key to continue\n");
        scanf("%s", &answer);
        done = 1;
        if (rev == 1){r4 = "-";}
        if (rev == 0){r4 = " ";}
        mode = axis;
        fprintf(fp, "    <axis index=\"%i\" name=\"mode\"/>\n", axis);
        break;
      }
    }
    // write the calibration values to a joystick configuration file
    fprintf(fp, "\n");
    fprintf(fp, "  <input/>\n");
    fprintf(fp, "  <messages period=\"0.025\">\n");
    fprintf(fp, "    <message class=\"datalink\" name=\"RC_4CH\" send_always=\"true\">\n");
    fprintf(fp, "      <field name=\"roll\" value=\"Fit(%croll,%i,%i,-127,127)\"/>\n", *r0, axis_min[roll], axis_max[roll]);
    fprintf(fp, "      <field name=\"pitch\" value=\"Fit(%cpitch,%i,%i,-127,127)\"/>\n", *r1, axis_min[pitch], axis_max[pitch]);
    fprintf(fp, "      <field name=\"yaw\" value=\"Fit(%cyaw,%i,%i,-127,127)\"/>\n", *r2, axis_min[yaw], axis_max[yaw]);
    fprintf(fp, "      <field name=\"throttle\" value=\"Fit(%cthrottle,%i,%i,0,127)\"/>\n", *r3, axis_min[throttle], axis_max[throttle]);
    fprintf(fp, "      <field name=\"mode\" value=\"Fit(%cmode,%i,%i,-127,127)\"/>\n", *r4, axis_min[mode], axis_max[mode]);
    fprintf(fp, "    </message>\n");
    fprintf(fp, "  </messages>\n");
    fprintf(fp, "<joystick/>\n");
    printf("The results of the calibration have been saved in joystick_calibration.xml\n");
    fclose(fp);
  }   
  return 0;
}
