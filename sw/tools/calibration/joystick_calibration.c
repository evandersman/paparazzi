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
#include <gtk/gtk.h>

#define AXIS_COUNT        32

// the axis of the joystick device
struct joystick_axis{
  int number;
  int16_t min, center, max;
  bool reverse;
  bool used;
};

// the axis used in the calibration file
struct output_axis{
  char name[10];
  int index;
  int16_t min, center, max;
  char convention[164];
};


volatile bool thread_done;
int current_axis;
char answer;
int i;

struct joystick_axis axis[AXIS_COUNT];

struct output_axis axis_output[5] = {
{
  .name = "roll",
  .index = -1,
  .max = -127,
  .center= 0,
  .min = 127,
  .convention = "Move ROLL axis to the right"
},
{
  .name = "pitch",
  .index = -1,
  .max = -127,
  .center= 0,
  .min = 127,
  .convention = "Move PITCH axis towards you (pitch up)"
},
{
  .name = "yaw",
  .index = -1,
  .max = -127,
  .center= 0,
  .min = 127,
  .convention = "Move YAW axis to the right"
},
{
  .name = "throttle",
  .index = -1,
  .max = 0,
  .center= 64,
  .min = 127,
  .convention = "Move THROTTLE axis up (full throttle)"
},
{
  .name = "mode",
  .index = -1,
  .max = 0,
  .center= 1,
  .min = 2,
  .convention = "Move MODE axis to navigation"
},
}; 

void *max_min_thread(void* data)
{
    int n;
    while (!thread_done)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        if (stick_axis_values[n] > axis[n].max){
          axis[n].max = stick_axis_values[n];
        }
        if (stick_axis_values[n] < axis[n].min){
          axis[n].min = stick_axis_values[n];
        }
      }
    }
}

void *center_thread(void* data)
{
    int n;
    while (!thread_done)
    {
      stick_read();
      for (n = 0; n < stick_axis_count; n++)
      {
        axis[n].center = stick_axis_values[n];
      }
    }
}

void *calibrate_axis_thread(void* data)
{
    int n;
    while (!thread_done)
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
          if (stick_axis_values[n] < 0.8*axis[n].min && stick_axis_values[n] < 0){
            axis[n].used = 1;
            current_axis = n;
            axis[n].number = n;
            axis[n].reverse = 1;
          }
        }
      }
    }
}

void flush_input ( FILE *in )
{
  int ch;

  do
    ch = fgetc ( in ); 
  while ( ch != EOF && ch != '\n' ); 

  clearerr ( in );
}

void find_max_min(void)
{
  //ask user for the max and min position
  printf("Move ALL axis from MAX to MIN.\n");
  thread_done = 0;
  //start max_min_thread
  pthread_t thread_max_min;
  pthread_create( &thread_max_min, NULL, max_min_thread, NULL);
  // clear input buffer
  flush_input( stdin );
  // ask user when to stop the thread
  printf("Done? Press any KEY + ENTER to continue...\n");
  // clear output buffer
  fflush( stdout );
  // wait until user is finished calibrating
  getchar();
  //stop max_min_thread
  thread_done = 1;
  pthread_join(thread_max_min, NULL);
}

void find_center(void)
{
  //ask user for the center position
  printf("Move ALL axis to the CENTER position\n");
  thread_done = 0;
  //start center_thread
  pthread_t thread_center;
  pthread_create( &thread_center, NULL, center_thread, NULL);
  // clear input buffer
  flush_input( stdin );
  //ask user when to stop the thread
  printf("Done? Press any KEY + ENTER to continue...\n");
  // clear output buffer
  fflush ( stdout );
  // wait until user is finished calibrating
  getchar();
  //stop center_thread
  thread_done = 1;
  pthread_join(thread_center, NULL);
}

void find_axis_info(void)
{
  printf("Start calibrating the axis:\n");
  printf("Do NOT move the axis away from the center position unless asked to do otherwise\n");
  for (i = 0; i < 5; i++)
  {
    printf("%s\n", axis_output[i].convention);
    thread_done = 0;
    pthread_t thread_axis;
    //start calibrate_axis_thread
    pthread_create( &thread_axis, NULL, calibrate_axis_thread, NULL);
    // clear input buffer
    flush_input( stdin );
    //ask user when to stop the thread
    printf("Done? Press any KEY + ENTER to continue...\n");
    // clear output buffer
    fflush ( stdout );
    // wait until user is finished calibrating
    getchar();
    //stop calibrate_axis_thread
    thread_done = 1;
    axis_output[i].index = current_axis;
    pthread_join(thread_axis, NULL);
  }
}

void print_to_file(void)
{
  // open a joystick calibration file in the conf/joystick directory
  FILE * fp;
  fp = fopen ("../../../conf/joystick/joystick_calibration.xml", "w+");
  // write the calibration values to the joystick configuration file
  fprintf(fp, "<joystick>\n");
  fprintf(fp, "  <input>\n");
  for (i = 0; i < 5; i++)
  {
    fprintf(fp, "    <axis index=\"%i\" name=\"%s\"/>\n", axis[axis_output[i].index].number, axis_output[i].name);
  }
  fprintf(fp, "\n");
  fprintf(fp, "  </input>\n");
  fprintf(fp, "  <messages period=\"0.025\">\n");
  fprintf(fp, "    <message class=\"datalink\" name=\"RC_4CH\" send_always=\"true\">\n");
  for (i = 0; i < 5; i++)
  {
    int out_idx = axis_output[i].index;
    // if reversed: max = -min and min = -max; this is necessary as the range is not equal on both sides.
    if(axis[out_idx].reverse == 0)
    {
      fprintf(fp, "      <field name=\"%s\" value=\"Fitcenter(%s,%d,%d,%d,%d,%d,%d)\"/>\n", axis_output[i].name, axis_output[i].name, axis[out_idx].min, axis[out_idx].center, axis[out_idx].max, axis_output[i].max, axis_output[i].center, axis_output[i].min);
    }
    if(axis[out_idx].reverse == 1)
    {
        fprintf(fp, "      <field name=\"%s\" value=\"Fitcenter(-%s,%d,%d,%d,%d,%d,%d)\"/>\n", axis_output[i].name, axis_output[i].name, -axis[out_idx].max, axis[out_idx].center, - axis[out_idx].min, axis_output[i].max, axis_output[i].center, axis_output[i].min);
    }
  }
  fprintf(fp, "    </message>\n");
  fprintf(fp, "  </messages>\n");
  fprintf(fp, "</joystick>\n");
  fclose(fp);
  printf("The results of the calibration have been saved in joystick_calibration.xml\n");
}

GtkWidget* build_gui ( void ) {
  GtkWidget *window, *vbox, *info_text, *hbox, *halign, *next, *previous;

  // create a new window
  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window), "Joystick calibration");
  gtk_window_set_default_size(GTK_WINDOW (window), 400, -1);

  // create a new vertical box
  vbox = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox);

  // add text box
  info_text = gtk_label_new ("Move ALL axis from MAX to MIN");
  gtk_box_pack_start (GTK_BOX (vbox), info_text, TRUE, TRUE, 0);

  hbox = gtk_hbox_new (FALSE, 0);
  halign = gtk_alignment_new(0.5, 1, 0.3, 0.2);
  gtk_container_add(GTK_CONTAINER(halign), hbox);
  gtk_container_add (GTK_CONTAINER (vbox), halign);

  previous = gtk_button_new_with_label("Previous");
  gtk_box_pack_start(GTK_BOX(hbox), previous, TRUE, TRUE, 5);
  next = gtk_button_new_with_label("Next");
  gtk_box_pack_start(GTK_BOX(hbox), next, TRUE, TRUE, 5);

  return window;
}


int main(int argc, char** argv)
{  
  gtk_init(&argc, &argv);
  GtkWidget* window = build_gui();
  gtk_widget_show_all(window);
  gtk_main();

  return 0;
  printf("Would you like to start the joystick calibration? Enter (Y/n) \n");
  scanf("%c", &answer);

  if (answer != 'y' && answer != 'Y'){
    return 0;
  }

  int device_index;
  int joystick_number;
  SDL_Joystick *sdl_joystick;

  /* Initialize SDL with joystick support */
  if (SDL_Init(SDL_INIT_JOYSTICK) < 0)
  {
    printf("Could not initialize SDL: %s.\n", SDL_GetError());
    exit(-1);
  }

  //Quit SDL at exit
  atexit(SDL_Quit);
  
  int j;
  int joysticks_available = 5;
  const char * name;
  //show available joysticks
  for (j = 0; j < joysticks_available; j++)
  {
    name = SDL_JoystickName(j);
    if (name != NULL)
    {
      printf("Input device name: \"%s\" on SDL Index %i\n", name, j);
    }
  }
 
  // user choses which joystick to configure
  printf("Which input device would you like to calibrate? Speficy SDL Index (0,1,2..) \n");
  scanf("%d", &joystick_number);
  device_index = joystick_number;
  int opened = stick_init(device_index);

  printf("The number of axis is %i\n", stick_axis_count);
  printf("\n");

  int cnt;
  //setting max, min, center, reverse and used to 0 (for all axis)
  for (cnt = 0; cnt < stick_axis_count; cnt++)
  {
    axis[cnt].center = 0;
    axis[cnt].min = 0;
    axis[cnt].max = 0;
    axis[cnt].reverse = 0;
    axis[cnt].used = 0;
  }

  find_max_min();
  find_center();
  find_axis_info();
  print_to_file();
}
