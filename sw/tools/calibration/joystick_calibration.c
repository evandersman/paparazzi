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
  GtkWidget *hscale;
  GtkWidget *adj;
  GtkWidget *name_label;
  GtkWidget *rev_label;
  GtkWidget *axisbox;
  GtkWidget *axisalign;
};

// the axis used in the calibration file
struct output_axis{
  char name[10];
  int index;
  int16_t min, center, max;
  char convention[164];
};

volatile enum STATE{
  FIND_MAX_MIN,
  FIND_CENTER,
  FIND_ROLL_AXIS,
  FIND_PITCH_AXIS,
  FIND_YAW_AXIS,
  FIND_THROTTLE_AXIS,
  SAVE_FILE,
};

enum STATE state;

char answer;
int i;

GtkWidget *window, *vbox, *hbox, *halign, *next, *previous, *vbox1, *valign;
GtkWidget *info_text;

struct joystick_axis axis[AXIS_COUNT];

struct output_axis axis_output[5] = {
{
  .name = "roll",
  .index = -1,
  .max = -127,
  .center= 0,
  .min = 127,
},
{
  .name = "pitch",
  .index = -1,
  .max = -127,
  .center= 0,
  .min = 127,
},
{
  .name = "yaw",
  .index = -1,
  .max = -127,
  .center= 0,
  .min = 127,
},
{
  .name = "throttle",
  .index = -1,
  .max = 0,
  .center= 64,
  .min = 127,
},
{
  .name = "mode",
  .index = -1,
  .max = 0,
  .center= 1,
  .min = 2,
},
}; 

void calibrate_axis_thread(int input_axis)
{
  int n;
  stick_read();
  for (n = 0; n < stick_axis_count; n++)
  {
    //show the axis values
    gtk_adjustment_set_value(axis[n].adj, stick_axis_values[n]);
    if (axis[n].used == 0){
      if (stick_axis_values[n] > 0.8*axis[n].max && stick_axis_values[n] > 0){
        axis[n].used = 1;
        axis[n].number = n;
        axis[n].reverse = 0;
        axis_output[input_axis].index = n;
        gtk_button_set_label (axis[n].rev_label,"normal");
        gtk_button_set_label (axis[n].name_label, axis_output[input_axis].name);
      }
      if (stick_axis_values[n] < 0.8*axis[n].min && stick_axis_values[n] < 0){
        axis[n].used = 1;
        axis[n].number = n;
        axis[n].reverse = 1;
        axis_output[input_axis].index = n;
        gtk_button_set_label (axis[n].rev_label,"reversed");
        gtk_button_set_label (axis[n].name_label, axis_output[input_axis].name);
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
}

void find_max_min(void){

  int n;
  stick_read();
  for (n = 0; n < stick_axis_count; n++)
  {
    //show the max and min values
    gtk_adjustment_set_value(axis[n].adj, stick_axis_values[n]);
    //store the max and min values
    if (stick_axis_values[n] > axis[n].max){
      axis[n].max = stick_axis_values[n];
    }
    if (stick_axis_values[n] < axis[n].min){
      axis[n].min = stick_axis_values[n];
    }
  }
}

void find_center(void){

  int n;
  stick_read();
  for (n = 0; n < stick_axis_count; n++)
  {
    //show the center values
    gtk_adjustment_set_value(axis[n].adj, stick_axis_values[n]);
    //store the center values
    axis[n].center = stick_axis_values[n];
  }
}

void state_definition(void){

  switch (state){
      case FIND_MAX_MIN:
        gtk_label_set_text (info_text, "Move ALL axis from MAX to MIN\n");
        break;
      case FIND_CENTER:
        gtk_label_set_text (info_text, "Move ALL to CENTER position\n");
        break;
      case FIND_ROLL_AXIS:
        gtk_label_set_text (info_text, "Move ROLL axis to the right and back to center\n");
        break;
      case FIND_PITCH_AXIS:
        gtk_label_set_text (info_text, "Move PITCH axis towards you (pitch up) and back to the center\n");
        break;
      case FIND_YAW_AXIS:
        gtk_label_set_text (info_text, "Move YAW axis to the right and back to the center\n");
        break;
      case FIND_THROTTLE_AXIS:
        gtk_label_set_text (info_text, "Move THROTTLE axis up (full throttle) and back to the center\n");
        gtk_button_set_label (next,"Save");
        break;
      case SAVE_FILE:
        print_to_file();
        gtk_label_set_text (info_text, "The results of the calibration have been saved in joystick_calibration.xml\n");
        break;
  }
}

void on_next (GtkWidget *widget,
             gpointer   data)
{
  int current_state;
  current_state = state;

  if ( current_state < SAVE_FILE){
    state++;
  }
  else{
    state = 0;
  }
  state_definition();
}

void on_previous (GtkWidget *widget,
             gpointer   data)
{
  int current_state;
  current_state = state;

  if ( current_state > FIND_MAX_MIN){
    state--;
  }
  else{
    state = 0;
  }
  state_definition();
}

static gboolean calibration_timer(GtkWidget *widget){

  switch (state){

      case FIND_MAX_MIN:
        find_max_min();
        break;
      case FIND_CENTER:
        find_center();
        break;
      case FIND_ROLL_AXIS:
        calibrate_axis_thread(0); 
        break;
      case FIND_PITCH_AXIS:
        calibrate_axis_thread(1);
        break;
      case FIND_YAW_AXIS:
        calibrate_axis_thread(2);
        break;
      case FIND_THROTTLE_AXIS:
        calibrate_axis_thread(3);
        break;
      case SAVE_FILE:
        break;
  }
  return TRUE;
  if(state == 7){
    return FALSE;
  }
}

GtkWidget* build_gui ( void ) {

  // create a new window
  window = gtk_window_new (GTK_WINDOW_TOPLEVEL);
  gtk_window_set_title (GTK_WINDOW (window), "Joystick calibration");
  gtk_window_set_default_size(GTK_WINDOW (window), 400, -1);

  // create a new vertical box
  vbox = gtk_vbox_new (FALSE, 0);
  gtk_container_add (GTK_CONTAINER (window), vbox);

  // add text box inside the vertical box with instructions
  info_text = gtk_label_new ("To start the calibration press NEXT");
  gtk_box_pack_start (GTK_BOX (vbox), info_text, TRUE, TRUE, 0);

  // add a vertically aligned box at the top inside the main vertical box
  vbox1 = gtk_vbox_new (FALSE, 0);
  valign = gtk_alignment_new(0.5, 0, 0.3, 0.2);
  gtk_container_add(GTK_CONTAINER(valign), vbox1);
  gtk_container_add (GTK_CONTAINER (vbox), valign);

  for (i = 0; i < stick_axis_count; i++)
  {
  axis[i].adj = gtk_adjustment_new (0.0, -32767.0, 32767.0, 1, 1.0, 1.0);
  axis[i].hscale = gtk_hscale_new (axis[i].adj);

  axis[i].name_label = gtk_button_new_with_label("axis name");
  axis[i].rev_label = gtk_button_new_with_label("axis sign");
  axis[i].axisbox = gtk_hbox_new (FALSE, 0);
  axis[i].axisalign = gtk_alignment_new(0.5, 1, 0.3, 0.2);
  gtk_container_add(GTK_CONTAINER(axis[i].axisalign), axis[i].axisbox);
  gtk_container_add (GTK_CONTAINER (vbox1), axis[i].axisalign);

  gtk_box_pack_start(GTK_BOX(vbox1), axis[i].hscale, TRUE, TRUE, 5);
  gtk_box_pack_start(GTK_BOX(axis[i].axisbox), axis[i].name_label, TRUE, TRUE, 5);
  gtk_box_pack_start(GTK_BOX(axis[i].axisbox), axis[i].rev_label, TRUE, TRUE, 5);
  }

  // add a horizontally aligned box at the bottom inside the vertical box
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
    state = -1;
  }

  gtk_init(&argc, &argv);
  GtkWidget* window = build_gui();
  g_signal_connect (window, "destroy", G_CALLBACK (gtk_main_quit), NULL);
  g_signal_connect (next, "clicked", G_CALLBACK (on_next), NULL);
  g_signal_connect (previous, "clicked", G_CALLBACK (on_previous), NULL);
  g_timeout_add(1000/60, calibration_timer, NULL);
  gtk_widget_show_all(window);
  gtk_main();

  return 0;

}
