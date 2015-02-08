# Hey Emacs, this is a -*- makefile -*-
#
#   Copyright (C) 2015 The Paparazzi Team
#
# This file is part of paparazzi.
#
# paparazzi is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2, or (at your option)
# any later version.
#
# paparazzi is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with paparazzi; see the file COPYING.  If not, write to
# the Free Software Foundation, 59 Temple Place - Suite 330,
# Boston, MA 02111-1307, USA.
#

# Quiet compilation
Q=@

CC = gcc
PKG = -package pprz,glibivy
LINKPKG = $(PKG) -linkpkg -dllpath-pkg pprz

# apparently on OSX `sdl-config --libs` also has -lSDLmain which we don't want
SDL_LDIRS = $(shell pkg-config sdl --libs-only-L)
SDL_LIBS = -lSDL
# we do however need -lSDLmain for test_stick, which is just c
SDL_LDFLAGS = $(shell sdl-config --libs)

GTK_CFLAGS = $(shell pkg-config gtk+-2.0 --cflags)
GTK_LDFLAGS = $(shell pkg-config gtk+-2.0 --libs)

all: joystick_calibration

joystick_calibration: joystick_calibration.o sdl_stick.o
	@echo BUILD $@
	$(Q)$(CC) -g -O2 -o $@ $^ $(SDL_LDFLAGS) $(GTK_LDFLAGS)

sdl_stick.o : ../../ground_segment/joystick/sdl_stick.c
	@echo CC ../../ground_segment/joystick/sdl_stick.o
	$(Q)$(CC) -c -O2 $(GTK_CFLAGS) $<

%.o : %.c
	@echo CC .$@
	$(Q)$(CC) -c -O2 $(GTK_CFLAGS) $<

clean:
	$(Q)rm -f *~ *.o *.bak joystick_calibration sdl_stick.o

.PHONY: all clean

