/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "mavlink_control.h"

int top(int argc, char **argv) {
  // parse command line
  char *uart_name = (char *)"/dev/ttyS0";
  int baudrate = 57600;
  bool debug = false;
  parse_commandline(argc, argv, &uart_name, &baudrate, &debug);

  // vars
  Serial_Port serial_port(uart_name, baudrate, debug);
  Autopilot_Interface autopilot_interface(&serial_port);

  // quit signals
  serial_port_quit = &serial_port;  // global var used in quit_handler
  autopilot_interface_quit = &autopilot_interface;  // ^^
  signal(SIGINT,
         quit_handler);  // used to end program safely when C-c is pressed

  // start up
  serial_port.start();
  autopilot_interface.start();

  // actual work
  commands(autopilot_interface);

  // stop
  autopilot_interface.stop();
  serial_port.stop();

  return 0;
}

void commands(Autopilot_Interface &api) {
  // --------------------------------------------------------------------------
  //   START OFFBOARD MODE
  // --------------------------------------------------------------------------

  api.enable_offboard_control();
  usleep(100);  // give some time to let it sink in

  // now the autopilot is accepting setpoint commands

  // --------------------------------------------------------------------------
  //   SEND OFFBOARD COMMANDS
  // --------------------------------------------------------------------------
  printf("SEND OFFBOARD COMMANDS\n");

  // initialize command data strtuctures
  mavlink_set_position_target_local_ned_t sp;
  mavlink_set_position_target_local_ned_t ip = api.initial_position;

  // autopilot_interface.h provides some helper functions to build the command

  // Example 1 - Set Velocity
  //	set_velocity( -1.0       , // [m/s]
  //				  -1.0       , // [m/s]
  //				   0.0       , // [m/s]
  //				   sp        );

  // Example 2 - Set Position
  set_position(ip.x - 5.0,  // [m]
               ip.y - 5.0,  // [m]
               ip.z,        // [m]
               sp);

  // Example 1.2 - Append Yaw Command
  set_yaw(ip.yaw,  // [rad]
          sp);

  // SEND THE COMMAND
  api.update_setpoint(sp);
  // NOW pixhawk will try to move

  // Wait for 8 seconds, check position
  for (int i = 0; i < 8; i++) {
    // mavlink_local_position_ned_t pos =
    // api.current_messages.local_position_ned;
    // printf("%i CURRENT POSITION XYZ = [ % .4f , % .4f , % .4f ] \n", i,
    // pos.x,
    //        pos.y, pos.z);
    sleep(10);
  }

  printf("\n");

  // --------------------------------------------------------------------------
  //   STOP OFFBOARD MODE
  // --------------------------------------------------------------------------

  api.disable_offboard_control();

  // now pixhawk isn't listening to setpoint commands

  // --------------------------------------------------------------------------
  //   GET A MESSAGE
  // --------------------------------------------------------------------------
  printf("READ SOME MESSAGES \n");

  // copy current messages
  Mavlink_Messages messages = api.current_messages;

  // local position in ned frame
  mavlink_local_position_ned_t pos = messages.local_position_ned;
  mavlink_global_position_int_t gpos = messages.global_position_int;
  printf(
      "Got message LOCAL_POSITION_NED (spec: "
      "https://pixhawk.ethz.ch/mavlink/#LOCAL_POSITION_NED)\n");
  printf("    pos  (NED):  %f %f %f (m)\n", pos.x, pos.y, pos.z);
  printf("    pos  (GPS):  %f %f %f (m)\n", gpos.lat, gpos.lon, gpos.alt);

  // hires imu
  mavlink_highres_imu_t imu = messages.highres_imu;
  printf(
      "Got message HIGHRES_IMU (spec: "
      "https://pixhawk.ethz.ch/mavlink/#HIGHRES_IMU)\n");
  printf("    ap time:     %llu \n", imu.time_usec);
  printf("    acc  (NED):  % f % f % f (m/s^2)\n", imu.xacc, imu.yacc,
         imu.zacc);
  printf("    gyro (NED):  % f % f % f (rad/s)\n", imu.xgyro, imu.ygyro,
         imu.zgyro);
  printf("    mag  (NED):  % f % f % f (Ga)\n", imu.xmag, imu.ymag, imu.zmag);
  printf("    baro:        %f (mBar) \n", imu.abs_pressure);
  printf("    altitude:    %f (m) \n", imu.pressure_alt);
  printf("    temperature: %f C \n", imu.temperature);

  printf("\n");

  return;
}

void parse_commandline(int argc, char **argv, char **uart_name, int *baudrate,
                       bool *debug) {
  // string for command line usage
  const char *commandline_usage =
      "usage: mavlink_control [-d <devicename>] [-b <baudrate>] [-t <debug>]";

  // Read input arguments
  for (int i = 1; i < argc; i++) {  // argv[0] is "mavlink"

    // Help
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printf("%s\n", commandline_usage);
      throw EXIT_FAILURE;
    }

    // UART device ID
    if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
      if (argc > i + 1) {
        *uart_name = argv[i + 1];

      } else {
        printf("%s\n", commandline_usage);
        throw EXIT_FAILURE;
      }
    }

    // Baud rate
    if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
      if (argc > i + 1) {
        *baudrate = atoi(argv[i + 1]);

      } else {
        printf("%s\n", commandline_usage);
        throw EXIT_FAILURE;
      }
    }

    // debugging
    if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--debug") == 0) {
      *debug = true;
    }
  }
  // end: for each input argument

  // Done!
  return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void quit_handler(int sig) {
  printf("\nTERMINATING AT USER REQUEST\n\n");

  // autopilot interface
  try {
    autopilot_interface_quit->handle_quit(sig);
  } catch (int error) {
  }

  // serial port
  try {
    serial_port_quit->handle_quit(sig);
  } catch (int error) {
  }

  // end program here
  exit(0);
}