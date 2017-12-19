#include "acLiDARldmrs.h"
#include "LiDARldmrs_c_types.h"

#include <err.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sickldmrs/sickldmrs.h>
#include <sickldmrs/sickldmrs.c>
#include <sickldmrs/commands.c>
#include <sickldmrs/print.c>

/* --- Task scan_pub ---------------------------------------------------- */


/* --- Activity open ---------------------------------------------------- */

/** Codel ldmrs_open of activity open.
 *
 * Triggered by LiDARldmrs_start.
 * Yields to LiDARldmrs_config.
 */
genom_event
ldmrs_open(const char ip[64], const char *port, LiDARldmrs_ids *ids,
           const genom_context self)
{
  int rc;
  sickldmrs_device *dev = (sickldmrs_device*)ids->LDMRS;

  dev = sickldmrs_init(ip, port, true);

  if (dev == NULL){
    printf("\n--ERROR: LDMRS INIT Failed--\n" );
    exit(2);
  }
  if ((rc = sickldmrs_get_status(dev, -1)) < 0){
    errx(2, "sickldmrs_get_status: %s\n", strerror(-rc));
  }
  if ((rc = sickldmrs_config_output(dev, 0x00ee, -1)) < 0){
    errx(2, "sickldmrs_config_output: %s\n", strerror(rc));
  }

  ids->devStatus.ip =  ip;
  ids->devStatus.port =  port;
  ids->devStatus.temperature = dev->temperature;
  ids->LDMRS = dev;

  printf("\n -- LDMRS init OK. -- \n" );
  /* skeleton sample: insert your code */
  /* skeleton sample */ return LiDARldmrs_config;
}

/** Codel ldmrs_conf of activity open.
 *
 * Triggered by LiDARldmrs_config.
 * Yields to LiDARldmrs_ether.
 */
genom_event
ldmrs_conf(LiDARldmrs_scan_freqs scan_freq, LiDARldmrs_ids *ids,
           const genom_context self)
{
  int rc;
  sickldmrs_device *dev = (sickldmrs_device*)ids->LDMRS;

  if (dev==NULL){
    printf("\n--LDMRS NULL--\n");
  }

  double Sfreq;
  if(scan_freq == 0){
    Sfreq=3200;             /* scan frequency -> 12.5 Hz */
    printf(" - - scan frequency -> 12.5 Hz - - \n" );
  }else if(scan_freq == 2){
    Sfreq=12800;             /* scan frequency -> 50 Hz */
    printf(" - - scan frequency -> 50 Hz - - \n" );
  }else{
    Sfreq=6400;             /* scan frequency -> 25 Hz */
    printf(" - - scan frequency -> 25 Hz - - \n" );
  }

  if ((rc = sickldmrs_set_parameter(dev,SLDMRS_PARAM_SCAN_FREQUENCY, Sfreq, -1)) < 0){
    printf("\n--Error in setting parameter--\n");
    errx(2, "sickldmrs_set_parameter: %s", strerror(rc));
  }
  if ((rc = sickldmrs_get_parameter(dev,SLDMRS_PARAM_SCAN_FREQUENCY, -1)) < 0){
    printf("\n--Error in getting parameter--\n");
    errx(2, "sickldmrs_get_parameter: %s", strerror(rc));
  }


  /*scan frequency  pp 20 */
  ids->devStatus.scanFreq = (int)dev->scan_frequency/256.0;
  ids->devStatus.temperature = dev->temperature;
  ids->devStatus.start_angle = dev->start_angle;
  ids->devStatus.end_angle = dev->end_angle;
  ids->devStatus.serial = dev->serial_number;
  ids->devStatus.firmware_version = dev->firmware_version;
  ids->devStatus.fpga_version = dev->fpga_version;

  ids->LDMRS = dev;


  printf("\n ------------------------------------- \n");
  printf("\n\t LAAS-CNRS Robotic Interaction Systems \n");
  printf("\t SICK LiDAR LD-MRS  \n");
  printf("\t Harold F MURCIA - September 2017  \n");
  printf("\n ------------------------------------- \n");

  printf("\t ip: %s \t\n",   ids->devStatus.ip);
  printf("\t Port: %s \t\n", ids->devStatus.port);
  printf("\t Firmware: %s\t\n", ids->devStatus.firmware_version);
  printf("\t FPGA Version: %s\t\n", ids->devStatus.fpga_version);
  printf("\t Serial: %s\t\n", ids->devStatus.serial);
  printf("\t Temperature: %02f\t\n", ids->devStatus.temperature);
  printf("\t Scan frequency [Hz]: %d\t\n", ids->devStatus.scanFreq);
  //printf("\t Start Angle: %.3lf\t\n", ids->devStatus.start_angle);
  //printf("\t End Angle: %.3lf\t\n", ids->devStatus.end_angle);

  printf("\n -- SICK LD-MRS CONFIG Ok --\n");
  /* skeleton sample: insert your code */
  /* skeleton sample */ return LiDARldmrs_ether;
}


/* --- Activity quit ---------------------------------------------------- */

/** Codel ldmrs_quit of activity quit.
 *
 * Triggered by LiDARldmrs_start.
 * Yields to LiDARldmrs_ether.
 */
genom_event
ldmrs_quit(LiDARldmrs_ids *ids, const genom_context self)
{
  int rc;
  sickldmrs_device *dev = (sickldmrs_device*)ids->LDMRS;
  sickldmrs_end(dev);
  /* skeleton sample: insert your code */
  /* skeleton sample */ return LiDARldmrs_ether;
}


/* --- Activity startAcquisition ---------------------------------------- */

/** Codel start_scanning of activity startAcquisition.
 *
 * Triggered by LiDARldmrs_start.
 * Yields to LiDARldmrs_scan.
 */
genom_event
start_scanning(LiDARldmrs_ids *ids, const genom_context self)
{
  //printf("\n - - SCANING - -\n");
  int rc;
  sickldmrs_device *dev = (sickldmrs_device*)ids->LDMRS;
  volatile int done = 0;
  static int last_frame = -1;
  /*scan frequency and deltaTime  pp 20 */
  int sleepTime = 1000000/(int)dev->scan_frequency/256.0;
  //printf("time: %d\n", sleepTime);
  usleep(sleepTime);	 /* [12.5, 25 or 50] Hz*/
  sickldmrs_lock(dev);
  if (dev->scan != NULL && dev->scan->scan_number != last_frame) {
    //sickldmrs_print_scan(dev->scan);
    last_frame = dev->scan->scan_number;
  }
  ids->LDMRS = dev;
  /* skeleton sample: insert your code */
  /* skeleton sample */ return LiDARldmrs_scan;
}

/** Codel get_data of activity startAcquisition.
 *
 * Triggered by LiDARldmrs_scan.
 * Yields to LiDARldmrs_stop.
 */
genom_event
get_data(const LiDARldmrs_sickScanner *sickScanner,
         LiDARldmrs_ids *ids, const genom_context self)
{
  int rc;
  sickldmrs_device *dev = (sickldmrs_device*)ids->LDMRS;
  LiDARldmrs_sickldmrsScan * pdata = sickScanner->data(self);

  pdata->scan_number = dev->scan->scan_number;
  //pdata->scanner_status = dev->scan->scanner_status;
  pdata->sync_phase_offset = dev->scan->sync_phase_offset;
  pdata->angle_ticks_per_rotation = dev->scan->angle_ticks_per_rotation;
  pdata->start_angle = dev->scan->start_angle;
  pdata->end_angle = dev->scan->end_angle;
  pdata->scan_points = dev->scan->scan_points;
  pdata->mount_yaw = dev->scan->mount_yaw;
  pdata->mount_pitch = dev->scan->mount_pitch;
  pdata->mount_roll = dev->scan->mount_roll;
  pdata->mount_x = dev->scan->mount_x;
  pdata->mount_y = dev->scan->mount_y;
  pdata->mount_z = dev->scan->mount_z;
  pdata->flags = dev->scan->flags;

  int N = dev->scan->scan_points;
  if(genom_sequence_reserve(&pdata->points,N)<0){
    printf("ERROR: Bad points sequence generation.\n" );
  }

  int i;
  float phi=0, theta=0;
  for(i=0;i<N;i++){
	   pdata->points._buffer[i].layer = dev->scan->points[i].layer;
     pdata->points._buffer[i].echo = dev->scan->points[i].echo;
     pdata->points._buffer[i].flags = dev->scan->points[i].flags;
     pdata->points._buffer[i].horizontal_angle = dev->scan->points[i].horizontal_angle;
     pdata->points._buffer[i].radial_distance = dev->scan->points[i].radial_distance;
     pdata->points._buffer[i].pulse_width = dev->scan->points[i].pulse_width;
  }
  pdata->points._length = N;
  sickScanner->write(self);
  /* skeleton sample: insert your code */
  /* skeleton sample */ return LiDARldmrs_stop;
}

/** Codel stop_scanning of activity startAcquisition.
 *
 * Triggered by LiDARldmrs_stop.
 * Yields to LiDARldmrs_ether.
 */
genom_event
stop_scanning(LiDARldmrs_ids *ids, const genom_context self)
{
  //printf("\n - - STOPPING - -\n");
  int rc;
  sickldmrs_device *dev = (sickldmrs_device*)ids->LDMRS;
  sickldmrs_unlock(dev);
  /* skeleton sample: insert your code */
  /* skeleton sample */ return LiDARldmrs_ether;
}
