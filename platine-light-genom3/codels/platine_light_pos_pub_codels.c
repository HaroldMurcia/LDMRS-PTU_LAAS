#include "acplatine_light.h"

#include "platine_light_c_types.h"
#include "platine_driver.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>


static genom_event ptu_error(const char *error, genom_context self);

static FILE *sfile = NULL;

/* --- Task pos_pub ----------------------------------------------------- */


/* --- Activity monitor_position ---------------------------------------- */

/** Codel platine_start_monitor_position of activity monitor_position.
 *
 * Triggered by platine_light_start.
 * Yields to platine_light_get.
 * Throws platine_light_e_file.
 */
genom_event
platine_start_monitor_position(platine_ptu_axis axis,
                               platine_ptu_unit unit,
                               platine_light_ids *ids,
                               const platine_light_ptu_position *ptu_position,
                               genom_context self)
{
  ids->monitor = true;
  platine_timed_pos *pos = ptu_position->data(self);
  pos->id=0;
  pos->axis = axis;
  pos->unit = unit;
  ptu_position->write(self);

  if(ids->save_type == platine_ENABLED){
    sfile = fopen(ids->save_file,"a");
    if(sfile == NULL)
      perror("Unable to open save file : ");
  }

  /* skeleton sample */ return platine_light_get;
}

/** Codel platine_get_position of activity monitor_position.
 *
 * Triggered by platine_light_get.
 * Yields to platine_light_get, platine_light_ether.
 * Throws platine_light_e_file.
 */
genom_event
platine_get_position(platine_ptu_axis axis, platine_ptu_unit unit,
                     platine_light_ids *ids,
                     const platine_light_ptu_position *ptu_position,
                     genom_context self)
{
  platine *p = (platine *)ids->ptu;
  if(p==NULL){
    return ptu_error("PTU not initialised", self);
  }

  const char *cmd;
  if(axis==platine_PAN){
    cmd = CMD_ABS_PAN_POS;
  }else{//platine_TILT
    cmd = CMD_ABS_TILT_POS;
  }
 
  double value;
  long lvalue;
  struct timespec time_stamp;

  if(platineCmdGetLongT(p,cmd,&lvalue,&time_stamp)<0){
    return ptu_error("Unable to send ptu set command", self);
  }
  
  if(unit==platine_STEP){
    value = (double)lvalue;
  }else if(unit==platine_DEG){
    value = PT_STEP_TO_DEG(p,lvalue);
  }else{//platine_RAD
    value = PT_STEP_TO_RAD(p,lvalue);
  }

  platine_timed_pos *pos = ptu_position->data(self);
  pos->id++;
  pos->time_stamp.sec = time_stamp.tv_sec;
  pos->time_stamp.nsec = time_stamp.tv_nsec;
  pos->value = value;

  ptu_position->write(self);

  if(sfile != NULL){
    fprintf(sfile,"%lf, %lf\n",
	    ((double)time_stamp.tv_sec)*1e3+((double)time_stamp.tv_nsec)*1e-6,
	    value);
}

  if(!ids->monitor){
    if(sfile != NULL) fclose(sfile);
    return platine_light_ether;
  }
  /* skeleton sample */ return platine_light_get;
}


static genom_event ptu_error(const char *error, genom_context self){
  platine_light_e_ptu_detail d;
  strcpy(d.what,error);
  return platine_light_e_ptu(&d, self);
}
