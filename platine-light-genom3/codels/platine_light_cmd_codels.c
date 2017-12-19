#include "acplatine_light.h"

#include "platine_light_c_types.h"

#include <stdlib.h>
#include <string.h>

#include "platine_driver.h"


static genom_event ptu_error(const char *error, genom_context self);

/* --- Task cmd --------------------------------------------------------- */


/* --- Activity open ---------------------------------------------------- */

/** Codel platine_open of activity open.
 *
 * Triggered by platine_light_start.
 * Yields to platine_light_update.
 * Throws platine_light_e_ptu.
 */
genom_event
platine_open(const char device[64], platine_baudrate_profile baudrate,
             platine_light_ids *ids, genom_context self)
{  

  platine *p = (platine *)ids->ptu;
  int br;

  if(p==NULL){
    ids->ptu = malloc(sizeof(platine));
    p = (platine *)ids->ptu;
  }

  
  if(baudrate==platine_BAUDRATE_DEFAULT){
    br = BAUDRATE_DEFAULT;
  }else{//platine_BAUDRATE_HIGHEST
    br = BAUDRATE_HIGHEST;
  }

  if(platineInit(p,device,br)<=0){
    printf("ERROR: Platine INIT failed\n");
    return ptu_error("unable to init ptu unit", self);
  }
  


  /* skeleton sample */ return platine_light_update;
}

/** Codel platine_update_state_poster of activity open.
 *
 * Triggered by platine_light_update.
 * Yields to platine_light_ether.
 * Throws platine_light_e_ptu.
 */
genom_event
platine_update_state_poster(platine_light_ids *ids,
                            const platine_light_ptu_state *ptu_state, 
			    genom_context self)
{  
  platine *p = (platine *)ids->ptu;

  if(p==NULL){
    return ptu_error("PTU driver not initialised", self);
  }

  platine_ptu_state *state = ptu_state->data(self);
  state->unit = platine_STEP;
  state->unit_res = platine_RAD;

  state->tilt_state.speed = p->state.stilt.speed;
  state->tilt_state.base_speed = p->state.stilt.base_speed;
  state->tilt_state.acceleration = p->state.stilt.acc;
  state->tilt_state.position = p->state.stilt.pos;
  state->tilt_state.min_position = p->state.stilt.min_pos;
  state->tilt_state.max_position = p->state.stilt.max_pos;
  state->tilt_state.resolution = p->state.rad_in_step;

  state->pan_state.speed = p->state.span.speed;
  state->pan_state.base_speed = p->state.span.base_speed;
  state->pan_state.acceleration = p->state.span.acc;
  state->pan_state.position = p->state.span.pos;  
  state->pan_state.min_position = p->state.span.min_pos;  
  state->pan_state.max_position = p->state.span.max_pos;  
  state->pan_state.resolution = p->state.rad_in_step;

  ptu_state->write(self);

  /* skeleton sample: insert your code */
  /* skeleton sample */ return platine_light_ether;
}


/* --- Activity set_baudrate -------------------------------------------- */

/** Codel platine_set_baudrate of activity set_baudrate.
 *
 * Triggered by platine_light_start.
 * Yields to platine_light_ether.
 * Throws platine_light_e_ptu.
 */
genom_event
platine_set_baudrate(const char device[64],
                     platine_baudrate_profile current_baudrate,
                     platine_baudrate_profile new_baudrate,
                     platine_light_ids *ids, genom_context self)
{
  
  platine p;
  int br;
  platine_light_e_ptu_detail d;

  if(ids->ptu!=NULL){
    return ptu_error("ptu unit connection already open.", self);
  }
  
  if(current_baudrate==platine_BAUDRATE_DEFAULT){
    br = BAUDRATE_DEFAULT;
  }else{//platine_BAUDRATE_HIGHEST
    br = BAUDRATE_HIGHEST;
  }

  if(platineInit(&p,device,br)<=0){
    return ptu_error("unable to init ptu unit.", self);
  }

  if(new_baudrate==platine_BAUDRATE_DEFAULT){
    br = BAUDRATE_DEFAULT;
  }else{//platine_BAUDRATE_HIGHEST
    br = BAUDRATE_HIGHEST;
  }
 
  if(ptuChangeBaudrate(p.fd,br)<0){
    return ptu_error("unable to change ptu baudrate.", self);
  }

  if(platineClose(&p)<0){
    return ptu_error("unable to close ptu.", self);
  }
  
  /* skeleton sample: insert your code */
  /* skeleton sample */ return platine_light_ether;
}


/* --- Activity goto_position ------------------------------------------- */

/** Codel platine_goto_position of activity goto_position.
 *
 * Triggered by platine_light_start.
 * Yields to platine_light_ether.
 * Throws platine_light_e_ptu.
 */
genom_event
platine_goto_position(platine_ptu_axis axis, double value,
                      platine_ptu_unit unit, platine_light_ids *ids,
                      genom_context self)
{
  platine *p = (platine *)ids->ptu;

  if(p==NULL){
    return ptu_error("PTU driver not initialised", self);
  }
  
  const char *cmd;
  if(axis==platine_PAN){
    cmd = CMD_ABS_PAN_POS;
  }else{//platine_TILT
    cmd = CMD_ABS_TILT_POS;
  }
  
  long svalue;
  if(unit==platine_STEP){
    svalue = value;
  }else if(unit==platine_DEG){
    svalue = PT_DEG_TO_STEP(p,value);
  }else{//platine_RAD
    svalue = PT_RAD_TO_STEP(p,value);
  }
  
  if(platineCmdSet(p,cmd,svalue)<0){
    return ptu_error("Unable to send ptu set command", self);
  }
  /* skeleton sample */ return platine_light_ether;
}


/* --- Activity set_speed_profile --------------------------------------- */

/** Codel platine_set_speed_profile of activity set_speed_profile.
 *
 * Triggered by platine_light_start.
 * Yields to platine_light_update.
 * Throws platine_light_e_ptu.
 */
genom_event
platine_set_speed_profile(platine_ptu_axis axis, double speed,
                          double base_speed, double acceleration,
                          platine_ptu_unit unit,
                          platine_light_ids *ids, genom_context self)
{
  platine *p = (platine *)ids->ptu;
  int error = 0;

  if(p==NULL){
    return ptu_error("PTU not initialised", self);
  }
  

  
  long sspeed,sbase_speed,sacceleration;
  if(unit==platine_STEP){
    sspeed = speed;
    sbase_speed = base_speed;
    sacceleration = acceleration;
  }else if(unit==platine_DEG){
    sspeed = PT_DEG_TO_STEP(p,speed);
    sbase_speed = PT_DEG_TO_STEP(p,base_speed);
    sacceleration = PT_DEG_TO_STEP(p,acceleration);
  }else{//platine_RAD
    sspeed = PT_RAD_TO_STEP(p,speed);
    sbase_speed = PT_RAD_TO_STEP(p,base_speed);
    sacceleration = PT_RAD_TO_STEP(p,acceleration);
  }
  
  if(axis==platine_PAN){
    error = platineCmdSet(p,CMD_ABS_PAN_SPEED,sspeed);
    if(error<0) return ptu_error("Unable to send ptu set command", self);
    error = platineCmdSet(p,CMD_BASE_PAN_SPEED,sbase_speed);
    if(error<0) return ptu_error("Unable to send ptu set command", self);
    error = platineCmdSet(p,CMD_ABS_PAN_ACC,sacceleration);
    if(error<0) return ptu_error("Unable to send ptu set command", self);
  }else{//platine_TILT
    error = platineCmdSet(p,CMD_ABS_TILT_SPEED,sspeed);
    if(error<0) return ptu_error("Unable to send ptu set command", self);
    error = platineCmdSet(p,CMD_BASE_TILT_SPEED,sbase_speed);
    if(error<0) return ptu_error("Unable to send ptu set command", self);
    error = platineCmdSet(p,CMD_ABS_TILT_ACC,sacceleration);
    if(error<0) return ptu_error("Unable to send ptu set command", self);
  }

  if(platineUpdateState(p)<0){
    return ptu_error("Unable to update ptu state.", self);
  }

  /* skeleton sample */ return platine_light_update;
}

/** Codel platine_update_state_poster of activity set_speed_profile.
 *
 * Triggered by platine_light_update.
 * Yields to platine_light_ether.
 * Throws platine_light_e_ptu.
 */
/* already defined in service open */





static genom_event ptu_error(const char *error, genom_context self){
  platine_light_e_ptu_detail d;
  strcpy(d.what,error);
  return platine_light_e_ptu(&d, self);
}
