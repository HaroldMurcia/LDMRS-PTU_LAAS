#include "acplatine_light.h"

#include "platine_light_c_types.h"
#include "platine_driver.h"

#include <stdio.h>
#include <stdlib.h>

/* --- Attribute set_savefile ------------------------------------------- */

/** Validation codel set_logfile of attribute set_savefile.
 *
 * Returns genom_ok.
 * Throws platine_light_e_file.
 */
genom_event
set_logfile(const char save_file[256], genom_context self)
{
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}


/* --- Function close --------------------------------------------------- */

/** Codel platine_close of function close.
 *
 * Returns genom_ok.
 * Throws platine_light_e_ptu.
 */
genom_event
platine_close(platine_light_ids *ids, genom_context self)
{
  platine *p = (platine *)ids->ptu;   
  platine_light_e_ptu_detail d;

  if(p==NULL){
    strcpy(d.what,"PTU driver not initialised");
    return platine_light_e_ptu(&d, self);
  }
  
  if(platineClose(p)<0){
    strcpy(d.what,"unable to close ptu file descriptor");
    return platine_light_e_ptu(&d, self);
  }
  
  ids->ptu=NULL;
  free(p);
  
  /* skeleton sample */ return genom_ok;
}


/* --- Function stop_monitor_position ----------------------------------- */

/** Codel platine_stop_monitor_position of function stop_monitor_position.
 *
 * Returns genom_ok.
 * Throws platine_light_e_ptu.
 */
genom_event
platine_stop_monitor_position(bool *monitor, genom_context self)
{
  *monitor = false;
  /* skeleton sample */ return genom_ok;
}
