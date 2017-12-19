#include "acLiDARldmrs.h"
#include "LiDARldmrs_c_types.h"

#include <sickldmrs/sickldmrs.h>

/* --- Function close --------------------------------------------------- */

/** Codel ldmrs_close of function close.
 *
 * Returns genom_ok.
 */
genom_event
ldmrs_close(LiDARldmrs_ids *ids, const genom_context self)
{
  //int rc;
  //sickldmrs_device *dev = (sickldmrs_device*)ids->LDMRS;
  //sickldmrs_end(dev);
  printf("\n - - - - LDMRS: Bye bye! - - - -\n");
  /* skeleton sample: insert your code */
  /* skeleton sample */ return genom_ok;
}
