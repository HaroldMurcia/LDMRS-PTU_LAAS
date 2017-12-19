#ifndef _PTU_DRIVER_H
#define _PTU_DRIVER_H
#include <time.h>
#include <math.h>

#define CMD_MAX_LENGTH 255


#define CMD_TILT_RES "tr"
#define CMD_PAN_RES "pr"

#define CMD_MIN_PAN_POS "pn"
#define CMD_MAX_PAN_POS "px"

#define CMD_MIN_TILT_POS "tn"
#define CMD_MAX_TILT_POS "tx"

#define CMD_ABS_TILT_POS "tp"
#define CMD_ABS_PAN_POS "pp"

#define CMD_ABS_TILT_SPEED "ts"
#define CMD_ABS_PAN_SPEED "ps"

#define CMD_ABS_TILT_ACC "ta"
#define CMD_ABS_PAN_ACC "pa"

#define CMD_BASE_TILT_SPEED "tb"
#define CMD_BASE_PAN_SPEED "pb"

#define CMD_FEEDBACK_VERBOSE "fv"
#define CMD_FEEDBACK_TERSE "ft"
#define CMD_FEEDBACK "f"

#define CMD_ECHO_DISABLE "ed"
#define CMD_ECHO_ENABLE "ee"
#define CMD_ECHO "e"

#define CMD_VERSION "v"

#define CMD_SET_BAUDRATE_TEMPLATE "@(%d,0,F)"

#define BAUDRATE_DEFAULT 9600
#define BAUDRATE_HIGHEST 38400

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif //M_PI

#define PT_DEG_TO_STEP(ps,deg) ((long)floor(deg / ps->state.deg_in_step))
#define PT_RAD_TO_STEP(ps,rad) ((long)floor(rad / ps->state.rad_in_step))
#define PT_STEP_TO_DEG(ps,step) (((double)step) * ps->state.deg_in_step)
#define PT_STEP_TO_RAD(ps,step) (((double)step) * ps->state.rad_in_step)

#define PT_DEG_RAD_STEP_INIT(ps) {ps->state.deg_in_step=ps->state.stilt.res/3600.0;ps->state.rad_in_step=M_PI*ps->state.deg_in_step/180.0;}


typedef struct platine_pt_state{
  double res;
  long min_pos;
  long max_pos;
  long speed;
  long base_speed;
  long acc;
  long pos;
}platine_pt_state;

typedef struct platine_state{
  platine_pt_state stilt;
  platine_pt_state span;
  double deg_in_step;
  double rad_in_step;
}platine_state;

typedef struct platine{
  int fd;
  platine_state state;
}platine;



void get_time_real(struct timespec *t);
long get_us_diff(struct timespec *t1,struct timespec *t2);

int ptuInitRS232(const char *device, int baudRate);

int ptuCloseRS232(int fd);
int ptuWriteAndRead(int fd,const char *message,size_t size,char * response,struct timespec *wtime_stamp,struct timespec *rtime_stamp);
int ptuDecodeDouble(const char *response,double *res);
int ptuDecodeLong(const char *response,long *res);

int ptuEncode(char * buff,size_t size,const char *cmd,long* param);

int ptuChangeBaudrate(int fd,int baudRate);

int platineInit(platine *p,const char *device,int baudrate);
int platineClose(platine *p);

int platineUpdateState(platine *p);

int platineCmdGetLongT(platine *p,const char *cmd,long *value,struct timespec *time_stamp);

int platineCmdSetT(platine *p,const char *cmd,long value,struct timespec *time_stamp);

int platineCmdGetLong(platine *p,const char *cmd,long *value);
int platineCmdGetDouble(platine *p,const char* cmd,double *value);
int platineCmdSet(platine *p,const char* cmd,long value);
int platineCmd(platine *p,const char*cmd);

#endif //_PTU_DRIVER_H
