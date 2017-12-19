#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

#include <unistd.h>
#include <stdlib.h>
#include <strings.h>
#include <string.h>
#include <errno.h>
#include <error.h>

#include "platine_driver.h"
#include "time_utils.h"
/* baudrate settings are defined in <asm/termbits.h>, which is
   included by <termios.h> */




 int ptuCloseRS232(int fd){
  int ret = close(fd);
  if(ret<0){
    perror("ptuCloseRS232 : ");
  }
  return ret;
}

int ptuWriteAndRead(int fd,const char *message,size_t size,char * response,struct timespec *wtime_stamp,struct timespec *rtime_stamp){
  char *buf = response;
  
  /* Reads message */
  /*      printf ("count=0\n"); */
  int count = 0;
  char c = '\0';
  
  if(tcflush(fd,TCIOFLUSH)<0){
    perror("ptuWriteAndRead error flushing IO on serial port :");
    return -1;
  }


  struct timespec tstart,tstop;
  if(wtime_stamp==NULL)
    wtime_stamp=&tstart;
  if(rtime_stamp==NULL)
    rtime_stamp=&tstop;
  
  get_time_real(wtime_stamp);
  int n = write(fd,message,size);
  

  if(n<0){
    perror("ptuDriver write error :");
    return n;
  }

  int error = -1;

  do {
    /*       printf ("read\n"); */
    int status = read(fd, &buf[count], 1);
    if (status == 1) {
      c  = buf[count];
      count++;
      /* if (isprint(c)) */
      /*   printf ("c%d=%c\n", count, c); */
      /* else */
      /*   printf ("c%d=%02x\n", count, c); */
      if(c=='*' || c=='!'){
	get_time_real(rtime_stamp);
	if(c == '*') error=0;
      }
      if(count==2 && (c=='T' || c=='P')){
	fprintf(stderr,"WARNING : Pan or Tilt synchro error (%c%c) ! \n",'!',c);
      }

    }else {
	if(errno != 0)
		fprintf(stderr, "ptuDriver read(%d) error : %d %s\n", 
			fd,
			status,
			strerror(errno));
	else
		fprintf(stderr,"ptuDriver read(%d) error : no bytes read\n",
			fd);
      return -1;
    }
  } while (c != '\n'/* && c != '\r'*/ && count<255);
    
  if (count == 255) {
    fprintf (stderr, "ptuDriver read error : message too long\n");
    return -1;
  }
  if(count < 2){
    fprintf (stderr, "ptuDriver read error : message too short\n");
    return -1;
  }
    
    
    
  buf[count]='\0';             /* set end of string, so we can printf */
  // printf("Response:[%s] (count=%d) (%f ms)\n", buf,count,get_ms_diff(wtime_stamp,rtime_stamp));
  if(error>=0)
    return count-1;
  else
   fprintf(stderr,"ptuDriver ptu error : %s",buf); 
   return error;
}


 int ptuDecodeDouble(const char *response,double *res){
  int i=0;
  char c=response[i];
  while(c!='\0' && !(isdigit(c) || c=='-' || c=='+')){
    c=response[++i];
  }
  if(c!='\0')
    return sscanf(response+i,"%lf",res);
  else
    return -1;
}

 int ptuDecodeLong(const char *response,long *res){
  int i=0;
  char c=response[i];
  while(c!='\0' && !(isdigit(c) || c=='-' || c=='+')){
    c=response[++i];
  }
  if(c!='\0')
    return sscanf(response+i,"%ld",res);
  else
    return -1;
}

 int ptuEncode(char * buff,size_t size,const char *cmd,long * param){
  if(param == NULL){
    return snprintf(buff,size,"%s ",cmd);
  }else{
    return snprintf(buff,size,"%s%ld ",cmd,*param);
  }
}


 int ptuChangeBaudrate(int fd,int baudrate){
  char cmd[CMD_MAX_LENGTH];
  char buf[CMD_MAX_LENGTH];
  snprintf(cmd,CMD_MAX_LENGTH,CMD_SET_BAUDRATE_TEMPLATE,baudrate);
  int n=ptuEncode(buf,CMD_MAX_LENGTH,cmd,NULL);
  return ptuWriteAndRead(fd,buf,n,cmd,NULL,NULL);
}


static struct speed {
    int speed;
    int termioSpeed;
} speeds[] = {
    { 0, B0 },
    { 50, B50 },
    { 75, B75 },
    { 110, B110 },
    { 300, B300 },
    { 600, B600 },
    { 1200, B1200 },
    { 2400, B2400 },
    { 4800, B4800 },
    { 9600, B9600 },
    { 19200, B19200 },
    { 38400, B38400 },
    { 57600, B57600 }};

#define SPEED_SIZE (sizeof(speeds)/sizeof(struct speed))

static int
get_termios_speed(int speed) 
{
    int i;

    for (i = 0; i < SPEED_SIZE; i++) {
	if (speed == speeds[i].speed) {
	  return speeds[i].termioSpeed;
	}
    } /* for */
    return -1;
}


 int 
ptuInitRS232(const char *device, int baudRate)
{
  int serialFd;
  struct termios tio;
  int speed;
  serialFd = open(device, O_RDWR|O_NONBLOCK, 0);

  if (serialFd == -1) {
    fprintf(stderr, "ptInitRS232 error open %s: ", device);
    perror(":");
    return -1;
  }
  if (tcgetattr(serialFd, &tio) < 0) {
    fprintf(stderr, "ptInitRS232 while setting options ");
    perror("tcgetattr");
    return -1;
  }
    /* restore old values to unhang the bastard, if hung */
  tio.c_iflag=1280;   /* IXON|ICRNL */
  tio.c_oflag=5;      /* ONLCR|OPOST */
  tio.c_cflag=3261;   /* CLOCAL|HUPCL|CREAD|CS8|B9600 */
  tio.c_lflag=35387;  /* IEXTEN|ECHOKE|ECHOCTL|ECHOK|ECHOE|ECHO|ISIG|ICANON */
  if (tcsetattr(serialFd, TCSANOW, &tio) < 0) {
    fprintf(stderr, "ptInitRS232 while setting options ");
    perror("tcgetattr");
    return -1;
  }
  close(serialFd);

  serialFd = open(device, O_RDWR);

  if (serialFd == -1) {
    fprintf(stderr, "ptInitRS232 error open %s: ", device);
    perror(":");
    return -1;
  }
  if (tcgetattr(serialFd, &tio) < 0) {
    fprintf(stderr, "ptInitRS232 while setting options ");
    perror("tcgetattr");
    return -1;
  }
  
  tio.c_cc[VTIME]    = 10;   /* timeout 1 sec. */
  tio.c_cc[VMIN]     = 0;   /* blocking read until 0 chars received */
  
  /* set flags -- don't fuck with these values */
  tio.c_iflag = IGNBRK | IGNPAR;
  tio.c_iflag &= ~(INLCR | ICRNL | IUCLC | ISTRIP | IXON | BRKINT);
  tio.c_oflag &= ~OPOST;
  tio.c_lflag &= ~(ICANON | ISIG | ECHO);
  tio.c_cflag = B9600 | CS8 | CREAD | CLOCAL;

  speed = get_termios_speed(baudRate);
  if (speed == -1) {
    fprintf(stderr, "ptInitRS232 error set baud rate %d\n", baudRate);
    return -1;
  }
	
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);

  if (tcsetattr(serialFd, TCSANOW, &tio) < 0) {
    fprintf(stderr, "ptInitRS232 error while setting options ");
    perror("tcsetattr");
    return -1;
  }

  if(tcflush(serialFd,TCIOFLUSH)<0){
   perror("ptInitRS232 error flushing IO on serial port :");
   return -1; 
}

  printf ("\nptInitRS232: Connection avec la ligne RS232"
	  " etablie en %s, Fd=%d\n",
	  device, serialFd);
  
  return (serialFd);
}


int platineInit(platine *p,const char *device,int baudrate){
  int error;

  p->fd = ptuInitRS232(device,baudrate);
  if(p->fd<0)
    return p->fd;

  //dummy version command to clean PTU buffer
  platineCmd(p,CMD_VERSION);
  
  if(platineCmd(p,CMD_FEEDBACK_TERSE)<0)
    return -1;

  if(platineCmd(p,CMD_ECHO_DISABLE)<0)
    return -1;

  return platineUpdateState(p);
}

int platineClose(platine *p){
  int error;
  error = ptuCloseRS232(p->fd);
  if(error<0)
    return error;

  p->fd = -1;
  return 0;
}

int platineUpdateState(platine *p){
  int error = 0;
  
  platine_state *state = &p->state;

  error = platineCmdGetDouble(p,CMD_TILT_RES,&state->stilt.res);
  if(error<0) return error;
  error = platineCmdGetDouble(p,CMD_PAN_RES,&state->span.res);
  if(error<0) return error;
  
  error = platineCmdGetLong(p,CMD_BASE_TILT_SPEED,&state->stilt.base_speed);
  if(error<0) return error;  
  error = platineCmdGetLong(p,CMD_BASE_PAN_SPEED,&state->span.base_speed);
  if(error<0) return error;

  error = platineCmdGetLong(p,CMD_ABS_TILT_SPEED,&state->stilt.speed);
  if(error<0) return error;
  error = platineCmdGetLong(p,CMD_ABS_PAN_SPEED,&state->span.speed);
  if(error<0) return error;

  error = platineCmdGetLong(p,CMD_ABS_TILT_ACC,&state->stilt.acc);
  if(error<0) return error;
  error = platineCmdGetLong(p,CMD_ABS_PAN_ACC,&state->span.acc);
  if(error<0) return error;
  
  error = platineCmdGetLong(p,CMD_ABS_TILT_POS,&state->stilt.pos);
  if(error<0) return error;
  error = platineCmdGetLong(p,CMD_ABS_PAN_POS,&state->span.pos);
  if(error<0) return error;

  error = platineCmdGetLong(p,CMD_MIN_TILT_POS,&state->stilt.min_pos);
  if(error<0) return error;
  error = platineCmdGetLong(p,CMD_MIN_PAN_POS,&state->span.min_pos);
  if(error<0) return error;
  
  error = platineCmdGetLong(p,CMD_MAX_TILT_POS,&state->stilt.max_pos);
 if(error<0) return error;
  error = platineCmdGetLong(p,CMD_MAX_PAN_POS,&state->span.max_pos);
  if(error<0) return error;

  PT_DEG_RAD_STEP_INIT(p)

  return error;
}

int platineCmdGetLong(platine *p,const char *cmd,long *value){
  char buf[CMD_MAX_LENGTH],response[CMD_MAX_LENGTH];
  int n=ptuEncode(buf,CMD_MAX_LENGTH,cmd,NULL);
  int error = ptuWriteAndRead(p->fd,buf,n,response,NULL,NULL);
  if(error<0)
    return error;
  
  return ptuDecodeLong(response,value);

}
int platineCmdGetDouble(platine *p,const char* cmd,double *value){
  char buf[CMD_MAX_LENGTH],response[CMD_MAX_LENGTH];
  int n=ptuEncode(buf,CMD_MAX_LENGTH,cmd,NULL);
  int error = ptuWriteAndRead(p->fd,buf,n,response,NULL,NULL);
  if(error<0)
    return error;
  
  return ptuDecodeDouble(response,value);

}
int platineCmdSet(platine *p,const char* cmd,long value){
  char buf[CMD_MAX_LENGTH],response[CMD_MAX_LENGTH];
  int n=ptuEncode(buf,CMD_MAX_LENGTH,cmd,&value);
  return ptuWriteAndRead(p->fd,buf,n,response,NULL,NULL);
 }

int platineCmd(platine *p,const char*cmd){
  char buf[CMD_MAX_LENGTH],response[CMD_MAX_LENGTH];
  int n=ptuEncode(buf,CMD_MAX_LENGTH,cmd,NULL);
  return ptuWriteAndRead(p->fd,buf,n,response,NULL,NULL);
}


int platineCmdGetLongT(platine *p,const char *cmd,long *value,struct timespec *time_stamp){
  
  struct timespec tw,tr;
  char buf[CMD_MAX_LENGTH],response[CMD_MAX_LENGTH];
  int n=ptuEncode(buf,CMD_MAX_LENGTH,cmd,NULL);
  int error = ptuWriteAndRead(p->fd,buf,n,response,&tw,&tr);
  if(error<0)
    return error;

  //  *time_stamp = timespec_mean(tw,tr);
  *time_stamp = tr;

  return ptuDecodeLong(response,value);
}

int platineCmdSetT(platine *p,const char *cmd,long value,struct timespec *time_stamp){
  struct timespec tw,tr;
  char buf[CMD_MAX_LENGTH],response[CMD_MAX_LENGTH];
  int n=ptuEncode(buf,CMD_MAX_LENGTH,cmd,&value);
  int error =  ptuWriteAndRead(p->fd,buf,n,response,&tw,&tr); 

  //*time_stamp = timespec_mean(tw,tr);
  *time_stamp = tr;

  return error;

}
