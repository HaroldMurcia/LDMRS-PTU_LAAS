#ifndef _TIME_UTILS_H
#define _TIME_UTILS_H

#include <time.h>
#include <sys/time.h>

#define MSEC_PER_SEC    1000L
#define USEC_PER_MSEC   1000L
#define NSEC_PER_USEC   1000L
#define NSEC_PER_MSEC   1000000L
#define USEC_PER_SEC    1000000L
#define NSEC_PER_SEC    1000000000L
#define FSEC_PER_SEC    1000000000000000LL

typedef long long               s64;

static inline void set_normalized_timespec(struct timespec *ts, time_t sec, s64 nsec)
{
  while (nsec >= NSEC_PER_SEC) {
    /*
     * The following asm() prevents the compiler from
     * optimising this loop into a modulo operation. See
     * also __iter_div_u64_rem() in include/linux/time.h
     */
    asm("" : "+rm"(nsec));
    nsec -= NSEC_PER_SEC;
    ++sec;
  }
  while (nsec < 0) {
    asm("" : "+rm"(nsec));
    nsec += NSEC_PER_SEC;
    --sec;
  }
  ts->tv_sec = sec;
  ts->tv_nsec = nsec;
}


inline struct timespec timespec_add(struct timespec lhs,
					   struct timespec rhs)
{
  struct timespec ts_delta;
  set_normalized_timespec(&ts_delta, lhs.tv_sec + rhs.tv_sec,
			  lhs.tv_nsec + rhs.tv_nsec);
  return ts_delta;
}

/*
 * sub = lhs - rhs, in normalized form
 */
inline struct timespec timespec_sub(struct timespec lhs,
					   struct timespec rhs)
{
  struct timespec ts_delta;
  set_normalized_timespec(&ts_delta, lhs.tv_sec - rhs.tv_sec,
			  lhs.tv_nsec - rhs.tv_nsec);
  return ts_delta;
}
  

inline struct timespec timespec_mean(struct timespec start,
					    struct timespec end)
{
  struct timespec ts_mean;
  ts_mean = timespec_sub(end,start);
  set_normalized_timespec(&ts_mean,start.tv_sec+ts_mean.tv_sec/2,start.tv_nsec+(ts_mean.tv_sec%2)*NSEC_PER_SEC/2+ts_mean.tv_nsec);
  return ts_mean;
}

inline void get_time_real(struct timespec *t)
{
  clock_gettime(CLOCK_REALTIME, t);
}

inline void get_time_monotonic(struct timespec *t)
{
  clock_gettime(CLOCK_MONOTONIC, t);
}

inline double get_ms_diff(struct timespec t1,struct timespec t2)
{
  struct timespec diff = timespec_sub(t2,t1);
  return ((double)diff.tv_sec)*1e3 + ((double)diff.tv_nsec)*1e-6;
}

#endif //_TIME_UTILS_H
