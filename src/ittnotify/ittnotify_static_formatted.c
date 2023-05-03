
typedef long int ptrdiff_t;
typedef long unsigned int size_t;
typedef int wchar_t;
typedef struct {
  long long __max_align_ll __attribute__((__aligned__(__alignof__(long long))));
  long double __max_align_ld
      __attribute__((__aligned__(__alignof__(long double))));
} max_align_t;


typedef unsigned char __u_char;
typedef unsigned short int __u_short;
typedef unsigned int __u_int;
typedef unsigned long int __u_long;

typedef signed char __int8_t;
typedef unsigned char __uint8_t;
typedef signed short int __int16_t;
typedef unsigned short int __uint16_t;
typedef signed int __int32_t;
typedef unsigned int __uint32_t;

typedef signed long int __int64_t;
typedef unsigned long int __uint64_t;

typedef __int8_t __int_least8_t;
typedef __uint8_t __uint_least8_t;
typedef __int16_t __int_least16_t;
typedef __uint16_t __uint_least16_t;
typedef __int32_t __int_least32_t;
typedef __uint32_t __uint_least32_t;
typedef __int64_t __int_least64_t;
typedef __uint64_t __uint_least64_t;

typedef long int __quad_t;
typedef unsigned long int __u_quad_t;

typedef long int __intmax_t;
typedef unsigned long int __uintmax_t;

typedef unsigned long int __dev_t;
typedef unsigned int __uid_t;
typedef unsigned int __gid_t;
typedef unsigned long int __ino_t;
typedef unsigned long int __ino64_t;
typedef unsigned int __mode_t;
typedef unsigned long int __nlink_t;
typedef long int __off_t;
typedef long int __off64_t;
typedef int __pid_t;
typedef struct {
  int __val[2];
} __fsid_t;
typedef long int __clock_t;
typedef unsigned long int __rlim_t;
typedef unsigned long int __rlim64_t;
typedef unsigned int __id_t;
typedef long int __time_t;
typedef unsigned int __useconds_t;
typedef long int __suseconds_t;
typedef long int __suseconds64_t;

typedef int __daddr_t;
typedef int __key_t;

typedef int __clockid_t;

typedef void *__timer_t;

typedef long int __blksize_t;

typedef long int __blkcnt_t;
typedef long int __blkcnt64_t;

typedef unsigned long int __fsblkcnt_t;
typedef unsigned long int __fsblkcnt64_t;

typedef unsigned long int __fsfilcnt_t;
typedef unsigned long int __fsfilcnt64_t;

typedef long int __fsword_t;

typedef long int __ssize_t;

typedef long int __syscall_slong_t;

typedef unsigned long int __syscall_ulong_t;

typedef __off64_t __loff_t;
typedef char *__caddr_t;

typedef long int __intptr_t;

typedef unsigned int __socklen_t;

typedef int __sig_atomic_t;

typedef __int8_t int8_t;
typedef __int16_t int16_t;
typedef __int32_t int32_t;
typedef __int64_t int64_t;

typedef __uint8_t uint8_t;
typedef __uint16_t uint16_t;
typedef __uint32_t uint32_t;
typedef __uint64_t uint64_t;

typedef __int_least8_t int_least8_t;
typedef __int_least16_t int_least16_t;
typedef __int_least32_t int_least32_t;
typedef __int_least64_t int_least64_t;

typedef __uint_least8_t uint_least8_t;
typedef __uint_least16_t uint_least16_t;
typedef __uint_least32_t uint_least32_t;
typedef __uint_least64_t uint_least64_t;

typedef signed char int_fast8_t;

typedef long int int_fast16_t;
typedef long int int_fast32_t;
typedef long int int_fast64_t;
typedef unsigned char uint_fast8_t;

typedef unsigned long int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
typedef unsigned long int uint_fast64_t;
typedef long int intptr_t;

typedef unsigned long int uintptr_t;
typedef __intmax_t intmax_t;
typedef __uintmax_t uintmax_t;


extern void *dlopen(const char *__file, int __mode)
    __attribute__((__nothrow__));

extern int dlclose(void *__handle) __attribute__((__nothrow__))
__attribute__((__nonnull__(1)));

extern void *dlsym(void *__restrict __handle, const char *__restrict __name)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));
extern char *dlerror(void) __attribute__((__nothrow__, __leaf__));


typedef __time_t time_t;


struct timespec {

  __time_t tv_sec;

  __syscall_slong_t tv_nsec;
};

typedef __pid_t pid_t;

struct sched_param {
  int sched_priority;
};


typedef unsigned long int __cpu_mask;

typedef struct {
  __cpu_mask __bits[1024 / (8 * sizeof(__cpu_mask))];
} cpu_set_t;

extern int __sched_cpucount(size_t __setsize, const cpu_set_t *__setp)
    __attribute__((__nothrow__, __leaf__));
extern cpu_set_t *__sched_cpualloc(size_t __count)
    __attribute__((__nothrow__, __leaf__));
extern void __sched_cpufree(cpu_set_t *__set)
    __attribute__((__nothrow__, __leaf__));


extern int sched_setparam(__pid_t __pid, const struct sched_param *__param)
    __attribute__((__nothrow__, __leaf__));

extern int sched_getparam(__pid_t __pid, struct sched_param *__param)
    __attribute__((__nothrow__, __leaf__));

extern int sched_setscheduler(__pid_t __pid, int __policy,
                              const struct sched_param *__param)
    __attribute__((__nothrow__, __leaf__));

extern int sched_getscheduler(__pid_t __pid)
    __attribute__((__nothrow__, __leaf__));

extern int sched_yield(void) __attribute__((__nothrow__, __leaf__));

extern int sched_get_priority_max(int __algorithm)
    __attribute__((__nothrow__, __leaf__));

extern int sched_get_priority_min(int __algorithm)
    __attribute__((__nothrow__, __leaf__));

extern int sched_rr_get_interval(__pid_t __pid, struct timespec *__t)
    __attribute__((__nothrow__, __leaf__));




typedef __clock_t clock_t;


struct tm {
  int tm_sec;
  int tm_min;
  int tm_hour;
  int tm_mday;
  int tm_mon;
  int tm_year;
  int tm_wday;
  int tm_yday;
  int tm_isdst;

  long int tm_gmtoff;
  const char *tm_zone;
};


typedef __clockid_t clockid_t;

typedef __timer_t timer_t;

struct itimerspec {
  struct timespec it_interval;
  struct timespec it_value;
};
struct sigevent;
struct __locale_struct {

  struct __locale_data *__locales[13];

  const unsigned short int *__ctype_b;
  const int *__ctype_tolower;
  const int *__ctype_toupper;

  const char *__names[13];
};

typedef struct __locale_struct *__locale_t;

typedef __locale_t locale_t;

extern clock_t clock(void) __attribute__((__nothrow__, __leaf__));

extern time_t time(time_t *__timer) __attribute__((__nothrow__, __leaf__));

extern double difftime(time_t __time1, time_t __time0)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__const__));

extern time_t mktime(struct tm *__tp) __attribute__((__nothrow__, __leaf__));
extern size_t strftime(char *__restrict __s, size_t __maxsize,
                       const char *__restrict __format,
                       const struct tm *__restrict __tp)
    __attribute__((__nothrow__, __leaf__));
extern size_t strftime_l(char *__restrict __s, size_t __maxsize,
                         const char *__restrict __format,
                         const struct tm *__restrict __tp, locale_t __loc)
    __attribute__((__nothrow__, __leaf__));
extern struct tm *gmtime(const time_t *__timer)
    __attribute__((__nothrow__, __leaf__));

extern struct tm *localtime(const time_t *__timer)
    __attribute__((__nothrow__, __leaf__));
extern struct tm *gmtime_r(const time_t *__restrict __timer,
                           struct tm *__restrict __tp)
    __attribute__((__nothrow__, __leaf__));

extern struct tm *localtime_r(const time_t *__restrict __timer,
                              struct tm *__restrict __tp)
    __attribute__((__nothrow__, __leaf__));
extern char *asctime(const struct tm *__tp)
    __attribute__((__nothrow__, __leaf__));

extern char *ctime(const time_t *__timer)
    __attribute__((__nothrow__, __leaf__));
extern char *asctime_r(const struct tm *__restrict __tp, char *__restrict __buf)
    __attribute__((__nothrow__, __leaf__));

extern char *ctime_r(const time_t *__restrict __timer, char *__restrict __buf)
    __attribute__((__nothrow__, __leaf__));
extern char *__tzname[2];
extern int __daylight;
extern long int __timezone;

extern char *tzname[2];

extern void tzset(void) __attribute__((__nothrow__, __leaf__));

extern int daylight;
extern long int timezone;
extern time_t timegm(struct tm *__tp) __attribute__((__nothrow__, __leaf__));
extern time_t timelocal(struct tm *__tp) __attribute__((__nothrow__, __leaf__));

extern int dysize(int __year) __attribute__((__nothrow__, __leaf__))
__attribute__((__const__));
extern int nanosleep(const struct timespec *__requested_time,
                     struct timespec *__remaining);

extern int clock_getres(clockid_t __clock_id, struct timespec *__res)
    __attribute__((__nothrow__, __leaf__));

extern int clock_gettime(clockid_t __clock_id, struct timespec *__tp)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));

extern int clock_settime(clockid_t __clock_id, const struct timespec *__tp)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));
extern int clock_nanosleep(clockid_t __clock_id, int __flags,
                           const struct timespec *__req,
                           struct timespec *__rem);
extern int clock_getcpuclockid(pid_t __pid, clockid_t *__clock_id)
    __attribute__((__nothrow__, __leaf__));

extern int timer_create(clockid_t __clock_id, struct sigevent *__restrict __evp,
                        timer_t *__restrict __timerid)
    __attribute__((__nothrow__, __leaf__));

extern int timer_delete(timer_t __timerid)
    __attribute__((__nothrow__, __leaf__));

extern int timer_settime(timer_t __timerid, int __flags,
                         const struct itimerspec *__restrict __value,
                         struct itimerspec *__restrict __ovalue)
    __attribute__((__nothrow__, __leaf__));

extern int timer_gettime(timer_t __timerid, struct itimerspec *__value)
    __attribute__((__nothrow__, __leaf__));
extern int timer_getoverrun(timer_t __timerid)
    __attribute__((__nothrow__, __leaf__));

extern int timespec_get(struct timespec *__ts, int __base)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));



typedef union {
  __extension__ unsigned long long int __value64;
  struct {
    unsigned int __low;
    unsigned int __high;
  } __value32;
} __atomic_wide_counter;

typedef struct __pthread_internal_list {
  struct __pthread_internal_list *__prev;
  struct __pthread_internal_list *__next;
} __pthread_list_t;

typedef struct __pthread_internal_slist {
  struct __pthread_internal_slist *__next;
} __pthread_slist_t;
struct __pthread_mutex_s {
  int __lock;
  unsigned int __count;
  int __owner;

  unsigned int __nusers;

  int __kind;

  short __spins;
  short __elision;
  __pthread_list_t __list;
};
struct __pthread_rwlock_arch_t {
  unsigned int __readers;
  unsigned int __writers;
  unsigned int __wrphase_futex;
  unsigned int __writers_futex;
  unsigned int __pad3;
  unsigned int __pad4;

  int __cur_writer;
  int __shared;
  signed char __rwelision;

  unsigned char __pad1[7];

  unsigned long int __pad2;

  unsigned int __flags;
};

struct __pthread_cond_s {
  __atomic_wide_counter __wseq;
  __atomic_wide_counter __g1_start;
  unsigned int __g_refs[2];
  unsigned int __g_size[2];
  unsigned int __g1_orig_size;
  unsigned int __wrefs;
  unsigned int __g_signals[2];
};

typedef unsigned int __tss_t;
typedef unsigned long int __thrd_t;

typedef struct {
  int __data;
} __once_flag;

typedef unsigned long int pthread_t;

typedef union {
  char __size[4];
  int __align;
} pthread_mutexattr_t;

typedef union {
  char __size[4];
  int __align;
} pthread_condattr_t;

typedef unsigned int pthread_key_t;

typedef int pthread_once_t;

union pthread_attr_t {
  char __size[56];
  long int __align;
};

typedef union pthread_attr_t pthread_attr_t;

typedef union {
  struct __pthread_mutex_s __data;
  char __size[40];
  long int __align;
} pthread_mutex_t;

typedef union {
  struct __pthread_cond_s __data;
  char __size[48];
  __extension__ long long int __align;
} pthread_cond_t;

typedef union {
  struct __pthread_rwlock_arch_t __data;
  char __size[56];
  long int __align;
} pthread_rwlock_t;

typedef union {
  char __size[8];
  long int __align;
} pthread_rwlockattr_t;

typedef volatile int pthread_spinlock_t;

typedef union {
  char __size[32];
  long int __align;
} pthread_barrier_t;

typedef union {
  char __size[4];
  int __align;
} pthread_barrierattr_t;

typedef long int __jmp_buf[8];


typedef struct {
  unsigned long int __val[(1024 / (8 * sizeof(unsigned long int)))];
} __sigset_t;
struct __jmp_buf_tag {

  __jmp_buf __jmpbuf;
  int __mask_was_saved;
  __sigset_t __saved_mask;
};


enum {
  PTHREAD_CREATE_JOINABLE,

  PTHREAD_CREATE_DETACHED

};

enum {
  PTHREAD_MUTEX_TIMED_NP,
  PTHREAD_MUTEX_RECURSIVE_NP,
  PTHREAD_MUTEX_ERRORCHECK_NP,
  PTHREAD_MUTEX_ADAPTIVE_NP

  ,
  PTHREAD_MUTEX_NORMAL = PTHREAD_MUTEX_TIMED_NP,
  PTHREAD_MUTEX_RECURSIVE = PTHREAD_MUTEX_RECURSIVE_NP,
  PTHREAD_MUTEX_ERRORCHECK = PTHREAD_MUTEX_ERRORCHECK_NP,
  PTHREAD_MUTEX_DEFAULT = PTHREAD_MUTEX_NORMAL

};

enum {
  PTHREAD_MUTEX_STALLED,
  PTHREAD_MUTEX_STALLED_NP = PTHREAD_MUTEX_STALLED,
  PTHREAD_MUTEX_ROBUST,
  PTHREAD_MUTEX_ROBUST_NP = PTHREAD_MUTEX_ROBUST
};

enum { PTHREAD_PRIO_NONE, PTHREAD_PRIO_INHERIT, PTHREAD_PRIO_PROTECT };
enum {
  PTHREAD_RWLOCK_PREFER_READER_NP,
  PTHREAD_RWLOCK_PREFER_WRITER_NP,
  PTHREAD_RWLOCK_PREFER_WRITER_NONRECURSIVE_NP,
  PTHREAD_RWLOCK_DEFAULT_NP = PTHREAD_RWLOCK_PREFER_READER_NP
};
enum {
  PTHREAD_INHERIT_SCHED,

  PTHREAD_EXPLICIT_SCHED

};

enum {
  PTHREAD_SCOPE_SYSTEM,

  PTHREAD_SCOPE_PROCESS

};

enum {
  PTHREAD_PROCESS_PRIVATE,

  PTHREAD_PROCESS_SHARED

};
struct _pthread_cleanup_buffer {
  void (*__routine)(void *);
  void *__arg;
  int __canceltype;
  struct _pthread_cleanup_buffer *__prev;
};

enum {
  PTHREAD_CANCEL_ENABLE,

  PTHREAD_CANCEL_DISABLE

};
enum {
  PTHREAD_CANCEL_DEFERRED,

  PTHREAD_CANCEL_ASYNCHRONOUS

};

extern int pthread_create(pthread_t *__restrict __newthread,
                          const pthread_attr_t *__restrict __attr,
                          void *(*__start_routine)(void *),
                          void *__restrict __arg) __attribute__((__nothrow__))
__attribute__((__nonnull__(1, 3)));

extern void pthread_exit(void *__retval) __attribute__((__noreturn__));

extern int pthread_join(pthread_t __th, void **__thread_return);
extern int pthread_detach(pthread_t __th)
    __attribute__((__nothrow__, __leaf__));

extern pthread_t pthread_self(void) __attribute__((__nothrow__, __leaf__))
__attribute__((__const__));

extern int pthread_equal(pthread_t __thread1, pthread_t __thread2)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__const__));

extern int pthread_attr_init(pthread_attr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_attr_destroy(pthread_attr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_attr_getdetachstate(const pthread_attr_t *__attr,
                                       int *__detachstate)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_attr_setdetachstate(pthread_attr_t *__attr,
                                       int __detachstate)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_attr_getguardsize(const pthread_attr_t *__attr,
                                     size_t *__guardsize)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_attr_setguardsize(pthread_attr_t *__attr, size_t __guardsize)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_attr_getschedparam(const pthread_attr_t *__restrict __attr,
                                      struct sched_param *__restrict __param)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int
pthread_attr_setschedparam(pthread_attr_t *__restrict __attr,
                           const struct sched_param *__restrict __param)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_attr_getschedpolicy(const pthread_attr_t *__restrict __attr,
                                       int *__restrict __policy)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_attr_setschedpolicy(pthread_attr_t *__attr, int __policy)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_attr_getinheritsched(const pthread_attr_t *__restrict __attr,
                                        int *__restrict __inherit)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_attr_setinheritsched(pthread_attr_t *__attr, int __inherit)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_attr_getscope(const pthread_attr_t *__restrict __attr,
                                 int *__restrict __scope)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_attr_setscope(pthread_attr_t *__attr, int __scope)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_attr_getstackaddr(const pthread_attr_t *__restrict __attr,
                                     void **__restrict __stackaddr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)))
    __attribute__((__deprecated__));

extern int pthread_attr_setstackaddr(pthread_attr_t *__attr, void *__stackaddr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)))
    __attribute__((__deprecated__));

extern int pthread_attr_getstacksize(const pthread_attr_t *__restrict __attr,
                                     size_t *__restrict __stacksize)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_attr_setstacksize(pthread_attr_t *__attr, size_t __stacksize)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_attr_getstack(const pthread_attr_t *__restrict __attr,
                                 void **__restrict __stackaddr,
                                 size_t *__restrict __stacksize)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__nonnull__(1, 2, 3)));

extern int pthread_attr_setstack(pthread_attr_t *__attr, void *__stackaddr,
                                 size_t __stacksize)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern int pthread_setschedparam(pthread_t __target_thread, int __policy,
                                 const struct sched_param *__param)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(3)));

extern int pthread_getschedparam(pthread_t __target_thread,
                                 int *__restrict __policy,
                                 struct sched_param *__restrict __param)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2, 3)));

extern int pthread_setschedprio(pthread_t __target_thread, int __prio)
    __attribute__((__nothrow__, __leaf__));
extern int pthread_getconcurrency(void) __attribute__((__nothrow__, __leaf__));

extern int pthread_setconcurrency(int __level)
    __attribute__((__nothrow__, __leaf__));
extern int pthread_once(pthread_once_t *__once_control,
                        void (*__init_routine)(void))
    __attribute__((__nonnull__(1, 2)));
extern int pthread_setcancelstate(int __state, int *__oldstate);

extern int pthread_setcanceltype(int __type, int *__oldtype);

extern int pthread_cancel(pthread_t __th);

extern void pthread_testcancel(void);

struct __cancel_jmp_buf_tag {
  __jmp_buf __cancel_jmp_buf;
  int __mask_was_saved;
};

typedef struct {
  struct __cancel_jmp_buf_tag __cancel_jmp_buf[1];
  void *__pad[4];
} __pthread_unwind_buf_t __attribute__((__aligned__));
struct __pthread_cleanup_frame {
  void (*__cancel_routine)(void *);
  void *__cancel_arg;
  int __do_it;
  int __cancel_type;
};
extern void __pthread_register_cancel(__pthread_unwind_buf_t *__buf);
extern void __pthread_unregister_cancel(__pthread_unwind_buf_t *__buf);
extern void __pthread_unwind_next(__pthread_unwind_buf_t *__buf)
    __attribute__((__noreturn__))

    __attribute__((__weak__))

    ;
extern int __sigsetjmp_cancel(struct __cancel_jmp_buf_tag __env[1],
                              int __savemask) __asm__(""
                                                      "__sigsetjmp")
    __attribute__((__nothrow__))

    __attribute__((__returns_twice__));
extern int pthread_mutex_init(pthread_mutex_t *__mutex,
                              const pthread_mutexattr_t *__mutexattr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_mutex_destroy(pthread_mutex_t *__mutex)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_mutex_trylock(pthread_mutex_t *__mutex)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_mutex_lock(pthread_mutex_t *__mutex)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_mutex_timedlock(pthread_mutex_t *__restrict __mutex,
                                   const struct timespec *__restrict __abstime)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1, 2)));
extern int pthread_mutex_unlock(pthread_mutex_t *__mutex)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int
pthread_mutex_getprioceiling(const pthread_mutex_t *__restrict __mutex,
                             int *__restrict __prioceiling)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_mutex_setprioceiling(pthread_mutex_t *__restrict __mutex,
                                        int __prioceiling,
                                        int *__restrict __old_ceiling)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 3)));

extern int pthread_mutex_consistent(pthread_mutex_t *__mutex)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern int pthread_mutexattr_init(pthread_mutexattr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_mutexattr_destroy(pthread_mutexattr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_mutexattr_getpshared(const pthread_mutexattr_t *__restrict __attr,
                             int *__restrict __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_mutexattr_setpshared(pthread_mutexattr_t *__attr,
                                        int __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_mutexattr_gettype(const pthread_mutexattr_t *__restrict __attr,
                          int *__restrict __kind)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_mutexattr_settype(pthread_mutexattr_t *__attr, int __kind)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_mutexattr_getprotocol(const pthread_mutexattr_t *__restrict __attr,
                              int *__restrict __protocol)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_mutexattr_setprotocol(pthread_mutexattr_t *__attr,
                                         int __protocol)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_mutexattr_getprioceiling(const pthread_mutexattr_t *__restrict __attr,
                                 int *__restrict __prioceiling)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_mutexattr_setprioceiling(pthread_mutexattr_t *__attr,
                                            int __prioceiling)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_mutexattr_getrobust(const pthread_mutexattr_t *__attr,
                                       int *__robustness)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));
extern int pthread_mutexattr_setrobust(pthread_mutexattr_t *__attr,
                                       int __robustness)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern int pthread_rwlock_init(pthread_rwlock_t *__restrict __rwlock,
                               const pthread_rwlockattr_t *__restrict __attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_rwlock_destroy(pthread_rwlock_t *__rwlock)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_rwlock_rdlock(pthread_rwlock_t *__rwlock)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_rwlock_tryrdlock(pthread_rwlock_t *__rwlock)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int
pthread_rwlock_timedrdlock(pthread_rwlock_t *__restrict __rwlock,
                           const struct timespec *__restrict __abstime)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1, 2)));
extern int pthread_rwlock_wrlock(pthread_rwlock_t *__rwlock)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_rwlock_trywrlock(pthread_rwlock_t *__rwlock)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int
pthread_rwlock_timedwrlock(pthread_rwlock_t *__restrict __rwlock,
                           const struct timespec *__restrict __abstime)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1, 2)));
extern int pthread_rwlock_unlock(pthread_rwlock_t *__rwlock)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_rwlockattr_init(pthread_rwlockattr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_rwlockattr_destroy(pthread_rwlockattr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_rwlockattr_getpshared(const pthread_rwlockattr_t *__restrict __attr,
                              int *__restrict __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_rwlockattr_setpshared(pthread_rwlockattr_t *__attr,
                                         int __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_rwlockattr_getkind_np(const pthread_rwlockattr_t *__restrict __attr,
                              int *__restrict __pref)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_rwlockattr_setkind_np(pthread_rwlockattr_t *__attr,
                                         int __pref)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_cond_init(pthread_cond_t *__restrict __cond,
                             const pthread_condattr_t *__restrict __cond_attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_cond_destroy(pthread_cond_t *__cond)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_cond_signal(pthread_cond_t *__cond)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_cond_broadcast(pthread_cond_t *__cond)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_cond_wait(pthread_cond_t *__restrict __cond,
                             pthread_mutex_t *__restrict __mutex)
    __attribute__((__nonnull__(1, 2)));
extern int pthread_cond_timedwait(pthread_cond_t *__restrict __cond,
                                  pthread_mutex_t *__restrict __mutex,
                                  const struct timespec *__restrict __abstime)
    __attribute__((__nonnull__(1, 2, 3)));
extern int pthread_condattr_init(pthread_condattr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_condattr_destroy(pthread_condattr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_condattr_getpshared(const pthread_condattr_t *__restrict __attr,
                            int *__restrict __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_condattr_setpshared(pthread_condattr_t *__attr,
                                       int __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_condattr_getclock(const pthread_condattr_t *__restrict __attr,
                          __clockid_t *__restrict __clock_id)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_condattr_setclock(pthread_condattr_t *__attr,
                                     __clockid_t __clock_id)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern int pthread_spin_init(pthread_spinlock_t *__lock, int __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_spin_destroy(pthread_spinlock_t *__lock)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_spin_lock(pthread_spinlock_t *__lock)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_spin_trylock(pthread_spinlock_t *__lock)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_spin_unlock(pthread_spinlock_t *__lock)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_barrier_init(pthread_barrier_t *__restrict __barrier,
                                const pthread_barrierattr_t *__restrict __attr,
                                unsigned int __count)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_barrier_destroy(pthread_barrier_t *__barrier)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_barrier_wait(pthread_barrier_t *__barrier)
    __attribute__((__nothrow__)) __attribute__((__nonnull__(1)));

extern int pthread_barrierattr_init(pthread_barrierattr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_barrierattr_destroy(pthread_barrierattr_t *__attr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int
pthread_barrierattr_getpshared(const pthread_barrierattr_t *__restrict __attr,
                               int *__restrict __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int pthread_barrierattr_setpshared(pthread_barrierattr_t *__attr,
                                          int __pshared)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern int pthread_key_create(pthread_key_t *__key,
                              void (*__destr_function)(void *))
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int pthread_key_delete(pthread_key_t __key)
    __attribute__((__nothrow__, __leaf__));

extern void *pthread_getspecific(pthread_key_t __key)
    __attribute__((__nothrow__, __leaf__));

extern int pthread_setspecific(pthread_key_t __key, const void *__pointer)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__access__(__none__, 2)));

extern int pthread_getcpuclockid(pthread_t __thread_id, __clockid_t *__clock_id)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));
extern int pthread_atfork(void (*__prepare)(void), void (*__parent)(void),
                          void (*__child)(void))
    __attribute__((__nothrow__, __leaf__));


typedef void *lib_t;
typedef pthread_t TIDT;
typedef pthread_mutex_t mutex_t;
static inline long __TBB_machine_fetchadd4(volatile void *ptr, long addend)
    __attribute__((always_inline, unused));
static inline long __TBB_machine_fetchadd4(volatile void *ptr, long addend) {
  long result;
  __asm__ __volatile__("lock\nxadd %0,%1"
                       : "=r"(result), "=m"(*(volatile int *)ptr)
                       : "0"(addend), "m"(*(volatile int *)ptr)
                       : "memory");
  return result;
}

static inline long __itt_interlocked_increment(volatile long *ptr)
    __attribute__((always_inline, unused));
static inline long __itt_interlocked_increment(volatile long *ptr) {
  return __TBB_machine_fetchadd4(ptr, 1) + 1L;
}
static inline long __itt_interlocked_compare_exchange(volatile long *ptr,
                                                      long exchange,
                                                      long comperand)
    __attribute__((always_inline, unused));
static inline long __itt_interlocked_compare_exchange(volatile long *ptr,
                                                      long exchange,
                                                      long comperand) {
  return __sync_val_compare_and_swap(ptr, exchange, comperand);
}

void *dlopen(const char *, int) __attribute__((weak));
void *dlsym(void *, const char *) __attribute__((weak));
int dlclose(void *) __attribute__((weak));

int pthread_mutex_init(pthread_mutex_t *, const pthread_mutexattr_t *)
    __attribute__((weak));
int pthread_mutex_lock(pthread_mutex_t *) __attribute__((weak));
int pthread_mutex_unlock(pthread_mutex_t *) __attribute__((weak));
int pthread_mutex_destroy(pthread_mutex_t *) __attribute__((weak));
int pthread_mutexattr_init(pthread_mutexattr_t *) __attribute__((weak));
int pthread_mutexattr_settype(pthread_mutexattr_t *, int) __attribute__((weak));
int pthread_mutexattr_destroy(pthread_mutexattr_t *) __attribute__((weak));
pthread_t pthread_self(void) __attribute__((weak));
typedef enum {
  __itt_thread_normal = 0,
  __itt_thread_ignored = 1
} __itt_thread_state;

#pragma pack(push, 8)

typedef struct ___itt_thread_info {
  const char *nameA;

  void *nameW;

  TIDT tid;
  __itt_thread_state state;
  int extra1;
  void *extra2;
  struct ___itt_thread_info *next;
} __itt_thread_info;

typedef enum ___itt_group_id {
  __itt_group_none = 0,
  __itt_group_legacy = 1 << 0,
  __itt_group_control = 1 << 1,
  __itt_group_thread = 1 << 2,
  __itt_group_mark = 1 << 3,
  __itt_group_sync = 1 << 4,
  __itt_group_fsync = 1 << 5,
  __itt_group_jit = 1 << 6,
  __itt_group_model = 1 << 7,
  __itt_group_splitter_min = 1 << 7,
  __itt_group_counter = 1 << 8,
  __itt_group_frame = 1 << 9,
  __itt_group_stitch = 1 << 10,
  __itt_group_heap = 1 << 11,
  __itt_group_splitter_max = 1 << 12,
  __itt_group_structure = 1 << 12,
  __itt_group_suppress = 1 << 13,
  __itt_group_arrays = 1 << 14,
  __itt_group_module = 1 << 15,
  __itt_group_all = -1
} __itt_group_id;

#pragma pack(push, 8)

typedef struct ___itt_group_list {
  __itt_group_id id;
  const char *name;
} __itt_group_list;

#pragma pack(pop)

typedef struct ___itt_api_info_20101001 {
  const char *name;
  void **func_ptr;
  void *init_func;
  __itt_group_id group;
} __itt_api_info_20101001;

typedef struct ___itt_api_info {
  const char *name;
  void **func_ptr;
  void *init_func;
  void *null_func;
  __itt_group_id group;
} __itt_api_info;

typedef struct __itt_counter_info {
  const char *nameA;

  void *nameW;

  const char *domainA;

  void *domainW;

  int type;
  long index;
  int extra1;
  void *extra2;
  struct __itt_counter_info *next;
} __itt_counter_info_t;

struct ___itt_domain;
struct ___itt_string_handle;
struct ___itt_histogram;

void __itt_pause(void);

void __itt_resume(void);

void __itt_detach(void);

typedef enum {
  __itt_collection_scope_host = 1 << 0,
  __itt_collection_scope_offload = 1 << 1,
  __itt_collection_scope_all = 0x7FFFFFFF
} __itt_collection_scope;

void __itt_pause_scoped(__itt_collection_scope);

void __itt_resume_scoped(__itt_collection_scope);
typedef unsigned char __itt_pt_region;
__itt_pt_region __itt_pt_region_create(const char *name);
void __itt_mark_pt_region_begin(__itt_pt_region region);

void __itt_mark_pt_region_end(__itt_pt_region region);
void __itt_thread_set_name(const char *name);
void __itt_thread_ignore(void);
void __itt_suppress_push(unsigned int mask);
void __itt_suppress_pop(void);
typedef enum __itt_suppress_mode {
  __itt_unsuppress_range,
  __itt_suppress_range
} __itt_suppress_mode_t;

typedef enum {
  __itt_collection_uninitialized = 0,
  __itt_collection_init_fail = 1,
  __itt_collection_collector_absent = 2,
  __itt_collection_collector_exists = 3,
  __itt_collection_init_successful = 4
} __itt_collection_state;

void __itt_suppress_mark_range(__itt_suppress_mode_t mode, unsigned int mask,
                               void *address, size_t size);
void __itt_suppress_clear_range(__itt_suppress_mode_t mode, unsigned int mask,
                                void *address, size_t size);
void __itt_sync_create(void *addr, const char *objtype, const char *objname,
                       int attribute);
void __itt_sync_rename(void *addr, const char *name);
void __itt_sync_destroy(void *addr);
void __itt_sync_prepare(void *addr);
void __itt_sync_cancel(void *addr);
void __itt_sync_acquired(void *addr);
void __itt_sync_releasing(void *addr);
void __itt_fsync_prepare(void *addr);
void __itt_fsync_cancel(void *addr);
void __itt_fsync_acquired(void *addr);
void __itt_fsync_releasing(void *addr);
typedef void *__itt_model_site;
typedef void *__itt_model_site_instance;
typedef void *__itt_model_task;
typedef void *__itt_model_task_instance;

typedef enum {
  __itt_model_disable_observation,
  __itt_model_disable_collection
} __itt_model_disable;
void __itt_model_site_begin(__itt_model_site *site,
                            __itt_model_site_instance *instance,
                            const char *name);

void __itt_model_site_beginA(const char *name);
void __itt_model_site_beginAL(const char *name, size_t siteNameLen);
void __itt_model_site_end(__itt_model_site *site,
                          __itt_model_site_instance *instance);
void __itt_model_site_end_2(void);
void __itt_model_task_begin(__itt_model_task *task,
                            __itt_model_task_instance *instance,
                            const char *name);

void __itt_model_task_beginA(const char *name);
void __itt_model_task_beginAL(const char *name, size_t taskNameLen);
void __itt_model_iteration_taskA(const char *name);
void __itt_model_iteration_taskAL(const char *name, size_t taskNameLen);
void __itt_model_task_end(__itt_model_task *task,
                          __itt_model_task_instance *instance);
void __itt_model_task_end_2(void);
void __itt_model_lock_acquire(void *lock);
void __itt_model_lock_acquire_2(void *lock);
void __itt_model_lock_release(void *lock);
void __itt_model_lock_release_2(void *lock);
void __itt_model_record_allocation(void *addr, size_t size);
void __itt_model_record_deallocation(void *addr);
void __itt_model_induction_uses(void *addr, size_t size);
void __itt_model_reduction_uses(void *addr, size_t size);
void __itt_model_observe_uses(void *addr, size_t size);
void __itt_model_clear_uses(void *addr);
void __itt_model_disable_push(__itt_model_disable x);
void __itt_model_disable_pop(void);
void __itt_model_aggregate_task(size_t x);
typedef void *__itt_heap_function;
__itt_heap_function __itt_heap_function_create(const char *name,
                                               const char *domain);
void __itt_heap_allocate_begin(__itt_heap_function h, size_t size,
                               int initialized);
void __itt_heap_allocate_end(__itt_heap_function h, void **addr, size_t size,
                             int initialized);
void __itt_heap_free_begin(__itt_heap_function h, void *addr);
void __itt_heap_free_end(__itt_heap_function h, void *addr);
void __itt_heap_reallocate_begin(__itt_heap_function h, void *addr,
                                 size_t new_size, int initialized);
void __itt_heap_reallocate_end(__itt_heap_function h, void *addr,
                               void **new_addr, size_t new_size,
                               int initialized);
void __itt_heap_internal_access_begin(void);
void __itt_heap_internal_access_end(void);
void __itt_heap_record_memory_growth_begin(void);
void __itt_heap_record_memory_growth_end(void);
void __itt_heap_reset_detection(unsigned int reset_mask);
void __itt_heap_record(unsigned int record_mask);
#pragma pack(push, 8)

typedef struct ___itt_domain {
  volatile int flags;
  const char *nameA;

  void *nameW;

  int extra1;
  void *extra2;
  struct ___itt_domain *next;
} __itt_domain;

#pragma pack(pop)
__itt_domain *__itt_domain_create(const char *name);
#pragma pack(push, 8)

typedef struct ___itt_id {
  unsigned long long d1, d2, d3;
} __itt_id;

#pragma pack(pop)

static const __itt_id __itt_null = {0, 0, 0};
static inline __itt_id __itt_id_make(void *addr, unsigned long long extra)
    __attribute__((always_inline, unused));
static inline __itt_id __itt_id_make(void *addr, unsigned long long extra) {
  __itt_id id = __itt_null;
  id.d1 = (unsigned long long)((uintptr_t)addr);
  id.d2 = (unsigned long long)extra;
  id.d3 = (unsigned long long)0;
  return id;
}
void __itt_id_create(const __itt_domain *domain, __itt_id id);
void __itt_id_destroy(const __itt_domain *domain, __itt_id id);
#pragma pack(push, 8)

typedef struct ___itt_string_handle {
  const char *strA;

  void *strW;

  int extra1;
  void *extra2;
  struct ___itt_string_handle *next;
} __itt_string_handle;

#pragma pack(pop)
__itt_string_handle *__itt_string_handle_create(const char *name);
typedef unsigned long long __itt_timestamp;
__itt_timestamp __itt_get_timestamp(void);
void __itt_region_begin(const __itt_domain *domain, __itt_id id,
                        __itt_id parentid, __itt_string_handle *name);
void __itt_region_end(const __itt_domain *domain, __itt_id id);
void __itt_frame_begin_v3(const __itt_domain *domain, __itt_id *id);
void __itt_frame_end_v3(const __itt_domain *domain, __itt_id *id);
void __itt_frame_submit_v3(const __itt_domain *domain, __itt_id *id,
                           __itt_timestamp begin, __itt_timestamp end);
void __itt_task_group(const __itt_domain *domain, __itt_id id,
                      __itt_id parentid, __itt_string_handle *name);
void __itt_task_begin(const __itt_domain *domain, __itt_id taskid,
                      __itt_id parentid, __itt_string_handle *name);
void __itt_task_begin_fn(const __itt_domain *domain, __itt_id taskid,
                         __itt_id parentid, void *fn);

void __itt_task_end(const __itt_domain *domain);
void __itt_task_begin_overlapped(const __itt_domain *domain, __itt_id taskid,
                                 __itt_id parentid, __itt_string_handle *name);

void __itt_task_end_overlapped(const __itt_domain *domain, __itt_id taskid);
typedef enum {
  __itt_scope_unknown = 0,
  __itt_scope_global,
  __itt_scope_track_group,
  __itt_scope_track,
  __itt_scope_task,
  __itt_scope_marker
} __itt_scope;
void __itt_marker(const __itt_domain *domain, __itt_id id,
                  __itt_string_handle *name, __itt_scope scope);
typedef enum {
  __itt_metadata_unknown = 0,
  __itt_metadata_u64,
  __itt_metadata_s64,
  __itt_metadata_u32,
  __itt_metadata_s32,
  __itt_metadata_u16,
  __itt_metadata_s16,
  __itt_metadata_float,
  __itt_metadata_double
} __itt_metadata_type;
void __itt_metadata_add(const __itt_domain *domain, __itt_id id,
                        __itt_string_handle *key, __itt_metadata_type type,
                        size_t count, void *data);
void __itt_metadata_str_add(const __itt_domain *domain, __itt_id id,
                            __itt_string_handle *key, const char *data,
                            size_t length);
void __itt_metadata_add_with_scope(const __itt_domain *domain,
                                   __itt_scope scope, __itt_string_handle *key,
                                   __itt_metadata_type type, size_t count,
                                   void *data);
void __itt_metadata_str_add_with_scope(const __itt_domain *domain,
                                       __itt_scope scope,
                                       __itt_string_handle *key,
                                       const char *data, size_t length);
typedef enum {
  __itt_relation_is_unknown = 0,
  __itt_relation_is_dependent_on,
  __itt_relation_is_sibling_of,
  __itt_relation_is_parent_of,
  __itt_relation_is_continuation_of,
  __itt_relation_is_child_of,
  __itt_relation_is_continued_by,
  __itt_relation_is_predecessor_to
} __itt_relation;
void __itt_relation_add_to_current(const __itt_domain *domain,
                                   __itt_relation relation, __itt_id tail);
void __itt_relation_add(const __itt_domain *domain, __itt_id head,
                        __itt_relation relation, __itt_id tail);
#pragma pack(push, 8)

typedef struct ___itt_clock_info {
  unsigned long long clock_freq;
  unsigned long long clock_base;
} __itt_clock_info;

#pragma pack(pop)

typedef void (*__itt_get_clock_info_fn)(__itt_clock_info *clock_info,
                                        void *data);

#pragma pack(push, 8)

typedef struct ___itt_clock_domain {
  __itt_clock_info info;
  __itt_get_clock_info_fn fn;
  void *fn_data;
  int extra1;
  void *extra2;
  struct ___itt_clock_domain *next;
} __itt_clock_domain;

#pragma pack(pop)
__itt_clock_domain *__itt_clock_domain_create(__itt_get_clock_info_fn fn,
                                              void *fn_data);
void __itt_clock_domain_reset(void);
void __itt_id_create_ex(const __itt_domain *domain,
                        __itt_clock_domain *clock_domain,
                        unsigned long long timestamp, __itt_id id);
void __itt_id_destroy_ex(const __itt_domain *domain,
                         __itt_clock_domain *clock_domain,
                         unsigned long long timestamp, __itt_id id);
void __itt_task_begin_ex(const __itt_domain *domain,
                         __itt_clock_domain *clock_domain,
                         unsigned long long timestamp, __itt_id taskid,
                         __itt_id parentid, __itt_string_handle *name);
void __itt_task_begin_fn_ex(const __itt_domain *domain,
                            __itt_clock_domain *clock_domain,
                            unsigned long long timestamp, __itt_id taskid,
                            __itt_id parentid, void *fn);
void __itt_task_end_ex(const __itt_domain *domain,
                       __itt_clock_domain *clock_domain,
                       unsigned long long timestamp);
typedef struct ___itt_counter *__itt_counter;
__itt_counter __itt_counter_create(const char *name, const char *domain);
void __itt_counter_inc(__itt_counter id);
void __itt_counter_inc_delta(__itt_counter id, unsigned long long value);
void __itt_counter_dec(__itt_counter id);
void __itt_counter_dec_delta(__itt_counter id, unsigned long long value);
void __itt_counter_inc_v3(const __itt_domain *domain,
                          __itt_string_handle *name);
void __itt_counter_inc_delta_v3(const __itt_domain *domain,
                                __itt_string_handle *name,
                                unsigned long long delta);
void __itt_counter_dec_v3(const __itt_domain *domain,
                          __itt_string_handle *name);
void __itt_counter_dec_delta_v3(const __itt_domain *domain,
                                __itt_string_handle *name,
                                unsigned long long delta);
void __itt_counter_set_value(__itt_counter id, void *value_ptr);
void __itt_counter_set_value_ex(__itt_counter id,
                                __itt_clock_domain *clock_domain,
                                unsigned long long timestamp, void *value_ptr);
__itt_counter __itt_counter_create_typed(const char *name, const char *domain,
                                         __itt_metadata_type type);
void __itt_counter_destroy(__itt_counter id);
void __itt_marker_ex(const __itt_domain *domain,
                     __itt_clock_domain *clock_domain,
                     unsigned long long timestamp, __itt_id id,
                     __itt_string_handle *name, __itt_scope scope);
void __itt_relation_add_to_current_ex(const __itt_domain *domain,
                                      __itt_clock_domain *clock_domain,
                                      unsigned long long timestamp,
                                      __itt_relation relation, __itt_id tail);
void __itt_relation_add_ex(const __itt_domain *domain,
                           __itt_clock_domain *clock_domain,
                           unsigned long long timestamp, __itt_id head,
                           __itt_relation relation, __itt_id tail);
typedef enum ___itt_track_group_type {
  __itt_track_group_type_normal = 0
} __itt_track_group_type;

#pragma pack(push, 8)

typedef struct ___itt_track_group {
  __itt_string_handle *name;
  struct ___itt_track *track;
  __itt_track_group_type tgtype;
  int extra1;
  void *extra2;
  struct ___itt_track_group *next;
} __itt_track_group;

#pragma pack(pop)

typedef enum ___itt_track_type {
  __itt_track_type_normal = 0

  ,
  __itt_track_type_queue

} __itt_track_type;

#pragma pack(push, 8)

typedef struct ___itt_track {
  __itt_string_handle *name;
  __itt_track_group *group;
  __itt_track_type ttype;
  int extra1;
  void *extra2;
  struct ___itt_track *next;
} __itt_track;

#pragma pack(pop)

__itt_track_group *
__itt_track_group_create(__itt_string_handle *name,
                         __itt_track_group_type track_group_type);
__itt_track *__itt_track_create(__itt_track_group *track_group,
                                __itt_string_handle *name,
                                __itt_track_type track_type);
void __itt_set_track(__itt_track *track);
typedef int __itt_event;
__itt_event __itt_event_create(const char *name, int namelen);
int __itt_event_start(__itt_event event);
int __itt_event_end(__itt_event event);
typedef enum {
  __itt_e_first = 0,
  __itt_e_char = 0,
  __itt_e_uchar,
  __itt_e_int16,
  __itt_e_uint16,
  __itt_e_int32,
  __itt_e_uint32,
  __itt_e_int64,
  __itt_e_uint64,
  __itt_e_float,
  __itt_e_double,
  __itt_e_last = __itt_e_double
} __itt_av_data_type;
int __itt_av_save(void *data, int rank, const int *dimensions, int type,
                  const char *filePath, int columnOrder);
void __itt_enable_attach(void);
void __itt_module_load(void *start_addr, void *end_addr, const char *path);
void __itt_module_unload(void *addr);
typedef enum {
  __itt_module_type_unknown = 0,
  __itt_module_type_elf,
  __itt_module_type_coff
} __itt_module_type;

typedef enum {
  itt_section_type_unknown,
  itt_section_type_bss,

  itt_section_type_data,

  itt_section_type_text

} __itt_section_type;
#pragma pack(push, 8)

typedef struct ___itt_section_info {
  const char *name;
  __itt_section_type type;
  size_t flags;

  void *start_addr;
  size_t size;
  size_t file_offset;
} __itt_section_info;

#pragma pack(pop)

#pragma pack(push, 8)

typedef struct ___itt_module_object {
  unsigned int version;
  __itt_id module_id;
  __itt_module_type module_type;
  const char *module_name;

  void *module_buffer;
  size_t module_size;

  __itt_section_info *section_array;
  size_t section_number;
} __itt_module_object;

#pragma pack(pop)
void __itt_module_load_with_sections(__itt_module_object *module_obj);
void __itt_module_unload_with_sections(__itt_module_object *module_obj);
#pragma pack(push, 8)

typedef struct ___itt_histogram {
  const __itt_domain *domain;
  const char *nameA;

  void *nameW;

  __itt_metadata_type x_type;
  __itt_metadata_type y_type;
  int extra1;
  void *extra2;
  struct ___itt_histogram *next;
} __itt_histogram;

#pragma pack(pop)
__itt_histogram *__itt_histogram_create(const __itt_domain *domain,
                                        const char *name,
                                        __itt_metadata_type x_type,
                                        __itt_metadata_type y_type);
void __itt_histogram_submit(__itt_histogram *hist, size_t length, void *x_data,
                            void *y_data);
__itt_collection_state __itt_get_collection_state(void);

void __itt_release_resources(void);
void __itt_task_begin_overlapped_ex(const __itt_domain *domain,
                                    __itt_clock_domain *clock_domain,
                                    unsigned long long timestamp,
                                    __itt_id taskid, __itt_id parentid,
                                    __itt_string_handle *name);
void __itt_task_end_overlapped_ex(const __itt_domain *domain,
                                  __itt_clock_domain *clock_domain,
                                  unsigned long long timestamp,
                                  __itt_id taskid);
typedef int __itt_mark_type;
__itt_mark_type __itt_mark_create(const char *name);
int __itt_mark(__itt_mark_type mt, const char *parameter);
int __itt_mark_global(__itt_mark_type mt, const char *parameter);
int __itt_mark_off(__itt_mark_type mt);
int __itt_mark_global_off(__itt_mark_type mt);
typedef struct ___itt_caller *__itt_caller;

__itt_caller __itt_stack_caller_create(void);
void __itt_stack_caller_destroy(__itt_caller id);
void __itt_stack_callee_enter(__itt_caller id);
void __itt_stack_callee_leave(__itt_caller id);

typedef __builtin_va_list __gnuc_va_list;
typedef __gnuc_va_list va_list;

typedef enum __itt_error_code {
  __itt_error_success = 0,
  __itt_error_no_module = 1,

  __itt_error_no_symbol = 2,

  __itt_error_unknown_group = 3,

  __itt_error_cant_read_env = 4,

  __itt_error_env_too_long = 5,

  __itt_error_system = 6

} __itt_error_code;

typedef void(__itt_error_handler_t)(__itt_error_code code, va_list);
__itt_error_handler_t *__itt_set_error_handler(__itt_error_handler_t *);

const char *__itt_api_version(void);

typedef struct ___itt_global {
  unsigned char magic[8];
  unsigned long version_major;
  unsigned long version_minor;
  unsigned long version_build;
  volatile long api_initialized;
  volatile long mutex_initialized;
  volatile long atomic_counter;
  mutex_t mutex;
  lib_t lib;
  void *error_handler;
  const char **dll_path_ptr;
  __itt_api_info *api_list_ptr;
  struct ___itt_global *next;

  __itt_thread_info *thread_list;
  struct ___itt_domain *domain_list;
  struct ___itt_string_handle *string_list;
  __itt_collection_state state;
  __itt_counter_info_t *counter_list;
  unsigned int ipt_collect_events;
  struct ___itt_histogram *histogram_list;
} __itt_global;

#pragma pack(pop)






extern int *__errno_location(void) __attribute__((__nothrow__, __leaf__))
__attribute__((__const__));





typedef struct {
  int __count;
  union {
    unsigned int __wch;
    char __wchb[4];
  } __value;
} __mbstate_t;

typedef struct _G_fpos_t {
  __off_t __pos;
  __mbstate_t __state;
} __fpos_t;
typedef struct _G_fpos64_t {
  __off64_t __pos;
  __mbstate_t __state;
} __fpos64_t;

struct _IO_FILE;
typedef struct _IO_FILE __FILE;

struct _IO_FILE;

typedef struct _IO_FILE FILE;
struct _IO_FILE;
struct _IO_marker;
struct _IO_codecvt;
struct _IO_wide_data;

typedef void _IO_lock_t;

struct _IO_FILE {
  int _flags;

  char *_IO_read_ptr;
  char *_IO_read_end;
  char *_IO_read_base;
  char *_IO_write_base;
  char *_IO_write_ptr;
  char *_IO_write_end;
  char *_IO_buf_base;
  char *_IO_buf_end;

  char *_IO_save_base;
  char *_IO_backup_base;
  char *_IO_save_end;

  struct _IO_marker *_markers;

  struct _IO_FILE *_chain;

  int _fileno;
  int _flags2;
  __off_t _old_offset;

  unsigned short _cur_column;
  signed char _vtable_offset;
  char _shortbuf[1];

  _IO_lock_t *_lock;

  __off64_t _offset;

  struct _IO_codecvt *_codecvt;
  struct _IO_wide_data *_wide_data;
  struct _IO_FILE *_freeres_list;
  void *_freeres_buf;
  size_t __pad5;
  int _mode;

  char _unused2[15 * sizeof(int) - 4 * sizeof(void *) - sizeof(size_t)];
};
typedef __off_t off_t;
typedef __ssize_t ssize_t;

typedef __fpos_t fpos_t;
extern FILE *stdin;
extern FILE *stdout;
extern FILE *stderr;

extern int remove(const char *__filename)
    __attribute__((__nothrow__, __leaf__));

extern int rename(const char *__old, const char *__new)
    __attribute__((__nothrow__, __leaf__));

extern int renameat(int __oldfd, const char *__old, int __newfd,
                    const char *__new) __attribute__((__nothrow__, __leaf__));
extern int fclose(FILE *__stream);
extern FILE *tmpfile(void) __attribute__((__malloc__))
__attribute__((__malloc__(fclose, 1)));
extern char *tmpnam(char[20]) __attribute__((__nothrow__, __leaf__));

extern char *tmpnam_r(char __s[20]) __attribute__((__nothrow__, __leaf__));
extern char *tempnam(const char *__dir, const char *__pfx)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__malloc__))
    __attribute__((__malloc__(__builtin_free, 1)));

extern int fflush(FILE *__stream);
extern int fflush_unlocked(FILE *__stream);
extern FILE *fopen(const char *__restrict __filename,
                   const char *__restrict __modes) __attribute__((__malloc__))
__attribute__((__malloc__(fclose, 1)));

extern FILE *freopen(const char *__restrict __filename,
                     const char *__restrict __modes, FILE *__restrict __stream);
extern FILE *fdopen(int __fd, const char *__modes)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__malloc__))
    __attribute__((__malloc__(fclose, 1)));
extern FILE *fmemopen(void *__s, size_t __len, const char *__modes)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__malloc__))
    __attribute__((__malloc__(fclose, 1)));

extern FILE *open_memstream(char **__bufloc, size_t *__sizeloc)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__malloc__))
    __attribute__((__malloc__(fclose, 1)));
extern void setbuf(FILE *__restrict __stream, char *__restrict __buf)
    __attribute__((__nothrow__, __leaf__));

extern int setvbuf(FILE *__restrict __stream, char *__restrict __buf,
                   int __modes, size_t __n)
    __attribute__((__nothrow__, __leaf__));

extern void setbuffer(FILE *__restrict __stream, char *__restrict __buf,
                      size_t __size) __attribute__((__nothrow__, __leaf__));

extern void setlinebuf(FILE *__stream) __attribute__((__nothrow__, __leaf__));

extern int fprintf(FILE *__restrict __stream, const char *__restrict __format,
                   ...);

extern int printf(const char *__restrict __format, ...);

extern int sprintf(char *__restrict __s, const char *__restrict __format, ...)
    __attribute__((__nothrow__));

extern int vfprintf(FILE *__restrict __s, const char *__restrict __format,
                    __gnuc_va_list __arg);

extern int vprintf(const char *__restrict __format, __gnuc_va_list __arg);

extern int vsprintf(char *__restrict __s, const char *__restrict __format,
                    __gnuc_va_list __arg) __attribute__((__nothrow__));

extern int snprintf(char *__restrict __s, size_t __maxlen,
                    const char *__restrict __format, ...)
    __attribute__((__nothrow__)) __attribute__((__format__(__printf__, 3, 4)));

extern int vsnprintf(char *__restrict __s, size_t __maxlen,
                     const char *__restrict __format, __gnuc_va_list __arg)
    __attribute__((__nothrow__)) __attribute__((__format__(__printf__, 3, 0)));
extern int vdprintf(int __fd, const char *__restrict __fmt,
                    __gnuc_va_list __arg)
    __attribute__((__format__(__printf__, 2, 0)));
extern int dprintf(int __fd, const char *__restrict __fmt, ...)
    __attribute__((__format__(__printf__, 2, 3)));

extern int fscanf(FILE *__restrict __stream, const char *__restrict __format,
                  ...);

extern int scanf(const char *__restrict __format, ...);

extern int sscanf(const char *__restrict __s, const char *__restrict __format,
                  ...) __attribute__((__nothrow__, __leaf__));


extern int fscanf(FILE *__restrict __stream, const char *__restrict __format,
                  ...) __asm__(""
                               "__isoc99_fscanf")

    ;
extern int scanf(const char *__restrict __format,
                 ...) __asm__(""
                              "__isoc99_scanf");
extern int sscanf(const char *__restrict __s, const char *__restrict __format,
                  ...) __asm__(""
                               "__isoc99_sscanf")
    __attribute__((__nothrow__, __leaf__))

    ;
extern int vfscanf(FILE *__restrict __s, const char *__restrict __format,
                   __gnuc_va_list __arg)
    __attribute__((__format__(__scanf__, 2, 0)));

extern int vscanf(const char *__restrict __format, __gnuc_va_list __arg)
    __attribute__((__format__(__scanf__, 1, 0)));

extern int vsscanf(const char *__restrict __s, const char *__restrict __format,
                   __gnuc_va_list __arg) __attribute__((__nothrow__, __leaf__))
__attribute__((__format__(__scanf__, 2, 0)));

extern int vfscanf(FILE *__restrict __s, const char *__restrict __format,
                   __gnuc_va_list __arg) __asm__(""
                                                 "__isoc99_vfscanf")

    __attribute__((__format__(__scanf__, 2, 0)));
extern int vscanf(const char *__restrict __format,
                  __gnuc_va_list __arg) __asm__(""
                                                "__isoc99_vscanf")

    __attribute__((__format__(__scanf__, 1, 0)));
extern int vsscanf(const char *__restrict __s, const char *__restrict __format,
                   __gnuc_va_list __arg) __asm__(""
                                                 "__isoc99_vsscanf")
    __attribute__((__nothrow__, __leaf__))

    __attribute__((__format__(__scanf__, 2, 0)));
extern int fgetc(FILE *__stream);
extern int getc(FILE *__stream);

extern int getchar(void);

extern int getc_unlocked(FILE *__stream);
extern int getchar_unlocked(void);
extern int fgetc_unlocked(FILE *__stream);
extern int fputc(int __c, FILE *__stream);
extern int putc(int __c, FILE *__stream);

extern int putchar(int __c);
extern int fputc_unlocked(int __c, FILE *__stream);

extern int putc_unlocked(int __c, FILE *__stream);
extern int putchar_unlocked(int __c);

extern int getw(FILE *__stream);

extern int putw(int __w, FILE *__stream);

extern char *fgets(char *__restrict __s, int __n, FILE *__restrict __stream)
    __attribute__((__access__(__write_only__, 1, 2)));
extern __ssize_t __getdelim(char **__restrict __lineptr, size_t *__restrict __n,
                            int __delimiter, FILE *__restrict __stream);
extern __ssize_t getdelim(char **__restrict __lineptr, size_t *__restrict __n,
                          int __delimiter, FILE *__restrict __stream);

extern __ssize_t getline(char **__restrict __lineptr, size_t *__restrict __n,
                         FILE *__restrict __stream);

extern int fputs(const char *__restrict __s, FILE *__restrict __stream);

extern int puts(const char *__s);

extern int ungetc(int __c, FILE *__stream);

extern size_t fread(void *__restrict __ptr, size_t __size, size_t __n,
                    FILE *__restrict __stream);

extern size_t fwrite(const void *__restrict __ptr, size_t __size, size_t __n,
                     FILE *__restrict __s);
extern size_t fread_unlocked(void *__restrict __ptr, size_t __size, size_t __n,
                             FILE *__restrict __stream);
extern size_t fwrite_unlocked(const void *__restrict __ptr, size_t __size,
                              size_t __n, FILE *__restrict __stream);

extern int fseek(FILE *__stream, long int __off, int __whence);

extern long int ftell(FILE *__stream);

extern void rewind(FILE *__stream);
extern int fseeko(FILE *__stream, __off_t __off, int __whence);

extern __off_t ftello(FILE *__stream);
extern int fgetpos(FILE *__restrict __stream, fpos_t *__restrict __pos);

extern int fsetpos(FILE *__stream, const fpos_t *__pos);
extern void clearerr(FILE *__stream) __attribute__((__nothrow__, __leaf__));

extern int feof(FILE *__stream) __attribute__((__nothrow__, __leaf__));

extern int ferror(FILE *__stream) __attribute__((__nothrow__, __leaf__));

extern void clearerr_unlocked(FILE *__stream)
    __attribute__((__nothrow__, __leaf__));
extern int feof_unlocked(FILE *__stream) __attribute__((__nothrow__, __leaf__));
extern int ferror_unlocked(FILE *__stream)
    __attribute__((__nothrow__, __leaf__));

extern void perror(const char *__s);

extern int fileno(FILE *__stream) __attribute__((__nothrow__, __leaf__));

extern int fileno_unlocked(FILE *__stream)
    __attribute__((__nothrow__, __leaf__));
extern int pclose(FILE *__stream);

extern FILE *popen(const char *__command, const char *__modes)
    __attribute__((__malloc__)) __attribute__((__malloc__(pclose, 1)));

extern char *ctermid(char *__s) __attribute__((__nothrow__, __leaf__))
__attribute__((__access__(__write_only__, 1)));
extern void flockfile(FILE *__stream) __attribute__((__nothrow__, __leaf__));

extern int ftrylockfile(FILE *__stream) __attribute__((__nothrow__, __leaf__));

extern void funlockfile(FILE *__stream) __attribute__((__nothrow__, __leaf__));
extern int __uflow(FILE *);
extern int __overflow(FILE *, int);



typedef struct {
  int quot;
  int rem;
} div_t;

typedef struct {
  long int quot;
  long int rem;
} ldiv_t;

__extension__ typedef struct {
  long long int quot;
  long long int rem;
} lldiv_t;
extern size_t __ctype_get_mb_cur_max(void)
    __attribute__((__nothrow__, __leaf__));

extern double atof(const char *__nptr) __attribute__((__nothrow__, __leaf__))
__attribute__((__pure__)) __attribute__((__nonnull__(1)));

extern int atoi(const char *__nptr) __attribute__((__nothrow__, __leaf__))
__attribute__((__pure__)) __attribute__((__nonnull__(1)));

extern long int atol(const char *__nptr) __attribute__((__nothrow__, __leaf__))
__attribute__((__pure__)) __attribute__((__nonnull__(1)));

__extension__ extern long long int atoll(const char *__nptr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1)));

extern double strtod(const char *__restrict __nptr, char **__restrict __endptr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern float strtof(const char *__restrict __nptr, char **__restrict __endptr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern long double strtold(const char *__restrict __nptr,
                           char **__restrict __endptr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern long int strtol(const char *__restrict __nptr,
                       char **__restrict __endptr, int __base)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern unsigned long int strtoul(const char *__restrict __nptr,
                                 char **__restrict __endptr, int __base)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

__extension__ extern long long int
strtoq(const char *__restrict __nptr, char **__restrict __endptr, int __base)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

__extension__ extern unsigned long long int
strtouq(const char *__restrict __nptr, char **__restrict __endptr, int __base)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

__extension__ extern long long int
strtoll(const char *__restrict __nptr, char **__restrict __endptr, int __base)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

__extension__ extern unsigned long long int
strtoull(const char *__restrict __nptr, char **__restrict __endptr, int __base)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern char *l64a(long int __n) __attribute__((__nothrow__, __leaf__));

extern long int a64l(const char *__s) __attribute__((__nothrow__, __leaf__))
__attribute__((__pure__)) __attribute__((__nonnull__(1)));


typedef __u_char u_char;
typedef __u_short u_short;
typedef __u_int u_int;
typedef __u_long u_long;
typedef __quad_t quad_t;
typedef __u_quad_t u_quad_t;
typedef __fsid_t fsid_t;

typedef __loff_t loff_t;

typedef __ino_t ino_t;
typedef __dev_t dev_t;

typedef __gid_t gid_t;

typedef __mode_t mode_t;

typedef __nlink_t nlink_t;

typedef __uid_t uid_t;
typedef __id_t id_t;
typedef __daddr_t daddr_t;
typedef __caddr_t caddr_t;

typedef __key_t key_t;

typedef unsigned long int ulong;
typedef unsigned short int ushort;
typedef unsigned int uint;

typedef __uint8_t u_int8_t;
typedef __uint16_t u_int16_t;
typedef __uint32_t u_int32_t;
typedef __uint64_t u_int64_t;

typedef int register_t __attribute__((__mode__(__word__)));
static __inline __uint16_t __bswap_16(__uint16_t __bsx) {

  return __builtin_bswap16(__bsx);
}

static __inline __uint32_t __bswap_32(__uint32_t __bsx) {

  return __builtin_bswap32(__bsx);
}
__extension__ static __inline __uint64_t __bswap_64(__uint64_t __bsx) {

  return __builtin_bswap64(__bsx);
}
static __inline __uint16_t __uint16_identity(__uint16_t __x) { return __x; }

static __inline __uint32_t __uint32_identity(__uint32_t __x) { return __x; }

static __inline __uint64_t __uint64_identity(__uint64_t __x) { return __x; }



typedef __sigset_t sigset_t;


struct timeval {

  __time_t tv_sec;
  __suseconds_t tv_usec;
};

typedef __suseconds_t suseconds_t;

typedef long int __fd_mask;
typedef struct {

  __fd_mask __fds_bits[1024 / (8 * (int)sizeof(__fd_mask))];

} fd_set;

typedef __fd_mask fd_mask;

extern int select(int __nfds, fd_set *__restrict __readfds,
                  fd_set *__restrict __writefds, fd_set *__restrict __exceptfds,
                  struct timeval *__restrict __timeout);
extern int pselect(int __nfds, fd_set *__restrict __readfds,
                   fd_set *__restrict __writefds,
                   fd_set *__restrict __exceptfds,
                   const struct timespec *__restrict __timeout,
                   const __sigset_t *__restrict __sigmask);


typedef __blksize_t blksize_t;

typedef __blkcnt_t blkcnt_t;

typedef __fsblkcnt_t fsblkcnt_t;

typedef __fsfilcnt_t fsfilcnt_t;


extern long int random(void) __attribute__((__nothrow__, __leaf__));

extern void srandom(unsigned int __seed) __attribute__((__nothrow__, __leaf__));

extern char *initstate(unsigned int __seed, char *__statebuf, size_t __statelen)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));

extern char *setstate(char *__statebuf) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1)));

struct random_data {
  int32_t *fptr;
  int32_t *rptr;
  int32_t *state;
  int rand_type;
  int rand_deg;
  int rand_sep;
  int32_t *end_ptr;
};

extern int random_r(struct random_data *__restrict __buf,
                    int32_t *__restrict __result)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int srandom_r(unsigned int __seed, struct random_data *__buf)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));

extern int initstate_r(unsigned int __seed, char *__restrict __statebuf,
                       size_t __statelen, struct random_data *__restrict __buf)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2, 4)));

extern int setstate_r(char *__restrict __statebuf,
                      struct random_data *__restrict __buf)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int rand(void) __attribute__((__nothrow__, __leaf__));

extern void srand(unsigned int __seed) __attribute__((__nothrow__, __leaf__));

extern int rand_r(unsigned int *__seed) __attribute__((__nothrow__, __leaf__));

extern double drand48(void) __attribute__((__nothrow__, __leaf__));
extern double erand48(unsigned short int __xsubi[3])
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern long int lrand48(void) __attribute__((__nothrow__, __leaf__));
extern long int nrand48(unsigned short int __xsubi[3])
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern long int mrand48(void) __attribute__((__nothrow__, __leaf__));
extern long int jrand48(unsigned short int __xsubi[3])
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern void srand48(long int __seedval) __attribute__((__nothrow__, __leaf__));
extern unsigned short int *seed48(unsigned short int __seed16v[3])
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern void lcong48(unsigned short int __param[7])
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

struct drand48_data {
  unsigned short int __x[3];
  unsigned short int __old_x[3];
  unsigned short int __c;
  unsigned short int __init;
  __extension__ unsigned long long int __a;
};

extern int drand48_r(struct drand48_data *__restrict __buffer,
                     double *__restrict __result)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));
extern int erand48_r(unsigned short int __xsubi[3],
                     struct drand48_data *__restrict __buffer,
                     double *__restrict __result)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int lrand48_r(struct drand48_data *__restrict __buffer,
                     long int *__restrict __result)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));
extern int nrand48_r(unsigned short int __xsubi[3],
                     struct drand48_data *__restrict __buffer,
                     long int *__restrict __result)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int mrand48_r(struct drand48_data *__restrict __buffer,
                     long int *__restrict __result)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));
extern int jrand48_r(unsigned short int __xsubi[3],
                     struct drand48_data *__restrict __buffer,
                     long int *__restrict __result)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int srand48_r(long int __seedval, struct drand48_data *__buffer)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));

extern int seed48_r(unsigned short int __seed16v[3],
                    struct drand48_data *__buffer)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern int lcong48_r(unsigned short int __param[7],
                     struct drand48_data *__buffer)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern __uint32_t arc4random(void) __attribute__((__nothrow__, __leaf__));

extern void arc4random_buf(void *__buf, size_t __size)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern __uint32_t arc4random_uniform(__uint32_t __upper_bound)
    __attribute__((__nothrow__, __leaf__));

extern void *malloc(size_t __size) __attribute__((__nothrow__, __leaf__))
__attribute__((__malloc__)) __attribute__((__alloc_size__(1)));

extern void *calloc(size_t __nmemb, size_t __size)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__malloc__))
    __attribute__((__alloc_size__(1, 2)));

extern void *realloc(void *__ptr, size_t __size)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__warn_unused_result__)) __attribute__((__alloc_size__(2)));

extern void free(void *__ptr) __attribute__((__nothrow__, __leaf__));

extern void *reallocarray(void *__ptr, size_t __nmemb, size_t __size)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__warn_unused_result__))
    __attribute__((__alloc_size__(2, 3)))
    __attribute__((__malloc__(__builtin_free, 1)));

extern void *reallocarray(void *__ptr, size_t __nmemb, size_t __size)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__malloc__(reallocarray, 1)));


extern void *alloca(size_t __size) __attribute__((__nothrow__, __leaf__));


extern void *valloc(size_t __size) __attribute__((__nothrow__, __leaf__))
__attribute__((__malloc__)) __attribute__((__alloc_size__(1)));

extern int posix_memalign(void **__memptr, size_t __alignment, size_t __size)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern void *aligned_alloc(size_t __alignment, size_t __size)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__malloc__))
    __attribute__((__alloc_align__(1))) __attribute__((__alloc_size__(2)));

extern void abort(void) __attribute__((__nothrow__, __leaf__))
__attribute__((__noreturn__));

extern int atexit(void (*__func)(void)) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1)));

extern int at_quick_exit(void (*__func)(void))
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int on_exit(void (*__func)(int __status, void *__arg), void *__arg)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern void exit(int __status) __attribute__((__nothrow__, __leaf__))
__attribute__((__noreturn__));

extern void quick_exit(int __status) __attribute__((__nothrow__, __leaf__))
__attribute__((__noreturn__));

extern void _Exit(int __status) __attribute__((__nothrow__, __leaf__))
__attribute__((__noreturn__));

extern char *getenv(const char *__name) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1)));
extern int putenv(char *__string) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1)));

extern int setenv(const char *__name, const char *__value, int __replace)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));

extern int unsetenv(const char *__name) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1)));

extern int clearenv(void) __attribute__((__nothrow__, __leaf__));
extern char *mktemp(char *__template) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1)));
extern int mkstemp(char *__template) __attribute__((__nonnull__(1)));
extern int mkstemps(char *__template, int __suffixlen)
    __attribute__((__nonnull__(1)));
extern char *mkdtemp(char *__template) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1)));
extern int system(const char *__command);
extern char *realpath(const char *__restrict __name,
                      char *__restrict __resolved)
    __attribute__((__nothrow__, __leaf__));

typedef int (*__compar_fn_t)(const void *, const void *);
extern void *bsearch(const void *__key, const void *__base, size_t __nmemb,
                     size_t __size, __compar_fn_t __compar)
    __attribute__((__nonnull__(1, 2, 5)));

extern void qsort(void *__base, size_t __nmemb, size_t __size,
                  __compar_fn_t __compar) __attribute__((__nonnull__(1, 4)));
extern int abs(int __x) __attribute__((__nothrow__, __leaf__))
__attribute__((__const__));
extern long int labs(long int __x) __attribute__((__nothrow__, __leaf__))
__attribute__((__const__));

__extension__ extern long long int llabs(long long int __x)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__const__));

extern div_t div(int __numer, int __denom)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__const__));
extern ldiv_t ldiv(long int __numer, long int __denom)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__const__));

__extension__ extern lldiv_t lldiv(long long int __numer, long long int __denom)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__const__));
extern char *ecvt(double __value, int __ndigit, int *__restrict __decpt,
                  int *__restrict __sign) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(3, 4)));

extern char *fcvt(double __value, int __ndigit, int *__restrict __decpt,
                  int *__restrict __sign) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(3, 4)));

extern char *gcvt(double __value, int __ndigit, char *__buf)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(3)));

extern char *qecvt(long double __value, int __ndigit, int *__restrict __decpt,
                   int *__restrict __sign)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(3, 4)));
extern char *qfcvt(long double __value, int __ndigit, int *__restrict __decpt,
                   int *__restrict __sign)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(3, 4)));
extern char *qgcvt(long double __value, int __ndigit, char *__buf)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(3)));

extern int ecvt_r(double __value, int __ndigit, int *__restrict __decpt,
                  int *__restrict __sign, char *__restrict __buf, size_t __len)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__nonnull__(3, 4, 5)));
extern int fcvt_r(double __value, int __ndigit, int *__restrict __decpt,
                  int *__restrict __sign, char *__restrict __buf, size_t __len)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__nonnull__(3, 4, 5)));

extern int qecvt_r(long double __value, int __ndigit, int *__restrict __decpt,
                   int *__restrict __sign, char *__restrict __buf, size_t __len)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__nonnull__(3, 4, 5)));
extern int qfcvt_r(long double __value, int __ndigit, int *__restrict __decpt,
                   int *__restrict __sign, char *__restrict __buf, size_t __len)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__nonnull__(3, 4, 5)));

extern int mblen(const char *__s, size_t __n)
    __attribute__((__nothrow__, __leaf__));

extern int mbtowc(wchar_t *__restrict __pwc, const char *__restrict __s,
                  size_t __n) __attribute__((__nothrow__, __leaf__));

extern int wctomb(char *__s, wchar_t __wchar)
    __attribute__((__nothrow__, __leaf__));

extern size_t mbstowcs(wchar_t *__restrict __pwcs, const char *__restrict __s,
                       size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__access__(__read_only__, 2)));

extern size_t wcstombs(char *__restrict __s, const wchar_t *__restrict __pwcs,
                       size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__access__(__write_only__, 1, 3)))
__attribute__((__access__(__read_only__, 2)));

extern int rpmatch(const char *__response)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));
extern int getsubopt(char **__restrict __optionp,
                     char *const *__restrict __tokens,
                     char **__restrict __valuep)
    __attribute__((__nothrow__, __leaf__))
    __attribute__((__nonnull__(1, 2, 3)));
extern int getloadavg(double __loadavg[], int __nelem)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));



extern void *memcpy(void *__restrict __dest, const void *__restrict __src,
                    size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1, 2)));

extern void *memmove(void *__dest, const void *__src, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern void *memccpy(void *__restrict __dest, const void *__restrict __src,
                     int __c, size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1, 2)))
__attribute__((__access__(__write_only__, 1, 4)));

extern void *memset(void *__s, int __c, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)));

extern int memcmp(const void *__s1, const void *__s2, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));
extern int __memcmpeq(const void *__s1, const void *__s2, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));
extern void *memchr(const void *__s, int __c, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1)));
extern char *strcpy(char *__restrict __dest, const char *__restrict __src)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern char *strncpy(char *__restrict __dest, const char *__restrict __src,
                     size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1, 2)));

extern char *strcat(char *__restrict __dest, const char *__restrict __src)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern char *strncat(char *__restrict __dest, const char *__restrict __src,
                     size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1, 2)));

extern int strcmp(const char *__s1, const char *__s2)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));

extern int strncmp(const char *__s1, const char *__s2, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));

extern int strcoll(const char *__s1, const char *__s2)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));

extern size_t strxfrm(char *__restrict __dest, const char *__restrict __src,
                      size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(2)))
__attribute__((__access__(__write_only__, 1, 3)));

extern int strcoll_l(const char *__s1, const char *__s2, locale_t __l)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2, 3)));

extern size_t strxfrm_l(char *__dest, const char *__src, size_t __n,
                        locale_t __l) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(2, 4)))
__attribute__((__access__(__write_only__, 1, 3)));

extern char *strdup(const char *__s) __attribute__((__nothrow__, __leaf__))
__attribute__((__malloc__)) __attribute__((__nonnull__(1)));

extern char *strndup(const char *__string, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__malloc__))
    __attribute__((__nonnull__(1)));
extern char *strchr(const char *__s, int __c)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1)));
extern char *strrchr(const char *__s, int __c)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1)));
extern size_t strcspn(const char *__s, const char *__reject)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));

extern size_t strspn(const char *__s, const char *__accept)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));
extern char *strpbrk(const char *__s, const char *__accept)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));
extern char *strstr(const char *__haystack, const char *__needle)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));

extern char *strtok(char *__restrict __s, const char *__restrict __delim)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2)));

extern char *__strtok_r(char *__restrict __s, const char *__restrict __delim,
                        char **__restrict __save_ptr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2, 3)));

extern char *strtok_r(char *__restrict __s, const char *__restrict __delim,
                      char **__restrict __save_ptr)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(2, 3)));
extern size_t strlen(const char *__s) __attribute__((__nothrow__, __leaf__))
__attribute__((__pure__)) __attribute__((__nonnull__(1)));

extern size_t strnlen(const char *__string, size_t __maxlen)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1)));

extern char *strerror(int __errnum) __attribute__((__nothrow__, __leaf__));
extern int strerror_r(int __errnum, char *__buf,
                      size_t __buflen) __asm__(""
                                               "__xpg_strerror_r")
    __attribute__((__nothrow__, __leaf__))

    __attribute__((__nonnull__(2)))
    __attribute__((__access__(__write_only__, 2, 3)));
extern char *strerror_l(int __errnum, locale_t __l)
    __attribute__((__nothrow__, __leaf__));


extern int bcmp(const void *__s1, const void *__s2, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));

extern void bcopy(const void *__src, void *__dest, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern void bzero(void *__s, size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1)));
extern char *index(const char *__s, int __c)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1)));
extern char *rindex(const char *__s, int __c)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1)));

extern int ffs(int __i) __attribute__((__nothrow__, __leaf__))
__attribute__((__const__));

extern int ffsl(long int __l) __attribute__((__nothrow__, __leaf__))
__attribute__((__const__));
__extension__ extern int ffsll(long long int __ll)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__const__));

extern int strcasecmp(const char *__s1, const char *__s2)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));

extern int strncasecmp(const char *__s1, const char *__s2, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2)));

extern int strcasecmp_l(const char *__s1, const char *__s2, locale_t __loc)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__pure__))
    __attribute__((__nonnull__(1, 2, 3)));

extern int strncasecmp_l(const char *__s1, const char *__s2, size_t __n,
                         locale_t __loc) __attribute__((__nothrow__, __leaf__))
__attribute__((__pure__)) __attribute__((__nonnull__(1, 2, 4)));


extern void explicit_bzero(void *__s, size_t __n)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1)))
    __attribute__((__access__(__write_only__, 1, 2)));

extern char *strsep(char **__restrict __stringp, const char *__restrict __delim)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern char *strsignal(int __sig) __attribute__((__nothrow__, __leaf__));
extern char *__stpcpy(char *__restrict __dest, const char *__restrict __src)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));
extern char *stpcpy(char *__restrict __dest, const char *__restrict __src)
    __attribute__((__nothrow__, __leaf__)) __attribute__((__nonnull__(1, 2)));

extern char *__stpncpy(char *__restrict __dest, const char *__restrict __src,
                       size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1, 2)));
extern char *stpncpy(char *__restrict __dest, const char *__restrict __src,
                     size_t __n) __attribute__((__nothrow__, __leaf__))
__attribute__((__nonnull__(1, 2)));



int __itt_thr_name_set(const char *name, int namelen);
void __itt_thr_ignore(void);
void __itt_sync_set_name(void *addr, const char *objtype, const char *objname,
                         int attribute);
int __itt_notify_sync_name(void *addr, const char *objtype, int typelen,
                           const char *objname, int namelen, int attribute);
void __itt_notify_sync_prepare(void *addr);
void __itt_notify_sync_cancel(void *addr);
void __itt_notify_sync_acquired(void *addr);
void __itt_notify_sync_releasing(void *addr);
void __itt_memory_read(void *addr, size_t size);
void __itt_memory_write(void *addr, size_t size);
void __itt_memory_update(void *address, size_t size);
typedef int __itt_state_t;

typedef enum __itt_obj_state {
  __itt_obj_state_err = 0,
  __itt_obj_state_clr = 1,
  __itt_obj_state_set = 2,
  __itt_obj_state_use = 3
} __itt_obj_state_t;

typedef enum __itt_thr_state {
  __itt_thr_state_err = 0,
  __itt_thr_state_clr = 1,
  __itt_thr_state_set = 2
} __itt_thr_state_t;

typedef enum __itt_obj_prop {
  __itt_obj_prop_watch = 1,
  __itt_obj_prop_ignore = 2,
  __itt_obj_prop_sharable = 3
} __itt_obj_prop_t;

typedef enum __itt_thr_prop { __itt_thr_prop_quiet = 1 } __itt_thr_prop_t;

__itt_state_t __itt_state_get(void);
__itt_state_t __itt_state_set(__itt_state_t s);
__itt_thr_state_t __itt_thr_mode_set(__itt_thr_prop_t p, __itt_thr_state_t s);
__itt_obj_state_t __itt_obj_mode_set(__itt_obj_prop_t p, __itt_obj_state_t s);
typedef struct __itt_frame_t *__itt_frame;
__itt_frame __itt_frame_create(const char *domain);
void __itt_frame_begin(__itt_frame frame);

void __itt_frame_end(__itt_frame frame);


static const char api_version[] = "ITT-API-Version "
                                  "3.23.0"
                                  " ("
                                  "20210712"
                                  ")"
                                  "\0\n@(#) $Revision$\n";
static const char *ittnotify_lib_name = "libittnotify.so";
typedef int(__itt_init_ittlib_t)(const char *, __itt_group_id);

int __itt_init_ittlib(const char *, __itt_group_id);
static __itt_init_ittlib_t *__itt_init_ittlib_ptr = __itt_init_ittlib;

typedef void(__itt_fini_ittlib_t)(void);

void __itt_fini_ittlib(void);
static __itt_fini_ittlib_t *__itt_fini_ittlib_ptr = __itt_fini_ittlib;

extern __itt_global __itt__ittapi_global;
static void __itt_detach_init_3_0(void);
typedef void __itt_detach_t(void);
__itt_detach_t *__itt_detach_ptr__3_0 = __itt_detach_init_3_0;
static void __itt_detach_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_detach_ptr__3_0 && __itt_detach_ptr__3_0 != __itt_detach_init_3_0)
    __itt_detach_ptr__3_0();
  else
    return;
}

static void __itt_sync_create_init_3_0(void *addr, const char *objtype,
                                       const char *objname, int attribute);
typedef void __itt_sync_create_t(void *addr, const char *objtype,
                                 const char *objname, int attribute);
__itt_sync_create_t *__itt_sync_create_ptr__3_0 = __itt_sync_create_init_3_0;
static void __itt_sync_create_init_3_0(void *addr, const char *objtype,
                                       const char *objname, int attribute) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_sync_create_ptr__3_0 &&
      __itt_sync_create_ptr__3_0 != __itt_sync_create_init_3_0)
    __itt_sync_create_ptr__3_0(addr, objtype, objname, attribute);
  else
    return;
}
static void __itt_sync_rename_init_3_0(void *addr, const char *name);
typedef void __itt_sync_rename_t(void *addr, const char *name);
__itt_sync_rename_t *__itt_sync_rename_ptr__3_0 = __itt_sync_rename_init_3_0;
static void __itt_sync_rename_init_3_0(void *addr, const char *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_sync_rename_ptr__3_0 &&
      __itt_sync_rename_ptr__3_0 != __itt_sync_rename_init_3_0)
    __itt_sync_rename_ptr__3_0(addr, name);
  else
    return;
}

static void __itt_sync_destroy_init_3_0(void *addr);
typedef void __itt_sync_destroy_t(void *addr);
__itt_sync_destroy_t *__itt_sync_destroy_ptr__3_0 = __itt_sync_destroy_init_3_0;
static void __itt_sync_destroy_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_sync_destroy_ptr__3_0 &&
      __itt_sync_destroy_ptr__3_0 != __itt_sync_destroy_init_3_0)
    __itt_sync_destroy_ptr__3_0(addr);
  else
    return;
}

static void __itt_sync_prepare_init_3_0(void *addr);
typedef void __itt_sync_prepare_t(void *addr);
__itt_sync_prepare_t *__itt_sync_prepare_ptr__3_0 = __itt_sync_prepare_init_3_0;
static void __itt_sync_prepare_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_sync_prepare_ptr__3_0 &&
      __itt_sync_prepare_ptr__3_0 != __itt_sync_prepare_init_3_0)
    __itt_sync_prepare_ptr__3_0(addr);
  else
    return;
}
static void __itt_sync_cancel_init_3_0(void *addr);
typedef void __itt_sync_cancel_t(void *addr);
__itt_sync_cancel_t *__itt_sync_cancel_ptr__3_0 = __itt_sync_cancel_init_3_0;
static void __itt_sync_cancel_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_sync_cancel_ptr__3_0 &&
      __itt_sync_cancel_ptr__3_0 != __itt_sync_cancel_init_3_0)
    __itt_sync_cancel_ptr__3_0(addr);
  else
    return;
}
static void __itt_sync_acquired_init_3_0(void *addr);
typedef void __itt_sync_acquired_t(void *addr);
__itt_sync_acquired_t *__itt_sync_acquired_ptr__3_0 =
    __itt_sync_acquired_init_3_0;
static void __itt_sync_acquired_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_sync_acquired_ptr__3_0 &&
      __itt_sync_acquired_ptr__3_0 != __itt_sync_acquired_init_3_0)
    __itt_sync_acquired_ptr__3_0(addr);
  else
    return;
}
static void __itt_sync_releasing_init_3_0(void *addr);
typedef void __itt_sync_releasing_t(void *addr);
__itt_sync_releasing_t *__itt_sync_releasing_ptr__3_0 =
    __itt_sync_releasing_init_3_0;
static void __itt_sync_releasing_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_sync_releasing_ptr__3_0 &&
      __itt_sync_releasing_ptr__3_0 != __itt_sync_releasing_init_3_0)
    __itt_sync_releasing_ptr__3_0(addr);
  else
    return;
}

static void __itt_suppress_push_init_3_0(unsigned int mask);
typedef void __itt_suppress_push_t(unsigned int mask);
__itt_suppress_push_t *__itt_suppress_push_ptr__3_0 =
    __itt_suppress_push_init_3_0;
static void __itt_suppress_push_init_3_0(unsigned int mask) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_suppress_push_ptr__3_0 &&
      __itt_suppress_push_ptr__3_0 != __itt_suppress_push_init_3_0)
    __itt_suppress_push_ptr__3_0(mask);
  else
    return;
}
static void __itt_suppress_pop_init_3_0(void);
typedef void __itt_suppress_pop_t(void);
__itt_suppress_pop_t *__itt_suppress_pop_ptr__3_0 = __itt_suppress_pop_init_3_0;
static void __itt_suppress_pop_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_suppress_pop_ptr__3_0 &&
      __itt_suppress_pop_ptr__3_0 != __itt_suppress_pop_init_3_0)
    __itt_suppress_pop_ptr__3_0();
  else
    return;
}
static void __itt_suppress_mark_range_init_3_0(__itt_suppress_mode_t mode,
                                               unsigned int mask, void *address,
                                               size_t size);
typedef void __itt_suppress_mark_range_t(__itt_suppress_mode_t mode,
                                         unsigned int mask, void *address,
                                         size_t size);
__itt_suppress_mark_range_t *__itt_suppress_mark_range_ptr__3_0 =
    __itt_suppress_mark_range_init_3_0;
static void __itt_suppress_mark_range_init_3_0(__itt_suppress_mode_t mode,
                                               unsigned int mask, void *address,
                                               size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_suppress_mark_range_ptr__3_0 &&
      __itt_suppress_mark_range_ptr__3_0 != __itt_suppress_mark_range_init_3_0)
    __itt_suppress_mark_range_ptr__3_0(mode, mask, address, size);
  else
    return;
}
static void __itt_suppress_clear_range_init_3_0(__itt_suppress_mode_t mode,
                                                unsigned int mask,
                                                void *address, size_t size);
typedef void __itt_suppress_clear_range_t(__itt_suppress_mode_t mode,
                                          unsigned int mask, void *address,
                                          size_t size);
__itt_suppress_clear_range_t *__itt_suppress_clear_range_ptr__3_0 =
    __itt_suppress_clear_range_init_3_0;
static void __itt_suppress_clear_range_init_3_0(__itt_suppress_mode_t mode,
                                                unsigned int mask,
                                                void *address, size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_suppress_clear_range_ptr__3_0 &&
      __itt_suppress_clear_range_ptr__3_0 !=
          __itt_suppress_clear_range_init_3_0)
    __itt_suppress_clear_range_ptr__3_0(mode, mask, address, size);
  else
    return;
}

static void __itt_fsync_prepare_init_3_0(void *addr);
typedef void __itt_fsync_prepare_t(void *addr);
__itt_fsync_prepare_t *__itt_fsync_prepare_ptr__3_0 =
    __itt_fsync_prepare_init_3_0;
static void __itt_fsync_prepare_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_fsync_prepare_ptr__3_0 &&
      __itt_fsync_prepare_ptr__3_0 != __itt_fsync_prepare_init_3_0)
    __itt_fsync_prepare_ptr__3_0(addr);
  else
    return;
}
static void __itt_fsync_cancel_init_3_0(void *addr);
typedef void __itt_fsync_cancel_t(void *addr);
__itt_fsync_cancel_t *__itt_fsync_cancel_ptr__3_0 = __itt_fsync_cancel_init_3_0;
static void __itt_fsync_cancel_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_fsync_cancel_ptr__3_0 &&
      __itt_fsync_cancel_ptr__3_0 != __itt_fsync_cancel_init_3_0)
    __itt_fsync_cancel_ptr__3_0(addr);
  else
    return;
}
static void __itt_fsync_acquired_init_3_0(void *addr);
typedef void __itt_fsync_acquired_t(void *addr);
__itt_fsync_acquired_t *__itt_fsync_acquired_ptr__3_0 =
    __itt_fsync_acquired_init_3_0;
static void __itt_fsync_acquired_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_fsync_acquired_ptr__3_0 &&
      __itt_fsync_acquired_ptr__3_0 != __itt_fsync_acquired_init_3_0)
    __itt_fsync_acquired_ptr__3_0(addr);
  else
    return;
}
static void __itt_fsync_releasing_init_3_0(void *addr);
typedef void __itt_fsync_releasing_t(void *addr);
__itt_fsync_releasing_t *__itt_fsync_releasing_ptr__3_0 =
    __itt_fsync_releasing_init_3_0;
static void __itt_fsync_releasing_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_fsync_releasing_ptr__3_0 &&
      __itt_fsync_releasing_ptr__3_0 != __itt_fsync_releasing_init_3_0)
    __itt_fsync_releasing_ptr__3_0(addr);
  else
    return;
}

static void __itt_model_site_begin_init_3_0(__itt_model_site *site,
                                            __itt_model_site_instance *instance,
                                            const char *name);
typedef void __itt_model_site_begin_t(__itt_model_site *site,
                                      __itt_model_site_instance *instance,
                                      const char *name);
__itt_model_site_begin_t *__itt_model_site_begin_ptr__3_0 =
    __itt_model_site_begin_init_3_0;
static void __itt_model_site_begin_init_3_0(__itt_model_site *site,
                                            __itt_model_site_instance *instance,
                                            const char *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_site_begin_ptr__3_0 &&
      __itt_model_site_begin_ptr__3_0 != __itt_model_site_begin_init_3_0)
    __itt_model_site_begin_ptr__3_0(site, instance, name);
  else
    return;
}
static void __itt_model_site_end_init_3_0(__itt_model_site *site,
                                          __itt_model_site_instance *instance);
typedef void __itt_model_site_end_t(__itt_model_site *site,
                                    __itt_model_site_instance *instance);
__itt_model_site_end_t *__itt_model_site_end_ptr__3_0 =
    __itt_model_site_end_init_3_0;
static void __itt_model_site_end_init_3_0(__itt_model_site *site,
                                          __itt_model_site_instance *instance) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_site_end_ptr__3_0 &&
      __itt_model_site_end_ptr__3_0 != __itt_model_site_end_init_3_0)
    __itt_model_site_end_ptr__3_0(site, instance);
  else
    return;
}
static void __itt_model_task_begin_init_3_0(__itt_model_task *task,
                                            __itt_model_task_instance *instance,
                                            const char *name);
typedef void __itt_model_task_begin_t(__itt_model_task *task,
                                      __itt_model_task_instance *instance,
                                      const char *name);
__itt_model_task_begin_t *__itt_model_task_begin_ptr__3_0 =
    __itt_model_task_begin_init_3_0;
static void __itt_model_task_begin_init_3_0(__itt_model_task *task,
                                            __itt_model_task_instance *instance,
                                            const char *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_task_begin_ptr__3_0 &&
      __itt_model_task_begin_ptr__3_0 != __itt_model_task_begin_init_3_0)
    __itt_model_task_begin_ptr__3_0(task, instance, name);
  else
    return;
}
static void __itt_model_task_end_init_3_0(__itt_model_task *task,
                                          __itt_model_task_instance *instance);
typedef void __itt_model_task_end_t(__itt_model_task *task,
                                    __itt_model_task_instance *instance);
__itt_model_task_end_t *__itt_model_task_end_ptr__3_0 =
    __itt_model_task_end_init_3_0;
static void __itt_model_task_end_init_3_0(__itt_model_task *task,
                                          __itt_model_task_instance *instance) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_task_end_ptr__3_0 &&
      __itt_model_task_end_ptr__3_0 != __itt_model_task_end_init_3_0)
    __itt_model_task_end_ptr__3_0(task, instance);
  else
    return;
}
static void __itt_model_lock_acquire_init_3_0(void *lock);
typedef void __itt_model_lock_acquire_t(void *lock);
__itt_model_lock_acquire_t *__itt_model_lock_acquire_ptr__3_0 =
    __itt_model_lock_acquire_init_3_0;
static void __itt_model_lock_acquire_init_3_0(void *lock) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_lock_acquire_ptr__3_0 &&
      __itt_model_lock_acquire_ptr__3_0 != __itt_model_lock_acquire_init_3_0)
    __itt_model_lock_acquire_ptr__3_0(lock);
  else
    return;
}
static void __itt_model_lock_release_init_3_0(void *lock);
typedef void __itt_model_lock_release_t(void *lock);
__itt_model_lock_release_t *__itt_model_lock_release_ptr__3_0 =
    __itt_model_lock_release_init_3_0;
static void __itt_model_lock_release_init_3_0(void *lock) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_lock_release_ptr__3_0 &&
      __itt_model_lock_release_ptr__3_0 != __itt_model_lock_release_init_3_0)
    __itt_model_lock_release_ptr__3_0(lock);
  else
    return;
}
static void __itt_model_record_allocation_init_3_0(void *addr, size_t size);
typedef void __itt_model_record_allocation_t(void *addr, size_t size);
__itt_model_record_allocation_t *__itt_model_record_allocation_ptr__3_0 =
    __itt_model_record_allocation_init_3_0;
static void __itt_model_record_allocation_init_3_0(void *addr, size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_record_allocation_ptr__3_0 &&
      __itt_model_record_allocation_ptr__3_0 !=
          __itt_model_record_allocation_init_3_0)
    __itt_model_record_allocation_ptr__3_0(addr, size);
  else
    return;
}
static void __itt_model_record_deallocation_init_3_0(void *addr);
typedef void __itt_model_record_deallocation_t(void *addr);
__itt_model_record_deallocation_t *__itt_model_record_deallocation_ptr__3_0 =
    __itt_model_record_deallocation_init_3_0;
static void __itt_model_record_deallocation_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_record_deallocation_ptr__3_0 &&
      __itt_model_record_deallocation_ptr__3_0 !=
          __itt_model_record_deallocation_init_3_0)
    __itt_model_record_deallocation_ptr__3_0(addr);
  else
    return;
}
static void __itt_model_induction_uses_init_3_0(void *addr, size_t size);
typedef void __itt_model_induction_uses_t(void *addr, size_t size);
__itt_model_induction_uses_t *__itt_model_induction_uses_ptr__3_0 =
    __itt_model_induction_uses_init_3_0;
static void __itt_model_induction_uses_init_3_0(void *addr, size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_induction_uses_ptr__3_0 &&
      __itt_model_induction_uses_ptr__3_0 !=
          __itt_model_induction_uses_init_3_0)
    __itt_model_induction_uses_ptr__3_0(addr, size);
  else
    return;
}
static void __itt_model_reduction_uses_init_3_0(void *addr, size_t size);
typedef void __itt_model_reduction_uses_t(void *addr, size_t size);
__itt_model_reduction_uses_t *__itt_model_reduction_uses_ptr__3_0 =
    __itt_model_reduction_uses_init_3_0;
static void __itt_model_reduction_uses_init_3_0(void *addr, size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_reduction_uses_ptr__3_0 &&
      __itt_model_reduction_uses_ptr__3_0 !=
          __itt_model_reduction_uses_init_3_0)
    __itt_model_reduction_uses_ptr__3_0(addr, size);
  else
    return;
}
static void __itt_model_observe_uses_init_3_0(void *addr, size_t size);
typedef void __itt_model_observe_uses_t(void *addr, size_t size);
__itt_model_observe_uses_t *__itt_model_observe_uses_ptr__3_0 =
    __itt_model_observe_uses_init_3_0;
static void __itt_model_observe_uses_init_3_0(void *addr, size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_observe_uses_ptr__3_0 &&
      __itt_model_observe_uses_ptr__3_0 != __itt_model_observe_uses_init_3_0)
    __itt_model_observe_uses_ptr__3_0(addr, size);
  else
    return;
}
static void __itt_model_clear_uses_init_3_0(void *addr);
typedef void __itt_model_clear_uses_t(void *addr);
__itt_model_clear_uses_t *__itt_model_clear_uses_ptr__3_0 =
    __itt_model_clear_uses_init_3_0;
static void __itt_model_clear_uses_init_3_0(void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_clear_uses_ptr__3_0 &&
      __itt_model_clear_uses_ptr__3_0 != __itt_model_clear_uses_init_3_0)
    __itt_model_clear_uses_ptr__3_0(addr);
  else
    return;
}

static void __itt_model_site_beginA_init_3_0(const char *name);
typedef void __itt_model_site_beginA_t(const char *name);
__itt_model_site_beginA_t *__itt_model_site_beginA_ptr__3_0 =
    __itt_model_site_beginA_init_3_0;
static void __itt_model_site_beginA_init_3_0(const char *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_site_beginA_ptr__3_0 &&
      __itt_model_site_beginA_ptr__3_0 != __itt_model_site_beginA_init_3_0)
    __itt_model_site_beginA_ptr__3_0(name);
  else
    return;
}
static void __itt_model_site_beginAL_init_3_0(const char *name, size_t len);
typedef void __itt_model_site_beginAL_t(const char *name, size_t len);
__itt_model_site_beginAL_t *__itt_model_site_beginAL_ptr__3_0 =
    __itt_model_site_beginAL_init_3_0;
static void __itt_model_site_beginAL_init_3_0(const char *name, size_t len) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_site_beginAL_ptr__3_0 &&
      __itt_model_site_beginAL_ptr__3_0 != __itt_model_site_beginAL_init_3_0)
    __itt_model_site_beginAL_ptr__3_0(name, len);
  else
    return;
}
static void __itt_model_task_beginA_init_3_0(const char *name);
typedef void __itt_model_task_beginA_t(const char *name);
__itt_model_task_beginA_t *__itt_model_task_beginA_ptr__3_0 =
    __itt_model_task_beginA_init_3_0;
static void __itt_model_task_beginA_init_3_0(const char *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_task_beginA_ptr__3_0 &&
      __itt_model_task_beginA_ptr__3_0 != __itt_model_task_beginA_init_3_0)
    __itt_model_task_beginA_ptr__3_0(name);
  else
    return;
}
static void __itt_model_task_beginAL_init_3_0(const char *name, size_t len);
typedef void __itt_model_task_beginAL_t(const char *name, size_t len);
__itt_model_task_beginAL_t *__itt_model_task_beginAL_ptr__3_0 =
    __itt_model_task_beginAL_init_3_0;
static void __itt_model_task_beginAL_init_3_0(const char *name, size_t len) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_task_beginAL_ptr__3_0 &&
      __itt_model_task_beginAL_ptr__3_0 != __itt_model_task_beginAL_init_3_0)
    __itt_model_task_beginAL_ptr__3_0(name, len);
  else
    return;
}
static void __itt_model_iteration_taskA_init_3_0(const char *name);
typedef void __itt_model_iteration_taskA_t(const char *name);
__itt_model_iteration_taskA_t *__itt_model_iteration_taskA_ptr__3_0 =
    __itt_model_iteration_taskA_init_3_0;
static void __itt_model_iteration_taskA_init_3_0(const char *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_iteration_taskA_ptr__3_0 &&
      __itt_model_iteration_taskA_ptr__3_0 !=
          __itt_model_iteration_taskA_init_3_0)
    __itt_model_iteration_taskA_ptr__3_0(name);
  else
    return;
}
static void __itt_model_iteration_taskAL_init_3_0(const char *name, size_t len);
typedef void __itt_model_iteration_taskAL_t(const char *name, size_t len);
__itt_model_iteration_taskAL_t *__itt_model_iteration_taskAL_ptr__3_0 =
    __itt_model_iteration_taskAL_init_3_0;
static void __itt_model_iteration_taskAL_init_3_0(const char *name,
                                                  size_t len) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_iteration_taskAL_ptr__3_0 &&
      __itt_model_iteration_taskAL_ptr__3_0 !=
          __itt_model_iteration_taskAL_init_3_0)
    __itt_model_iteration_taskAL_ptr__3_0(name, len);
  else
    return;
}
static void __itt_model_site_end_2_init_3_0(void);
typedef void __itt_model_site_end_2_t(void);
__itt_model_site_end_2_t *__itt_model_site_end_2_ptr__3_0 =
    __itt_model_site_end_2_init_3_0;
static void __itt_model_site_end_2_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_site_end_2_ptr__3_0 &&
      __itt_model_site_end_2_ptr__3_0 != __itt_model_site_end_2_init_3_0)
    __itt_model_site_end_2_ptr__3_0();
  else
    return;
}
static void __itt_model_task_end_2_init_3_0(void);
typedef void __itt_model_task_end_2_t(void);
__itt_model_task_end_2_t *__itt_model_task_end_2_ptr__3_0 =
    __itt_model_task_end_2_init_3_0;
static void __itt_model_task_end_2_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_task_end_2_ptr__3_0 &&
      __itt_model_task_end_2_ptr__3_0 != __itt_model_task_end_2_init_3_0)
    __itt_model_task_end_2_ptr__3_0();
  else
    return;
}
static void __itt_model_lock_acquire_2_init_3_0(void *lock);
typedef void __itt_model_lock_acquire_2_t(void *lock);
__itt_model_lock_acquire_2_t *__itt_model_lock_acquire_2_ptr__3_0 =
    __itt_model_lock_acquire_2_init_3_0;
static void __itt_model_lock_acquire_2_init_3_0(void *lock) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_lock_acquire_2_ptr__3_0 &&
      __itt_model_lock_acquire_2_ptr__3_0 !=
          __itt_model_lock_acquire_2_init_3_0)
    __itt_model_lock_acquire_2_ptr__3_0(lock);
  else
    return;
}
static void __itt_model_lock_release_2_init_3_0(void *lock);
typedef void __itt_model_lock_release_2_t(void *lock);
__itt_model_lock_release_2_t *__itt_model_lock_release_2_ptr__3_0 =
    __itt_model_lock_release_2_init_3_0;
static void __itt_model_lock_release_2_init_3_0(void *lock) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_lock_release_2_ptr__3_0 &&
      __itt_model_lock_release_2_ptr__3_0 !=
          __itt_model_lock_release_2_init_3_0)
    __itt_model_lock_release_2_ptr__3_0(lock);
  else
    return;
}
static void __itt_model_aggregate_task_init_3_0(size_t count);
typedef void __itt_model_aggregate_task_t(size_t count);
__itt_model_aggregate_task_t *__itt_model_aggregate_task_ptr__3_0 =
    __itt_model_aggregate_task_init_3_0;
static void __itt_model_aggregate_task_init_3_0(size_t count) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_aggregate_task_ptr__3_0 &&
      __itt_model_aggregate_task_ptr__3_0 !=
          __itt_model_aggregate_task_init_3_0)
    __itt_model_aggregate_task_ptr__3_0(count);
  else
    return;
}
static void __itt_model_disable_push_init_3_0(__itt_model_disable x);
typedef void __itt_model_disable_push_t(__itt_model_disable x);
__itt_model_disable_push_t *__itt_model_disable_push_ptr__3_0 =
    __itt_model_disable_push_init_3_0;
static void __itt_model_disable_push_init_3_0(__itt_model_disable x) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_disable_push_ptr__3_0 &&
      __itt_model_disable_push_ptr__3_0 != __itt_model_disable_push_init_3_0)
    __itt_model_disable_push_ptr__3_0(x);
  else
    return;
}
static void __itt_model_disable_pop_init_3_0(void);
typedef void __itt_model_disable_pop_t(void);
__itt_model_disable_pop_t *__itt_model_disable_pop_ptr__3_0 =
    __itt_model_disable_pop_init_3_0;
static void __itt_model_disable_pop_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_model_disable_pop_ptr__3_0 &&
      __itt_model_disable_pop_ptr__3_0 != __itt_model_disable_pop_init_3_0)
    __itt_model_disable_pop_ptr__3_0();
  else
    return;
}

static __itt_heap_function
__itt_heap_function_create_init_3_0(const char *name, const char *domain);
typedef __itt_heap_function __itt_heap_function_create_t(const char *name,
                                                         const char *domain);
__itt_heap_function_create_t *__itt_heap_function_create_ptr__3_0 =
    __itt_heap_function_create_init_3_0;
static __itt_heap_function
__itt_heap_function_create_init_3_0(const char *name, const char *domain) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_function_create_ptr__3_0 &&
      __itt_heap_function_create_ptr__3_0 !=
          __itt_heap_function_create_init_3_0)
    return __itt_heap_function_create_ptr__3_0(name, domain);
  else
    return (__itt_heap_function)0;
}

static void __itt_heap_allocate_begin_init_3_0(__itt_heap_function h,
                                               size_t size, int initialized);
typedef void __itt_heap_allocate_begin_t(__itt_heap_function h, size_t size,
                                         int initialized);
__itt_heap_allocate_begin_t *__itt_heap_allocate_begin_ptr__3_0 =
    __itt_heap_allocate_begin_init_3_0;
static void __itt_heap_allocate_begin_init_3_0(__itt_heap_function h,
                                               size_t size, int initialized) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_allocate_begin_ptr__3_0 &&
      __itt_heap_allocate_begin_ptr__3_0 != __itt_heap_allocate_begin_init_3_0)
    __itt_heap_allocate_begin_ptr__3_0(h, size, initialized);
  else
    return;
}
static void __itt_heap_allocate_end_init_3_0(__itt_heap_function h, void **addr,
                                             size_t size, int initialized);
typedef void __itt_heap_allocate_end_t(__itt_heap_function h, void **addr,
                                       size_t size, int initialized);
__itt_heap_allocate_end_t *__itt_heap_allocate_end_ptr__3_0 =
    __itt_heap_allocate_end_init_3_0;
static void __itt_heap_allocate_end_init_3_0(__itt_heap_function h, void **addr,
                                             size_t size, int initialized) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_allocate_end_ptr__3_0 &&
      __itt_heap_allocate_end_ptr__3_0 != __itt_heap_allocate_end_init_3_0)
    __itt_heap_allocate_end_ptr__3_0(h, addr, size, initialized);
  else
    return;
}
static void __itt_heap_free_begin_init_3_0(__itt_heap_function h, void *addr);
typedef void __itt_heap_free_begin_t(__itt_heap_function h, void *addr);
__itt_heap_free_begin_t *__itt_heap_free_begin_ptr__3_0 =
    __itt_heap_free_begin_init_3_0;
static void __itt_heap_free_begin_init_3_0(__itt_heap_function h, void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_free_begin_ptr__3_0 &&
      __itt_heap_free_begin_ptr__3_0 != __itt_heap_free_begin_init_3_0)
    __itt_heap_free_begin_ptr__3_0(h, addr);
  else
    return;
}
static void __itt_heap_free_end_init_3_0(__itt_heap_function h, void *addr);
typedef void __itt_heap_free_end_t(__itt_heap_function h, void *addr);
__itt_heap_free_end_t *__itt_heap_free_end_ptr__3_0 =
    __itt_heap_free_end_init_3_0;
static void __itt_heap_free_end_init_3_0(__itt_heap_function h, void *addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_free_end_ptr__3_0 &&
      __itt_heap_free_end_ptr__3_0 != __itt_heap_free_end_init_3_0)
    __itt_heap_free_end_ptr__3_0(h, addr);
  else
    return;
}
static void __itt_heap_reallocate_begin_init_3_0(__itt_heap_function h,
                                                 void *addr, size_t new_size,
                                                 int initialized);
typedef void __itt_heap_reallocate_begin_t(__itt_heap_function h, void *addr,
                                           size_t new_size, int initialized);
__itt_heap_reallocate_begin_t *__itt_heap_reallocate_begin_ptr__3_0 =
    __itt_heap_reallocate_begin_init_3_0;
static void __itt_heap_reallocate_begin_init_3_0(__itt_heap_function h,
                                                 void *addr, size_t new_size,
                                                 int initialized) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_reallocate_begin_ptr__3_0 &&
      __itt_heap_reallocate_begin_ptr__3_0 !=
          __itt_heap_reallocate_begin_init_3_0)
    __itt_heap_reallocate_begin_ptr__3_0(h, addr, new_size, initialized);
  else
    return;
}
static void __itt_heap_reallocate_end_init_3_0(__itt_heap_function h,
                                               void *addr, void **new_addr,
                                               size_t new_size,
                                               int initialized);
typedef void __itt_heap_reallocate_end_t(__itt_heap_function h, void *addr,
                                         void **new_addr, size_t new_size,
                                         int initialized);
__itt_heap_reallocate_end_t *__itt_heap_reallocate_end_ptr__3_0 =
    __itt_heap_reallocate_end_init_3_0;
static void __itt_heap_reallocate_end_init_3_0(__itt_heap_function h,
                                               void *addr, void **new_addr,
                                               size_t new_size,
                                               int initialized) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_reallocate_end_ptr__3_0 &&
      __itt_heap_reallocate_end_ptr__3_0 != __itt_heap_reallocate_end_init_3_0)
    __itt_heap_reallocate_end_ptr__3_0(h, addr, new_addr, new_size,
                                       initialized);
  else
    return;
}
static void __itt_heap_internal_access_begin_init_3_0(void);
typedef void __itt_heap_internal_access_begin_t(void);
__itt_heap_internal_access_begin_t *__itt_heap_internal_access_begin_ptr__3_0 =
    __itt_heap_internal_access_begin_init_3_0;
static void __itt_heap_internal_access_begin_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_internal_access_begin_ptr__3_0 &&
      __itt_heap_internal_access_begin_ptr__3_0 !=
          __itt_heap_internal_access_begin_init_3_0)
    __itt_heap_internal_access_begin_ptr__3_0();
  else
    return;
}
static void __itt_heap_internal_access_end_init_3_0(void);
typedef void __itt_heap_internal_access_end_t(void);
__itt_heap_internal_access_end_t *__itt_heap_internal_access_end_ptr__3_0 =
    __itt_heap_internal_access_end_init_3_0;
static void __itt_heap_internal_access_end_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_internal_access_end_ptr__3_0 &&
      __itt_heap_internal_access_end_ptr__3_0 !=
          __itt_heap_internal_access_end_init_3_0)
    __itt_heap_internal_access_end_ptr__3_0();
  else
    return;
}
static void __itt_heap_record_memory_growth_begin_init_3_0(void);
typedef void __itt_heap_record_memory_growth_begin_t(void);
__itt_heap_record_memory_growth_begin_t
    *__itt_heap_record_memory_growth_begin_ptr__3_0 =
        __itt_heap_record_memory_growth_begin_init_3_0;
static void __itt_heap_record_memory_growth_begin_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_record_memory_growth_begin_ptr__3_0 &&
      __itt_heap_record_memory_growth_begin_ptr__3_0 !=
          __itt_heap_record_memory_growth_begin_init_3_0)
    __itt_heap_record_memory_growth_begin_ptr__3_0();
  else
    return;
}
static void __itt_heap_record_memory_growth_end_init_3_0(void);
typedef void __itt_heap_record_memory_growth_end_t(void);
__itt_heap_record_memory_growth_end_t
    *__itt_heap_record_memory_growth_end_ptr__3_0 =
        __itt_heap_record_memory_growth_end_init_3_0;
static void __itt_heap_record_memory_growth_end_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_record_memory_growth_end_ptr__3_0 &&
      __itt_heap_record_memory_growth_end_ptr__3_0 !=
          __itt_heap_record_memory_growth_end_init_3_0)
    __itt_heap_record_memory_growth_end_ptr__3_0();
  else
    return;
}
static void __itt_heap_reset_detection_init_3_0(unsigned int reset_mask);
typedef void __itt_heap_reset_detection_t(unsigned int reset_mask);
__itt_heap_reset_detection_t *__itt_heap_reset_detection_ptr__3_0 =
    __itt_heap_reset_detection_init_3_0;
static void __itt_heap_reset_detection_init_3_0(unsigned int reset_mask) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_reset_detection_ptr__3_0 &&
      __itt_heap_reset_detection_ptr__3_0 !=
          __itt_heap_reset_detection_init_3_0)
    __itt_heap_reset_detection_ptr__3_0(reset_mask);
  else
    return;
}
static void __itt_heap_record_init_3_0(unsigned int record_mask);
typedef void __itt_heap_record_t(unsigned int record_mask);
__itt_heap_record_t *__itt_heap_record_ptr__3_0 = __itt_heap_record_init_3_0;
static void __itt_heap_record_init_3_0(unsigned int record_mask) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_heap_record_ptr__3_0 &&
      __itt_heap_record_ptr__3_0 != __itt_heap_record_init_3_0)
    __itt_heap_record_ptr__3_0(record_mask);
  else
    return;
}

static void __itt_id_create_init_3_0(const __itt_domain *domain, __itt_id id);
typedef void __itt_id_create_t(const __itt_domain *domain, __itt_id id);
__itt_id_create_t *__itt_id_create_ptr__3_0 = __itt_id_create_init_3_0;
static void __itt_id_create_init_3_0(const __itt_domain *domain, __itt_id id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_id_create_ptr__3_0 &&
      __itt_id_create_ptr__3_0 != __itt_id_create_init_3_0)
    __itt_id_create_ptr__3_0(domain, id);
  else
    return;
}
static void __itt_id_destroy_init_3_0(const __itt_domain *domain, __itt_id id);
typedef void __itt_id_destroy_t(const __itt_domain *domain, __itt_id id);
__itt_id_destroy_t *__itt_id_destroy_ptr__3_0 = __itt_id_destroy_init_3_0;
static void __itt_id_destroy_init_3_0(const __itt_domain *domain, __itt_id id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_id_destroy_ptr__3_0 &&
      __itt_id_destroy_ptr__3_0 != __itt_id_destroy_init_3_0)
    __itt_id_destroy_ptr__3_0(domain, id);
  else
    return;
}

static __itt_timestamp __itt_get_timestamp_init_3_0(void);
typedef __itt_timestamp __itt_get_timestamp_t(void);
__itt_get_timestamp_t *__itt_get_timestamp_ptr__3_0 =
    __itt_get_timestamp_init_3_0;
static __itt_timestamp __itt_get_timestamp_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_get_timestamp_ptr__3_0 &&
      __itt_get_timestamp_ptr__3_0 != __itt_get_timestamp_init_3_0)
    return __itt_get_timestamp_ptr__3_0();
  else
    return (__itt_timestamp)0;
}

static void __itt_region_begin_init_3_0(const __itt_domain *domain, __itt_id id,
                                        __itt_id parent,
                                        __itt_string_handle *name);
typedef void __itt_region_begin_t(const __itt_domain *domain, __itt_id id,
                                  __itt_id parent, __itt_string_handle *name);
__itt_region_begin_t *__itt_region_begin_ptr__3_0 = __itt_region_begin_init_3_0;
static void __itt_region_begin_init_3_0(const __itt_domain *domain, __itt_id id,
                                        __itt_id parent,
                                        __itt_string_handle *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_region_begin_ptr__3_0 &&
      __itt_region_begin_ptr__3_0 != __itt_region_begin_init_3_0)
    __itt_region_begin_ptr__3_0(domain, id, parent, name);
  else
    return;
}
static void __itt_region_end_init_3_0(const __itt_domain *domain, __itt_id id);
typedef void __itt_region_end_t(const __itt_domain *domain, __itt_id id);
__itt_region_end_t *__itt_region_end_ptr__3_0 = __itt_region_end_init_3_0;
static void __itt_region_end_init_3_0(const __itt_domain *domain, __itt_id id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_region_end_ptr__3_0 &&
      __itt_region_end_ptr__3_0 != __itt_region_end_init_3_0)
    __itt_region_end_ptr__3_0(domain, id);
  else
    return;
}

static void __itt_frame_begin_v3_init_3_0(const __itt_domain *domain,
                                          __itt_id *id);
typedef void __itt_frame_begin_v3_t(const __itt_domain *domain, __itt_id *id);
__itt_frame_begin_v3_t *__itt_frame_begin_v3_ptr__3_0 =
    __itt_frame_begin_v3_init_3_0;
static void __itt_frame_begin_v3_init_3_0(const __itt_domain *domain,
                                          __itt_id *id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_frame_begin_v3_ptr__3_0 &&
      __itt_frame_begin_v3_ptr__3_0 != __itt_frame_begin_v3_init_3_0)
    __itt_frame_begin_v3_ptr__3_0(domain, id);
  else
    return;
}
static void __itt_frame_end_v3_init_3_0(const __itt_domain *domain,
                                        __itt_id *id);
typedef void __itt_frame_end_v3_t(const __itt_domain *domain, __itt_id *id);
__itt_frame_end_v3_t *__itt_frame_end_v3_ptr__3_0 = __itt_frame_end_v3_init_3_0;
static void __itt_frame_end_v3_init_3_0(const __itt_domain *domain,
                                        __itt_id *id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_frame_end_v3_ptr__3_0 &&
      __itt_frame_end_v3_ptr__3_0 != __itt_frame_end_v3_init_3_0)
    __itt_frame_end_v3_ptr__3_0(domain, id);
  else
    return;
}
static void __itt_frame_submit_v3_init_3_0(const __itt_domain *domain,
                                           __itt_id *id, __itt_timestamp begin,
                                           __itt_timestamp end);
typedef void __itt_frame_submit_v3_t(const __itt_domain *domain, __itt_id *id,
                                     __itt_timestamp begin,
                                     __itt_timestamp end);
__itt_frame_submit_v3_t *__itt_frame_submit_v3_ptr__3_0 =
    __itt_frame_submit_v3_init_3_0;
static void __itt_frame_submit_v3_init_3_0(const __itt_domain *domain,
                                           __itt_id *id, __itt_timestamp begin,
                                           __itt_timestamp end) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_frame_submit_v3_ptr__3_0 &&
      __itt_frame_submit_v3_ptr__3_0 != __itt_frame_submit_v3_init_3_0)
    __itt_frame_submit_v3_ptr__3_0(domain, id, begin, end);
  else
    return;
}

static void __itt_task_group_init_3_0(const __itt_domain *domain, __itt_id id,
                                      __itt_id parent,
                                      __itt_string_handle *name);
typedef void __itt_task_group_t(const __itt_domain *domain, __itt_id id,
                                __itt_id parent, __itt_string_handle *name);
__itt_task_group_t *__itt_task_group_ptr__3_0 = __itt_task_group_init_3_0;
static void __itt_task_group_init_3_0(const __itt_domain *domain, __itt_id id,
                                      __itt_id parent,
                                      __itt_string_handle *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_group_ptr__3_0 &&
      __itt_task_group_ptr__3_0 != __itt_task_group_init_3_0)
    __itt_task_group_ptr__3_0(domain, id, parent, name);
  else
    return;
}

static void __itt_task_begin_init_3_0(const __itt_domain *domain, __itt_id id,
                                      __itt_id parent,
                                      __itt_string_handle *name);
typedef void __itt_task_begin_t(const __itt_domain *domain, __itt_id id,
                                __itt_id parent, __itt_string_handle *name);
__itt_task_begin_t *__itt_task_begin_ptr__3_0 = __itt_task_begin_init_3_0;
static void __itt_task_begin_init_3_0(const __itt_domain *domain, __itt_id id,
                                      __itt_id parent,
                                      __itt_string_handle *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_begin_ptr__3_0 &&
      __itt_task_begin_ptr__3_0 != __itt_task_begin_init_3_0)
    __itt_task_begin_ptr__3_0(domain, id, parent, name);
  else
    return;
}
static void __itt_task_begin_fn_init_3_0(const __itt_domain *domain,
                                         __itt_id id, __itt_id parent,
                                         void *fn);
typedef void __itt_task_begin_fn_t(const __itt_domain *domain, __itt_id id,
                                   __itt_id parent, void *fn);
__itt_task_begin_fn_t *__itt_task_begin_fn_ptr__3_0 =
    __itt_task_begin_fn_init_3_0;
static void __itt_task_begin_fn_init_3_0(const __itt_domain *domain,
                                         __itt_id id, __itt_id parent,
                                         void *fn) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_begin_fn_ptr__3_0 &&
      __itt_task_begin_fn_ptr__3_0 != __itt_task_begin_fn_init_3_0)
    __itt_task_begin_fn_ptr__3_0(domain, id, parent, fn);
  else
    return;
}
static void __itt_task_end_init_3_0(const __itt_domain *domain);
typedef void __itt_task_end_t(const __itt_domain *domain);
__itt_task_end_t *__itt_task_end_ptr__3_0 = __itt_task_end_init_3_0;
static void __itt_task_end_init_3_0(const __itt_domain *domain) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_end_ptr__3_0 &&
      __itt_task_end_ptr__3_0 != __itt_task_end_init_3_0)
    __itt_task_end_ptr__3_0(domain);
  else
    return;
}

static void __itt_counter_inc_v3_init_3_0(const __itt_domain *domain,
                                          __itt_string_handle *name);
typedef void __itt_counter_inc_v3_t(const __itt_domain *domain,
                                    __itt_string_handle *name);
__itt_counter_inc_v3_t *__itt_counter_inc_v3_ptr__3_0 =
    __itt_counter_inc_v3_init_3_0;
static void __itt_counter_inc_v3_init_3_0(const __itt_domain *domain,
                                          __itt_string_handle *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_inc_v3_ptr__3_0 &&
      __itt_counter_inc_v3_ptr__3_0 != __itt_counter_inc_v3_init_3_0)
    __itt_counter_inc_v3_ptr__3_0(domain, name);
  else
    return;
}
static void __itt_counter_inc_delta_v3_init_3_0(const __itt_domain *domain,
                                                __itt_string_handle *name,
                                                unsigned long long value);
typedef void __itt_counter_inc_delta_v3_t(const __itt_domain *domain,
                                          __itt_string_handle *name,
                                          unsigned long long value);
__itt_counter_inc_delta_v3_t *__itt_counter_inc_delta_v3_ptr__3_0 =
    __itt_counter_inc_delta_v3_init_3_0;
static void __itt_counter_inc_delta_v3_init_3_0(const __itt_domain *domain,
                                                __itt_string_handle *name,
                                                unsigned long long value) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_inc_delta_v3_ptr__3_0 &&
      __itt_counter_inc_delta_v3_ptr__3_0 !=
          __itt_counter_inc_delta_v3_init_3_0)
    __itt_counter_inc_delta_v3_ptr__3_0(domain, name, value);
  else
    return;
}
static void __itt_counter_dec_v3_init_3_0(const __itt_domain *domain,
                                          __itt_string_handle *name);
typedef void __itt_counter_dec_v3_t(const __itt_domain *domain,
                                    __itt_string_handle *name);
__itt_counter_dec_v3_t *__itt_counter_dec_v3_ptr__3_0 =
    __itt_counter_dec_v3_init_3_0;
static void __itt_counter_dec_v3_init_3_0(const __itt_domain *domain,
                                          __itt_string_handle *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_dec_v3_ptr__3_0 &&
      __itt_counter_dec_v3_ptr__3_0 != __itt_counter_dec_v3_init_3_0)
    __itt_counter_dec_v3_ptr__3_0(domain, name);
  else
    return;
}
static void __itt_counter_dec_delta_v3_init_3_0(const __itt_domain *domain,
                                                __itt_string_handle *name,
                                                unsigned long long value);
typedef void __itt_counter_dec_delta_v3_t(const __itt_domain *domain,
                                          __itt_string_handle *name,
                                          unsigned long long value);
__itt_counter_dec_delta_v3_t *__itt_counter_dec_delta_v3_ptr__3_0 =
    __itt_counter_dec_delta_v3_init_3_0;
static void __itt_counter_dec_delta_v3_init_3_0(const __itt_domain *domain,
                                                __itt_string_handle *name,
                                                unsigned long long value) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_dec_delta_v3_ptr__3_0 &&
      __itt_counter_dec_delta_v3_ptr__3_0 !=
          __itt_counter_dec_delta_v3_init_3_0)
    __itt_counter_dec_delta_v3_ptr__3_0(domain, name, value);
  else
    return;
}

static void __itt_marker_init_3_0(const __itt_domain *domain, __itt_id id,
                                  __itt_string_handle *name, __itt_scope scope);
typedef void __itt_marker_t(const __itt_domain *domain, __itt_id id,
                            __itt_string_handle *name, __itt_scope scope);
__itt_marker_t *__itt_marker_ptr__3_0 = __itt_marker_init_3_0;
static void __itt_marker_init_3_0(const __itt_domain *domain, __itt_id id,
                                  __itt_string_handle *name,
                                  __itt_scope scope) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_marker_ptr__3_0 && __itt_marker_ptr__3_0 != __itt_marker_init_3_0)
    __itt_marker_ptr__3_0(domain, id, name, scope);
  else
    return;
}

static void __itt_metadata_add_init_3_0(const __itt_domain *domain, __itt_id id,
                                        __itt_string_handle *key,
                                        __itt_metadata_type type, size_t count,
                                        void *data);
typedef void __itt_metadata_add_t(const __itt_domain *domain, __itt_id id,
                                  __itt_string_handle *key,
                                  __itt_metadata_type type, size_t count,
                                  void *data);
__itt_metadata_add_t *__itt_metadata_add_ptr__3_0 = __itt_metadata_add_init_3_0;
static void __itt_metadata_add_init_3_0(const __itt_domain *domain, __itt_id id,
                                        __itt_string_handle *key,
                                        __itt_metadata_type type, size_t count,
                                        void *data) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_metadata_add_ptr__3_0 &&
      __itt_metadata_add_ptr__3_0 != __itt_metadata_add_init_3_0)
    __itt_metadata_add_ptr__3_0(domain, id, key, type, count, data);
  else
    return;
}

static void __itt_metadata_str_add_init_3_0(const __itt_domain *domain,
                                            __itt_id id,
                                            __itt_string_handle *key,
                                            const char *data, size_t length);
typedef void __itt_metadata_str_add_t(const __itt_domain *domain, __itt_id id,
                                      __itt_string_handle *key,
                                      const char *data, size_t length);
__itt_metadata_str_add_t *__itt_metadata_str_add_ptr__3_0 =
    __itt_metadata_str_add_init_3_0;
static void __itt_metadata_str_add_init_3_0(const __itt_domain *domain,
                                            __itt_id id,
                                            __itt_string_handle *key,
                                            const char *data, size_t length) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_metadata_str_add_ptr__3_0 &&
      __itt_metadata_str_add_ptr__3_0 != __itt_metadata_str_add_init_3_0)
    __itt_metadata_str_add_ptr__3_0(domain, id, key, data, length);
  else
    return;
}

static void __itt_relation_add_to_current_init_3_0(const __itt_domain *domain,
                                                   __itt_relation relation,
                                                   __itt_id tail);
typedef void __itt_relation_add_to_current_t(const __itt_domain *domain,
                                             __itt_relation relation,
                                             __itt_id tail);
__itt_relation_add_to_current_t *__itt_relation_add_to_current_ptr__3_0 =
    __itt_relation_add_to_current_init_3_0;
static void __itt_relation_add_to_current_init_3_0(const __itt_domain *domain,
                                                   __itt_relation relation,
                                                   __itt_id tail) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_relation_add_to_current_ptr__3_0 &&
      __itt_relation_add_to_current_ptr__3_0 !=
          __itt_relation_add_to_current_init_3_0)
    __itt_relation_add_to_current_ptr__3_0(domain, relation, tail);
  else
    return;
}
static void __itt_relation_add_init_3_0(const __itt_domain *domain,
                                        __itt_id head, __itt_relation relation,
                                        __itt_id tail);
typedef void __itt_relation_add_t(const __itt_domain *domain, __itt_id head,
                                  __itt_relation relation, __itt_id tail);
__itt_relation_add_t *__itt_relation_add_ptr__3_0 = __itt_relation_add_init_3_0;
static void __itt_relation_add_init_3_0(const __itt_domain *domain,
                                        __itt_id head, __itt_relation relation,
                                        __itt_id tail) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_relation_add_ptr__3_0 &&
      __itt_relation_add_ptr__3_0 != __itt_relation_add_init_3_0)
    __itt_relation_add_ptr__3_0(domain, head, relation, tail);
  else
    return;
}

static __itt_event __itt_event_create_init_3_0(const char *name, int namelen);
typedef __itt_event __itt_event_create_t(const char *name, int namelen);
__itt_event_create_t *__itt_event_create_ptr__3_0 = __itt_event_create_init_3_0;
static __itt_event __itt_event_create_init_3_0(const char *name, int namelen) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_event_create_ptr__3_0 &&
      __itt_event_create_ptr__3_0 != __itt_event_create_init_3_0)
    return __itt_event_create_ptr__3_0(name, namelen);
  else
    return (__itt_event)0;
}

static int __itt_event_start_init_3_0(__itt_event event);
typedef int __itt_event_start_t(__itt_event event);
__itt_event_start_t *__itt_event_start_ptr__3_0 = __itt_event_start_init_3_0;
static int __itt_event_start_init_3_0(__itt_event event) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_event_start_ptr__3_0 &&
      __itt_event_start_ptr__3_0 != __itt_event_start_init_3_0)
    return __itt_event_start_ptr__3_0(event);
  else
    return (int)0;
}
static int __itt_event_end_init_3_0(__itt_event event);
typedef int __itt_event_end_t(__itt_event event);
__itt_event_end_t *__itt_event_end_ptr__3_0 = __itt_event_end_init_3_0;
static int __itt_event_end_init_3_0(__itt_event event) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_event_end_ptr__3_0 &&
      __itt_event_end_ptr__3_0 != __itt_event_end_init_3_0)
    return __itt_event_end_ptr__3_0(event);
  else
    return (int)0;
}

static void __itt_sync_set_name_init_3_0(void *addr, const char *objtype,
                                         const char *objname, int attribute);
typedef void __itt_sync_set_name_t(void *addr, const char *objtype,
                                   const char *objname, int attribute);
__itt_sync_set_name_t *__itt_sync_set_name_ptr__3_0 =
    __itt_sync_set_name_init_3_0;
static void __itt_sync_set_name_init_3_0(void *addr, const char *objtype,
                                         const char *objname, int attribute) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_sync_set_name_ptr__3_0 &&
      __itt_sync_set_name_ptr__3_0 != __itt_sync_set_name_init_3_0)
    __itt_sync_set_name_ptr__3_0(addr, objtype, objname, attribute);
  else
    return;
}

static int __itt_notify_sync_name_init_3_0(void *p, const char *objtype,
                                           int typelen, const char *objname,
                                           int namelen, int attribute);
typedef int __itt_notify_sync_name_t(void *p, const char *objtype, int typelen,
                                     const char *objname, int namelen,
                                     int attribute);
__itt_notify_sync_name_t *__itt_notify_sync_name_ptr__3_0 =
    __itt_notify_sync_name_init_3_0;
static int __itt_notify_sync_name_init_3_0(void *p, const char *objtype,
                                           int typelen, const char *objname,
                                           int namelen, int attribute) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_notify_sync_name_ptr__3_0 &&
      __itt_notify_sync_name_ptr__3_0 != __itt_notify_sync_name_init_3_0)
    return __itt_notify_sync_name_ptr__3_0(p, objtype, typelen, objname,
                                           namelen, attribute);
  else
    return (int)0;
}

static void __itt_notify_sync_prepare_init_3_0(void *p);
typedef void __itt_notify_sync_prepare_t(void *p);
__itt_notify_sync_prepare_t *__itt_notify_sync_prepare_ptr__3_0 =
    __itt_notify_sync_prepare_init_3_0;
static void __itt_notify_sync_prepare_init_3_0(void *p) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_notify_sync_prepare_ptr__3_0 &&
      __itt_notify_sync_prepare_ptr__3_0 != __itt_notify_sync_prepare_init_3_0)
    __itt_notify_sync_prepare_ptr__3_0(p);
  else
    return;
}
static void __itt_notify_sync_cancel_init_3_0(void *p);
typedef void __itt_notify_sync_cancel_t(void *p);
__itt_notify_sync_cancel_t *__itt_notify_sync_cancel_ptr__3_0 =
    __itt_notify_sync_cancel_init_3_0;
static void __itt_notify_sync_cancel_init_3_0(void *p) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_notify_sync_cancel_ptr__3_0 &&
      __itt_notify_sync_cancel_ptr__3_0 != __itt_notify_sync_cancel_init_3_0)
    __itt_notify_sync_cancel_ptr__3_0(p);
  else
    return;
}
static void __itt_notify_sync_acquired_init_3_0(void *p);
typedef void __itt_notify_sync_acquired_t(void *p);
__itt_notify_sync_acquired_t *__itt_notify_sync_acquired_ptr__3_0 =
    __itt_notify_sync_acquired_init_3_0;
static void __itt_notify_sync_acquired_init_3_0(void *p) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_notify_sync_acquired_ptr__3_0 &&
      __itt_notify_sync_acquired_ptr__3_0 !=
          __itt_notify_sync_acquired_init_3_0)
    __itt_notify_sync_acquired_ptr__3_0(p);
  else
    return;
}
static void __itt_notify_sync_releasing_init_3_0(void *p);
typedef void __itt_notify_sync_releasing_t(void *p);
__itt_notify_sync_releasing_t *__itt_notify_sync_releasing_ptr__3_0 =
    __itt_notify_sync_releasing_init_3_0;
static void __itt_notify_sync_releasing_init_3_0(void *p) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_notify_sync_releasing_ptr__3_0 &&
      __itt_notify_sync_releasing_ptr__3_0 !=
          __itt_notify_sync_releasing_init_3_0)
    __itt_notify_sync_releasing_ptr__3_0(p);
  else
    return;
}

static void __itt_memory_read_init_3_0(void *addr, size_t size);
typedef void __itt_memory_read_t(void *addr, size_t size);
__itt_memory_read_t *__itt_memory_read_ptr__3_0 = __itt_memory_read_init_3_0;
static void __itt_memory_read_init_3_0(void *addr, size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_memory_read_ptr__3_0 &&
      __itt_memory_read_ptr__3_0 != __itt_memory_read_init_3_0)
    __itt_memory_read_ptr__3_0(addr, size);
  else
    return;
}
static void __itt_memory_write_init_3_0(void *addr, size_t size);
typedef void __itt_memory_write_t(void *addr, size_t size);
__itt_memory_write_t *__itt_memory_write_ptr__3_0 = __itt_memory_write_init_3_0;
static void __itt_memory_write_init_3_0(void *addr, size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_memory_write_ptr__3_0 &&
      __itt_memory_write_ptr__3_0 != __itt_memory_write_init_3_0)
    __itt_memory_write_ptr__3_0(addr, size);
  else
    return;
}
static void __itt_memory_update_init_3_0(void *addr, size_t size);
typedef void __itt_memory_update_t(void *addr, size_t size);
__itt_memory_update_t *__itt_memory_update_ptr__3_0 =
    __itt_memory_update_init_3_0;
static void __itt_memory_update_init_3_0(void *addr, size_t size) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_memory_update_ptr__3_0 &&
      __itt_memory_update_ptr__3_0 != __itt_memory_update_init_3_0)
    __itt_memory_update_ptr__3_0(addr, size);
  else
    return;
}

static __itt_state_t __itt_state_get_init_3_0(void);
typedef __itt_state_t __itt_state_get_t(void);
__itt_state_get_t *__itt_state_get_ptr__3_0 = __itt_state_get_init_3_0;
static __itt_state_t __itt_state_get_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_state_get_ptr__3_0 &&
      __itt_state_get_ptr__3_0 != __itt_state_get_init_3_0)
    return __itt_state_get_ptr__3_0();
  else
    return (__itt_state_t)0;
}
static __itt_state_t __itt_state_set_init_3_0(__itt_state_t s);
typedef __itt_state_t __itt_state_set_t(__itt_state_t s);
__itt_state_set_t *__itt_state_set_ptr__3_0 = __itt_state_set_init_3_0;
static __itt_state_t __itt_state_set_init_3_0(__itt_state_t s) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_state_set_ptr__3_0 &&
      __itt_state_set_ptr__3_0 != __itt_state_set_init_3_0)
    return __itt_state_set_ptr__3_0(s);
  else
    return (__itt_state_t)0;
}
static __itt_obj_state_t __itt_obj_mode_set_init_3_0(__itt_obj_prop_t p,
                                                     __itt_obj_state_t s);
typedef __itt_obj_state_t __itt_obj_mode_set_t(__itt_obj_prop_t p,
                                               __itt_obj_state_t s);
__itt_obj_mode_set_t *__itt_obj_mode_set_ptr__3_0 = __itt_obj_mode_set_init_3_0;
static __itt_obj_state_t __itt_obj_mode_set_init_3_0(__itt_obj_prop_t p,
                                                     __itt_obj_state_t s) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_obj_mode_set_ptr__3_0 &&
      __itt_obj_mode_set_ptr__3_0 != __itt_obj_mode_set_init_3_0)
    return __itt_obj_mode_set_ptr__3_0(p, s);
  else
    return (__itt_obj_state_t)0;
}
static __itt_thr_state_t __itt_thr_mode_set_init_3_0(__itt_thr_prop_t p,
                                                     __itt_thr_state_t s);
typedef __itt_thr_state_t __itt_thr_mode_set_t(__itt_thr_prop_t p,
                                               __itt_thr_state_t s);
__itt_thr_mode_set_t *__itt_thr_mode_set_ptr__3_0 = __itt_thr_mode_set_init_3_0;
static __itt_thr_state_t __itt_thr_mode_set_init_3_0(__itt_thr_prop_t p,
                                                     __itt_thr_state_t s) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_thr_mode_set_ptr__3_0 &&
      __itt_thr_mode_set_ptr__3_0 != __itt_thr_mode_set_init_3_0)
    return __itt_thr_mode_set_ptr__3_0(p, s);
  else
    return (__itt_thr_state_t)0;
}

static __itt_frame __itt_frame_create_init_3_0(const char *domain);
typedef __itt_frame __itt_frame_create_t(const char *domain);
__itt_frame_create_t *__itt_frame_create_ptr__3_0 = __itt_frame_create_init_3_0;
static __itt_frame __itt_frame_create_init_3_0(const char *domain) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_frame_create_ptr__3_0 &&
      __itt_frame_create_ptr__3_0 != __itt_frame_create_init_3_0)
    return __itt_frame_create_ptr__3_0(domain);
  else
    return (__itt_frame)0;
}

static __itt_pt_region __itt_pt_region_create_init_3_0(const char *name);
typedef __itt_pt_region __itt_pt_region_create_t(const char *name);
__itt_pt_region_create_t *__itt_pt_region_create_ptr__3_0 =
    __itt_pt_region_create_init_3_0;
static __itt_pt_region __itt_pt_region_create_init_3_0(const char *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_pt_region_create_ptr__3_0 &&
      __itt_pt_region_create_ptr__3_0 != __itt_pt_region_create_init_3_0)
    return __itt_pt_region_create_ptr__3_0(name);
  else
    return (__itt_pt_region)0;
}

static void __itt_frame_begin_init_3_0(__itt_frame frame);
typedef void __itt_frame_begin_t(__itt_frame frame);
__itt_frame_begin_t *__itt_frame_begin_ptr__3_0 = __itt_frame_begin_init_3_0;
static void __itt_frame_begin_init_3_0(__itt_frame frame) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_frame_begin_ptr__3_0 &&
      __itt_frame_begin_ptr__3_0 != __itt_frame_begin_init_3_0)
    __itt_frame_begin_ptr__3_0(frame);
  else
    return;
}
static void __itt_frame_end_init_3_0(__itt_frame frame);
typedef void __itt_frame_end_t(__itt_frame frame);
__itt_frame_end_t *__itt_frame_end_ptr__3_0 = __itt_frame_end_init_3_0;
static void __itt_frame_end_init_3_0(__itt_frame frame) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_frame_end_ptr__3_0 &&
      __itt_frame_end_ptr__3_0 != __itt_frame_end_init_3_0)
    __itt_frame_end_ptr__3_0(frame);
  else
    return;
}

static void __itt_counter_destroy_init_3_0(__itt_counter id);
typedef void __itt_counter_destroy_t(__itt_counter id);
__itt_counter_destroy_t *__itt_counter_destroy_ptr__3_0 =
    __itt_counter_destroy_init_3_0;
static void __itt_counter_destroy_init_3_0(__itt_counter id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_destroy_ptr__3_0 &&
      __itt_counter_destroy_ptr__3_0 != __itt_counter_destroy_init_3_0)
    __itt_counter_destroy_ptr__3_0(id);
  else
    return;
}
static void __itt_counter_inc_init_3_0(__itt_counter id);
typedef void __itt_counter_inc_t(__itt_counter id);
__itt_counter_inc_t *__itt_counter_inc_ptr__3_0 = __itt_counter_inc_init_3_0;
static void __itt_counter_inc_init_3_0(__itt_counter id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_inc_ptr__3_0 &&
      __itt_counter_inc_ptr__3_0 != __itt_counter_inc_init_3_0)
    __itt_counter_inc_ptr__3_0(id);
  else
    return;
}
static void __itt_counter_inc_delta_init_3_0(__itt_counter id,
                                             unsigned long long value);
typedef void __itt_counter_inc_delta_t(__itt_counter id,
                                       unsigned long long value);
__itt_counter_inc_delta_t *__itt_counter_inc_delta_ptr__3_0 =
    __itt_counter_inc_delta_init_3_0;
static void __itt_counter_inc_delta_init_3_0(__itt_counter id,
                                             unsigned long long value) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_inc_delta_ptr__3_0 &&
      __itt_counter_inc_delta_ptr__3_0 != __itt_counter_inc_delta_init_3_0)
    __itt_counter_inc_delta_ptr__3_0(id, value);
  else
    return;
}
static void __itt_counter_dec_init_3_0(__itt_counter id);
typedef void __itt_counter_dec_t(__itt_counter id);
__itt_counter_dec_t *__itt_counter_dec_ptr__3_0 = __itt_counter_dec_init_3_0;
static void __itt_counter_dec_init_3_0(__itt_counter id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_dec_ptr__3_0 &&
      __itt_counter_dec_ptr__3_0 != __itt_counter_dec_init_3_0)
    __itt_counter_dec_ptr__3_0(id);
  else
    return;
}
static void __itt_counter_dec_delta_init_3_0(__itt_counter id,
                                             unsigned long long value);
typedef void __itt_counter_dec_delta_t(__itt_counter id,
                                       unsigned long long value);
__itt_counter_dec_delta_t *__itt_counter_dec_delta_ptr__3_0 =
    __itt_counter_dec_delta_init_3_0;
static void __itt_counter_dec_delta_init_3_0(__itt_counter id,
                                             unsigned long long value) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_dec_delta_ptr__3_0 &&
      __itt_counter_dec_delta_ptr__3_0 != __itt_counter_dec_delta_init_3_0)
    __itt_counter_dec_delta_ptr__3_0(id, value);
  else
    return;
}
static void __itt_counter_set_value_init_3_0(__itt_counter id, void *value_ptr);
typedef void __itt_counter_set_value_t(__itt_counter id, void *value_ptr);
__itt_counter_set_value_t *__itt_counter_set_value_ptr__3_0 =
    __itt_counter_set_value_init_3_0;
static void __itt_counter_set_value_init_3_0(__itt_counter id,
                                             void *value_ptr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_set_value_ptr__3_0 &&
      __itt_counter_set_value_ptr__3_0 != __itt_counter_set_value_init_3_0)
    __itt_counter_set_value_ptr__3_0(id, value_ptr);
  else
    return;
}
static void __itt_counter_set_value_ex_init_3_0(
    __itt_counter id, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, void *value_ptr);
typedef void __itt_counter_set_value_ex_t(__itt_counter id,
                                          __itt_clock_domain *clock_domain,
                                          unsigned long long timestamp,
                                          void *value_ptr);
__itt_counter_set_value_ex_t *__itt_counter_set_value_ex_ptr__3_0 =
    __itt_counter_set_value_ex_init_3_0;
static void __itt_counter_set_value_ex_init_3_0(
    __itt_counter id, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, void *value_ptr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_counter_set_value_ex_ptr__3_0 &&
      __itt_counter_set_value_ex_ptr__3_0 !=
          __itt_counter_set_value_ex_init_3_0)
    __itt_counter_set_value_ex_ptr__3_0(id, clock_domain, timestamp, value_ptr);
  else
    return;
}

static __itt_mark_type __itt_mark_create_init_3_0(const char *name);
typedef __itt_mark_type __itt_mark_create_t(const char *name);
__itt_mark_create_t *__itt_mark_create_ptr__3_0 = __itt_mark_create_init_3_0;
static __itt_mark_type __itt_mark_create_init_3_0(const char *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_mark_create_ptr__3_0 &&
      __itt_mark_create_ptr__3_0 != __itt_mark_create_init_3_0)
    return __itt_mark_create_ptr__3_0(name);
  else
    return (__itt_mark_type)0;
}

static int __itt_mark_init_3_0(__itt_mark_type mt, const char *parameter);
typedef int __itt_mark_t(__itt_mark_type mt, const char *parameter);
__itt_mark_t *__itt_mark_ptr__3_0 = __itt_mark_init_3_0;
static int __itt_mark_init_3_0(__itt_mark_type mt, const char *parameter) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_mark_ptr__3_0 && __itt_mark_ptr__3_0 != __itt_mark_init_3_0)
    return __itt_mark_ptr__3_0(mt, parameter);
  else
    return (int)0;
}

static int __itt_mark_off_init_3_0(__itt_mark_type mt);
typedef int __itt_mark_off_t(__itt_mark_type mt);
__itt_mark_off_t *__itt_mark_off_ptr__3_0 = __itt_mark_off_init_3_0;
static int __itt_mark_off_init_3_0(__itt_mark_type mt) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_mark_off_ptr__3_0 &&
      __itt_mark_off_ptr__3_0 != __itt_mark_off_init_3_0)
    return __itt_mark_off_ptr__3_0(mt);
  else
    return (int)0;
}

static int __itt_mark_global_init_3_0(__itt_mark_type mt,
                                      const char *parameter);
typedef int __itt_mark_global_t(__itt_mark_type mt, const char *parameter);
__itt_mark_global_t *__itt_mark_global_ptr__3_0 = __itt_mark_global_init_3_0;
static int __itt_mark_global_init_3_0(__itt_mark_type mt,
                                      const char *parameter) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_mark_global_ptr__3_0 &&
      __itt_mark_global_ptr__3_0 != __itt_mark_global_init_3_0)
    return __itt_mark_global_ptr__3_0(mt, parameter);
  else
    return (int)0;
}

static int __itt_mark_global_off_init_3_0(__itt_mark_type mt);
typedef int __itt_mark_global_off_t(__itt_mark_type mt);
__itt_mark_global_off_t *__itt_mark_global_off_ptr__3_0 =
    __itt_mark_global_off_init_3_0;
static int __itt_mark_global_off_init_3_0(__itt_mark_type mt) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_mark_global_off_ptr__3_0 &&
      __itt_mark_global_off_ptr__3_0 != __itt_mark_global_off_init_3_0)
    return __itt_mark_global_off_ptr__3_0(mt);
  else
    return (int)0;
}

static __itt_caller __itt_stack_caller_create_init_3_0(void);
typedef __itt_caller __itt_stack_caller_create_t(void);
__itt_stack_caller_create_t *__itt_stack_caller_create_ptr__3_0 =
    __itt_stack_caller_create_init_3_0;
static __itt_caller __itt_stack_caller_create_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_stack_caller_create_ptr__3_0 &&
      __itt_stack_caller_create_ptr__3_0 != __itt_stack_caller_create_init_3_0)
    return __itt_stack_caller_create_ptr__3_0();
  else
    return (__itt_caller)0;
}

static void __itt_stack_caller_destroy_init_3_0(__itt_caller id);
typedef void __itt_stack_caller_destroy_t(__itt_caller id);
__itt_stack_caller_destroy_t *__itt_stack_caller_destroy_ptr__3_0 =
    __itt_stack_caller_destroy_init_3_0;
static void __itt_stack_caller_destroy_init_3_0(__itt_caller id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_stack_caller_destroy_ptr__3_0 &&
      __itt_stack_caller_destroy_ptr__3_0 !=
          __itt_stack_caller_destroy_init_3_0)
    __itt_stack_caller_destroy_ptr__3_0(id);
  else
    return;
}
static void __itt_stack_callee_enter_init_3_0(__itt_caller id);
typedef void __itt_stack_callee_enter_t(__itt_caller id);
__itt_stack_callee_enter_t *__itt_stack_callee_enter_ptr__3_0 =
    __itt_stack_callee_enter_init_3_0;
static void __itt_stack_callee_enter_init_3_0(__itt_caller id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_stack_callee_enter_ptr__3_0 &&
      __itt_stack_callee_enter_ptr__3_0 != __itt_stack_callee_enter_init_3_0)
    __itt_stack_callee_enter_ptr__3_0(id);
  else
    return;
}
static void __itt_stack_callee_leave_init_3_0(__itt_caller id);
typedef void __itt_stack_callee_leave_t(__itt_caller id);
__itt_stack_callee_leave_t *__itt_stack_callee_leave_ptr__3_0 =
    __itt_stack_callee_leave_init_3_0;
static void __itt_stack_callee_leave_init_3_0(__itt_caller id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_stack_callee_leave_ptr__3_0 &&
      __itt_stack_callee_leave_ptr__3_0 != __itt_stack_callee_leave_init_3_0)
    __itt_stack_callee_leave_ptr__3_0(id);
  else
    return;
}

static __itt_clock_domain *
__itt_clock_domain_create_init_3_0(__itt_get_clock_info_fn fn, void *fn_data);
typedef __itt_clock_domain *
__itt_clock_domain_create_t(__itt_get_clock_info_fn fn, void *fn_data);
__itt_clock_domain_create_t *__itt_clock_domain_create_ptr__3_0 =
    __itt_clock_domain_create_init_3_0;
static __itt_clock_domain *
__itt_clock_domain_create_init_3_0(__itt_get_clock_info_fn fn, void *fn_data) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_clock_domain_create_ptr__3_0 &&
      __itt_clock_domain_create_ptr__3_0 != __itt_clock_domain_create_init_3_0)
    return __itt_clock_domain_create_ptr__3_0(fn, fn_data);
  else
    return (__itt_clock_domain *)0;
}
static void __itt_clock_domain_reset_init_3_0(void);
typedef void __itt_clock_domain_reset_t(void);
__itt_clock_domain_reset_t *__itt_clock_domain_reset_ptr__3_0 =
    __itt_clock_domain_reset_init_3_0;
static void __itt_clock_domain_reset_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_clock_domain_reset_ptr__3_0 &&
      __itt_clock_domain_reset_ptr__3_0 != __itt_clock_domain_reset_init_3_0)
    __itt_clock_domain_reset_ptr__3_0();
  else
    return;
}
static void __itt_id_create_ex_init_3_0(const __itt_domain *domain,
                                        __itt_clock_domain *clock_domain,
                                        unsigned long long timestamp,
                                        __itt_id id);
typedef void __itt_id_create_ex_t(const __itt_domain *domain,
                                  __itt_clock_domain *clock_domain,
                                  unsigned long long timestamp, __itt_id id);
__itt_id_create_ex_t *__itt_id_create_ex_ptr__3_0 = __itt_id_create_ex_init_3_0;
static void __itt_id_create_ex_init_3_0(const __itt_domain *domain,
                                        __itt_clock_domain *clock_domain,
                                        unsigned long long timestamp,
                                        __itt_id id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_id_create_ex_ptr__3_0 &&
      __itt_id_create_ex_ptr__3_0 != __itt_id_create_ex_init_3_0)
    __itt_id_create_ex_ptr__3_0(domain, clock_domain, timestamp, id);
  else
    return;
}
static void __itt_id_destroy_ex_init_3_0(const __itt_domain *domain,
                                         __itt_clock_domain *clock_domain,
                                         unsigned long long timestamp,
                                         __itt_id id);
typedef void __itt_id_destroy_ex_t(const __itt_domain *domain,
                                   __itt_clock_domain *clock_domain,
                                   unsigned long long timestamp, __itt_id id);
__itt_id_destroy_ex_t *__itt_id_destroy_ex_ptr__3_0 =
    __itt_id_destroy_ex_init_3_0;
static void __itt_id_destroy_ex_init_3_0(const __itt_domain *domain,
                                         __itt_clock_domain *clock_domain,
                                         unsigned long long timestamp,
                                         __itt_id id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_id_destroy_ex_ptr__3_0 &&
      __itt_id_destroy_ex_ptr__3_0 != __itt_id_destroy_ex_init_3_0)
    __itt_id_destroy_ex_ptr__3_0(domain, clock_domain, timestamp, id);
  else
    return;
}
static void __itt_task_begin_ex_init_3_0(const __itt_domain *domain,
                                         __itt_clock_domain *clock_domain,
                                         unsigned long long timestamp,
                                         __itt_id id, __itt_id parentid,
                                         __itt_string_handle *name);
typedef void __itt_task_begin_ex_t(const __itt_domain *domain,
                                   __itt_clock_domain *clock_domain,
                                   unsigned long long timestamp, __itt_id id,
                                   __itt_id parentid,
                                   __itt_string_handle *name);
__itt_task_begin_ex_t *__itt_task_begin_ex_ptr__3_0 =
    __itt_task_begin_ex_init_3_0;
static void __itt_task_begin_ex_init_3_0(const __itt_domain *domain,
                                         __itt_clock_domain *clock_domain,
                                         unsigned long long timestamp,
                                         __itt_id id, __itt_id parentid,
                                         __itt_string_handle *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_begin_ex_ptr__3_0 &&
      __itt_task_begin_ex_ptr__3_0 != __itt_task_begin_ex_init_3_0)
    __itt_task_begin_ex_ptr__3_0(domain, clock_domain, timestamp, id, parentid,
                                 name);
  else
    return;
}
static void __itt_task_begin_fn_ex_init_3_0(const __itt_domain *domain,
                                            __itt_clock_domain *clock_domain,
                                            unsigned long long timestamp,
                                            __itt_id id, __itt_id parentid,
                                            void *fn);
typedef void __itt_task_begin_fn_ex_t(const __itt_domain *domain,
                                      __itt_clock_domain *clock_domain,
                                      unsigned long long timestamp, __itt_id id,
                                      __itt_id parentid, void *fn);
__itt_task_begin_fn_ex_t *__itt_task_begin_fn_ex_ptr__3_0 =
    __itt_task_begin_fn_ex_init_3_0;
static void __itt_task_begin_fn_ex_init_3_0(const __itt_domain *domain,
                                            __itt_clock_domain *clock_domain,
                                            unsigned long long timestamp,
                                            __itt_id id, __itt_id parentid,
                                            void *fn) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_begin_fn_ex_ptr__3_0 &&
      __itt_task_begin_fn_ex_ptr__3_0 != __itt_task_begin_fn_ex_init_3_0)
    __itt_task_begin_fn_ex_ptr__3_0(domain, clock_domain, timestamp, id,
                                    parentid, fn);
  else
    return;
}
static void __itt_task_end_ex_init_3_0(const __itt_domain *domain,
                                       __itt_clock_domain *clock_domain,
                                       unsigned long long timestamp);
typedef void __itt_task_end_ex_t(const __itt_domain *domain,
                                 __itt_clock_domain *clock_domain,
                                 unsigned long long timestamp);
__itt_task_end_ex_t *__itt_task_end_ex_ptr__3_0 = __itt_task_end_ex_init_3_0;
static void __itt_task_end_ex_init_3_0(const __itt_domain *domain,
                                       __itt_clock_domain *clock_domain,
                                       unsigned long long timestamp) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_end_ex_ptr__3_0 &&
      __itt_task_end_ex_ptr__3_0 != __itt_task_end_ex_init_3_0)
    __itt_task_end_ex_ptr__3_0(domain, clock_domain, timestamp);
  else
    return;
}
static void __itt_task_begin_overlapped_init_3_0(const __itt_domain *domain,
                                                 __itt_id id, __itt_id parent,
                                                 __itt_string_handle *name);
typedef void __itt_task_begin_overlapped_t(const __itt_domain *domain,
                                           __itt_id id, __itt_id parent,
                                           __itt_string_handle *name);
__itt_task_begin_overlapped_t *__itt_task_begin_overlapped_ptr__3_0 =
    __itt_task_begin_overlapped_init_3_0;
static void __itt_task_begin_overlapped_init_3_0(const __itt_domain *domain,
                                                 __itt_id id, __itt_id parent,
                                                 __itt_string_handle *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_begin_overlapped_ptr__3_0 &&
      __itt_task_begin_overlapped_ptr__3_0 !=
          __itt_task_begin_overlapped_init_3_0)
    __itt_task_begin_overlapped_ptr__3_0(domain, id, parent, name);
  else
    return;
}
static void __itt_task_begin_overlapped_ex_init_3_0(
    const __itt_domain *domain, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, __itt_id id, __itt_id parentid,
    __itt_string_handle *name);
typedef void __itt_task_begin_overlapped_ex_t(const __itt_domain *domain,
                                              __itt_clock_domain *clock_domain,
                                              unsigned long long timestamp,
                                              __itt_id id, __itt_id parentid,
                                              __itt_string_handle *name);
__itt_task_begin_overlapped_ex_t *__itt_task_begin_overlapped_ex_ptr__3_0 =
    __itt_task_begin_overlapped_ex_init_3_0;
static void __itt_task_begin_overlapped_ex_init_3_0(
    const __itt_domain *domain, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, __itt_id id, __itt_id parentid,
    __itt_string_handle *name) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_begin_overlapped_ex_ptr__3_0 &&
      __itt_task_begin_overlapped_ex_ptr__3_0 !=
          __itt_task_begin_overlapped_ex_init_3_0)
    __itt_task_begin_overlapped_ex_ptr__3_0(domain, clock_domain, timestamp, id,
                                            parentid, name);
  else
    return;
}
static void __itt_task_end_overlapped_init_3_0(const __itt_domain *domain,
                                               __itt_id id);
typedef void __itt_task_end_overlapped_t(const __itt_domain *domain,
                                         __itt_id id);
__itt_task_end_overlapped_t *__itt_task_end_overlapped_ptr__3_0 =
    __itt_task_end_overlapped_init_3_0;
static void __itt_task_end_overlapped_init_3_0(const __itt_domain *domain,
                                               __itt_id id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_end_overlapped_ptr__3_0 &&
      __itt_task_end_overlapped_ptr__3_0 != __itt_task_end_overlapped_init_3_0)
    __itt_task_end_overlapped_ptr__3_0(domain, id);
  else
    return;
}
static void __itt_task_end_overlapped_ex_init_3_0(
    const __itt_domain *domain, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, __itt_id id);
typedef void __itt_task_end_overlapped_ex_t(const __itt_domain *domain,
                                            __itt_clock_domain *clock_domain,
                                            unsigned long long timestamp,
                                            __itt_id id);
__itt_task_end_overlapped_ex_t *__itt_task_end_overlapped_ex_ptr__3_0 =
    __itt_task_end_overlapped_ex_init_3_0;
static void __itt_task_end_overlapped_ex_init_3_0(
    const __itt_domain *domain, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, __itt_id id) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_task_end_overlapped_ex_ptr__3_0 &&
      __itt_task_end_overlapped_ex_ptr__3_0 !=
          __itt_task_end_overlapped_ex_init_3_0)
    __itt_task_end_overlapped_ex_ptr__3_0(domain, clock_domain, timestamp, id);
  else
    return;
}
static void __itt_marker_ex_init_3_0(const __itt_domain *domain,
                                     __itt_clock_domain *clock_domain,
                                     unsigned long long timestamp, __itt_id id,
                                     __itt_string_handle *name,
                                     __itt_scope scope);
typedef void __itt_marker_ex_t(const __itt_domain *domain,
                               __itt_clock_domain *clock_domain,
                               unsigned long long timestamp, __itt_id id,
                               __itt_string_handle *name, __itt_scope scope);
__itt_marker_ex_t *__itt_marker_ex_ptr__3_0 = __itt_marker_ex_init_3_0;
static void __itt_marker_ex_init_3_0(const __itt_domain *domain,
                                     __itt_clock_domain *clock_domain,
                                     unsigned long long timestamp, __itt_id id,
                                     __itt_string_handle *name,
                                     __itt_scope scope) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_marker_ex_ptr__3_0 &&
      __itt_marker_ex_ptr__3_0 != __itt_marker_ex_init_3_0)
    __itt_marker_ex_ptr__3_0(domain, clock_domain, timestamp, id, name, scope);
  else
    return;
}
static void __itt_metadata_add_with_scope_init_3_0(const __itt_domain *domain,
                                                   __itt_scope scope,
                                                   __itt_string_handle *key,
                                                   __itt_metadata_type type,
                                                   size_t count, void *data);
typedef void __itt_metadata_add_with_scope_t(const __itt_domain *domain,
                                             __itt_scope scope,
                                             __itt_string_handle *key,
                                             __itt_metadata_type type,
                                             size_t count, void *data);
__itt_metadata_add_with_scope_t *__itt_metadata_add_with_scope_ptr__3_0 =
    __itt_metadata_add_with_scope_init_3_0;
static void __itt_metadata_add_with_scope_init_3_0(const __itt_domain *domain,
                                                   __itt_scope scope,
                                                   __itt_string_handle *key,
                                                   __itt_metadata_type type,
                                                   size_t count, void *data) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_metadata_add_with_scope_ptr__3_0 &&
      __itt_metadata_add_with_scope_ptr__3_0 !=
          __itt_metadata_add_with_scope_init_3_0)
    __itt_metadata_add_with_scope_ptr__3_0(domain, scope, key, type, count,
                                           data);
  else
    return;
}

static void __itt_metadata_str_add_with_scope_init_3_0(
    const __itt_domain *domain, __itt_scope scope, __itt_string_handle *key,
    const char *data, size_t length);
typedef void __itt_metadata_str_add_with_scope_t(const __itt_domain *domain,
                                                 __itt_scope scope,
                                                 __itt_string_handle *key,
                                                 const char *data,
                                                 size_t length);
__itt_metadata_str_add_with_scope_t
    *__itt_metadata_str_add_with_scope_ptr__3_0 =
        __itt_metadata_str_add_with_scope_init_3_0;
static void __itt_metadata_str_add_with_scope_init_3_0(
    const __itt_domain *domain, __itt_scope scope, __itt_string_handle *key,
    const char *data, size_t length) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_metadata_str_add_with_scope_ptr__3_0 &&
      __itt_metadata_str_add_with_scope_ptr__3_0 !=
          __itt_metadata_str_add_with_scope_init_3_0)
    __itt_metadata_str_add_with_scope_ptr__3_0(domain, scope, key, data,
                                               length);
  else
    return;
}

static void __itt_relation_add_to_current_ex_init_3_0(
    const __itt_domain *domain, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, __itt_relation relation, __itt_id tail);
typedef void __itt_relation_add_to_current_ex_t(
    const __itt_domain *domain, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, __itt_relation relation, __itt_id tail);
__itt_relation_add_to_current_ex_t *__itt_relation_add_to_current_ex_ptr__3_0 =
    __itt_relation_add_to_current_ex_init_3_0;
static void __itt_relation_add_to_current_ex_init_3_0(
    const __itt_domain *domain, __itt_clock_domain *clock_domain,
    unsigned long long timestamp, __itt_relation relation, __itt_id tail) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_relation_add_to_current_ex_ptr__3_0 &&
      __itt_relation_add_to_current_ex_ptr__3_0 !=
          __itt_relation_add_to_current_ex_init_3_0)
    __itt_relation_add_to_current_ex_ptr__3_0(domain, clock_domain, timestamp,
                                              relation, tail);
  else
    return;
}
static void __itt_relation_add_ex_init_3_0(const __itt_domain *domain,
                                           __itt_clock_domain *clock_domain,
                                           unsigned long long timestamp,
                                           __itt_id head,
                                           __itt_relation relation,
                                           __itt_id tail);
typedef void __itt_relation_add_ex_t(const __itt_domain *domain,
                                     __itt_clock_domain *clock_domain,
                                     unsigned long long timestamp,
                                     __itt_id head, __itt_relation relation,
                                     __itt_id tail);
__itt_relation_add_ex_t *__itt_relation_add_ex_ptr__3_0 =
    __itt_relation_add_ex_init_3_0;
static void __itt_relation_add_ex_init_3_0(const __itt_domain *domain,
                                           __itt_clock_domain *clock_domain,
                                           unsigned long long timestamp,
                                           __itt_id head,
                                           __itt_relation relation,
                                           __itt_id tail) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_relation_add_ex_ptr__3_0 &&
      __itt_relation_add_ex_ptr__3_0 != __itt_relation_add_ex_init_3_0)
    __itt_relation_add_ex_ptr__3_0(domain, clock_domain, timestamp, head,
                                   relation, tail);
  else
    return;
}
static __itt_track_group *
__itt_track_group_create_init_3_0(__itt_string_handle *name,
                                  __itt_track_group_type track_group_type);
typedef __itt_track_group *
__itt_track_group_create_t(__itt_string_handle *name,
                           __itt_track_group_type track_group_type);
__itt_track_group_create_t *__itt_track_group_create_ptr__3_0 =
    __itt_track_group_create_init_3_0;
static __itt_track_group *
__itt_track_group_create_init_3_0(__itt_string_handle *name,
                                  __itt_track_group_type track_group_type) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_track_group_create_ptr__3_0 &&
      __itt_track_group_create_ptr__3_0 != __itt_track_group_create_init_3_0)
    return __itt_track_group_create_ptr__3_0(name, track_group_type);
  else
    return (__itt_track_group *)0;
}
static __itt_track *__itt_track_create_init_3_0(__itt_track_group *track_group,
                                                __itt_string_handle *name,
                                                __itt_track_type track_type);
typedef __itt_track *__itt_track_create_t(__itt_track_group *track_group,
                                          __itt_string_handle *name,
                                          __itt_track_type track_type);
__itt_track_create_t *__itt_track_create_ptr__3_0 = __itt_track_create_init_3_0;
static __itt_track *__itt_track_create_init_3_0(__itt_track_group *track_group,
                                                __itt_string_handle *name,
                                                __itt_track_type track_type) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_track_create_ptr__3_0 &&
      __itt_track_create_ptr__3_0 != __itt_track_create_init_3_0)
    return __itt_track_create_ptr__3_0(track_group, name, track_type);
  else
    return (__itt_track *)0;
}
static void __itt_set_track_init_3_0(__itt_track *track);
typedef void __itt_set_track_t(__itt_track *track);
__itt_set_track_t *__itt_set_track_ptr__3_0 = __itt_set_track_init_3_0;
static void __itt_set_track_init_3_0(__itt_track *track) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_set_track_ptr__3_0 &&
      __itt_set_track_ptr__3_0 != __itt_set_track_init_3_0)
    __itt_set_track_ptr__3_0(track);
  else
    return;
}

static const char *__itt_api_version_init_3_0(void);
typedef const char *__itt_api_version_t(void);
__itt_api_version_t *__itt_api_version_ptr__3_0 = __itt_api_version_init_3_0;
static const char *__itt_api_version_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_api_version_ptr__3_0 &&
      __itt_api_version_ptr__3_0 != __itt_api_version_init_3_0)
    return __itt_api_version_ptr__3_0();
  else
    return (const char *)0;
}

static int __itt_av_save_init_3_0(void *data, int rank, const int *dimensions,
                                  int type, const char *filePath,
                                  int columnOrder);
typedef int __itt_av_save_t(void *data, int rank, const int *dimensions,
                            int type, const char *filePath, int columnOrder);
__itt_av_save_t *__itt_av_save_ptr__3_0 = __itt_av_save_init_3_0;
static int __itt_av_save_init_3_0(void *data, int rank, const int *dimensions,
                                  int type, const char *filePath,
                                  int columnOrder) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_av_save_ptr__3_0 &&
      __itt_av_save_ptr__3_0 != __itt_av_save_init_3_0)
    return __itt_av_save_ptr__3_0(data, rank, dimensions, type, filePath,
                                  columnOrder);
  else
    return (int)0;
}

static void __itt_module_load_init_3_0(void *start_addr, void *end_addr,
                                       const char *path);
typedef void __itt_module_load_t(void *start_addr, void *end_addr,
                                 const char *path);
__itt_module_load_t *__itt_module_load_ptr__3_0 = __itt_module_load_init_3_0;
static void __itt_module_load_init_3_0(void *start_addr, void *end_addr,
                                       const char *path) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_module_load_ptr__3_0 &&
      __itt_module_load_ptr__3_0 != __itt_module_load_init_3_0)
    __itt_module_load_ptr__3_0(start_addr, end_addr, path);
  else
    return;
}

static void __itt_module_unload_init_3_0(void *start_addr);
typedef void __itt_module_unload_t(void *start_addr);
__itt_module_unload_t *__itt_module_unload_ptr__3_0 =
    __itt_module_unload_init_3_0;
static void __itt_module_unload_init_3_0(void *start_addr) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_module_unload_ptr__3_0 &&
      __itt_module_unload_ptr__3_0 != __itt_module_unload_init_3_0)
    __itt_module_unload_ptr__3_0(start_addr);
  else
    return;
}

static void __itt_histogram_submit_init_3_0(__itt_histogram *hist,
                                            size_t length, void *x_data,
                                            void *y_data);
typedef void __itt_histogram_submit_t(__itt_histogram *hist, size_t length,
                                      void *x_data, void *y_data);
__itt_histogram_submit_t *__itt_histogram_submit_ptr__3_0 =
    __itt_histogram_submit_init_3_0;
static void __itt_histogram_submit_init_3_0(__itt_histogram *hist,
                                            size_t length, void *x_data,
                                            void *y_data) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0))
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  if (__itt_histogram_submit_ptr__3_0 &&
      __itt_histogram_submit_ptr__3_0 != __itt_histogram_submit_init_3_0)
    __itt_histogram_submit_ptr__3_0(hist, length, x_data, y_data);
  else
    return;
}
static __itt_domain *__itt_domain_create_init_3_0(const char *name);
typedef __itt_domain *__itt_domain_create_t(const char *name);
__itt_domain_create_t *__itt_domain_create_ptr__3_0 =
    __itt_domain_create_init_3_0;

static void
__itt_module_load_with_sections_init_3_0(__itt_module_object *module_obj);
typedef void __itt_module_load_with_sections_t(__itt_module_object *module_obj);
__itt_module_load_with_sections_t *__itt_module_load_with_sections_ptr__3_0 =
    __itt_module_load_with_sections_init_3_0;
static void
__itt_module_unload_with_sections_init_3_0(__itt_module_object *module_obj);
typedef void
__itt_module_unload_with_sections_t(__itt_module_object *module_obj);
__itt_module_unload_with_sections_t
    *__itt_module_unload_with_sections_ptr__3_0 =
        __itt_module_unload_with_sections_init_3_0;

static __itt_string_handle *
__itt_string_handle_create_init_3_0(const char *name);
typedef __itt_string_handle *__itt_string_handle_create_t(const char *name);
__itt_string_handle_create_t *__itt_string_handle_create_ptr__3_0 =
    __itt_string_handle_create_init_3_0;

static __itt_counter __itt_counter_create_init_3_0(const char *name,
                                                   const char *domain);
typedef __itt_counter __itt_counter_create_t(const char *name,
                                             const char *domain);
__itt_counter_create_t *__itt_counter_create_ptr__3_0 =
    __itt_counter_create_init_3_0;

static __itt_counter
__itt_counter_create_typed_init_3_0(const char *name, const char *domain,
                                    __itt_metadata_type type);
typedef __itt_counter __itt_counter_create_typed_t(const char *name,
                                                   const char *domain,
                                                   __itt_metadata_type type);
__itt_counter_create_typed_t *__itt_counter_create_typed_ptr__3_0 =
    __itt_counter_create_typed_init_3_0;

static void __itt_pause_init_3_0(void);
typedef void __itt_pause_t(void);
__itt_pause_t *__itt_pause_ptr__3_0 = __itt_pause_init_3_0;
static void __itt_resume_init_3_0(void);
typedef void __itt_resume_t(void);
__itt_resume_t *__itt_resume_ptr__3_0 = __itt_resume_init_3_0;
static void __itt_pause_scoped_init_3_0(__itt_collection_scope scope);
typedef void __itt_pause_scoped_t(__itt_collection_scope scope);
__itt_pause_scoped_t *__itt_pause_scoped_ptr__3_0 = __itt_pause_scoped_init_3_0;
static void __itt_resume_scoped_init_3_0(__itt_collection_scope scope);
typedef void __itt_resume_scoped_t(__itt_collection_scope scope);
__itt_resume_scoped_t *__itt_resume_scoped_ptr__3_0 =
    __itt_resume_scoped_init_3_0;

static void __itt_thread_set_name_init_3_0(const char *name);
typedef void __itt_thread_set_name_t(const char *name);
__itt_thread_set_name_t *__itt_thread_set_name_ptr__3_0 =
    __itt_thread_set_name_init_3_0;

static void __itt_thread_ignore_init_3_0(void);
typedef void __itt_thread_ignore_t(void);
__itt_thread_ignore_t *__itt_thread_ignore_ptr__3_0 =
    __itt_thread_ignore_init_3_0;

static int __itt_thr_name_set_init_3_0(const char *name, int namelen);
typedef int __itt_thr_name_set_t(const char *name, int namelen);
__itt_thr_name_set_t *__itt_thr_name_set_ptr__3_0 = __itt_thr_name_set_init_3_0;

static void __itt_thr_ignore_init_3_0(void);
typedef void __itt_thr_ignore_t(void);
__itt_thr_ignore_t *__itt_thr_ignore_ptr__3_0 = __itt_thr_ignore_init_3_0;

static __itt_histogram *
__itt_histogram_create_init_3_0(const __itt_domain *domain, const char *name,
                                __itt_metadata_type x_type,
                                __itt_metadata_type y_type);
typedef __itt_histogram *__itt_histogram_create_t(const __itt_domain *domain,
                                                  const char *name,
                                                  __itt_metadata_type x_type,
                                                  __itt_metadata_type y_type);
__itt_histogram_create_t *__itt_histogram_create_ptr__3_0 =
    __itt_histogram_create_init_3_0;

static void __itt_enable_attach_init_3_0(void);
typedef void __itt_enable_attach_t(void);
__itt_enable_attach_t *__itt_enable_attach_ptr__3_0 =
    __itt_enable_attach_init_3_0;

static __itt_group_list group_list[] = {{__itt_group_all, "all"},
                                        {__itt_group_control, "control"},
                                        {__itt_group_thread, "thread"},
                                        {__itt_group_mark, "mark"},
                                        {__itt_group_sync, "sync"},
                                        {__itt_group_fsync, "fsync"},
                                        {__itt_group_jit, "jit"},
                                        {__itt_group_model, "model"},
                                        {__itt_group_counter, "counter"},
                                        {__itt_group_frame, "frame"},
                                        {__itt_group_stitch, "stitch"},
                                        {__itt_group_heap, "heap"},
                                        {__itt_group_structure, "structure"},
                                        {__itt_group_suppress, "suppress"},
                                        {__itt_group_arrays, "arrays"},
                                        {__itt_group_module, "module"},
                                        {__itt_group_none,((void *)0)}};

#pragma pack(push, 8)

typedef struct ___itt_group_alias {
  const char *env_var;
  __itt_group_id groups;
} __itt_group_alias;

static __itt_group_alias group_alias[] = {
    {"KMP_FOR_TPROFILE",
     (__itt_group_id)(__itt_group_control | __itt_group_thread |
                      __itt_group_sync | __itt_group_mark)},
    {"KMP_FOR_TCHECK",
     (__itt_group_id)(__itt_group_control | __itt_group_thread |
                      __itt_group_sync | __itt_group_fsync | __itt_group_mark |
                      __itt_group_suppress)},
    {((void *)0),
        (__itt_group_none)},
    {api_version, (__itt_group_none)}};

#pragma pack(pop)
static __itt_api_info api_list[] = {

    {"__itt_domain_create", (void **)(void *)&__itt_domain_create_ptr__3_0,
     (void *)(size_t)&__itt_domain_create_init_3_0,
     (void *)(size_t)&__itt_domain_create_init_3_0,
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_module_load_with_sections",
     (void **)(void *)&__itt_module_load_with_sections_ptr__3_0,
     (void *)(size_t)&__itt_module_load_with_sections_init_3_0,
     (void *)(size_t)&__itt_module_load_with_sections_init_3_0,
     (__itt_group_id)(__itt_group_module)},
    {"__itt_module_unload_with_sections",
     (void **)(void *)&__itt_module_unload_with_sections_ptr__3_0,
     (void *)(size_t)&__itt_module_unload_with_sections_init_3_0,
     (void *)(size_t)&__itt_module_unload_with_sections_init_3_0,
     (__itt_group_id)(__itt_group_module)},

    {"__itt_string_handle_create",
     (void **)(void *)&__itt_string_handle_create_ptr__3_0,
     (void *)(size_t)&__itt_string_handle_create_init_3_0,
     (void *)(size_t)&__itt_string_handle_create_init_3_0,
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_counter_create", (void **)(void *)&__itt_counter_create_ptr__3_0,
     (void *)(size_t)&__itt_counter_create_init_3_0,
     (void *)(size_t)&__itt_counter_create_init_3_0,
     (__itt_group_id)(__itt_group_counter)},

    {"__itt_counter_create_typed",
     (void **)(void *)&__itt_counter_create_typed_ptr__3_0,
     (void *)(size_t)&__itt_counter_create_typed_init_3_0,
     (void *)(size_t)&__itt_counter_create_typed_init_3_0,
     (__itt_group_id)(__itt_group_counter)},

    {"__itt_pause", (void **)(void *)&__itt_pause_ptr__3_0,
     (void *)(size_t)&__itt_pause_init_3_0,
     (void *)(size_t)&__itt_pause_init_3_0,
     (__itt_group_id)(__itt_group_control | __itt_group_legacy)},
    {"__itt_resume", (void **)(void *)&__itt_resume_ptr__3_0,
     (void *)(size_t)&__itt_resume_init_3_0,
     (void *)(size_t)&__itt_resume_init_3_0,
     (__itt_group_id)(__itt_group_control | __itt_group_legacy)},
    {"__itt_pause_scoped", (void **)(void *)&__itt_pause_scoped_ptr__3_0,
     (void *)(size_t)&__itt_pause_scoped_init_3_0,
     (void *)(size_t)&__itt_pause_scoped_init_3_0,
     (__itt_group_id)(__itt_group_control)},
    {"__itt_resume_scoped", (void **)(void *)&__itt_resume_scoped_ptr__3_0,
     (void *)(size_t)&__itt_resume_scoped_init_3_0,
     (void *)(size_t)&__itt_resume_scoped_init_3_0,
     (__itt_group_id)(__itt_group_control)},

    {"__itt_thread_set_name", (void **)(void *)&__itt_thread_set_name_ptr__3_0,
     (void *)(size_t)&__itt_thread_set_name_init_3_0,
     (void *)(size_t)&__itt_thread_set_name_init_3_0,
     (__itt_group_id)(__itt_group_thread)},

    {"__itt_thread_ignore", (void **)(void *)&__itt_thread_ignore_ptr__3_0,
     (void *)(size_t)&__itt_thread_ignore_init_3_0,
     (void *)(size_t)&__itt_thread_ignore_init_3_0,
     (__itt_group_id)(__itt_group_thread)},

    {"__itt_thr_name_set", (void **)(void *)&__itt_thr_name_set_ptr__3_0,
     (void *)(size_t)&__itt_thr_name_set_init_3_0,
     (void *)(size_t)&__itt_thr_name_set_init_3_0,
     (__itt_group_id)(__itt_group_thread | __itt_group_legacy)},

    {"__itt_thr_ignore", (void **)(void *)&__itt_thr_ignore_ptr__3_0,
     (void *)(size_t)&__itt_thr_ignore_init_3_0,
     (void *)(size_t)&__itt_thr_ignore_init_3_0,
     (__itt_group_id)(__itt_group_thread | __itt_group_legacy)},

    {"__itt_histogram_create",
     (void **)(void *)&__itt_histogram_create_ptr__3_0,
     (void *)(size_t)&__itt_histogram_create_init_3_0,
     (void *)(size_t)&__itt_histogram_create_init_3_0,
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_enable_attach", (void **)(void *)&__itt_enable_attach_ptr__3_0,
     (void *)(size_t)&__itt_enable_attach_init_3_0,
     (void *)(size_t)&__itt_enable_attach_init_3_0,
     (__itt_group_id)(__itt_group_all)},

    {"__itt_detach", (void **)(void *)&__itt_detach_ptr__3_0,
     (void *)(size_t)&__itt_detach_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_control | __itt_group_legacy)},

    {"__itt_sync_create", (void **)(void *)&__itt_sync_create_ptr__3_0,
     (void *)(size_t)&__itt_sync_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync)},
    {"__itt_sync_rename", (void **)(void *)&__itt_sync_rename_ptr__3_0,
     (void *)(size_t)&__itt_sync_rename_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync)},

    {"__itt_sync_destroy", (void **)(void *)&__itt_sync_destroy_ptr__3_0,
     (void *)(size_t)&__itt_sync_destroy_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync)},

    {"__itt_sync_prepare", (void **)(void *)&__itt_sync_prepare_ptr__3_0,
     (void *)(size_t)&__itt_sync_prepare_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync)},
    {"__itt_sync_cancel", (void **)(void *)&__itt_sync_cancel_ptr__3_0,
     (void *)(size_t)&__itt_sync_cancel_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync)},
    {"__itt_sync_acquired", (void **)(void *)&__itt_sync_acquired_ptr__3_0,
     (void *)(size_t)&__itt_sync_acquired_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync)},
    {"__itt_sync_releasing", (void **)(void *)&__itt_sync_releasing_ptr__3_0,
     (void *)(size_t)&__itt_sync_releasing_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync)},

    {"__itt_suppress_push", (void **)(void *)&__itt_suppress_push_ptr__3_0,
     (void *)(size_t)&__itt_suppress_push_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_suppress)},
    {"__itt_suppress_pop", (void **)(void *)&__itt_suppress_pop_ptr__3_0,
     (void *)(size_t)&__itt_suppress_pop_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_suppress)},
    {"__itt_suppress_mark_range",
     (void **)(void *)&__itt_suppress_mark_range_ptr__3_0,
     (void *)(size_t)&__itt_suppress_mark_range_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_suppress)},
    {"__itt_suppress_clear_range",
     (void **)(void *)&__itt_suppress_clear_range_ptr__3_0,
     (void *)(size_t)&__itt_suppress_clear_range_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_suppress)},

    {"__itt_sync_prepare", (void **)(void *)&__itt_fsync_prepare_ptr__3_0,
     (void *)(size_t)&__itt_fsync_prepare_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_fsync)},
    {"__itt_sync_cancel", (void **)(void *)&__itt_fsync_cancel_ptr__3_0,
     (void *)(size_t)&__itt_fsync_cancel_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_fsync)},
    {"__itt_sync_acquired", (void **)(void *)&__itt_fsync_acquired_ptr__3_0,
     (void *)(size_t)&__itt_fsync_acquired_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_fsync)},
    {"__itt_sync_releasing", (void **)(void *)&__itt_fsync_releasing_ptr__3_0,
     (void *)(size_t)&__itt_fsync_releasing_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_fsync)},

    {"__itt_model_site_begin",
     (void **)(void *)&__itt_model_site_begin_ptr__3_0,
     (void *)(size_t)&__itt_model_site_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_site_end", (void **)(void *)&__itt_model_site_end_ptr__3_0,
     (void *)(size_t)&__itt_model_site_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_task_begin",
     (void **)(void *)&__itt_model_task_begin_ptr__3_0,
     (void *)(size_t)&__itt_model_task_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_task_end", (void **)(void *)&__itt_model_task_end_ptr__3_0,
     (void *)(size_t)&__itt_model_task_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_lock_acquire",
     (void **)(void *)&__itt_model_lock_acquire_ptr__3_0,
     (void *)(size_t)&__itt_model_lock_acquire_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_lock_release",
     (void **)(void *)&__itt_model_lock_release_ptr__3_0,
     (void *)(size_t)&__itt_model_lock_release_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_record_allocation",
     (void **)(void *)&__itt_model_record_allocation_ptr__3_0,
     (void *)(size_t)&__itt_model_record_allocation_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_record_deallocation",
     (void **)(void *)&__itt_model_record_deallocation_ptr__3_0,
     (void *)(size_t)&__itt_model_record_deallocation_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_induction_uses",
     (void **)(void *)&__itt_model_induction_uses_ptr__3_0,
     (void *)(size_t)&__itt_model_induction_uses_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_reduction_uses",
     (void **)(void *)&__itt_model_reduction_uses_ptr__3_0,
     (void *)(size_t)&__itt_model_reduction_uses_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_observe_uses",
     (void **)(void *)&__itt_model_observe_uses_ptr__3_0,
     (void *)(size_t)&__itt_model_observe_uses_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_clear_uses",
     (void **)(void *)&__itt_model_clear_uses_ptr__3_0,
     (void *)(size_t)&__itt_model_clear_uses_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},

    {"__itt_model_site_beginA",
     (void **)(void *)&__itt_model_site_beginA_ptr__3_0,
     (void *)(size_t)&__itt_model_site_beginA_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_site_beginAL",
     (void **)(void *)&__itt_model_site_beginAL_ptr__3_0,
     (void *)(size_t)&__itt_model_site_beginAL_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_task_beginA",
     (void **)(void *)&__itt_model_task_beginA_ptr__3_0,
     (void *)(size_t)&__itt_model_task_beginA_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_task_beginAL",
     (void **)(void *)&__itt_model_task_beginAL_ptr__3_0,
     (void *)(size_t)&__itt_model_task_beginAL_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_iteration_taskA",
     (void **)(void *)&__itt_model_iteration_taskA_ptr__3_0,
     (void *)(size_t)&__itt_model_iteration_taskA_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_iteration_taskAL",
     (void **)(void *)&__itt_model_iteration_taskAL_ptr__3_0,
     (void *)(size_t)&__itt_model_iteration_taskAL_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_site_end_2",
     (void **)(void *)&__itt_model_site_end_2_ptr__3_0,
     (void *)(size_t)&__itt_model_site_end_2_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_task_end_2",
     (void **)(void *)&__itt_model_task_end_2_ptr__3_0,
     (void *)(size_t)&__itt_model_task_end_2_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_lock_acquire_2",
     (void **)(void *)&__itt_model_lock_acquire_2_ptr__3_0,
     (void *)(size_t)&__itt_model_lock_acquire_2_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_lock_release_2",
     (void **)(void *)&__itt_model_lock_release_2_ptr__3_0,
     (void *)(size_t)&__itt_model_lock_release_2_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_aggregate_task",
     (void **)(void *)&__itt_model_aggregate_task_ptr__3_0,
     (void *)(size_t)&__itt_model_aggregate_task_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_disable_push",
     (void **)(void *)&__itt_model_disable_push_ptr__3_0,
     (void *)(size_t)&__itt_model_disable_push_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},
    {"__itt_model_disable_pop",
     (void **)(void *)&__itt_model_disable_pop_ptr__3_0,
     (void *)(size_t)&__itt_model_disable_pop_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_model)},

    {"__itt_heap_function_create",
     (void **)(void *)&__itt_heap_function_create_ptr__3_0,
     (void *)(size_t)&__itt_heap_function_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},

    {"__itt_heap_allocate_begin",
     (void **)(void *)&__itt_heap_allocate_begin_ptr__3_0,
     (void *)(size_t)&__itt_heap_allocate_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_allocate_end",
     (void **)(void *)&__itt_heap_allocate_end_ptr__3_0,
     (void *)(size_t)&__itt_heap_allocate_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_free_begin", (void **)(void *)&__itt_heap_free_begin_ptr__3_0,
     (void *)(size_t)&__itt_heap_free_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_free_end", (void **)(void *)&__itt_heap_free_end_ptr__3_0,
     (void *)(size_t)&__itt_heap_free_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_reallocate_begin",
     (void **)(void *)&__itt_heap_reallocate_begin_ptr__3_0,
     (void *)(size_t)&__itt_heap_reallocate_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_reallocate_end",
     (void **)(void *)&__itt_heap_reallocate_end_ptr__3_0,
     (void *)(size_t)&__itt_heap_reallocate_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_internal_access_begin",
     (void **)(void *)&__itt_heap_internal_access_begin_ptr__3_0,
     (void *)(size_t)&__itt_heap_internal_access_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_internal_access_end",
     (void **)(void *)&__itt_heap_internal_access_end_ptr__3_0,
     (void *)(size_t)&__itt_heap_internal_access_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_record_memory_growth_begin",
     (void **)(void *)&__itt_heap_record_memory_growth_begin_ptr__3_0,
     (void *)(size_t)&__itt_heap_record_memory_growth_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_record_memory_growth_end",
     (void **)(void *)&__itt_heap_record_memory_growth_end_ptr__3_0,
     (void *)(size_t)&__itt_heap_record_memory_growth_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_reset_detection",
     (void **)(void *)&__itt_heap_reset_detection_ptr__3_0,
     (void *)(size_t)&__itt_heap_reset_detection_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},
    {"__itt_heap_record", (void **)(void *)&__itt_heap_record_ptr__3_0,
     (void *)(size_t)&__itt_heap_record_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_heap)},

    {"__itt_id_create", (void **)(void *)&__itt_id_create_ptr__3_0,
     (void *)(size_t)&__itt_id_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_id_destroy", (void **)(void *)&__itt_id_destroy_ptr__3_0,
     (void *)(size_t)&__itt_id_destroy_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_get_timestamp", (void **)(void *)&__itt_get_timestamp_ptr__3_0,
     (void *)(size_t)&__itt_get_timestamp_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_region_begin", (void **)(void *)&__itt_region_begin_ptr__3_0,
     (void *)(size_t)&__itt_region_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_region_end", (void **)(void *)&__itt_region_end_ptr__3_0,
     (void *)(size_t)&__itt_region_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_frame_begin_v3", (void **)(void *)&__itt_frame_begin_v3_ptr__3_0,
     (void *)(size_t)&__itt_frame_begin_v3_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_frame_end_v3", (void **)(void *)&__itt_frame_end_v3_ptr__3_0,
     (void *)(size_t)&__itt_frame_end_v3_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_frame_submit_v3", (void **)(void *)&__itt_frame_submit_v3_ptr__3_0,
     (void *)(size_t)&__itt_frame_submit_v3_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_task_group", (void **)(void *)&__itt_task_group_ptr__3_0,
     (void *)(size_t)&__itt_task_group_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_task_begin", (void **)(void *)&__itt_task_begin_ptr__3_0,
     (void *)(size_t)&__itt_task_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_begin_fn", (void **)(void *)&__itt_task_begin_fn_ptr__3_0,
     (void *)(size_t)&__itt_task_begin_fn_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_end", (void **)(void *)&__itt_task_end_ptr__3_0,
     (void *)(size_t)&__itt_task_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_counter_inc_v3", (void **)(void *)&__itt_counter_inc_v3_ptr__3_0,
     (void *)(size_t)&__itt_counter_inc_v3_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_counter_inc_delta_v3",
     (void **)(void *)&__itt_counter_inc_delta_v3_ptr__3_0,
     (void *)(size_t)&__itt_counter_inc_delta_v3_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_counter_dec_v3", (void **)(void *)&__itt_counter_dec_v3_ptr__3_0,
     (void *)(size_t)&__itt_counter_dec_v3_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_counter_dec_delta_v3",
     (void **)(void *)&__itt_counter_dec_delta_v3_ptr__3_0,
     (void *)(size_t)&__itt_counter_dec_delta_v3_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_marker", (void **)(void *)&__itt_marker_ptr__3_0,
     (void *)(size_t)&__itt_marker_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_metadata_add", (void **)(void *)&__itt_metadata_add_ptr__3_0,
     (void *)(size_t)&__itt_metadata_add_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_metadata_str_add",
     (void **)(void *)&__itt_metadata_str_add_ptr__3_0,
     (void *)(size_t)&__itt_metadata_str_add_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_relation_add_to_current",
     (void **)(void *)&__itt_relation_add_to_current_ptr__3_0,
     (void *)(size_t)&__itt_relation_add_to_current_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_relation_add", (void **)(void *)&__itt_relation_add_ptr__3_0,
     (void *)(size_t)&__itt_relation_add_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_event_create", (void **)(void *)&__itt_event_create_ptr__3_0,
     (void *)(size_t)&__itt_event_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_mark | __itt_group_legacy)},

    {"__itt_event_start", (void **)(void *)&__itt_event_start_ptr__3_0,
     (void *)(size_t)&__itt_event_start_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_mark | __itt_group_legacy)},
    {"__itt_event_end", (void **)(void *)&__itt_event_end_ptr__3_0,
     (void *)(size_t)&__itt_event_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_mark | __itt_group_legacy)},

    {"__itt_sync_set_name", (void **)(void *)&__itt_sync_set_name_ptr__3_0,
     (void *)(size_t)&__itt_sync_set_name_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync |
                      __itt_group_legacy)},

    {"__itt_notify_sync_name",
     (void **)(void *)&__itt_notify_sync_name_ptr__3_0,
     (void *)(size_t)&__itt_notify_sync_name_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync |
                      __itt_group_legacy)},

    {"__itt_notify_sync_prepare",
     (void **)(void *)&__itt_notify_sync_prepare_ptr__3_0,
     (void *)(size_t)&__itt_notify_sync_prepare_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync |
                      __itt_group_legacy)},
    {"__itt_notify_sync_cancel",
     (void **)(void *)&__itt_notify_sync_cancel_ptr__3_0,
     (void *)(size_t)&__itt_notify_sync_cancel_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync |
                      __itt_group_legacy)},
    {"__itt_notify_sync_acquired",
     (void **)(void *)&__itt_notify_sync_acquired_ptr__3_0,
     (void *)(size_t)&__itt_notify_sync_acquired_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync |
                      __itt_group_legacy)},
    {"__itt_notify_sync_releasing",
     (void **)(void *)&__itt_notify_sync_releasing_ptr__3_0,
     (void *)(size_t)&__itt_notify_sync_releasing_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_sync | __itt_group_fsync |
                      __itt_group_legacy)},

    {"__itt_memory_read", (void **)(void *)&__itt_memory_read_ptr__3_0,
     (void *)(size_t)&__itt_memory_read_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_legacy)},
    {"__itt_memory_write", (void **)(void *)&__itt_memory_write_ptr__3_0,
     (void *)(size_t)&__itt_memory_write_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_legacy)},
    {"__itt_memory_update", (void **)(void *)&__itt_memory_update_ptr__3_0,
     (void *)(size_t)&__itt_memory_update_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_legacy)},

    {"__itt_state_get", (void **)(void *)&__itt_state_get_ptr__3_0,
     (void *)(size_t)&__itt_state_get_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_legacy)},
    {"__itt_state_set", (void **)(void *)&__itt_state_set_ptr__3_0,
     (void *)(size_t)&__itt_state_set_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_legacy)},
    {"__itt_obj_mode_set", (void **)(void *)&__itt_obj_mode_set_ptr__3_0,
     (void *)(size_t)&__itt_obj_mode_set_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_legacy)},
    {"__itt_thr_mode_set", (void **)(void *)&__itt_thr_mode_set_ptr__3_0,
     (void *)(size_t)&__itt_thr_mode_set_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_legacy)},

    {"__itt_frame_create", (void **)(void *)&__itt_frame_create_ptr__3_0,
     (void *)(size_t)&__itt_frame_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_frame)},

    {"__itt_pt_region_create",
     (void **)(void *)&__itt_pt_region_create_ptr__3_0,
     (void *)(size_t)&__itt_pt_region_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_frame_begin", (void **)(void *)&__itt_frame_begin_ptr__3_0,
     (void *)(size_t)&__itt_frame_begin_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_frame)},
    {"__itt_frame_end", (void **)(void *)&__itt_frame_end_ptr__3_0,
     (void *)(size_t)&__itt_frame_end_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_frame)},

    {"__itt_counter_destroy", (void **)(void *)&__itt_counter_destroy_ptr__3_0,
     (void *)(size_t)&__itt_counter_destroy_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_counter)},
    {"__itt_counter_inc", (void **)(void *)&__itt_counter_inc_ptr__3_0,
     (void *)(size_t)&__itt_counter_inc_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_counter)},
    {"__itt_counter_inc_delta",
     (void **)(void *)&__itt_counter_inc_delta_ptr__3_0,
     (void *)(size_t)&__itt_counter_inc_delta_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_counter)},
    {"__itt_counter_dec", (void **)(void *)&__itt_counter_dec_ptr__3_0,
     (void *)(size_t)&__itt_counter_dec_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_counter)},
    {"__itt_counter_dec_delta",
     (void **)(void *)&__itt_counter_dec_delta_ptr__3_0,
     (void *)(size_t)&__itt_counter_dec_delta_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_counter)},
    {"__itt_counter_set_value",
     (void **)(void *)&__itt_counter_set_value_ptr__3_0,
     (void *)(size_t)&__itt_counter_set_value_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_counter)},
    {"__itt_counter_set_value_ex",
     (void **)(void *)&__itt_counter_set_value_ex_ptr__3_0,
     (void *)(size_t)&__itt_counter_set_value_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_counter)},

    {"__itt_mark_create", (void **)(void *)&__itt_mark_create_ptr__3_0,
     (void *)(size_t)&__itt_mark_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_mark)},

    {"__itt_mark", (void **)(void *)&__itt_mark_ptr__3_0,
     (void *)(size_t)&__itt_mark_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_mark)},

    {"__itt_mark_off", (void **)(void *)&__itt_mark_off_ptr__3_0,
     (void *)(size_t)&__itt_mark_off_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_mark)},

    {"__itt_mark_global", (void **)(void *)&__itt_mark_global_ptr__3_0,
     (void *)(size_t)&__itt_mark_global_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_mark)},

    {"__itt_mark_global_off", (void **)(void *)&__itt_mark_global_off_ptr__3_0,
     (void *)(size_t)&__itt_mark_global_off_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_mark)},

    {"__itt_stack_caller_create",
     (void **)(void *)&__itt_stack_caller_create_ptr__3_0,
     (void *)(size_t)&__itt_stack_caller_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_stitch)},

    {"__itt_stack_caller_destroy",
     (void **)(void *)&__itt_stack_caller_destroy_ptr__3_0,
     (void *)(size_t)&__itt_stack_caller_destroy_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_stitch)},
    {"__itt_stack_callee_enter",
     (void **)(void *)&__itt_stack_callee_enter_ptr__3_0,
     (void *)(size_t)&__itt_stack_callee_enter_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_stitch)},
    {"__itt_stack_callee_leave",
     (void **)(void *)&__itt_stack_callee_leave_ptr__3_0,
     (void *)(size_t)&__itt_stack_callee_leave_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_stitch)},

    {"__itt_clock_domain_create",
     (void **)(void *)&__itt_clock_domain_create_ptr__3_0,
     (void *)(size_t)&__itt_clock_domain_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_clock_domain_reset",
     (void **)(void *)&__itt_clock_domain_reset_ptr__3_0,
     (void *)(size_t)&__itt_clock_domain_reset_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_id_create_ex", (void **)(void *)&__itt_id_create_ex_ptr__3_0,
     (void *)(size_t)&__itt_id_create_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_id_destroy_ex", (void **)(void *)&__itt_id_destroy_ex_ptr__3_0,
     (void *)(size_t)&__itt_id_destroy_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_begin_ex", (void **)(void *)&__itt_task_begin_ex_ptr__3_0,
     (void *)(size_t)&__itt_task_begin_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_begin_fn_ex",
     (void **)(void *)&__itt_task_begin_fn_ex_ptr__3_0,
     (void *)(size_t)&__itt_task_begin_fn_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_end_ex", (void **)(void *)&__itt_task_end_ex_ptr__3_0,
     (void *)(size_t)&__itt_task_end_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_begin_overlapped",
     (void **)(void *)&__itt_task_begin_overlapped_ptr__3_0,
     (void *)(size_t)&__itt_task_begin_overlapped_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_begin_overlapped_ex",
     (void **)(void *)&__itt_task_begin_overlapped_ex_ptr__3_0,
     (void *)(size_t)&__itt_task_begin_overlapped_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_end_overlapped",
     (void **)(void *)&__itt_task_end_overlapped_ptr__3_0,
     (void *)(size_t)&__itt_task_end_overlapped_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_task_end_overlapped_ex",
     (void **)(void *)&__itt_task_end_overlapped_ex_ptr__3_0,
     (void *)(size_t)&__itt_task_end_overlapped_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_marker_ex", (void **)(void *)&__itt_marker_ex_ptr__3_0,
     (void *)(size_t)&__itt_marker_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_metadata_add_with_scope",
     (void **)(void *)&__itt_metadata_add_with_scope_ptr__3_0,
     (void *)(size_t)&__itt_metadata_add_with_scope_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_metadata_str_add_with_scope",
     (void **)(void *)&__itt_metadata_str_add_with_scope_ptr__3_0,
     (void *)(size_t)&__itt_metadata_str_add_with_scope_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_relation_add_to_current_ex",
     (void **)(void *)&__itt_relation_add_to_current_ex_ptr__3_0,
     (void *)(size_t)&__itt_relation_add_to_current_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_relation_add_ex", (void **)(void *)&__itt_relation_add_ex_ptr__3_0,
     (void *)(size_t)&__itt_relation_add_ex_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_track_group_create",
     (void **)(void *)&__itt_track_group_create_ptr__3_0,
     (void *)(size_t)&__itt_track_group_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_track_create", (void **)(void *)&__itt_track_create_ptr__3_0,
     (void *)(size_t)&__itt_track_create_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {"__itt_set_track", (void **)(void *)&__itt_set_track_ptr__3_0,
     (void *)(size_t)&__itt_set_track_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},

    {"__itt_api_version", (void **)(void *)&__itt_api_version_ptr__3_0,
     (void *)(size_t)&__itt_api_version_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_all & ~__itt_group_legacy)},

    {"__itt_av_save", (void **)(void *)&__itt_av_save_ptr__3_0,
     (void *)(size_t)&__itt_av_save_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_arrays)},

    {"__itt_module_load", (void **)(void *)&__itt_module_load_ptr__3_0,
     (void *)(size_t)&__itt_module_load_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_module)},

    {"__itt_module_unload", (void **)(void *)&__itt_module_unload_ptr__3_0,
     (void *)(size_t)&__itt_module_unload_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_module)},

    {"__itt_histogram_submit",
     (void **)(void *)&__itt_histogram_submit_ptr__3_0,
     (void *)(size_t)&__itt_histogram_submit_init_3_0,((void *)0),
     (__itt_group_id)(__itt_group_structure)},
    {((void *)0),((void *)0),((void *)0),((void *)0),
        __itt_group_none}};
__itt_global __itt__ittapi_global = {
    {0xED, 0xAB, 0xAB, 0xEC, 0x0D, 0xEE, 0xDA, 0x30},
    3,
    0,
    20210712,
    0,
    0,
    0,

    {{0, 0, 0, 0, PTHREAD_MUTEX_TIMED_NP, 0, 0, {0, 0}}}
    ,((void *)0),((void *)0),((void *)0),
    (__itt_api_info *)&api_list,((void *)0),((void *)0),((void *)0),((void *)0),
    __itt_collection_uninitialized,((void *)0),
    0,((void *)0)};

typedef void(__itt_api_init_t)(__itt_global *, __itt_group_id);
typedef void(__itt_api_fini_t)(__itt_global *);

static __itt_domain dummy_domain;
static void __itt_report_error(int code, ...) {
  va_list args;

  __builtin_va_start(
      args
      ,
      code
  )
      ;
  if (__itt__ittapi_global.error_handler !=((void *)0)) {
    __itt_error_handler_t *handler =
        (__itt_error_handler_t *)(size_t)__itt__ittapi_global.error_handler;
    handler((__itt_error_code)code, args);
  }

  __builtin_va_end(
      args
  )
      ;
}

static int __itt_is_collector_available(void);
static __itt_domain *__itt_domain_create_init_3_0(const char *name)

{
  __itt_domain *h_tail =((void *)0),
               *h =((void *)0);

  if (name ==((void *)0)) {
    return((void *)0);
  }

  {
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self)) {
      if (!__itt__ittapi_global.mutex_initialized) {
        if (__itt_interlocked_compare_exchange(
                &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
          {
            pthread_mutexattr_t mutex_attr;
            int error_code = pthread_mutexattr_init(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                 error_code);
            error_code =
                pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_settype", error_code);
            error_code =
                pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                 error_code);
            error_code = pthread_mutexattr_destroy(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_destroy", error_code);
          };
          __itt__ittapi_global.mutex_initialized = 1;
        } else
          while (!__itt__ittapi_global.mutex_initialized)
            sched_yield();
      }
      pthread_mutex_lock(&__itt__ittapi_global.mutex);
    }
  };
  if (__itt__ittapi_global.api_initialized) {

    if (__itt_domain_create_ptr__3_0 &&
        __itt_domain_create_ptr__3_0 != __itt_domain_create_init_3_0) {
      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);
      return __itt_domain_create_ptr__3_0(name);
    }

    else {

      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);

      return &dummy_domain;
    }
  }
  if (__itt_is_collector_available()) {
    for (h_tail =((void *)0),
        h = __itt__ittapi_global.domain_list;
         h !=((void *)0);
         h_tail = h, h = h->next) {
      if (h->nameA !=((void *)0)&& !strcmp(h->nameA, name))
        break;
    }
    if (h ==((void *)0)) {
      {
        h = (__itt_domain *)malloc(sizeof(__itt_domain));
        if (h !=((void *)0)) {
          h->flags = 1;
          char *name_copy =((void *)0);
          do {
            if (name !=((void *)0)) {
              size_t s_len = strlen(name);
              name_copy = (char *)malloc(s_len + 1);
              if (name_copy !=((void *)0)) {
                {
                  if (s_len + 1 > 0) {
                    volatile size_t num_to_copy =
                        (size_t)(s_len + 1 - 1) < (size_t)(s_len)
                            ? (size_t)(s_len + 1 - 1)
                            : (size_t)(s_len);
                    strncpy(name_copy, name, num_to_copy);
                    name_copy[num_to_copy] = 0;
                  }
                };
              }
            }
          } while (0);
          h->nameA = name_copy;
          h->nameW =((void *)0);
          h->extra1 = 0;
          h->extra2 =((void *)0);
          h->next =((void *)0);
          if (h_tail ==((void *)0))
            (&__itt__ittapi_global)->domain_list = h;
          else
            h_tail->next = h;
        }
      };
    }
  }
  if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
       pthread_mutex_destroy && pthread_mutexattr_init &&
       pthread_mutexattr_settype && pthread_mutexattr_destroy && pthread_self))
    pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  return h;
}

static void
__itt_module_load_with_sections_init_3_0(__itt_module_object *module_obj) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }
  if (__itt_module_load_with_sections_ptr__3_0 &&
      __itt_module_load_with_sections_ptr__3_0 !=
          __itt_module_load_with_sections_init_3_0) {
    if (module_obj !=((void *)0)) {
      module_obj->version = 1;
      __itt_module_load_with_sections_ptr__3_0(module_obj);
    }
  }
}

static void
__itt_module_unload_with_sections_init_3_0(__itt_module_object *module_obj) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }
  if (__itt_module_unload_with_sections_ptr__3_0 &&
      __itt_module_unload_with_sections_ptr__3_0 !=
          __itt_module_unload_with_sections_init_3_0) {
    if (module_obj !=((void *)0)) {
      module_obj->version = 1;
      __itt_module_unload_with_sections_ptr__3_0(module_obj);
    }
  }
}
static __itt_string_handle *
__itt_string_handle_create_init_3_0(const char *name)

{
  __itt_string_handle *h_tail =((void *)0),
                      *h =((void *)0);

  if (name ==((void *)0)) {
    return((void *)0);
  }

  {
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self)) {
      if (!__itt__ittapi_global.mutex_initialized) {
        if (__itt_interlocked_compare_exchange(
                &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
          {
            pthread_mutexattr_t mutex_attr;
            int error_code = pthread_mutexattr_init(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                 error_code);
            error_code =
                pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_settype", error_code);
            error_code =
                pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                 error_code);
            error_code = pthread_mutexattr_destroy(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_destroy", error_code);
          };
          __itt__ittapi_global.mutex_initialized = 1;
        } else
          while (!__itt__ittapi_global.mutex_initialized)
            sched_yield();
      }
      pthread_mutex_lock(&__itt__ittapi_global.mutex);
    }
  };
  if (__itt__ittapi_global.api_initialized) {

    if (__itt_string_handle_create_ptr__3_0 &&
        __itt_string_handle_create_ptr__3_0 !=
            __itt_string_handle_create_init_3_0) {
      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);
      return __itt_string_handle_create_ptr__3_0(name);
    }

    else {

      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);

      return((void *)0);
    }
  }
  if (__itt_is_collector_available()) {
    for (h_tail =((void *)0),
        h = __itt__ittapi_global.string_list;
         h !=((void *)0);
         h_tail = h, h = h->next) {
      if (h->strA !=((void *)0)&& !strcmp(h->strA, name))
        break;
    }
    if (h ==((void *)0)) {
      {
        h = (__itt_string_handle *)malloc(sizeof(__itt_string_handle));
        if (h !=((void *)0)) {
          char *name_copy =((void *)0);
          do {
            if (name !=((void *)0)) {
              size_t s_len = strlen(name);
              name_copy = (char *)malloc(s_len + 1);
              if (name_copy !=((void *)0)) {
                {
                  if (s_len + 1 > 0) {
                    volatile size_t num_to_copy =
                        (size_t)(s_len + 1 - 1) < (size_t)(s_len)
                            ? (size_t)(s_len + 1 - 1)
                            : (size_t)(s_len);
                    strncpy(name_copy, name, num_to_copy);
                    name_copy[num_to_copy] = 0;
                  }
                };
              }
            }
          } while (0);
          h->strA = name_copy;
          h->strW =((void *)0);
          h->extra1 = 0;
          h->extra2 =((void *)0);
          h->next =((void *)0);
          if (h_tail ==((void *)0))
            (&__itt__ittapi_global)->string_list = h;
          else
            h_tail->next = h;
        }
      };
    }
  }
  if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
       pthread_mutex_destroy && pthread_mutexattr_init &&
       pthread_mutexattr_settype && pthread_mutexattr_destroy && pthread_self))
    pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  return h;
}
static __itt_counter __itt_counter_create_init_3_0(const char *name,
                                                   const char *domain)

{
  __itt_counter_info_t *h_tail =((void *)0),
                       *h =((void *)0);
  __itt_metadata_type type = __itt_metadata_u64;

  if (name ==((void *)0)) {
    return((void *)0);
  }

  {
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self)) {
      if (!__itt__ittapi_global.mutex_initialized) {
        if (__itt_interlocked_compare_exchange(
                &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
          {
            pthread_mutexattr_t mutex_attr;
            int error_code = pthread_mutexattr_init(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                 error_code);
            error_code =
                pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_settype", error_code);
            error_code =
                pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                 error_code);
            error_code = pthread_mutexattr_destroy(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_destroy", error_code);
          };
          __itt__ittapi_global.mutex_initialized = 1;
        } else
          while (!__itt__ittapi_global.mutex_initialized)
            sched_yield();
      }
      pthread_mutex_lock(&__itt__ittapi_global.mutex);
    }
  };
  if (__itt__ittapi_global.api_initialized) {

    if (__itt_counter_create_ptr__3_0 &&
        __itt_counter_create_ptr__3_0 != __itt_counter_create_init_3_0) {
      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);
      return __itt_counter_create_ptr__3_0(name, domain);
    }

    else {

      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);

      return((void *)0);
    }
  }
  if (__itt_is_collector_available()) {
    for (h_tail =((void *)0),
        h = __itt__ittapi_global.counter_list;
         h !=((void *)0);
         h_tail = h, h = h->next) {
      if (h->nameA !=((void *)0)&& h->type == (int)type && !strcmp(h->nameA, name) &&
          ((h->domainA ==((void *)0)&& domain ==((void *)0)) ||
           (h->domainA !=((void *)0)&& domain !=((void *)0)&& !strcmp(h->domainA, domain))))
        break;
    }
    if (h ==((void *)0)) {
      {
        h = (__itt_counter_info_t *)malloc(sizeof(__itt_counter_info_t));
        if (h !=((void *)0)) {
          char *name_copy =((void *)0);
          do {
            if (name !=((void *)0)) {
              size_t s_len = strlen(name);
              name_copy = (char *)malloc(s_len + 1);
              if (name_copy !=((void *)0)) {
                {
                  if (s_len + 1 > 0) {
                    volatile size_t num_to_copy =
                        (size_t)(s_len + 1 - 1) < (size_t)(s_len)
                            ? (size_t)(s_len + 1 - 1)
                            : (size_t)(s_len);
                    strncpy(name_copy, name, num_to_copy);
                    name_copy[num_to_copy] = 0;
                  }
                };
              }
            }
          } while (0);
          h->nameA = name_copy;
          h->nameW =((void *)0);
          char *domain_copy =((void *)0);
          do {
            if (domain !=((void *)0)) {
              size_t s_len = strlen(domain);
              domain_copy = (char *)malloc(s_len + 1);
              if (domain_copy !=((void *)0)) {
                {
                  if (s_len + 1 > 0) {
                    volatile size_t num_to_copy =
                        (size_t)(s_len + 1 - 1) < (size_t)(s_len)
                            ? (size_t)(s_len + 1 - 1)
                            : (size_t)(s_len);
                    strncpy(domain_copy, domain, num_to_copy);
                    domain_copy[num_to_copy] = 0;
                  }
                };
              }
            }
          } while (0);
          h->domainA = domain_copy;
          h->domainW =((void *)0);
          h->type = type;
          h->index = 0;
          h->next =((void *)0);
          if (h_tail ==((void *)0))
            (&__itt__ittapi_global)->counter_list = h;
          else
            h_tail->next = h;
        }
      };
    }
  }
  if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
       pthread_mutex_destroy && pthread_mutexattr_init &&
       pthread_mutexattr_settype && pthread_mutexattr_destroy && pthread_self))
    pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  return (__itt_counter)h;
}
static __itt_counter
__itt_counter_create_typed_init_3_0(const char *name, const char *domain,
                                    __itt_metadata_type type)

{
  __itt_counter_info_t *h_tail =((void *)0),
                       *h =((void *)0);

  if (name ==((void *)0)) {
    return((void *)0);
  }

  {
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self)) {
      if (!__itt__ittapi_global.mutex_initialized) {
        if (__itt_interlocked_compare_exchange(
                &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
          {
            pthread_mutexattr_t mutex_attr;
            int error_code = pthread_mutexattr_init(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                 error_code);
            error_code =
                pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_settype", error_code);
            error_code =
                pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                 error_code);
            error_code = pthread_mutexattr_destroy(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_destroy", error_code);
          };
          __itt__ittapi_global.mutex_initialized = 1;
        } else
          while (!__itt__ittapi_global.mutex_initialized)
            sched_yield();
      }
      pthread_mutex_lock(&__itt__ittapi_global.mutex);
    }
  };
  if (__itt__ittapi_global.api_initialized) {

    if (__itt_counter_create_typed_ptr__3_0 &&
        __itt_counter_create_typed_ptr__3_0 !=
            __itt_counter_create_typed_init_3_0) {
      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);
      return __itt_counter_create_typed_ptr__3_0(name, domain, type);
    }

    else {

      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);

      return((void *)0);
    }
  }
  if (__itt_is_collector_available()) {
    for (h_tail =((void *)0),
        h = __itt__ittapi_global.counter_list;
         h !=((void *)0);
         h_tail = h, h = h->next) {
      if (h->nameA !=((void *)0)&& h->type == (int)type && !strcmp(h->nameA, name) &&
          ((h->domainA ==((void *)0)&& domain ==((void *)0)) ||
           (h->domainA !=((void *)0)&& domain !=((void *)0)&& !strcmp(h->domainA, domain))))
        break;
    }
    if (h ==((void *)0)) {
      {
        h = (__itt_counter_info_t *)malloc(sizeof(__itt_counter_info_t));
        if (h !=((void *)0)) {
          char *name_copy =((void *)0);
          do {
            if (name !=((void *)0)) {
              size_t s_len = strlen(name);
              name_copy = (char *)malloc(s_len + 1);
              if (name_copy !=((void *)0)) {
                {
                  if (s_len + 1 > 0) {
                    volatile size_t num_to_copy =
                        (size_t)(s_len + 1 - 1) < (size_t)(s_len)
                            ? (size_t)(s_len + 1 - 1)
                            : (size_t)(s_len);
                    strncpy(name_copy, name, num_to_copy);
                    name_copy[num_to_copy] = 0;
                  }
                };
              }
            }
          } while (0);
          h->nameA = name_copy;
          h->nameW =((void *)0);
          char *domain_copy =((void *)0);
          do {
            if (domain !=((void *)0)) {
              size_t s_len = strlen(domain);
              domain_copy = (char *)malloc(s_len + 1);
              if (domain_copy !=((void *)0)) {
                {
                  if (s_len + 1 > 0) {
                    volatile size_t num_to_copy =
                        (size_t)(s_len + 1 - 1) < (size_t)(s_len)
                            ? (size_t)(s_len + 1 - 1)
                            : (size_t)(s_len);
                    strncpy(domain_copy, domain, num_to_copy);
                    domain_copy[num_to_copy] = 0;
                  }
                };
              }
            }
          } while (0);
          h->domainA = domain_copy;
          h->domainW =((void *)0);
          h->type = type;
          h->index = 0;
          h->next =((void *)0);
          if (h_tail ==((void *)0))
            (&__itt__ittapi_global)->counter_list = h;
          else
            h_tail->next = h;
        }
      };
    }
  }
  if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
       pthread_mutex_destroy && pthread_mutexattr_init &&
       pthread_mutexattr_settype && pthread_mutexattr_destroy && pthread_self))
    pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  return (__itt_counter)h;
}
static __itt_histogram *
__itt_histogram_create_init_3_0(const __itt_domain *domain, const char *name,
                                __itt_metadata_type x_type,
                                __itt_metadata_type y_type)

{
  __itt_histogram *h_tail =((void *)0),
                  *h =((void *)0);

  if (domain ==((void *)0)|| name ==((void *)0)) {
    return((void *)0);
  }

  {
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self)) {
      if (!__itt__ittapi_global.mutex_initialized) {
        if (__itt_interlocked_compare_exchange(
                &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
          {
            pthread_mutexattr_t mutex_attr;
            int error_code = pthread_mutexattr_init(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                 error_code);
            error_code =
                pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_settype", error_code);
            error_code =
                pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                 error_code);
            error_code = pthread_mutexattr_destroy(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_destroy", error_code);
          };
          __itt__ittapi_global.mutex_initialized = 1;
        } else
          while (!__itt__ittapi_global.mutex_initialized)
            sched_yield();
      }
      pthread_mutex_lock(&__itt__ittapi_global.mutex);
    }
  };
  if (__itt__ittapi_global.api_initialized) {

    if (__itt_histogram_create_ptr__3_0 &&
        __itt_histogram_create_ptr__3_0 != __itt_histogram_create_init_3_0) {
      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);
      return __itt_histogram_create_ptr__3_0(domain, name, x_type, y_type);
    }

    else {

      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self))
        pthread_mutex_unlock(&__itt__ittapi_global.mutex);

      return((void *)0);
    }
  }
  if (__itt_is_collector_available()) {
    for (h_tail =((void *)0),
        h = __itt__ittapi_global.histogram_list;
         h !=((void *)0);
         h_tail = h, h = h->next) {
      if (h->domain ==((void *)0))
        continue;
      else if (h->domain == domain &&
               h->nameA !=((void *)0)&& !strcmp(h->nameA, name))
        break;
    }
    if (h ==((void *)0)) {
      {
        h = (__itt_histogram *)malloc(sizeof(__itt_histogram));
        if (h !=((void *)0)) {
          h->domain = domain;
          char *name_copy =((void *)0);
          do {
            if (name !=((void *)0)) {
              size_t s_len = strlen(name);
              name_copy = (char *)malloc(s_len + 1);
              if (name_copy !=((void *)0)) {
                {
                  if (s_len + 1 > 0) {
                    volatile size_t num_to_copy =
                        (size_t)(s_len + 1 - 1) < (size_t)(s_len)
                            ? (size_t)(s_len + 1 - 1)
                            : (size_t)(s_len);
                    strncpy(name_copy, name, num_to_copy);
                    name_copy[num_to_copy] = 0;
                  }
                };
              }
            }
          } while (0);
          h->nameA = name_copy;
          h->nameW =((void *)0);
          h->x_type = x_type;
          h->y_type = y_type;
          h->extra1 = 0;
          h->extra2 =((void *)0);
          if (h_tail ==((void *)0))
            (&__itt__ittapi_global)->histogram_list = h;
          else
            h_tail->next = h;
        }
      };
    }
  }
  if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
       pthread_mutex_destroy && pthread_mutexattr_init &&
       pthread_mutexattr_settype && pthread_mutexattr_destroy && pthread_self))
    pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  return (__itt_histogram *)h;
}

static void __itt_pause_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }
  if (__itt_pause_ptr__3_0 && __itt_pause_ptr__3_0 != __itt_pause_init_3_0) {
    __itt_pause_ptr__3_0();
  }
}

static void __itt_resume_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }
  if (__itt_resume_ptr__3_0 && __itt_resume_ptr__3_0 != __itt_resume_init_3_0) {
    __itt_resume_ptr__3_0();
  }
}

static void __itt_pause_scoped_init_3_0(__itt_collection_scope scope) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }
  if (__itt_pause_scoped_ptr__3_0 &&
      __itt_pause_scoped_ptr__3_0 != __itt_pause_scoped_init_3_0) {
    __itt_pause_scoped_ptr__3_0(scope);
  }
}

static void __itt_resume_scoped_init_3_0(__itt_collection_scope scope) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }
  if (__itt_resume_scoped_ptr__3_0 &&
      __itt_resume_scoped_ptr__3_0 != __itt_resume_scoped_init_3_0) {
    __itt_resume_scoped_ptr__3_0(scope);
  }
}
static void __itt_thread_set_name_init_3_0(const char *name)

{
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }

  if (__itt_thread_set_name_ptr__3_0 &&
      __itt_thread_set_name_ptr__3_0 != __itt_thread_set_name_init_3_0) {
    __itt_thread_set_name_ptr__3_0(name);
  }
}
static int __itt_thr_name_set_init_3_0(const char *name, int namelen) {
  (void)namelen;
  __itt_thread_set_name_init_3_0(name);
  return 0;
}

static void __itt_thread_ignore_init_3_0(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }
  if (__itt_thread_ignore_ptr__3_0 &&
      __itt_thread_ignore_ptr__3_0 != __itt_thread_ignore_init_3_0) {
    __itt_thread_ignore_ptr__3_0();
  }
}

static void __itt_thr_ignore_init_3_0(void) { __itt_thread_ignore_init_3_0(); }

static void __itt_enable_attach_init_3_0(void) {}

static const char *__itt_fsplit(const char *s, const char *sep,
                                const char **out, int *len) {
  int i;
  int j;

  if (!s || !sep || !out || !len)
    return((void *)0);

  for (i = 0; s[i]; i++) {
    int b = 0;
    for (j = 0; sep[j]; j++)
      if (s[i] == sep[j]) {
        b = 1;
        break;
      }
    if (!b)
      break;
  }

  if (!s[i])
    return((void *)0);

  *len = 0;
  *out = &s[i];

  for (; s[i]; i++, (*len)++) {
    int b = 0;
    for (j = 0; sep[j]; j++)
      if (s[i] == sep[j]) {
        b = 1;
        break;
      }
    if (b)
      break;
  }

  for (; s[i]; i++) {
    int b = 0;
    for (j = 0; sep[j]; j++)
      if (s[i] == sep[j]) {
        b = 1;
        break;
      }
    if (!b)
      break;
  }

  return &s[i];
}

static const char *__itt_get_env_var(const char *name) {

  static char env_buff[4086];
  static char *env_value = (char *)env_buff;

  if (name !=((void *)0)) {
    char *env = getenv(name);
    if (env !=((void *)0)) {
      size_t len = strlen(env);
      size_t max_len = 4086 - (size_t)(env_value - env_buff);
      if (len < max_len) {
        const char *ret = (const char *)env_value;
        {
          if (max_len > 0) {
            volatile size_t num_to_copy =
                (size_t)(max_len - 1) < (size_t)(len + 1)
                    ? (size_t)(max_len - 1)
                    : (size_t)(len + 1);
            strncpy(env_value, env, num_to_copy);
            env_value[num_to_copy] = 0;
          }
        };
        env_value += len + 1;
        return ret;
      } else
        __itt_report_error(__itt_error_env_too_long, name, (size_t)len,
                           (size_t)(max_len - 1));
    }
  }
  return((void *)0);
}

static const char *__itt_get_lib_name(void) {
  const char *lib_name = __itt_get_env_var("INTEL_LIBITTNOTIFY64");
  return lib_name;
}

static __itt_group_id __itt_get_groups(void) {
  int i;
  __itt_group_id res = __itt_group_none;
  const char *var_name = "INTEL_ITTNOTIFY_GROUPS";
  const char *group_str = __itt_get_env_var(var_name);

  if (group_str !=((void *)0)) {
    int len;
    char gr[255];
    const char *chunk;
    while ((group_str = __itt_fsplit(group_str, ",; ", &chunk, &len)) !=((void *)0)) {
      int min_len =
          ((len) < ((int)(sizeof(gr) - 1)) ? (len) : ((int)(sizeof(gr) - 1)));
      {
        if (sizeof(gr) - 1 > 0) {
          volatile size_t num_to_copy =
              (size_t)(sizeof(gr) - 1 - 1) < (size_t)(min_len)
                  ? (size_t)(sizeof(gr) - 1 - 1)
                  : (size_t)(min_len);
          strncpy(gr, chunk, num_to_copy);
          gr[num_to_copy] = 0;
        }
      };
      gr[min_len] = 0;

      for (i = 0; group_list[i].name !=((void *)0);
           i++) {
        if (!strcmp(gr, group_list[i].name)) {
          res = (__itt_group_id)(res | group_list[i].id);
          break;
        }
      }
    }

    for (i = 0; group_list[i].id != __itt_group_none; i++)
      if (group_list[i].id != __itt_group_all &&
          group_list[i].id > __itt_group_splitter_min &&
          group_list[i].id < __itt_group_splitter_max)
        res = (__itt_group_id)(res | group_list[i].id);
    return res;
  } else {
    for (i = 0; group_alias[i].env_var !=((void *)0);
         i++)
      if (__itt_get_env_var(group_alias[i].env_var) !=((void *)0))
        return group_alias[i].groups;
  }

  return res;
}

static int __itt_lib_version(lib_t lib) {
  if (lib ==((void *)0))
    return 0;
  if (dlsym(lib, "__itt_api_init"))
    return 2;
  if (dlsym(lib, "__itt_api_version"))
    return 1;
  return 0;
}
static void __itt_nullify_all_pointers(void) {
  int i;

  for (i = 0; __itt__ittapi_global.api_list_ptr[i].name !=((void *)0);
       i++)
    *__itt__ittapi_global.api_list_ptr[i].func_ptr =
        __itt__ittapi_global.api_list_ptr[i].null_func;
}

static int __itt_is_collector_available(void) {
  int is_available;

  {
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self)) {
      if (!__itt__ittapi_global.mutex_initialized) {
        if (__itt_interlocked_compare_exchange(
                &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
          {
            pthread_mutexattr_t mutex_attr;
            int error_code = pthread_mutexattr_init(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                 error_code);
            error_code =
                pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_settype", error_code);
            error_code =
                pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                 error_code);
            error_code = pthread_mutexattr_destroy(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_destroy", error_code);
          };
          __itt__ittapi_global.mutex_initialized = 1;
        } else
          while (!__itt__ittapi_global.mutex_initialized)
            sched_yield();
      }
      pthread_mutex_lock(&__itt__ittapi_global.mutex);
    }
  };
  if (__itt__ittapi_global.state == __itt_collection_uninitialized) {
    __itt__ittapi_global.state = (((void *)0)== __itt_get_lib_name())
                                     ? __itt_collection_collector_absent
                                     : __itt_collection_collector_exists;
  }
  is_available =
      (__itt__ittapi_global.state == __itt_collection_collector_exists ||
       __itt__ittapi_global.state == __itt_collection_init_successful);
  pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  return is_available;
}
void __itt_fini_ittlib(void) {
  __itt_api_fini_t *__itt_api_fini_ptr =((void *)0);
  static volatile TIDT current_thread = 0;

  if (__itt__ittapi_global.api_initialized) {
    {
      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self)) {
        if (!__itt__ittapi_global.mutex_initialized) {
          if (__itt_interlocked_compare_exchange(
                  &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
            {
              pthread_mutexattr_t mutex_attr;
              int error_code = pthread_mutexattr_init(&mutex_attr);
              if (error_code)
                __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                   error_code);
              error_code = pthread_mutexattr_settype(&mutex_attr,
                                                     PTHREAD_MUTEX_RECURSIVE);
              if (error_code)
                __itt_report_error(__itt_error_system,
                                   "pthread_mutexattr_settype", error_code);
              error_code =
                  pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
              if (error_code)
                __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                   error_code);
              error_code = pthread_mutexattr_destroy(&mutex_attr);
              if (error_code)
                __itt_report_error(__itt_error_system,
                                   "pthread_mutexattr_destroy", error_code);
            };
            __itt__ittapi_global.mutex_initialized = 1;
          } else
            while (!__itt__ittapi_global.mutex_initialized)
              sched_yield();
        }
        pthread_mutex_lock(&__itt__ittapi_global.mutex);
      }
    };
    if (__itt__ittapi_global.api_initialized) {
      if (current_thread == 0) {
        if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
             pthread_mutex_destroy && pthread_mutexattr_init &&
             pthread_mutexattr_settype && pthread_mutexattr_destroy &&
             pthread_self))
          current_thread = pthread_self();
        if (__itt__ittapi_global.lib !=((void *)0)) {
          __itt_api_fini_ptr = (__itt_api_fini_t *)(size_t)dlsym(
              __itt__ittapi_global.lib, "__itt_api_fini");
        }
        if (__itt_api_fini_ptr) {
          __itt_api_fini_ptr(&__itt__ittapi_global);
        }

        __itt_nullify_all_pointers();

        __itt__ittapi_global.api_initialized = 0;
        current_thread = 0;
      }
    }
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self))
      pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  }
}

static void __itt_free_allocated_resources(void) {
  __itt_string_handle *current_string = __itt__ittapi_global.string_list;
  while (current_string !=((void *)0)) {
    __itt_string_handle *tmp = current_string->next;
    free((char *)current_string->strA);

    free(current_string);
    current_string = tmp;
  }
  __itt__ittapi_global.string_list =((void *)0);

  __itt_domain *current_domain = __itt__ittapi_global.domain_list;
  while (current_domain !=((void *)0)) {
    __itt_domain *tmp = current_domain->next;
    free((char *)current_domain->nameA);

    free(current_domain);
    current_domain = tmp;
  }
  __itt__ittapi_global.domain_list =((void *)0);

  __itt_counter_info_t *current_couter = __itt__ittapi_global.counter_list;
  while (current_couter !=((void *)0)) {
    __itt_counter_info_t *tmp = current_couter->next;
    free((char *)current_couter->nameA);
    free((char *)current_couter->domainA);

    free(current_couter);
    current_couter = tmp;
  }
  __itt__ittapi_global.counter_list =((void *)0);

  __itt_histogram *current_histogram = __itt__ittapi_global.histogram_list;
  while (current_histogram !=((void *)0)) {
    __itt_histogram *tmp = current_histogram->next;
    free((char *)current_histogram->nameA);

    free(current_histogram);
    current_histogram = tmp;
  }
  __itt__ittapi_global.histogram_list =((void *)0);
}

int __itt_init_ittlib(const char *lib_name, __itt_group_id init_groups) {
  int i;
  __itt_group_id groups;

  static volatile TIDT current_thread = 0;

  if (!__itt__ittapi_global.api_initialized) {

    {
      if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
           pthread_mutex_destroy && pthread_mutexattr_init &&
           pthread_mutexattr_settype && pthread_mutexattr_destroy &&
           pthread_self)) {
        if (!__itt__ittapi_global.mutex_initialized) {
          if (__itt_interlocked_compare_exchange(
                  &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
            {
              pthread_mutexattr_t mutex_attr;
              int error_code = pthread_mutexattr_init(&mutex_attr);
              if (error_code)
                __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                   error_code);
              error_code = pthread_mutexattr_settype(&mutex_attr,
                                                     PTHREAD_MUTEX_RECURSIVE);
              if (error_code)
                __itt_report_error(__itt_error_system,
                                   "pthread_mutexattr_settype", error_code);
              error_code =
                  pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
              if (error_code)
                __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                   error_code);
              error_code = pthread_mutexattr_destroy(&mutex_attr);
              if (error_code)
                __itt_report_error(__itt_error_system,
                                   "pthread_mutexattr_destroy", error_code);
            };
            __itt__ittapi_global.mutex_initialized = 1;
          } else
            while (!__itt__ittapi_global.mutex_initialized)
              sched_yield();
        }
        pthread_mutex_lock(&__itt__ittapi_global.mutex);
      }
    };

    if (!__itt__ittapi_global.api_initialized) {
      if (current_thread == 0) {
        if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
             pthread_mutex_destroy && pthread_mutexattr_init &&
             pthread_mutexattr_settype && pthread_mutexattr_destroy &&
             pthread_self))
          current_thread = pthread_self();
        if (lib_name ==((void *)0)) {
          lib_name = __itt_get_lib_name();
        }
        groups = __itt_get_groups();
        if ((dlopen && dlsym && dlclose) &&
            (groups != __itt_group_none || lib_name !=((void *)0))) {
          __itt__ittapi_global.lib = dlopen((lib_name ==((void *)0))
                                                ? ittnotify_lib_name
                                                : lib_name,
                                            0x00001
          );

          if (__itt__ittapi_global.lib !=((void *)0)) {
            __itt__ittapi_global.state = __itt_collection_init_successful;
            __itt_api_init_t *__itt_api_init_ptr;
            int lib_version = __itt_lib_version(__itt__ittapi_global.lib);

            switch (lib_version) {
            case 0:
              groups = __itt_group_legacy;
              __attribute__((fallthrough));
            case 1:

              for (i = 0; __itt__ittapi_global.api_list_ptr[i].name !=((void *)0);
                   i++) {
                if (__itt__ittapi_global.api_list_ptr[i].group & groups &
                    init_groups) {
                  *__itt__ittapi_global.api_list_ptr[i].func_ptr =
                      (void *)dlsym(__itt__ittapi_global.lib,
                                    __itt__ittapi_global.api_list_ptr[i].name);
                  if (*__itt__ittapi_global.api_list_ptr[i].func_ptr ==((void *)0)) {

                    *__itt__ittapi_global.api_list_ptr[i].func_ptr =
                        __itt__ittapi_global.api_list_ptr[i].null_func;
                    __itt_report_error(
                        __itt_error_no_symbol, lib_name,
                        __itt__ittapi_global.api_list_ptr[i].name);
                  }
                } else
                  *__itt__ittapi_global.api_list_ptr[i].func_ptr =
                      __itt__ittapi_global.api_list_ptr[i].null_func;
              }

              if (groups == __itt_group_legacy) {

                __itt_thread_ignore_ptr__3_0 = __itt_thr_ignore_ptr__3_0;

                __itt_sync_create_ptr__3_0 = __itt_sync_set_name_ptr__3_0;

                __itt_sync_prepare_ptr__3_0 =
                    __itt_notify_sync_prepare_ptr__3_0;
                __itt_sync_cancel_ptr__3_0 = __itt_notify_sync_cancel_ptr__3_0;
                __itt_sync_acquired_ptr__3_0 =
                    __itt_notify_sync_acquired_ptr__3_0;
                __itt_sync_releasing_ptr__3_0 =
                    __itt_notify_sync_releasing_ptr__3_0;
              }

              break;
            case 2:
              __itt_api_init_ptr = (__itt_api_init_t *)(size_t)dlsym(
                  __itt__ittapi_global.lib, "__itt_api_init");
              if (__itt_api_init_ptr)
                __itt_api_init_ptr(&__itt__ittapi_global, init_groups);
              break;
            }
          } else {
            __itt__ittapi_global.state = __itt_collection_init_fail;
            __itt_free_allocated_resources();
            __itt_nullify_all_pointers();

            __itt_report_error(__itt_error_no_module, lib_name,

                               dlerror()

            );
          }
        } else {
          __itt__ittapi_global.state = __itt_collection_collector_absent;
          __itt_nullify_all_pointers();
        }
        __itt__ittapi_global.api_initialized = 1;
        current_thread = 0;

        if (__itt_fini_ittlib_ptr == __itt_fini_ittlib)
          current_thread = 0;
      }
    }

    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self))
      pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  }

  for (i = 0; __itt__ittapi_global.api_list_ptr[i].name !=((void *)0);
       i++) {
    if (*__itt__ittapi_global.api_list_ptr[i].func_ptr !=
            __itt__ittapi_global.api_list_ptr[i].null_func &&
        __itt__ittapi_global.api_list_ptr[i].group & init_groups) {
      return 1;
    }
  }
  return 0;
}

__itt_error_handler_t *__itt_set_error_handler(__itt_error_handler_t *handler) {
  __itt_error_handler_t *prev =
      (__itt_error_handler_t *)(size_t)__itt__ittapi_global.error_handler;
  __itt__ittapi_global.error_handler = (void *)(size_t)handler;
  return prev;
}
void __itt_mark_pt_region_begin(__itt_pt_region region) {
  (void)region;
}

void __itt_mark_pt_region_end(__itt_pt_region region) {
  (void)region;
}

__itt_collection_state(__itt_get_collection_state)(void) {
  if (!__itt__ittapi_global.api_initialized &&
      __itt__ittapi_global.thread_list ==((void *)0)) {
    __itt_init_ittlib_ptr(((void *)0),
        __itt_group_all);
  }
  return __itt__ittapi_global.state;
}

void(__itt_release_resources)(void) {
  {
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self)) {
      if (!__itt__ittapi_global.mutex_initialized) {
        if (__itt_interlocked_compare_exchange(
                &__itt__ittapi_global.atomic_counter, 1, 0) == 0) {
          {
            pthread_mutexattr_t mutex_attr;
            int error_code = pthread_mutexattr_init(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutexattr_init",
                                 error_code);
            error_code =
                pthread_mutexattr_settype(&mutex_attr, PTHREAD_MUTEX_RECURSIVE);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_settype", error_code);
            error_code =
                pthread_mutex_init(&__itt__ittapi_global.mutex, &mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system, "pthread_mutex_init",
                                 error_code);
            error_code = pthread_mutexattr_destroy(&mutex_attr);
            if (error_code)
              __itt_report_error(__itt_error_system,
                                 "pthread_mutexattr_destroy", error_code);
          };
          __itt__ittapi_global.mutex_initialized = 1;
        } else
          while (!__itt__ittapi_global.mutex_initialized)
            sched_yield();
      }
      pthread_mutex_lock(&__itt__ittapi_global.mutex);
    }
  };
  __itt_free_allocated_resources();
  if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
       pthread_mutex_destroy && pthread_mutexattr_init &&
       pthread_mutexattr_settype && pthread_mutexattr_destroy && pthread_self))
    pthread_mutex_unlock(&__itt__ittapi_global.mutex);
  {
    if ((pthread_mutex_init && pthread_mutex_lock && pthread_mutex_unlock &&
         pthread_mutex_destroy && pthread_mutexattr_init &&
         pthread_mutexattr_settype && pthread_mutexattr_destroy &&
         pthread_self)) {
      if (__itt__ittapi_global.mutex_initialized) {
        if (__itt_interlocked_compare_exchange(
                &__itt__ittapi_global.atomic_counter, 0, 1) == 1) {
          pthread_mutex_destroy(&__itt__ittapi_global.mutex);
          __itt__ittapi_global.mutex_initialized = 0;
        }
      }
    }
  };
}
