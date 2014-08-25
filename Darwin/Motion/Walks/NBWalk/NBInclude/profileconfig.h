
#ifndef _profileconfig_h
#define _profileconfig_h


// Turn on/off profiling function calls
#define USE_TIME_PROFILING_OFF
#ifdef  USE_TIME_PROFILING_ON
#  define USE_TIME_PROFILING
#else
#  undef  USE_TIME_PROFILING
#endif

// Turn on/off automatic profiling summary printing
#define USE_PROFILER_AUTO_PRINT_OFF
#ifdef  USE_PROFILER_AUTO_PRINT_ON
#  define USE_PROFILER_AUTO_PRINT
#else
#  undef  USE_PROFILER_AUTO_PRINT
#endif


#endif // !_profileconfig_h
