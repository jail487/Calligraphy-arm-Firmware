#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "cmsis_os.h"

// 前向宣告 clock_gettime 以避免隱式宣告警告
int clock_gettime(int clock_id, struct timespec * tp);

// 定義 rcutils 類型以避免依賴標頭檔
typedef int64_t rcutils_time_point_value_t;
typedef int rcutils_ret_t;
#define RCUTILS_RET_OK 0
#define RCUTILS_RET_INVALID_ARGUMENT 1
#define RCUTILS_RET_ERROR 2

#define MICROSECONDS_PER_SECOND    ( 1000000LL )                                   /**< Microseconds per second. */
#define NANOSECONDS_PER_SECOND     ( 1000000000LL )                                /**< Nanoseconds per second. */
#define NANOSECONDS_PER_TICK       ( NANOSECONDS_PER_SECOND / configTICK_RATE_HZ ) /**< Nanoseconds per FreeRTOS tick. */

// 補丁：實現 usleep (micro-ROS 需要)
int usleep(useconds_t usec)
{
    osDelay(usec / 1000);
    return 0;
}

// 補丁：實現 _gettimeofday (Newlib Nano 需要)
int _gettimeofday(struct timeval *tv, void *tzvp)
{
    if (tv != NULL) {
        // 簡單實現：使用 RTOS ticks
        uint64_t ticks = xTaskGetTickCount();
        tv->tv_sec = ticks / configTICK_RATE_HZ;
        tv->tv_usec = (ticks % configTICK_RATE_HZ) * (1000000 / configTICK_RATE_HZ);
    }
    return 0;
}

// 補丁：實現 rcutils_system_time_now (如果靜態庫遺失此符號)
rcutils_ret_t rcutils_system_time_now(rcutils_time_point_value_t * now)
{
    if (!now) return RCUTILS_RET_INVALID_ARGUMENT;
    
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    *now = ((int64_t)ts.tv_sec * 1000000000LL) + ts.tv_nsec;
    return RCUTILS_RET_OK;
}

rcutils_ret_t rcutils_steady_time_now(rcutils_time_point_value_t * now)
{
    return rcutils_system_time_now(now);
}

void UTILS_NanosecondsToTimespec( int64_t llSource,
                                  struct timespec * const pxDestination )
{
    long lCarrySec = 0;

    /* Convert to timespec. */
    pxDestination->tv_sec = ( time_t ) ( llSource / NANOSECONDS_PER_SECOND );
    pxDestination->tv_nsec = ( long ) ( llSource % NANOSECONDS_PER_SECOND );

    /* Subtract from tv_sec if tv_nsec < 0. */
    if( pxDestination->tv_nsec < 0L )
    {
        /* Compute the number of seconds to carry. */
        lCarrySec = ( pxDestination->tv_nsec / ( long ) NANOSECONDS_PER_SECOND ) + 1L;

        pxDestination->tv_sec -= ( time_t ) ( lCarrySec );
        pxDestination->tv_nsec += lCarrySec * ( long ) NANOSECONDS_PER_SECOND;
    }
}

int clock_gettime( int clock_id,
                   struct timespec * tp )
{
    TimeOut_t xCurrentTime = { 0 };

    /* Intermediate variable used to convert TimeOut_t to struct timespec.
     * Also used to detect overflow issues. It must be unsigned because the
     * behavior of signed integer overflow is undefined. */
    uint64_t ullTickCount = 0ULL;

    /* Silence warnings about unused parameters. */
    ( void ) clock_id;

    /* Get the current tick count and overflow count. vTaskSetTimeOutState()
     * is used to get these values because they are both static in tasks.c. */
    vTaskSetTimeOutState( &xCurrentTime );

    /* Adjust the tick count for the number of times a TickType_t has overflowed.
     * portMAX_DELAY should be the maximum value of a TickType_t. */
    ullTickCount = ( uint64_t ) ( xCurrentTime.xOverflowCount ) << ( sizeof( TickType_t ) * 8 );

    /* Add the current tick count. */
    ullTickCount += xCurrentTime.xTimeOnEntering;

    /* Convert ullTickCount to timespec. */
    UTILS_NanosecondsToTimespec( ( int64_t ) ullTickCount * NANOSECONDS_PER_TICK, tp );

    return 0;
}