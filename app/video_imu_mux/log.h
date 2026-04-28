#ifndef VIDEO_IMU_MUX_LOG_H
#define VIDEO_IMU_MUX_LOG_H

#include <stdint.h>
#include <stdio.h>
#include <time.h>

/* ------------------------------------------------------------------ */
/* Compile-time log level                                              */
/* Override per-target: -DLOG_LEVEL=LOG_LEVEL_DEBUG                   */
/* ------------------------------------------------------------------ */
#define LOG_LEVEL_ERROR  0
#define LOG_LEVEL_WARN   1
#define LOG_LEVEL_INFO   2
#define LOG_LEVEL_DEBUG  3
#define LOG_LEVEL_TRACE  4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

/* ------------------------------------------------------------------ */
/* Per-translation-unit module tag                                     */
/* Define before #include "log.h":  #define LOG_MODULE "IMU"          */
/* ------------------------------------------------------------------ */
#ifndef LOG_MODULE
#define LOG_MODULE "APP"
#endif

/* ------------------------------------------------------------------ */
/* Timestamp — static inline, zero external linkage                   */
/* ------------------------------------------------------------------ */
static inline uint64_t log_time_us(void)
{
	struct timespec _ts;
	clock_gettime(CLOCK_MONOTONIC, &_ts);
	return (uint64_t)_ts.tv_sec * 1000000ULL + (uint64_t)_ts.tv_nsec / 1000ULL;
}

/* __LINE__ stringification so the format string is a pure literal. */
#define _LOG_STR(x)  #x
#define _LOG_XSTR(x) _LOG_STR(x)

/* ------------------------------------------------------------------ */
/* Internal print primitives — not for direct use                     */
/*                                                                     */
/* _LOG_OUT : stdout, no file:line  (INFO / DEBUG / TRACE)            */
/* _LOG_ERR : stderr + file:line + fflush  (ERROR / WARN)             */
/* ------------------------------------------------------------------ */
#define _LOG_OUT(lvl, fmt, ...) \
	do { \
		uint64_t _t = log_time_us(); \
		fprintf(stdout, \
		        "[%llu.%06llu][" lvl "][" LOG_MODULE "] " fmt, \
		        (unsigned long long)(_t / 1000000ULL), \
		        (unsigned long long)(_t % 1000000ULL), \
		        ##__VA_ARGS__); \
		fflush(stdout); \
	} while (0)

#define _LOG_ERR(lvl, fmt, ...) \
	do { \
		uint64_t _t = log_time_us(); \
		fprintf(stderr, \
		        "[%llu.%06llu][" lvl "][" LOG_MODULE "][" __FILE__ ":" _LOG_XSTR(__LINE__) "] " fmt, \
		        (unsigned long long)(_t / 1000000ULL), \
		        (unsigned long long)(_t % 1000000ULL), \
		        ##__VA_ARGS__); \
		fflush(stderr); \
	} while (0)

/* ------------------------------------------------------------------ */
/* Public level macros                                                 */
/* Disabled levels expand to ((void)0) — zero code generated.         */
/* ------------------------------------------------------------------ */
#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOGE(fmt, ...) _LOG_ERR("E", fmt, ##__VA_ARGS__)
#define LOGP(msg) \
	do { \
		uint64_t _t = log_time_us(); \
		fprintf(stderr, \
		        "[%llu.%06llu][E][" LOG_MODULE "][" __FILE__ ":" _LOG_XSTR(__LINE__) "] ", \
		        (unsigned long long)(_t / 1000000ULL), \
		        (unsigned long long)(_t % 1000000ULL)); \
		perror(msg); \
		fflush(stderr); \
	} while (0)
#else
#define LOGE(fmt, ...) ((void)0)
#define LOGP(msg)      ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOGW(fmt, ...) _LOG_ERR("W", fmt, ##__VA_ARGS__)
#else
#define LOGW(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOGI(fmt, ...) _LOG_OUT("I", fmt, ##__VA_ARGS__)
#else
#define LOGI(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOGD(fmt, ...) _LOG_OUT("D", fmt, ##__VA_ARGS__)
#else
#define LOGD(fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_TRACE
#define LOGT(fmt, ...) _LOG_OUT("T", fmt, ##__VA_ARGS__)
#else
#define LOGT(fmt, ...) ((void)0)
#endif

/* ------------------------------------------------------------------ */
/* Rate-limited variants                                               */
/* Each call site gets its own static timer — no shared global state. */
/* When the underlying level is compiled out the whole block is gone. */
/* ------------------------------------------------------------------ */
#define _LOG_RL(interval_us, macro, fmt, ...) \
	do { \
		static uint64_t _rl_last = 0; \
		uint64_t _rl_now = log_time_us(); \
		if (_rl_now - _rl_last >= (uint64_t)(interval_us)) { \
			_rl_last = _rl_now; \
			macro(fmt, ##__VA_ARGS__); \
		} \
	} while (0)

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOGI_RL(interval_us, fmt, ...) _LOG_RL((interval_us), LOGI, fmt, ##__VA_ARGS__)
#else
#define LOGI_RL(interval_us, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOGD_RL(interval_us, fmt, ...) _LOG_RL((interval_us), LOGD, fmt, ##__VA_ARGS__)
#else
#define LOGD_RL(interval_us, fmt, ...) ((void)0)
#endif

#if LOG_LEVEL >= LOG_LEVEL_TRACE
#define LOGT_RL(interval_us, fmt, ...) _LOG_RL((interval_us), LOGT, fmt, ##__VA_ARGS__)
#else
#define LOGT_RL(interval_us, fmt, ...) ((void)0)
#endif

#endif /* VIDEO_IMU_MUX_LOG_H */
