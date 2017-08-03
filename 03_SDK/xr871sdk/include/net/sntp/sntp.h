#ifndef __SNTP_H__
#define __SNTP_H__

typedef struct {
        uint8_t sec;       /**< Seconds after the minute   - [0,59]  */
        uint8_t min;       /**< Minutes after the hour     - [0,59]  */
        uint8_t hour;      /**< Hours after the midnight   - [0,23]  */
        uint8_t day;       /**< Day of the month           - [1,31]  */
        uint8_t mon;       /**< Months                     - [1,12]  */
        uint8_t week;      /**< Days in a week             - [0,6]   */
        uint8_t year;      /**< Years                      - [0,127] */
} sntp_time;

void* sntp_obtain_time();
void sntp_thread(void *arg);
void sntp_thread_stop();
int sntp_request(void *arg);
#endif /* __SNTP_H__ */
