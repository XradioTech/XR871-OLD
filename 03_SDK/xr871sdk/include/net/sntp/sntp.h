#ifndef _NET_SNTP_H__
#define _NET_SNTP_H__

typedef struct {
        uint8_t sec;       /**< Seconds after the minute   - [0,59]  */
        uint8_t min;       /**< Minutes after the hour     - [0,59]  */
        uint8_t hour;      /**< Hours after the midnight   - [0,23]  */
        uint8_t day;       /**< Day of the month           - [1,31]  */
        uint8_t mon;       /**< Months                     - [1,12]  */
        uint8_t week;      /**< Days in a week             - [0,6]   */
        uint8_t year;      /**< Years                      - [0,127] */
} sntp_time;

sntp_time *sntp_obtain_time(void);
int sntp_request(void *arg);

#endif /* _NET_SNTP_H__ */
