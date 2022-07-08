#ifndef RTC_H
#define RTC_H
#include <stdint.h>
#include <time.h>

typedef struct _DateTime  DateTime_t;// определено в r3_object.h

void rtc_time_set  (const struct tm *timer);
void rtc_time_get  (struct tm *timer);
void rtc_open  (int32_t flag_idx);// подписка на периодическое срабатывание. период 1 сек.
void rtc_alarmA_set(const struct tm *timer, int32_t flag_idx);// установить время следующего срабатывания
void rtc_alarmB_set(const struct tm *timer, int32_t flag_idx);// установить время следующего срабатывания

#endif//RTC_H
