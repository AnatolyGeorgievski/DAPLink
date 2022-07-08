/*! 
\see [RM0366] Real-time clock (RTC)
таймер реального времени, считает дату и время в bcd формате.

	\n[ 0.. 5] (6 бит) секунды
	\n[ 6..11] (6 бит) минуты
	\n[12..16] (5 бит) часы

	\todo флаг валидности времени, устанавливается только когда небыло отключения питания и была произведена настройка времени, вызов функции rtc_set_date().
	
Задача
Расписание и времена строятся на базе DateTime

 */
#include "board.h"
#include "rtc.h"
#include "gd32e10x_rtc.h"
#include "cmsis_os.h"
#include <stdio.h>

#define RTC_EXTI_LINE_TIMESTAMP_EVENT (1<<21)	//!< External interrupt line 21 Connected to the RTC Tamper and Time Stamp events
#define RTC_EXTI_LINE_ALARM_EVENT (1<<17)
#define RCC_DBP_TIMEOUT_VALUE 100 /* ms */

//extern uint32_t HAL_GetTick();
void rtc_configuration_mode_enter()
{
	RTC_CTL |= RTC_CTL_CMF;
}
void rtc_configuration_mode_exit()
{
    RTC_CTL &= ~RTC_CTL_CMF;
}
/* RTC register high / low bits mask */
#define RTC_HIGH_BITS_MASK         ((uint32_t)0x000F0000U)  /* RTC high bits mask */
#define RTC_LOW_BITS_MASK          ((uint32_t)0x0000FFFFU)  /* RTC low bits mask */

/* RTC register high bits offset */
#define RTC_HIGH_BITS_OFFSET       ((uint32_t)16U)
#define RTC_SUSEC 32768
void rtc_prescaler_set(uint32_t psc)
{
    rtc_configuration_mode_enter();
    /* set the RTC prescaler high bits */
    RTC_PSCH = ((psc & RTC_HIGH_BITS_MASK) >> RTC_HIGH_BITS_OFFSET);
    /* set the RTC prescaler low bits */
    RTC_PSCL = (psc & RTC_LOW_BITS_MASK);
    rtc_configuration_mode_exit();
}
static uint32_t rtc_flag=0;
static uint32_t alarmA_flag=0;
static uint32_t alarmB_flag=0;
static osThreadId rtc_owner_thread=NULL;
static osThreadId alarmA_owner_thread=NULL;
static osThreadId alarmB_owner_thread=NULL;

void rtc_lwoff_wait()
{
    /* loop until LWOFF flag is set */
    while(RESET == (RTC_CTL & RTC_CTL_LWOFF)){
    }	
}
uint32_t rtc_counter_get(void)
{
    uint32_t temp;
	do {
		temp  =  RTC_CNTL;
		temp |= (RTC_CNTH << RTC_HIGH_BITS_OFFSET);
	} while (RTC_CNTL != (temp & RTC_LOW_BITS_MASK));
    return temp;
}
void rtc_counter_set(uint32_t cnt)
{
    rtc_configuration_mode_enter();
    /* set the RTC counter high bits */
    RTC_CNTH = (cnt >> RTC_HIGH_BITS_OFFSET);
    /* set the RTC counter low bits */
    RTC_CNTL = (cnt & RTC_LOW_BITS_MASK);
    rtc_configuration_mode_exit();
}
void rtc_alarm_config(uint32_t alarm)
{
    rtc_configuration_mode_enter();
    /* set the alarm high bits */
    RTC_ALRMH = (alarm >> RTC_HIGH_BITS_OFFSET);
    /* set the alarm low bits */
    RTC_ALRML = (alarm & RTC_LOW_BITS_MASK);
    rtc_configuration_mode_exit();
}
static void __attribute__((constructor)) rtc_init()//void* data)
{
	RCU_APB1EN |= (RCU_APB1EN_PMUEN|RCU_APB1EN_BKPIEN);
	PMU_CTL |= PMU_CTL_BKPWEN; /* разрешить доступ Disable Backup Domain write protection */
    /* reset backup domain */
	RCU_BDCTL |=  RCU_BDCTL_BKPRST;
	RCU_BDCTL &= ~RCU_BDCTL_BKPRST;
#if defined(BOARD_RTC_SOURCE_LSE) 
	RCU_BDCTL |= RCU_BDCTL_LXTALEN; // low speed crystal oscillator (LXTAL) enable
	while (0 == (RCU_CTL & RCU_CTL_LXTALSTB));
	RCU_BDCTL |= (RCU_BDCTL & ~RCU_BDCTL_RTCSRC) | RCU_RTCSRC_LXTAL;
#elif defined(BOARD_RTC_SOURCE_HSE)
	RCU_CTL |= RCU_CTL_HXTALEN; // High speed crystal oscillator (HXTAL) enable
	while (0 == (RCU_CTL & RCU_CTL_HXTALSTB));
	RCU_BDCTL |= (RCU_BDCTL & ~RCU_BDCTL_RTCSRC) | RCU_RTCSRC_HXTAL_DIV_128;
#else
	RCU_RSTSCK|= RCU_RSTSCK_IRC40KEN;
	while (0 == (RCU_RSTSCK & RCU_RSTSCK_IRC40KSTB));
	RCU_BDCTL |= (RCU_BDCTL & ~RCU_BDCTL_RTCSRC) | RCU_RTCSRC_IRC40K;
#endif
	RCU_BDCTL |= RCU_BDCTL_RTCEN;/* enable RTC Clock */
	//      wait RTC registers synchronized flag set
	RTC_CTL &= ~RTC_CTL_RSYNF;/* wait for RTC registers synchronization */
	while(RESET == (RTC_CTL & RTC_CTL_RSYNF)){
    }
	/* wait until last write operation on RTC registers has finished */
	rtc_lwoff_wait();
#if   defined(BOARD_RTC_SOURCE_LSE) 
	rtc_prescaler_set(32767);
#elif defined(BOARD_RTC_SOURCE_HSE)
	rtc_prescaler_set(HXTAL_VALUE/128-1);
#else
	rtc_prescaler_set(40000-1);
#endif
	rtc_lwoff_wait();
    /* enable the RTC second and alarm interrupt*/
    RTC_INTEN |= (RTC_INT_SECOND|RTC_INT_ALARM);
	rtc_lwoff_wait();
	
	NVIC_EnableIRQ(RTC_IRQn);
	printf ("RTC enable\r\n");
}
//static int32_t* rtc_sec_flag =NULL;
static uint32_t timestamp =0;
void RTC_IRQHandler()
{
	if(RESET != (RTC_CTL & RTC_CTL_SCIF)){
		/* clear the RTC second interrupt flag*/
		RTC_CTL &= ~RTC_CTL_SCIF;// second interrupt flag
		++timestamp;// BCD format
/*
		if (((ts>>0) & 0xF)>=10) ts+=6; 	// коррекция BCD
		if (((ts>>4) & 0xF)>= 6) ts+=10<<4; // коррекция BCD 60 сек
		if (((ts>>8) & 0xF)>=10) ts+=6<<8; 	// коррекция BCD
		if (((ts>>12)& 0xF)>= 6) ts+=10<<12;// коррекция BCD 60 min
		if (((ts>>16)& 0xF)>=10) ts+=6<<16; // коррекция BCD
		if (((ts>>16)& 0xFF)>=0x24) {// коррекция BCD 24 часа
			ts-=(0x24<<16); 
			//days++;
		}
		timestamp = ts; */
		if (rtc_owner_thread) {
			osSignalSet (rtc_owner_thread, 1UL<<rtc_flag);
			osThreadNotify(rtc_owner_thread);
		}
//	debug("+");
	}
	if(RESET != (RTC_CTL & RTC_CTL_ALRMIF)){
		RTC_CTL &= ~RTC_CTL_ALRMIF;// Alarm interrupt flag
		if (alarmA_owner_thread) {
			osSignalSet (alarmA_owner_thread, 1UL<<alarmA_flag);
			osThreadNotify(alarmA_owner_thread);
			alarmA_owner_thread = NULL;
		}
			//if (1) debug("AlarmA ");
	}
	if(RESET != (RTC_CTL & RTC_CTL_OVIF)){
		RTC_CTL &= ~RTC_CTL_OVIF;// counter overflow interrupt flag
	}
	__DMB();
}

static unsigned int bcd2bin(uint8_t bcd)
{
	return bcd - (bcd>>4)*6;
}
void rtc_open  (int32_t flag_idx)
{
	rtc_flag = flag_idx;
	rtc_owner_thread = osThreadGetId();
}
/*
 Q = (A* 0xA3D70A3EULL)>>38; Q=A/100
 Q = (A* 0x88888889ULL)>>37; Q=A/60 кратно /15
 Q = (A* 0xCCCСCСCDULL)>>35; Q=A/10 кратно /5 работает для всех A
 Q = (A* 0xCCCСCСCDULL)>>34; Q=A/5 работает для всех A
 Q = (A* 0xAAAAAAABULL)>>33; Q=A/3 работает для всех A
 Q = (A* 0xССCD)>>19; работает до A<81920
 Q = (A* 0xCD)>>11; работает до A<1029
*/
static inline 
unsigned int div10h(uint8_t val){
	return (val* 0xCCCD)>>19;
}
static inline 
unsigned int div100(unsigned int val){
	return (val* 0xA3D70A3EULL)>>38;
}
static inline 
unsigned int div10(unsigned int val){
	return (val* 0xCCCCCCCDULL)>>35;
}

static unsigned int bin2bcd(uint8_t val)
{
//	return ((val/10)*6) + val; // деление на /10 можно выразить через умножение 0xСССD и сдвиг на 19
	return div10h(val)*6+val;
}

void rtc_time_get(struct tm *t)
{
	uint32_t tr,dr, ss;

#if 0 // C11 time

#else // BCD format
	
#endif

}
/*!
	\brief Установить системное время
	
	\param t дата и время в формате DateTime
 */
void rtc_time_set(const struct tm *t)
{
#if 0 // C11 time
	uint32_t dr =(((uint32_t)bin2bcd(t->tm_year%100) << 16) | 
				((uint32_t)bin2bcd(t->tm_mon+1) << 8) | 
				((uint32_t)bin2bcd(t->tm_mday)) | 
				((uint32_t)t->tm_wday << 13));	/* (Sunday = 0) */
    uint32_t ts =(((uint32_t)bin2bcd(t->tm_hour) << 16) |
				  ((uint32_t)bin2bcd(t->tm_min)  <<  8) |
				  ((uint32_t)bin2bcd(t->tm_sec)));
#else // BCD format
	uint32_t dr =(((uint32_t)bcd2bin(t->tm_year) << 16) | 
				  ((uint32_t)bcd2bin(t->tm_mon ) <<  8) | 
				  ((uint32_t)bcd2bin(t->tm_mday)) | 
				  ((uint32_t)bcd2bin(t->tm_wday) << 13));
    uint32_t ts =(((uint32_t)bcd2bin(t->tm_hour)*(RTC_SUSEC*60*60)) +
				  ((uint32_t)bcd2bin(t->tm_min )*(RTC_SUSEC*60)) +
				  ((uint32_t)bcd2bin(t->tm_sec )*RTC_SUSEC));
#endif
	uint32_t ss = t->tm_usec * 10;
// \todo Вынести эту операцию в SVC для атомарности,

}
/*
	большие числа в разрядах таймера маскируют использование разряда.
 */
void rtc_alarmA_set(const struct tm *t, int32_t flag_idx)
{
	
}
