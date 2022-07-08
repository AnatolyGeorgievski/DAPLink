/*! \ingroup _hal
	\defgroup _hal_tim Таймеры - счетчики
	\{
*/
#ifndef TIM_H
#define TIM_H
#include "gd32e10x_timer.h"
/// источник синхронизации и способ синхронизации
enum {
	TIM_MODE_MASTER = 0,// \see SMC[2:0] Slave mode control
	TIM_MODE_MSM 	=  TIMER_SMCFG_MSM,// выравнивание задержек для каскадного включения таймеров
	TIM_MODE_ITR0	= (0<<4),
	TIM_MODE_ITR1	= (1<<4),
	TIM_MODE_ITR2	= (2<<4),
	TIM_MODE_ITR3	= (3<<4),
	TIM_MODE_ONEPULSE=(1<<16),
	TIM_MODE_DIR=(1<<17),
	TIM_MODE_CAM_UP =(1<<18),// Center aligned mode counting up
	TIM_MODE_CAM_DOWN =(2<<18),// Center aligned mode counting down
	TIM_MODE_CAM_UPDOWN =(3<<18),// Center aligned mode counting up,down
};


/* 	TIMERx(x=0..13) definitions 
	для таймеров TIMER0-TIMER7 Advanced Timer
	для таймеров TIMER1-TIMER4 Basic Timer
*/
static inline void TIM_pwm_init(uint32_t timx, uint32_t period, uint32_t prescaler, uint32_t mode)
{
	// slave mode control | trigger selection | master-slave mode
	TIMER_SMCFG(timx) &=~(TIMER_SMCFG_SMC|TIMER_SMCFG_TRGS|TIMER_SMCFG_MSM);// очистить slave mode configuration register 
	TIMER_SMCFG(timx) |= mode & (TIMER_SMCFG_SMC|TIMER_SMCFG_TRGS|TIMER_SMCFG_MSM);
	//	tim->SMCR |= TIM_SMCR_ECE; // external clock enable
	TIMER_PSC(timx) = prescaler-1;// prescaler register делитель частоты CK_CNT = f/(PSC+1)
	TIMER_CAR(timx) = period-1;			// [RM00090] 14.4.12 TIMx auto-reload register (TIMx_ARR)
	
	register uint32_t timr = TIMER_CTL0_ARSE;// Auto-reload shadow enable	
	if (mode&TIM_MODE_ONEPULSE) timr |= TIMER_CTL0_SPM;// Single pulse mode
	if (mode&TIM_MODE_DIR) timr |= TIMER_CTL0_DIR;// Direction count down
	if (mode&TIM_MODE_CAM_UPDOWN) timr |= (mode>>(18-5)) & TIMER_CTL0_CAM;

	TIMER_CTL0(timx) = timr;
	TIMER_CNT(timx) = 0; 				// counter register начальное значение счетчика
	TIMER_CH0CV(timx) =0, TIMER_CH1CV(timx)=0, TIMER_CH2CV(timx)=0, TIMER_CH3CV(timx)=0;
//	tim->CCR1 = 0;  tim->CCR2 = 0;  tim->CCR3 = 0;  tim->CCR4 = 0;
	TIMER_SWEVG(timx) = TIMER_SWEVG_UPG; 
//	tim->EGR = TIM_EGR_UG; // update event для записи значения ARR
  /* Select the Output Compare Mode */
	TIMER_CHCTL0(timx) = (0x06<<4) | (0x06<<12);// PWM0 PWM1 - PWM1 инвертирует выход при той же полярности
//	tim->CCMR1 = (0x06<<4) | (0x06<<12);// PWM1 PWM2 - PWM2 инвертирует выход при той же полярности
	TIMER_CHCTL1(timx) = (0x06<<4) | (0x06<<12);// PWM0 PWM1 - PWM1 инвертирует выход при той же полярности
//	tim->CCMR2 = (0x06<<4) | (0x06<<12);// PWM1 PWM2
	TIMER_CHCTL0(timx) |= TIMER_CHCTL0_CH0COMSEN |TIMER_CHCTL0_CH1COMSEN;
	TIMER_CHCTL1(timx) |= TIMER_CHCTL1_CH2COMSEN |TIMER_CHCTL1_CH3COMSEN;
//	tim->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;// preload enable
//	tim->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
	// за одно движение выставляем полярность всех каналов \see TIM_channel_enable

//	tim->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;
//	tim->CCR1 = 0;  tim->CCR2 = 0;  tim->CCR3 = 0;  tim->CCR4 = 0;
//	tim->BDTR |= TIM_BDTR_AOE;
	TIMER_CCHP(timx) |= TIMER_CCHP_OAEN;
	TIMER_SWEVG(timx) = TIMER_SWEVG_UPG; 
	//tim->EGR = TIM_EGR_UG; /* Generate an update event to reload the Prescaler value immediatly */
}
static inline void TIM_repetition(uint32_t timx, uint32_t count)
{
	TIMER_CREP(timx) = count;
}

static inline void TIM_channel_set(uint32_t timx, int ch, uint32_t pulse)
{
	volatile uint32_t *ccr = (volatile uint32_t *)(timx+0x34U);
	ccr[ch] = pulse;
	//tim->EGR = TIM_EGR_UG; 
}
static inline uint32_t TIM_channel_get(uint32_t timx, int ch)
{
	volatile uint32_t *ccr = (volatile uint32_t *)(timx+0x34U);
	return ccr[ch];
}
/*! чтобы выключить или включить одновременно несколько каналов на до обращаться к нулевому каналу 
	включить можно сопряженный канал TIM_CCER_СС1x N,P выбрать полярность сигналаP
	\see TIMERx_CHCTL2
 */
static inline void TIM_channel_enable (uint32_t timx, int ch, uint32_t mask, int enable)
{
	if (enable) 
		TIMER_CHCTL2(timx) |=   mask<<(ch*4);
	else
		TIMER_CHCTL2(timx) &= ~(mask<<(ch*4));
}
static inline void TIM_channel_config(uint32_t timx, int ch, uint16_t flags)
{
	if(ch&2) {// обновление через frozen
		TIMER_CHCTL1(timx) &= ~((ch&1)?0xFF00:0x00FF);
		TIMER_CHCTL1(timx) |=  ((ch&1)?flags<<8:flags);
	} else {
		TIMER_CHCTL0(timx) &= ~((ch&1)?0xFF00:0x00FF);
		TIMER_CHCTL0(timx) |=  ((ch&1)?flags<<8:flags);
	}
}
/*!	\brief установить режим работы при синхронизации от другого таймера
	\param mode - slave TRIGGER/GATED/RESET MODE
	\param trigger_source - ITR 0/1/2/3 источник синхронизации
*/
static inline void TIM_slave_mode(uint32_t timx, int  mode)
{
	TIMER_SMCFG(timx) = mode & (TIMER_SMCFG_SMC|TIMER_SMCFG_TRGS|TIMER_SMCFG_MSM);
//	tim->SMCR = mode & (TIM_SMCR_TS|TIM_SMCR_SMS|TIM_SMCR_MSM);// | ((trigger_source<<4)&TIM_SMCR_TS);
}
/*static inline void TIM_trigger_output(TIM_TypeDef* tim, int  mode)
{
	tim->CR2 = mode;
}*/
static inline void TIM_enable(uint32_t timx, int enable)
{
	if (enable) 
		TIMER_CTL0(timx) |=  TIMER_CTL0_CEN;
	else 
		TIMER_CTL0(timx) &= ~TIMER_CTL0_CEN;
	TIMER_SWEVG(timx) = TIMER_SWEVG_UPG; 
//	tim->EGR  |= TIM_EGR_UG;
}
/*
enum _TIM_CO_FORCE {OC_FROZEN=0, OC_FORCE_INACTIVE=4,OC_FORCE_ACTIVE=5, OC_PWM_MODE1=6};
static inline void TIM_force_output(uint32_t timx, int ch,  enum _TIM_CO_FORCE mode)
{
	register uint32_t timr;
	if(ch&2) {
		tim->CCMR2 &= ~((ch&1)?TIM_CCMR2_OC4M:TIM_CCMR2_OC3M);
		tim->CCMR2 |= ((ch&1)?mode<<TIM_CCMR2_OC4M_Pos : mode<<TIM_CCMR2_OC3M_Pos);
	} else {
		tim->CCMR1 &= ~((ch&1)?TIM_CCMR1_OC2M:TIM_CCMR1_OC1M);
		tim->CCMR1 |= ((ch&1)?mode<<TIM_CCMR1_OC2M_Pos : mode<<TIM_CCMR1_OC1M_Pos);
	}

}*/

#endif// TIM_H
//! \}
