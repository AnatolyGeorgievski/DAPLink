#include <cmsis_os.h>
#include <stdio.h>
#include "board.h"
#include "i2c.h"
#define I2CCLK_MAX ((uint32_t)0x7FU) /*!< i2cclk maximum value */
#define I2CCLK_MIN ((uint32_t)0x02U) /*!< i2cclk minimum value */

struct _i2c_state {
	osThreadId owner;
	int flag;
	I2C_Transfer* tr;
	int tx_idx;
	int rx_idx;
	int status;
	
//	uint8_t slave_address;// 0xA0 младший бит отвечает за отсылку или прием данных
};

#define DEBUG_I2C 0
#if 1//defined(BOARD_I2C0)
static struct _i2c_state i2c0;
/*!    \brief      handle I2C0 event interrupt request
*/
char buf[20];
int events;
void I2C0_EV_IRQHandler(void)
{
	uint32_t stat0 = I2C_STAT0(I2C0);
	struct _i2c_state *ctx = &i2c0;
	I2C_Transfer* tr = ctx->tr;
    if(stat0 & I2C_STAT0_SBSEND){// в режиме мастера передать на линию адрес
		if (tr->rx_size==0) {// transmitter
			I2C_DATA(I2C0) = (tr->address & 0xFEU);
if(DEBUG_I2C)			debug("S");
		} else {// receiver
			I2C_DATA(I2C0) = (tr->address | 0x01U);
if(DEBUG_I2C) {
			snprintf(buf,16, "%dR", events);
			debug(buf);
}

		}
		events = 0;
		I2C_CTL1(I2C0) |=(I2C_CTL1_BUFIE);
    }else 
	if(stat0 & I2C_STAT0_ADDSEND){ /*clear ADDSEND bit */
		(void) I2C_STAT1(I2C0);// очистка флага происходит после чтения регистра I2C_STAT1
if(DEBUG_I2C)			debug("A");
		if (1==tr->rx_size){// ~ACK | STOP после передачи последнего символа
			I2C_CTL0(I2C0) = (I2C_CTL0(I2C0)&~I2C_CTL0_ACKEN) | I2C_CTL0_STOP; 
		}
    }else 
	if(stat0 & I2C_STAT0_ADD10SEND){ /*clear ADD10SEND bit */
if(DEBUG_I2C)		debug ("A2");
    }else
	if(stat0 & I2C_STAT0_STPDET){ /*clear STPDET bit */
if(DEBUG_I2C)		debug ("P");
    }else
	if(stat0 & I2C_STAT0_RBNE){
		uint8_t data = I2C_DATA(I2C0);

		int idx = ctx->rx_idx;
		if (idx < tr->rx_size) {
			tr->rx_buffer[idx++] = data;
if(DEBUG_I2C)			debug("#");
			ctx->rx_idx = idx;
			if (idx+1==tr->rx_size){// ~ACK | STOP после передачи последнего символа
				I2C_CTL0(I2C0) = (I2C_CTL0(I2C0)&~I2C_CTL0_ACKEN) | I2C_CTL0_STOP;
			} else
			if (idx == tr->rx_size){
if(DEBUG_I2C) {
	uint32_t stat1 = I2C_STAT1(I2C0);
	snprintf(buf,16, "P%d %X/%X\n", events, stat0, stat1);
	debug(buf);
}
				tr->status=ERR_OK;
				osSignalSet(ctx->owner, 1<<ctx->flag);
			}
			/* disable the I2C0 interrupt */
			 //I2C_CTL1(I2C0) &=~(I2C_CTL1_ERRIE|I2C_CTL1_BUFIE|I2C_CTL1_EVIE);	

		} else {
if(DEBUG_I2C) {
	snprintf(buf,10, "?%08X ", stat0);
	debug(buf);
}	
		}
    }else 
	if(stat0 & I2C_STAT0_TBE){// transmit buffer empty
		int idx = ctx->tx_idx;
        if(idx < tr->tx_size){
			I2C_DATA(I2C0) = tr->tx_buffer[idx++];
			I2C_CTL1(I2C0) &=~(I2C_CTL1_BUFIE);
if(DEBUG_I2C) debug("$");
			ctx->tx_idx = idx;
			if (idx == tr->tx_size) {
				if (tr->rx_size!=0){
					//I2C_CTL0(I2C0) |= I2C_CTL0_START;
				} else
					I2C_CTL0(I2C0) |= I2C_CTL0_STOP;
			}
        } else 
		if (stat0 & I2C_STAT0_BTC)
		{// TBE && BTC -- всегда так
			if (tr->rx_size!=0) {
				if (tr->tx_size !=0) 
				{// использую как флаг события
					I2C_CTL0(I2C0) |= (I2C_CTL0_START);
					tr->tx_size =0;
					//ctx->rx_idx =0;
//if(DEBUG_I2C) debug("B");
				}
			} else {
if(DEBUG_I2C) {
			snprintf(buf,16, "P%d\n", events);
			debug(buf);
}

				tr->status=ERR_OK;
				osSignalSet(ctx->owner, 1<<ctx->flag);
			}
		} else {
			//if (tr->tx_size==0 && tr->rx_size==0) I2C_CTL0(I2C0) |= (I2C_CTL0_STOP);
if(DEBUG_I2C) debug("X");
if(0 && tr->rx_size!=0) {
			I2C_CTL0(I2C0) |= (I2C_CTL0_START);
			tr->tx_size =0;
			ctx->rx_idx =0;
}
		}
//	uint32_t stat1 = I2C_STAT1(I2C0);
//	snprintf(buf,16, "%%%c%04X %04X ", (tr->rx_size!=0?'t':'r'),stat0, stat1);
//	debug(buf);

			
	} else {// один раз попадаем сюда после завершения передачи STOP
		//stat0 = I2C_STAT1(I2C0);
		//I2C_CTL1(I2C0) &=~(I2C_CTL1_ERRIE|I2C_CTL1_BUFIE|I2C_CTL1_EVIE);
if (DEBUG_I2C) {
	uint32_t stat1 = I2C_STAT1(I2C0);
	snprintf(buf,16, "?%04X/%04X\n", stat0, stat1);
	debug(buf);
}
	}
if (DEBUG_I2C) 	events++;
}
void I2C0_ER_IRQHandler(void)
{
	struct _i2c_state *i2c = &i2c0;
	uint32_t stat0 = I2C_STAT0(I2C0);
	uint32_t stat1 = I2C_STAT1(I2C0);
	int err;
    /* no acknowledge received */
    if(stat0 & I2C_STAT0_AERR){
		// нормально при завершении трансфера
		I2C_STAT0(I2C0) &=~I2C_STAT0_AERR;
		err = ERR_I2C_ACK;
		if (stat1 & I2C_STAT1_MASTER) I2C_CTL0(I2C0) |= I2C_CTL0_STOP;

    }
    /* SMBus alert */
    if(stat0 & I2C_STAT0_SMBALT){
		I2C_STAT0(I2C0) &=~I2C_STAT0_SMBALT;
		err = ERR_I2C_ALERT;
    }
    /* bus timeout in SMBus mode */
    if(stat0 & I2C_STAT0_SMBTO){
		I2C_STAT0(I2C0) &=~I2C_STAT0_SMBTO;
		err = ERR_I2C_TIMEOUT;
    }
    /* over-run or under-run when SCL stretch is disabled */
    if(stat0 & I2C_STAT0_OUERR){
		I2C_STAT0(I2C0) &=~I2C_STAT0_OUERR;
		err = ERR_I2C_OVERRUN;
    }
    /* arbitration lost */
    if(stat0 & I2C_STAT0_LOSTARB){
        I2C_STAT0(I2C0) &=~I2C_STAT0_LOSTARB;
		err = ERR_I2C_ARLO;
    }
    /* bus error */
    if(stat0 & I2C_STAT0_BERR){
        I2C_STAT0(I2C0) &=~I2C_STAT0_BERR;
		err = ERR_I2C_BUS;
    }
    /* CRC value doesn't match */
    if(stat0 & I2C_STAT0_PECERR){
        I2C_STAT0(I2C0) &=~I2C_STAT0_PECERR;
		err = ERR_I2C_PEC;
    }
    /* disable the error interrupt */
	//I2C_CTL1(I2C0) &=~(I2C_CTL1_ERRIE|I2C_CTL1_BUFIE|I2C_CTL1_EVIE);
	

if (DEBUG_I2C)	{
	debug("%I2C ERR");
	snprintf(buf,16, "%04X/%04X", stat0, stat1);
	debug(buf);
}
	i2c->status = err;
	osSignalSet(i2c->owner, 1UL<<i2c->flag);
}
static struct _i2c_state i2c1;
#endif
int I2C_send_recv(uint32_t device_fd, I2C_Transfer* tr)
{
	struct _i2c_state * ctx=NULL;
	if (device_fd == I2C0) {
		ctx = &i2c0;
		ctx->tr = tr;
	} else 
	if (device_fd == I2C1) {
		ctx = &i2c1;
	}
	if (ctx!=NULL) {
		if (0) while(I2C_CTL0(device_fd)&I2C_CTL0_STOP) 
		{
			//I2C_CTL0(device_fd)&=~I2C_CTL0_STOP;
			printf(">> STP \n");
			//osDelay(1);
			osThreadYield();
		}
		uint32_t stat0, stat1;
		if(1)while((stat1 = I2C_STAT1(device_fd)) & I2C_STAT1_I2CBSY) {
			//stat0 = I2C_STAT0(device_fd);
			//printf("BSY%08X %08X\r\n", stat0, stat1);
			printf("BSY ");
			osDelay(1);
		}
		//printf("+");
		//while(I2C_CTL0(device_fd)&I2C_CTL0_STOP);
		ctx->tx_idx = 0;
		ctx->rx_idx = 0;
		tr->status = ERR_BUSY;
		I2C_CTL1(device_fd) |= (I2C_CTL1_ERRIE|I2C_CTL1_BUFIE|I2C_CTL1_EVIE);
		//printf("i2c_start\r\n");
		I2C_CTL0(device_fd) |= (I2C_CTL0_ACKEN|I2C_CTL0_START);// начать передачу в режиме мастера
		
	}
	return 0;
}
#if 0
void i2c_clock_config(uint32_t i2c_periph, uint32_t clkspeed, uint32_t dutycyc)
{
    uint32_t pclk1, clkc, freq, risetime;
    uint32_t temp;
    
    pclk1 = BOARD_PCLK1*1000000U;//rcu_clock_freq_get(CK_APB1);
    /* I2C peripheral clock frequency */
    freq = (uint32_t)(pclk1/1000000U);
    if(freq >= I2CCLK_MAX){
        freq = I2CCLK_MAX;
    }
    temp = I2C_CTL1(i2c_periph);
    temp &= ~I2C_CTL1_I2CCLK;
    temp |= freq;
    
    I2C_CTL1(i2c_periph) = temp;
    
    if(100000U >= clkspeed){
        /* the maximum SCL rise time is 1000ns in standard mode */
        risetime = (uint32_t)((pclk1/1000000U)+1U);
        if(risetime >= I2CCLK_MAX){
            I2C_RT(i2c_periph) = I2CCLK_MAX;
        }else if(risetime <= I2CCLK_MIN){
            I2C_RT(i2c_periph) = I2CCLK_MIN;
        }else{
            I2C_RT(i2c_periph) = risetime;
        }
        clkc = (uint32_t)(pclk1/(clkspeed*2U)); 
        if(clkc < 0x04U){
            /* the CLKC in standard mode minmum value is 4 */
            clkc = 0x04U;
        }
        I2C_CKCFG(i2c_periph) |= (I2C_CKCFG_CLKC & clkc);

    }else if(400000U >= clkspeed){
        /* the maximum SCL rise time is 300ns in fast mode */
        I2C_RT(i2c_periph) = (uint32_t)(((freq*(uint32_t)300U)/(uint32_t)1000U)+(uint32_t)1U);
        if(I2C_DTCY_2 == dutycyc){
            /* I2C duty cycle is 2 */
            clkc = (uint32_t)(pclk1/(clkspeed*3U));
            I2C_CKCFG(i2c_periph) &= ~I2C_CKCFG_DTCY;
        }else{
            /* I2C duty cycle is 16/9 */
            clkc = (uint32_t)(pclk1/(clkspeed*25U));
            I2C_CKCFG(i2c_periph) |= I2C_CKCFG_DTCY;
        }
        if(0U == (clkc & I2C_CKCFG_CLKC)){
            /* the CLKC in fast mode minmum value is 1 */
            clkc |= 0x0001U;  
        }
        I2C_CKCFG(i2c_periph) |= I2C_CKCFG_FAST;
        I2C_CKCFG(i2c_periph) |= clkc;
    }else{
        /* fast mode plus, the maximum SCL rise time is 120ns */
        I2C_RT(i2c_periph) = (uint32_t)(((freq*(uint32_t)120U)/(uint32_t)1000U)+(uint32_t)1U);
        if(I2C_DTCY_2 == dutycyc){
            /* I2C duty cycle is 2 */
            clkc = (uint32_t)(pclk1/(clkspeed*3U));
            I2C_CKCFG(i2c_periph) &= ~I2C_CKCFG_DTCY;
        }else{
            /* I2C duty cycle is 16/9 */
            clkc = (uint32_t)(pclk1/(clkspeed*25U));
            I2C_CKCFG(i2c_periph) |= I2C_CKCFG_DTCY;
        }
        /* enable fast mode */
        I2C_CKCFG(i2c_periph) |= I2C_CKCFG_FAST;
        I2C_CKCFG(i2c_periph) |= clkc;
        /* enable I2C fast mode plus */
        I2C_FMPCFG(i2c_periph) = I2C_FMPCFG_FMPEN;
    }
}
#endif
int I2C_open(uint32_t device_fd, int32_t event_flag, int mode)
{
	I2C_CTL0(device_fd) |= I2C_CTL0_SRESET;
	I2C_CTL0(device_fd) &=~I2C_CTL0_SRESET;
	// BOARD_PCLK1 максимум 60 МГц 
	I2C_CTL1(device_fd) = (I2C_CTL1(device_fd) &~I2C_CTL1_I2CCLK) | (BOARD_PCLK1&I2C_CTL1_I2CCLK);
    /* Rise time the maximum SCL rise time is 300ns in fast mode */
	I2C_RT(device_fd) = (uint32_t)(((BOARD_PCLK1)/12U)+(uint32_t)1U);
	//uint32_t clkc = (uint32_t)(BOARD_PCLK1/10);
	I2C_CKCFG(device_fd)  = I2C_CKCFG_FAST| I2C_DTCY_16_9 | (BOARD_PCLK1/10);//&= ~I2C_CKCFG_DTCY;
	//I2C_CKCFG(device_fd)  = I2C_CKCFG_FAST| I2C_DTCY_2    | ((BOARD_PCLK1*10)/12);//&= ~I2C_CKCFG_DTCY;

//	i2c_clock_config(device_fd, 400000, I2C_DTCY_16_9);
	struct _i2c_state * ctx=NULL;
	int fd;
	if (device_fd == I2C0) {
		ctx = &i2c0;
	}else 
	if (device_fd == I2C1) {
		ctx = &i2c1;
	} else 
		fd = -1;
	if (ctx!=NULL){
    /* enable I2C0 */
		// if (mode == SLAVE)

		ctx->tr = NULL;
		ctx->owner = osThreadGetId();
		ctx->flag  = event_flag;
		fd = event_flag;//osSignalAlloc(); // выделить номер сигнала
		I2C_CTL0(device_fd) |= I2C_CTL0_I2CEN;
		I2C_CTL0(device_fd) |= I2C_CTL0_ACKEN;
		NVIC_SetPriority(I2C0_EV_IRQn, 3);
		NVIC_ClearPendingIRQ(I2C0_EV_IRQn);
		NVIC_EnableIRQ(I2C0_EV_IRQn);
		NVIC_SetPriority(I2C0_ER_IRQn, 2);
		NVIC_ClearPendingIRQ(I2C0_ER_IRQn);
		NVIC_EnableIRQ(I2C0_ER_IRQn);
	}
	return fd;
}

void I2C_close(uint32_t device_fd)
{
	struct _i2c_state * ctx=NULL;
	if (device_fd == I2C0) {
		ctx = &i2c0;
	} 
	else 
	if (device_fd == I2C1) {
		ctx = &i2c1;
	}
	if (ctx!=NULL){
		ctx->owner = NULL;
		//osSignalFree(fd);
	}
}
