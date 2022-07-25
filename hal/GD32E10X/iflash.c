/*! \ingroup _hal
	\defgroup _hal_iflash Запись конфигурационной информации во внутренний флеш 

	\see PM0042 Reading/programming the STM32F10xxx embedded Flash memory
	\see PM0063 Programming manual STM32F100xx value line Flash programming
	\see RM0091 Reading/programming the STM32F10xxx embedded Flash memory
	\see RM0313 Reading/programming the STM32F10xxx embedded Flash memory
	\see RM0366 Reading/programming the STM32F10xxx embedded Flash memory

	\{
*/
#include "board.h"
#include "cmsis_os.h"
#include "atomic.h"
#include "r3rtos.h"
#include "iflash.h"
#include "gd32e10x_fmc.h"


#ifndef IFLASH_PAGE_SIZE
	#define IFLASH_PAGE_SIZE 1024  //STM32F03x STM32F04x STM32F05x  STM32F100xB
	// GD32E10X
#endif
#define IFLASH_WORD_SIZE 4
//#define IFLASH_WORD_SIZE 8


extern unsigned char _iflash_segment_end[];


#ifndef UNLOCK_KEY1
#define UNLOCK_KEY0	0x45670123
#define UNLOCK_KEY1	0xCDEF89AB
#endif

#define FLASH_CR_PSIZE_x16 (FLASH_CR_PSIZE_0)
#define FLASH_CR_SNB_MASK (FLASH_CR_SNB_0|FLASH_CR_SNB_1|FLASH_CR_SNB_2|FLASH_CR_SNB_3)
static int iFLASH_init(void* base)
{
	return 0;
}
/*!	\brief Загрузка блоков данных с блочного устройства
*/
static void*  iFLASH_load(void* base, void* buffer, uint32_t block, uint32_t size)
{
	unsigned long *src = (void*)((char*)base + (block << 9));
	return src;
}

/*! \brief Производит подготовку записи страницы во флеш
*/
__attribute__((section(".ramfunc")))
int iFLASH_store(void* base, const void* buffer, uint32_t  block, uint32_t  size)
{ 
	while (FMC_STAT & FMC_STAT_BUSY); // ждем завершения предыдущей операции

	if (FMC_CTL & FMC_CTL_LK)
	{ // unlock 
		FMC_KEY = UNLOCK_KEY0;
		FMC_KEY = UNLOCK_KEY1;
	}

	uint32_t *dst = (void*)((char*)base + (block << 9));

	const uint32_t sector_size_mask = IFLASH_PAGE_SIZE-1;// Надо взять актуальное значение размера страницы
	/* 2. определить начало нового сектора и выполнить очистку страницы флеш. */

/* 1. Check that no Flash memory operation is ongoing by checking the BSY bit in the FLASH_CR register. */
/* 2. Set the PER bit in the FLASH_CR register */
/* 3. Program the FLASH_AR register to select a page to erase */
/* 4. Set the STRT bit in the FLASH_CR register */
/* 5. Wait for the BSY bit to be reset */
	if (((uint32_t)dst & (sector_size_mask)) == 0)// новая страница ФЛЕШ-памяти вызывает очистку
	{
		FMC_CTL |= FMC_CTL_PER;    /* (2) set PER page erase bit */
		FMC_ADDR  = (uint32_t)dst; /* (3) */
		FMC_CTL |= FMC_CTL_START;  /* (4) op start */
		while (FMC_STAT & FMC_STAT_BUSY);	/* (5) */
		FMC_CTL &= ~FMC_CTL_PER;
	}

/* (1) Set the PG bit in the FLASH_CR register to enable programming */
/* (2) Perform the data write (half-word) at the desired address */
/* (3) Wait until the BSY bit is reset in the FLASH_SR register */
/* (4) Reset the PG Bit to disable programming */	
	
	uint32_t const*src = buffer;
	size >>=2; // по словам 32 бит
	if (size) {
		do {
			FMC_CTL |=  FMC_CTL_PG; // program
			*dst++ = *src++;
			while (FMC_STAT & FMC_STAT_BUSY);	/* (3) */
		}	while (--size);
		FMC_CTL &= ~FMC_CTL_PG; // program
	}
	// заблокировать запись во флеш
	FMC_CTL |= FMC_CTL_LK;
	return 0;
}

int iFLASH_log(void* base, uint32_t offset, const void* buffer, uint32_t  size)
{
	uint32_t *dst = base+offset;
	uint32_t const*src = buffer;
	while (FMC_STAT & FMC_STAT_BUSY); // ждем завершения предыдущей операции

	if (FMC_CTL & FMC_CTL_LK)
	{ // unlock 
		FMC_KEY = UNLOCK_KEY0;
		FMC_KEY = UNLOCK_KEY1;
	}

	size >>=2; // по словам 32 бит
	if (size) {
		FMC_CTL |=  FMC_CTL_PG; // program
		do {
			*dst++ = *src++;
			while (FMC_STAT & FMC_STAT_BUSY);
		}   while (--size);
		FMC_CTL &= ~FMC_CTL_PG; // program
	}
	FMC_CTL |= FMC_CTL_LK;// заблокировать запись во флеш
	return 0;
}
__attribute__((section(".ramfunc")))
void iflash_bootloader(uint32_t offset, size_t size) 
{
	__disable_irq();
	uint8_t const *src = (void*)(_iflash_segment_end + offset);// + (block<<9));
	uint8_t *base = (void*)(FLASH_BASE);
	while (size>=IFLASH_PAGE_SIZE){// запись по страницам
		iFLASH_store(base, src, 0, IFLASH_PAGE_SIZE);
		base+=IFLASH_PAGE_SIZE;
		src +=IFLASH_PAGE_SIZE;
		size-=IFLASH_PAGE_SIZE;
	}
	if (size>0){// запись последней не полной страницы
		iFLASH_store(base, src, 0, size);
	}
	NVIC_SystemReset();
}
//! \{
BlockMedia iflash_block_media = {
    .data = (void*)_iflash_segment_end,//IFLASH_FS_BASE,
    .init = iFLASH_init,
    .read = iFLASH_load,
    .send = iFLASH_store,
	.append=iFLASH_log,
    .unref=NULL,
};
typedef struct _FLASH_Handle FLASH_Handle;
struct _FLASH_Handle {
	osThreadId owner;
	int32_t flag;
	int status;
	uint32_t* dst;
	uint32_t* src;
	uint16_t size;
	uint16_t idx;
};
static FLASH_Handle iflash_handle;

/*! \brief открыть канал записи во внутренний FLASH 
	\param h -- просто затычка для унификации интерфейса, ссылка на ресурс FLASH
	\param 
 */
void* iflash_open(uint32_t block, int32_t flag)
{
	FLASH_Handle* iflash = &iflash_handle;
	iflash->owner= osThreadGetId();
	iflash->flag = flag;
	iflash->dst  = (void*)(_iflash_segment_end + (block<<9));
	// снять блокировку записи
	if (FMC_CTL & FMC_CTL_LK) { // Unlock the FMC_CTL register if necessary. 
		FMC_KEY = UNLOCK_KEY0;
		FMC_KEY = UNLOCK_KEY1;
	}
	NVIC_ClearPendingIRQ(FMC_IRQn);
	NVIC_EnableIRQ(FMC_IRQn);
	return iflash;
}
void iflash_close(void* vh)
{
	FLASH_Handle* iflash = vh;
	NVIC_DisableIRQ(FMC_IRQn);
	// заблокировать запись во флеш
	FMC_CTL |= FMC_CTL_LK;
	iflash->owner=NULL;
}
void* iflash_recv(void* vh, uint32_t block)
{
	return (void*)(_iflash_segment_end + (block<<9));
}
void iflash_send(void* vh, void* data, uint16_t size)
{
	FLASH_Handle* iflash =vh;
	iflash->src = data;
	iflash->size = size;
	iflash->idx  = 0;
	iflash->status = ERR_FLASH_BUSY;
	//uint16_t *dst = (void*)((char*)vh + (block << 9));
	if (((uint32_t)iflash->dst & (IFLASH_PAGE_SIZE-1) )==0){
		//printf("DST=%08X\r\n", iflash->dst);
		FMC_CTL |= FMC_CTL_PER;  /* (2) set PER page erase bit */
		FMC_ADDR = (uint32_t)iflash->dst; /* (3) */
		FMC_CTL |= FMC_CTL_START; /* (4) op start */
	} else {
		#if (IFLASH_WORD_SIZE == 8) 
		 FMC_WS |= FMC_WS_PGW;
		#endif
		FMC_CTL |=  FMC_CTL_PG; // program
		iflash->idx+=IFLASH_WORD_SIZE;// 32 или 64 бита
		*iflash->dst++ = *iflash->src++; 
		#if (IFLASH_WORD_SIZE == 8) 
		*iflash->dst++ = *iflash->src++; 
		#endif
	}
	FMC_CTL |= (FMC_CTL_ENDIE | FMC_CTL_ERRIE);
}
	
/*! \brief обработчик прерываний по готовности FLASH 
			PROG		ERASE	PAGE
GD32E10X 	37-44us		3.2-4ms	1KB
GD32E23X 	37-44us		3.2-4ms	1KB
GD32E503	37.5us		11ms	8KB
GD32F130 	200-400us	100ms	1KB
GD32F303	37.5		45ms	(bank0 256 x 2KB) +(bank1 N x 4KB)
GD32F3X0	1KB

	Время очистки страницы флеш памяти составляет 20-40мс. 
	Время записи во флеш одного слова составляет 40-60мкс. 
 */
void FMC_IRQHandler(void)
{
	FLASH_Handle* iflash =&iflash_handle;
	if ((FMC_STAT & FMC_STAT_ENDF)!=0) {/* завершение операции */
		FMC_STAT |= FMC_STAT_ENDF;
		if ((FMC_CTL & FMC_CTL_PER)!=0) {/* очистка страницы памяти завершена */
			FMC_CTL &= ~FMC_CTL_PER;
			#if (IFLASH_WORD_SIZE != 8) 
				FMC_WS &= ~FMC_WS_PGW;
			#endif
			//printf("FLASH PER-Ok\r\n");
		}
		if (iflash->idx < iflash->size) {
			FMC_CTL |=  FMC_CTL_PG; // program
			*iflash->dst++ = *iflash->src++;
			#if (IFLASH_WORD_SIZE == 8) 
			*iflash->dst++ = *iflash->src++;
			#endif
			iflash->idx+=IFLASH_WORD_SIZE;
		} else {
			FMC_CTL &= ~FMC_CTL_PG; // program
			// osConfirm(osThreadId, flag);
			// printf("FLASH PG-Ok\r\n");
			iflash->status = ERR_FLASH_OK;
			if (iflash->owner) {
				osSignalSet(iflash->owner, 1UL<<iflash->flag);
				osThreadNotify(iflash->owner);
			}
		}
	} else
	if ((FMC_STAT & FMC_STAT_WPERR)!=0) {/* защита от записи */
		FMC_STAT |= FMC_STAT_WPERR;
		iflash->status = ERR_FLASH_WRP;
		printf("FLASH ERR-WRP\r\n");
	} else
	if ((FMC_STAT & FMC_STAT_PGERR)!=0) {/* ошибка программирования, когда содержимое памяти не равно 0xFFFF */
		FMC_STAT |= FMC_STAT_PGERR;
		iflash->status = ERR_FLASH_PAGE;
		printf("FLASH ERR-PG\r\n");
	} else
	if ((FMC_STAT & FMC_STAT_PGAERR)!=0) {// ошибка выравнивания памяти
		FMC_STAT |= FMC_STAT_PGAERR;
		iflash->status = ERR_FLASH_ALIGN;
		printf("FLASH ERR-ALIGN\r\n");
	}
		
}
