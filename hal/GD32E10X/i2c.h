#ifndef I2C_H
#define I2C_H
#include "board.h"
#include <cmsis_os.h>
//#include "errno.h"

typedef struct _i2c_transfer I2C_Transfer;
struct _i2c_transfer {
	uint8_t* tx_buffer;
	uint8_t* rx_buffer;
	uint16_t address;
	uint8_t tx_size;
	uint8_t rx_size;
	uint8_t status;
};
// переименовать на ERR_I2C_ и записать в errno.h
enum {ERR_OK=0, ERR_BUSY,  ERR_I2C_ACK, ERR_I2C_BUS, ERR_I2C_OVERRUN, ERR_I2C_ARLO, ERR_I2C_TIMEOUT, ERR_I2C_PEC, ERR_I2C_ALERT, ERR_UNKNOWN};

enum {I2C_MODE_SLAVE, I2C_MODE_MASTER};

//#include <fcntl.h>
//int open(const char *path, int oflag, ...);
//int fcntl(int, int, ...);
//#include <unistd.h>
//int close(int);
int  I2C_open(uint32_t i2c, int32_t flag, int mode);
int  I2C_send_recv(uint32_t i2c, I2C_Transfer* tr);
void I2C_close(uint32_t i2c);

#endif // I2C_H
