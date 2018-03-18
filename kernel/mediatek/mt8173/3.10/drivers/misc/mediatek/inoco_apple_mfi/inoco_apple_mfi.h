/*****************************************************************************
* Filename: inoco_apple_mfi.h
* Author: Innocomm
****************************************************************************/
#define I2C_DRIVER_NAME	"apple_mfi"
#define APPLE_MFI_DRIVER_NAME "inoco_apple_mfi"

#define I2C_MASK_FLAG	      (0x00ff)
#define I2C_RETRY_COUNT        10
#define MFI_MAX_I2C_LEN         255
#define MFI_FIFO_SIZE               8192 //(PAGE_SIZE*2)
#define MFI_I2C_FIFO_SIZE       2
#define MFI_RESPONSE_SIZE     128
#define MFI_STRING_SIZE          512
#define MAX_MFI_REG_COUNT 34

/* apple mfi register table */
struct mfi_cmd_table {
	char *func;
	unsigned char cmd_addr;
	int nbyte;
};

struct mfi_register_list {
	struct mfi_cmd_table cmd;
};

struct mfi_register_table {
	struct mfi_register_list *list;
};

/* error code definition */
#define ENOERR            0x00    /* No error */
#define EREGR               0x01    /* Invalid register for read */
#define EREGW              0x02    /* Invalid register for write */
#define ECHARESLEN    0x03    /* Invalid challenge response length */
#define ECHALEN          0x04    /* Invalid challenge length */
#define ECERLEN           0x05    /* Invalid certificate length */
#define ECHARESGEN   0x06    /* Internal process error during challenge response generation */
#define ECHAGEN         0x07    /* Internal process error during challenge generation */
#define ECHARESVER    0x08    /* Internal process error during challenge response verification */
#define ECERVALID       0x09    /* Internal process error during certificate validation */
#define EPROCTRL        0x0A     /* Invalid process control */
#define EPROCSEQ        0x0B    /* Process control out of sequence */

static int inoco_apple_mfi_tx_data(uint8_t *databuf, int length);
static int inoco_apple_mfi_rx_data(char *buf, int length);
void inoco_apple_mfi_i2c_exit_handle(void);
