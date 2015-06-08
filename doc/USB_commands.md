
/* resend received */
#define USB_CMD_LOOP                0x01

#define USB_CMD_SEND_STATUS_8BIT    0x02

#define USB_CMD_SEND_STATUS_FLOAT   0x03

#define USB_CMD_GLOBAL_FLAGS        0x04

/*******************/
/* Config Commands
 *  (except USB_CMD_CONFIG_MODE)
 * */

/* enter and leave config mode*/
#define USB_CMD_CONFIG_MODE         0xC0

/* get or update whole configuration*/
#define USB_CMD_GET_CONFIG          0xC1
#define USB_CMD_UPDATE_CONFIG       0xC2

/* direct EEPROM access
 * only working if you are in config mode
 *
 * needs Address: MSB first
 * structure: USB_CMD; ADDR_MSB; ADDR_LSB; DATA;
 * */
#define USB_CMD_READ_BYTE           0xC3
#define USB_CMD_READ_2BYTES         0xC4
#define USB_CMD_READ_4BYTES         0xC5

#define USB_CMD_WRITE_BYTE          0xC6
#define USB_CMD_WRITE_2BYTES        0xC7
#define USB_CMD_WRITE_4BYTES        0xC8


/* save config
 * triggers reset
 */
#define USB_CMD_SAVE_CONFIG         0xCE
/* restore hardcoded config values*/
#define USB_CMD_RESTORE_CONFIG      0xCF

/*******************/

#define USB_CMD_RESET               0xFF