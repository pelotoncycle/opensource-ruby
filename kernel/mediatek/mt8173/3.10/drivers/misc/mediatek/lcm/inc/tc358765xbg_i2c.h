/*****************************************************************************
* Filename: tc358765xbg_i2c.h
* Author: Innocomm
****************************************************************************/

#ifndef BUILD_LK
extern int tc358765xbg_read_byte(kal_uint8 cmd, kal_uint8 *returnData);
extern int tc358765xbg_write_byte(kal_uint8* value);
extern kal_uint32 tc358765xbg_read_reg (kal_uint8* addr, kal_uint8 *dataBuffer);
#endif

