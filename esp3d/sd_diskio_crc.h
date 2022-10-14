#ifndef SD_DISKIO_CRC_H
#define SD_DISKIO_CRC_H

char CRC7(const void* data, int length);
unsigned short CRC16(const void* data, int length);

#endif
