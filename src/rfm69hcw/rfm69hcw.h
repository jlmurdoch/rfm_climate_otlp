#ifndef __RFM69HCW_H_
#define __RFM69HCW_H_

#include <SPI.h>
#include <stdint.h>

// Standard Clock speed
#define FXOSC         32000000.0 // Clock speed used with RxBw

enum { FSK_500KHZ, FSK_250KHZ, FSK_125KHZ, FSK_62KHZ, FSK_31KHZ, FSK_16KHZ, FSK_7_8KHZ, FSK_3_9KHZ,
	FSK_400KHZ, FSK_200KHZ, FSK_100KHZ, FSK_50KHZ, FSK_25KHZ, FSK_12KHZ, FSK_6_3KHZ, FSK_3_1KHZ,
	FSK_333KHZ, FSK_167KHZ, FSK_83KHZ, FSK_42KHZ, FSK_21KHZ, FSK_10KHZ, FSK_5_2KHZ, FSK_2_6KHZ };

enum { OOK_250KHZ, OOK_125KHZ, OOK_62KHZ, OOK_31KHZ, OOK_16KHZ, OOK_7_8KHZ, OOK_3_9KHZ, OOK_2KHZ,
	OOK_200KHZ, OOK_100KHZ, OOK_50KHZ, OOK_25KHZ, OOK_12KHZ, OOK_6_3KHZ, OOK_3_1KHZ, OOK_1_6KHZ,
	OOK_167KHZ, OOK_83KHZ, OOK_42KHZ, OOK_21KHZ, OOK_10KHZ, OOK_5_2KHZ, OOK_2_6KHZ, OOK_1_3KHZ };

// Bandwidth Macro Enumeration
enum { DCCFREQ_16, DCCFREQ_8, DCCFREQ_4, DCCFREQ_2, 
       DCCFREQ_1, DCCFREQ_0_5, DCCFREQ_0_25, DCCFREQ_0_125 };
enum { RXBWMANT_16, RXBWMANT_20, RXBWMANT_24 };
enum { RXBWEXP_0, RXBWEXP_1, RXBWEXP_2,  RXBWEXP_3,  
       RXBWEXP_4, RXBWEXP_5,  RXBWEXP_6,  RXBWEXP_7 };

enum { PKT_FSK_NONE = 0, PKT_FSK_LOW, PKT_FSK_MID, PKT_FSK_HIGH,
       PKT_OOK_NONE = 8, PKT_OOK_LOW, PKT_OOK_HIGH,
       CONT_SYNC_FSK_NONE = 64, CONT_SYNC_FSK_LOW, CONT_SYNC_FSK_MID, CONT_SYNC_FSK_HIGH,
       CONT_SYNC_OOK_NONE = 72, CONT_SYNC_OOK_LOW, CONT_SYNC_OOK_HIGH,
       CONT_NONE_FSK_NONE = 96, CONT_NONE_FSK_LOW, CONT_NONE_FSK_MID, CONT_NONE_FSK_HIGH,
       CONT_NONE_OOK_NONE = 104, CONT_NONE_OOK_LOW, CONT_NONE_OOK_HIGH };

uint8_t spi_cmd(SPIClass *vspi, uint8_t addr, uint8_t value);

uint8_t spi_single_write(SPIClass *vspi, uint8_t addr, uint8_t value);

uint8_t spi_single_read(SPIClass *vspi, uint8_t addr);

void rfminit(SPIClass *vspi, uint8_t rfm_rst, uint8_t modulation, float freq, uint16_t bitrate, uint8_t bandwidth);

uint8_t crc8(uint8_t buf[], int begin, int end);

uint16_t crc16(uint8_t buf[], int begin, int end);
#endif
