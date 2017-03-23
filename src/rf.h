#ifndef __RF_H__
#define __RF_H__

void RFM95_GPIO_Configuration(void);
void StartRFchip(void);
void TX_OGN_packet_test(uint8_t *);
int RX_OGN_packet_test(uint8_t *);

#endif
