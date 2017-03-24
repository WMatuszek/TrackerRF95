#ifndef __RF_H__
#define __RF_H__

struct RFM95_RX_Stats {
    uint16_t rxPacketCnt;
    uint16_t byteErrorCnt;
};

void RFM95_GPIO_Configuration(void);
void StartRFchip(void);
void RFM95_TransmitPacket(uint8_t *txPakData);
uint8_t RFM95_ReceivePacket(uint8_t *pakData, uint8_t *pakErr);
RFM95_RX_Stats RFM95_Receive_TEST(uint8_t *pakData, uint8_t *expectedData, uint8_t rcvSlot_ms);

#endif
