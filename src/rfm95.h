#ifndef rfm95_h
#define rfm95_h

#include "spi.h"
#include "stm32f10x.h"

const uint8_t ManchesterEncode[0x10] =  // lookup table for 4-bit nibbles for
                                        // quick Manchester encoding
    {
        0xAA,  // hex: 0, bin: 0000, manch: 10101010
        0xA9,  // hex: 1, bin: 0001, manch: 10101001
        0xA6,  // hex: 2, bin: 0010, manch: 10100110
        0xA5,  // hex: 3, bin: 0011, manch: 10100101
        0x9A,  // hex: 4, bin: 0100, manch: 10011010
        0x99,  // hex: 5, bin: 0101, manch: 10011001
        0x96,  // hex: 6, bin: 0110, manch: 10010110
        0x95,  // hex: 7, bin: 0111, manch: 10010101
        0x6A,  // hex: 8, bin: 1000, manch: 01101010
        0x69,  // hex: 9, bin: 1001, manch: 01101001
        0x66,  // hex: A, bin: 1010, manch: 01100110
        0x65,  // hex: B, bin: 1011, manch: 01100101
        0x5A,  // hex: C, bin: 1100, manch: 01011010
        0x59,  // hex: D, bin: 1101, manch: 01011001
        0x56,  // hex: E, bin: 1110, manch: 01010110
        0x55   // hex: F, bin: 1111, manch: 01010101
};

const uint8_t ManchesterDecode[0x100] =  // lookup table for quick Manchester decoding
    {                                    // lower nibble has the data bits and the upper nibble the error pattern
        0xF0, 0xE1, 0xE0, 0xF1, 0xD2, 0xC3, 0xC2, 0xD3, 0xD0, 0xC1, 0xC0, 0xD1, 0xF2, 0xE3, 0xE2, 0xF3, 0xB4, 0xA5, 0xA4, 0xB5, 0x96, 0x87,
        0x86, 0x97, 0x94, 0x85, 0x84, 0x95, 0xB6, 0xA7, 0xA6, 0xB7, 0xB0, 0xA1, 0xA0, 0xB1, 0x92, 0x83, 0x82, 0x93, 0x90, 0x81, 0x80, 0x91,
        0xB2, 0xA3, 0xA2, 0xB3, 0xF4, 0xE5, 0xE4, 0xF5, 0xD6, 0xC7, 0xC6, 0xD7, 0xD4, 0xC5, 0xC4, 0xD5, 0xF6, 0xE7, 0xE6, 0xF7, 0x78, 0x69,
        0x68, 0x79, 0x5A, 0x4B, 0x4A, 0x5B, 0x58, 0x49, 0x48, 0x59, 0x7A, 0x6B, 0x6A, 0x7B, 0x3C, 0x2D, 0x2C, 0x3D, 0x1E, 0x0F, 0x0E, 0x1F,
        0x1C, 0x0D, 0x0C, 0x1D, 0x3E, 0x2F, 0x2E, 0x3F, 0x38, 0x29, 0x28, 0x39, 0x1A, 0x0B, 0x0A, 0x1B, 0x18, 0x09, 0x08, 0x19, 0x3A, 0x2B,
        0x2A, 0x3B, 0x7C, 0x6D, 0x6C, 0x7D, 0x5E, 0x4F, 0x4E, 0x5F, 0x5C, 0x4D, 0x4C, 0x5D, 0x7E, 0x6F, 0x6E, 0x7F, 0x70, 0x61, 0x60, 0x71,
        0x52, 0x43, 0x42, 0x53, 0x50, 0x41, 0x40, 0x51, 0x72, 0x63, 0x62, 0x73, 0x34, 0x25, 0x24, 0x35, 0x16, 0x07, 0x06, 0x17, 0x14, 0x05,
        0x04, 0x15, 0x36, 0x27, 0x26, 0x37, 0x30, 0x21, 0x20, 0x31, 0x12, 0x03, 0x02, 0x13, 0x10, 0x01, 0x00, 0x11, 0x32, 0x23, 0x22, 0x33,
        0x74, 0x65, 0x64, 0x75, 0x56, 0x47, 0x46, 0x57, 0x54, 0x45, 0x44, 0x55, 0x76, 0x67, 0x66, 0x77, 0xF8, 0xE9, 0xE8, 0xF9, 0xDA, 0xCB,
        0xCA, 0xDB, 0xD8, 0xC9, 0xC8, 0xD9, 0xFA, 0xEB, 0xEA, 0xFB, 0xBC, 0xAD, 0xAC, 0xBD, 0x9E, 0x8F, 0x8E, 0x9F, 0x9C, 0x8D, 0x8C, 0x9D,
        0xBE, 0xAF, 0xAE, 0xBF, 0xB8, 0xA9, 0xA8, 0xB9, 0x9A, 0x8B, 0x8A, 0x9B, 0x98, 0x89, 0x88, 0x99, 0xBA, 0xAB, 0xAA, 0xBB, 0xFC, 0xED,
        0xEC, 0xFD, 0xDE, 0xCF, 0xCE, 0xDF, 0xDC, 0xCD, 0xCC, 0xDD, 0xFE, 0xEF, 0xEE, 0xFF};

// Register names (FSK Mode, from table 86)
#define RH_RF95_REG_00_FIFO 0x00
#define RH_RF95_REG_01_OP_MODE 0x01
#define RH_RF95_REG_02_BITRATE_MSB 0x02
#define RH_RF95_REG_03_BITRATE_LSB 0x03
#define RH_RF95_REG_04_FDEV_MSB 0x04
#define RH_RF95_REG_05_FDEV_LSB 0x05
#define RH_RF95_REG_06_FRF_MSB 0x06
#define RH_RF95_REG_07_FRF_MID 0x07
#define RH_RF95_REG_08_FRF_LSB 0x08
#define RH_RF95_REG_09_PA_CONFIG 0x09
#define RH_RF95_REG_0A_PA_RAMP 0x0a
#define RH_RF95_REG_0B_OCP 0x0b
#define RH_RF95_REG_0C_LNA 0x0c
#define RH_RF95_REG_0D_RX_CONFIG 0x0d
#define RH_RF95_REG_0E_RSSI_CONFIG 0x0e
#define RH_RF95_REG_0F_RSSI_COLLISION 0x0f
#define RH_RF95_REG_10_RSSI_THRESH 0x10
#define RH_RF95_REG_11_RSSI_VALUE 0x11
#define RH_RF95_REG_12_RX_BW 0x12
#define RH_RF95_REG_13_AFC_BW 0x13
#define RH_RF95_REG_14_OOK_PEAK 0x14
#define RH_RF95_REG_15_OOK_FIX 0x15
#define RH_RF95_REG_16_OOK_AVG 0x16
#define RH_RF95_REG_17_RESERVED 0x17
#define RH_RF95_REG_18_RESERVED 0x18
#define RH_RF95_REG_19_RESERVED 0x19
#define RH_RF95_REG_1A_AFC_FEI 0x1a
#define RH_RF95_REG_1B_AFC_MSB 0x1b
#define RH_RF95_REG_1C_AFC_LSB 0x1c
#define RH_RF95_REG_1D_FEI_MSB 0x1d
#define RH_RF95_REG_1E_FEI_LSB 0x1e
#define RH_RF95_REG_1F_PREAMBLE_DETECT 0x1f
#define RH_RF95_REG_20_RX_TIMEOUT1 0x20
#define RH_RF95_REG_21_RX_TIMEOUT2 0x21
#define RH_RF95_REG_22_RX_TIMEOUT3 0x22
#define RH_RF95_REG_23_RX_DELAY 0x23
#define RH_RF95_REG_24_REG_OSC 0x24
#define RH_RF95_REG_25_PREAMBLE_MSB 0x25
#define RH_RF95_REG_26_PREAMBLE_LSB 0x26
#define RH_RF95_REG_27_SYNC_CONFIG 0x27
#define RH_RF95_REG_28_SYNC_VALUE_1 0x28
#define RH_RF95_REG_29_SYNC_VALUE_2 0x29
#define RH_RF95_REG_2A_SYNC_VALUE_3 0x2a
#define RH_RF95_REG_2B_SYNC_VALUE_4 0x2b
#define RH_RF95_REG_2C_SYNC_VALUE_5 0x2c
#define RH_RF95_REG_2D_SYNC_VALUE_6 0x2d
#define RH_RF95_REG_2E_SYNC_VALUE_7 0x2e
#define RH_RF95_REG_2F_SYNC_VALUE_8 0x2f
#define RH_RF95_REG_30_PACKET_CONFIG_1 0x30
#define RH_RF95_REG_31_PACKET_CONFIG_2 0x31
#define RH_RF95_REG_32_PAYLOAD_LENGTH 0x32
#define RH_RF95_REG_33_NODE_ADRS 0x33
#define RH_RF95_REG_34_BROADCAST_ADRS 0x34
#define RH_RF95_REG_35_FIFO_THRESH 0x35
#define RH_RF95_REG_36_SEQ_CONFIG_1 0x36
#define RH_RF95_REG_37_SEQ_CONFIG_2 0x37
#define RH_RF95_REG_38_REG_TIMER_RESOL 0x38
#define RH_RF95_REG_39_REG_TIMER1_COEF 0x39
#define RH_RF95_REG_3A_REG_TIMER2_COEF 0x3a
#define RH_RF95_REG_3B_IMAGE_CAL 0x3b
#define RH_RF95_REG_3C_TEMP 0x3c
#define RH_RF95_REG_3D_LOW_BAT 0x3d
#define RH_RF95_REG_3E_IRQ_FLAGS_1 0x3e
#define RH_RF95_REG_3F_IRQ_FLAGS_2 0x3f
#define RH_RF95_REG_40_DIO_MAPPING1 0x40
#define RH_RF95_REG_41_DIO_MAPPING2 0x41
#define RH_RF95_REG_42_VERSION 0x42
#define RH_RF95_REG_44_PLL_HOP 0x44
#define RH_RF95_REG_4B_TCXO 0x4b
#define RH_RF95_REG_4D_PA_DAC 0x4d
#define RH_RF95_REG_5B_FORMER_TEMP 0x5b
#define RH_RF95_REG_5D_BITRATE_FRAC 0x5d
#define RH_RF95_REG_61_AGC_REF 0x61
#define RH_RF95_REG_62_AGC_THRESH1 0x62
#define RH_RF95_REG_63_AGC_THRESH2 0x63
#define RH_RF95_REG_64_AGC_THRESH3 0x64
#define RH_RF95_REG_70_PLL_LF 0x70

// RH_RF95_REG_01_OP_MODE                             0x01
#define RH_RF95_LONG_RANGE_MODE 0x80
#define RH_RF95_ACCESS_SHARED_REG 0x40
#define RH_RF95_LOW_FREQUENCY_MODE 0x08
#define RH_RF95_MODE 0x07
#define RH_RF95_MODE_SLEEP 0x00
#define RH_RF95_MODE_STDBY 0x01
#define RH_RF95_MODE_FSTX 0x02
#define RH_RF95_MODE_TX 0x03
#define RH_RF95_MODE_FSRX 0x04
#define RH_RF95_MODE_RX 0x05

// RH_RF95_REG_09_PA_CONFIG                           0x09
#define RH_RF95_PA_SELECT 0x80
#define RH_RF95_MAX_POWER 0x70
#define RH_RF95_OUTPUT_POWER 0x0f

// RH_RF95_REG_4D_PA_DAC                              0x4d
#define RH_RF95_PA_DAC_DISABLE 0x04
#define RH_RF95_PA_DAC_ENABLE 0x07

// RH_RF95_REG_3F_IRQ_FLAGS_2
#define RH_RF95_IRQ_FifoFull 0x0080      //
#define RH_RF95_IRQ_FifoNotEmpty 0x0040  // at least one byte in the FIFO
#define RH_RF95_IRQ_FifoLevel 0x0020     // more bytes than FifoThreshold
#define RH_RF95_IRQ_FifoOverrun 0x0010   // write this bit to clear the FIFO
#define RH_RF95_IRQ_PacketSent 0x0008    // packet transmission was completed
#define RH_RF95_IRQ_PayloadReady 0x0004
#define RH_RF95_IRQ_CrcOk 0x0002
#define RH_RF95_IRQ_Unused 0x0001

#define RH_RF95_IRQ_RxReady 0x4000      // RxReady
#define RH_RF95_IRQ_TxReady 0x2000      // TxReady
#define RH_RF95_IRQ_PllLock 0x1000      // PllLock
#define RH_RF95_IRQ_PreambleDet 0x0400  // PreambleDetect

class RFM95 {
 public:
    void (*Select)(void);              // activate SPI select
    void (*Deselect)(void);            // deactivate SPI select
    uint8_t (*TransferByte)(uint8_t);  // exchange one byte through SPI
    bool (*DIO0_isOn)(void);           // read DIO0 (packet is ready)
    bool (*DIO4_isOn)(void);           // read DIO4
    void (*RESET_On)(void);            // activate RF chip reset
    void (*RESET_Off)(void);           // deactivate RF chip reset

    uint32_t BaseFrequency;
    int32_t FrequencyCorrection;
    uint32_t ChannelSpacing;
    int16_t Channel;

 private:
    uint8_t WriteByte(uint8_t Byte, uint8_t Addr = 0) const {
        uint8_t Old;

        Select();
        TransferByte(Addr | 0x80);
        Old = TransferByte(Byte);
        Deselect();

        return Old;
    }

    uint16_t WriteWord(uint16_t Word, uint8_t Addr = 0) const {
        uint16_t Old;

        Select();
        TransferByte(Addr | 0x80);
        Old = TransferByte(Word >> 8);
        Old = (Old << 8) | (Word & 0xff);
        Deselect();

        return Old;
    }

    void WriteBytes(const uint8_t *Data, uint8_t Len, uint8_t Addr = 0) const {
        uint8_t Idx;

        Select();
        TransferByte(Addr | 0x80);
        for (Idx = 0; Idx < Len; Idx++) {
            TransferByte(Data[Idx]);
        }
        Deselect();
    }

    uint8_t ReadByte(uint8_t Addr = 0) const {
        uint8_t Byte;

        Select();
        TransferByte(Addr);
        Byte = TransferByte(0);
        Deselect();

        return Byte;
    }

    uint16_t ReadWord(uint8_t Addr = 0) const {
        uint16_t Word;

        Select();
        TransferByte(Addr);
        Word = TransferByte(0);
        Word = (Word << 8) | TransferByte(0);
        Deselect();

        return Word;
    }

 public:
    uint32_t WriteFreq(uint32_t Freq) const {
        const uint8_t Addr = RH_RF95_REG_06_FRF_MSB;
        uint32_t Old;

        Select();
        TransferByte(Addr | 0x80);
        Old = TransferByte(Freq >> 16);
        Old = (Old << 8) | TransferByte(Freq >> 8);
        Old = (Old << 8) | TransferByte(Freq);
        Deselect();

        return Old;
    }

    void setChannel(int16_t newChannel) {
        Channel = newChannel;
        WriteFreq(BaseFrequency + ChannelSpacing * Channel + FrequencyCorrection);
    }

    void WritePacket(const uint8_t *Data, uint8_t Len = 26) const {  // changed from uint8_t
        const uint8_t Addr = RH_RF95_REG_00_FIFO;
        uint8_t Idx, Byte;

        Select();
        TransferByte(Addr | 0x80);
        for (Idx = 0; Idx < Len; Idx++) {
            Byte = Data[Idx];
            TransferByte(ManchesterEncode[Byte >> 4]);
            TransferByte(ManchesterEncode[Byte & 0x0f]);
        }
        Deselect();
    }

    void ReadPacket(uint8_t *Data, uint8_t *Err, uint8_t Len = 26) const {
        const uint8_t Addr = RH_RF95_REG_00_FIFO;
        uint8_t Idx, ByteH, ByteL, ErrH, ErrL;

        Select();
        TransferByte(Addr);
        for (Idx = 0; Idx < Len; Idx++) {
            ByteH = 0;
            ByteH = TransferByte(ByteH);
            ByteH = ManchesterDecode[ByteH];
            ErrH = ByteH >> 4;
            ByteH &= 0x0f;

            ByteL = 0;
            ByteL = TransferByte(ByteL);
            ByteL = ManchesterDecode[ByteL];
            ErrL = ByteL >> 4;
            ByteL &= 0x0f;

            Data[Idx] = (ByteH << 4) | ByteL;
            Err[Idx] = (ErrH << 4) | ErrL;
        }
        Deselect();
    }

    void WriteSYNC(uint8_t WriteSize, uint8_t SyncTol, const uint8_t *SyncData) const {
        if (SyncTol > 7) SyncTol = 7;
        if (WriteSize > 8) WriteSize = 8;
        WriteBytes(SyncData + (8 - WriteSize), WriteSize, RH_RF95_REG_28_SYNC_VALUE_1);  // write the SYNC, skip some initial bytes
        WriteByte(0x80 | ((WriteSize - 1) << 3) | SyncTol, RH_RF95_REG_27_SYNC_CONFIG);  // write SYNC length [bytes]
        WriteWord(9 - WriteSize, RH_RF95_REG_25_PREAMBLE_MSB);                           // write preamble length [bytes]
    }

    void WriteMode(uint8_t Mode = RH_RF95_MODE_STDBY) const { WriteByte(Mode, RH_RF95_REG_01_OP_MODE); }
    uint8_t ReadMode(void) const { return ReadByte(RH_RF95_REG_01_OP_MODE); }
    uint8_t ModeReady(void) const { return ReadByte(RH_RF95_REG_3E_IRQ_FLAGS_1) & 0x80; }
    uint16_t ReadIrqFlags(void) const { return ReadWord(RH_RF95_REG_3E_IRQ_FLAGS_1); }
    void ClearIrqFlags(void) const { WriteWord(0x0810, RH_RF95_REG_3E_IRQ_FLAGS_1); }
    void WriteTxPower_W(int8_t TxPower) const {}   // was RFM69W specific
    void WriteTxPower_HW(int8_t TxPower) const {}  // was RFM69HW specific

    void WriteTxPower(int8_t TxPower, uint8_t isHW = 0) const {
        uint8_t Old;
        uint8_t useRFO = 0;

        //	if(isHW) WriteTxPower_HW(TxPower);
        //	else WriteTxPower_W(TxPower);

        if (useRFO) {
            if (TxPower > 14) TxPower = 14;
            if (TxPower < -1) TxPower = -1;
            // Set RFO, MaxPower = 111, OutputPower = TxP+1
            WriteByte(RH_RF95_MAX_POWER | (TxPower + 1), RH_RF95_REG_09_PA_CONFIG);
        } else {
            if (TxPower > 23) TxPower = 23;
            if (TxPower < 5) TxPower = 5;

            if (TxPower > 20) {
                Old = ReadByte(RH_RF95_REG_4D_PA_DAC);
                Old = (Old & 0xf8) | 0x07;  // retain default 7-3 bit val
                WriteByte(Old, RH_RF95_REG_4D_PA_DAC);
                TxPower -= 3;

            } else {
                Old = ReadByte(RH_RF95_REG_4D_PA_DAC);
                Old = (Old & 0xf8) | 0x04;  // retain default 7-3 bit val
                WriteByte(Old, RH_RF95_REG_4D_PA_DAC);
            }
            // Set PA_Boost, OutputPower = TxP - 5 (MaxPower unused in PA_Boost mode)
            WriteByte(RH_RF95_PA_SELECT | (TxPower - 5), RH_RF95_REG_09_PA_CONFIG);
        }
    }

    void WriteTxPowerMin(void) const { WriteTxPower(-1, 0); }

    void WriteTxPowerMax(void) const {
        uint8_t Old;
        Old = ReadByte(RH_RF95_REG_4D_PA_DAC);
        Old = (Old & 0xf8) | 0x07;  // enable pa boost
        WriteByte(Old, RH_RF95_REG_4D_PA_DAC);
        WriteByte(RH_RF95_PA_SELECT | 0x0f, RH_RF95_REG_09_PA_CONFIG);  // set output power to 1111
        WriteByte(0x3b, RH_RF95_REG_0B_OCP);                            // set overcurrent protection to max
    }

    int Configure(int16_t Channel, const uint8_t *Sync) {
        WriteMode(RH_RF95_MODE_STDBY);  // change mode to STBY

        ClearIrqFlags();
        // #TODO bad register, ramp etc. configured in 0x0A reg
        WriteByte(0x49, RH_RF95_REG_0A_PA_RAMP);           // gaussian filter BT = 0.5,
                                                           // rise/fall time = 40us, packet
                                                           // mode, fsk mode enabled by
                                                           // default (reg 0x31)
        WriteWord(0x0140, RH_RF95_REG_02_BITRATE_MSB);     // bitrate = 100kbps
        WriteWord(0x0333, RH_RF95_REG_04_FDEV_MSB);        // FSK deviation 50kHz
        setChannel(Channel);                               // channel
        WriteSYNC(8, 7, Sync);                             // sync pattern
        WriteByte(0x00, RH_RF95_REG_30_PACKET_CONFIG_1);   // fixed packet size, no
                                                           // DC-free encoding, no CRC,
                                                           // no address filtering
        WriteByte(0x80 + 51, RH_RF95_REG_35_FIFO_THRESH);  // TxStartCondition=FifoNotEmpty, FIFO
                                                           // threshold = 51 bytes
        WriteByte(2 * 26, RH_RF95_REG_32_PAYLOAD_LENGTH);  // Packet size = 26 bytes
                                                           // Manchester encoded into
                                                           // 52 bytes
        WriteByte(0x08, RH_RF95_REG_0C_LNA);
        WriteByte(2 * 114, RH_RF95_REG_10_RSSI_THRESH);  // RSSI threshold = -114dBm
        WriteByte(0x0a, RH_RF95_REG_12_RX_BW);           // +/-100kHz Rx bandwidth => p.27+67
        WriteByte(0x0a, RH_RF95_REG_13_AFC_BW);          // +/-200kHz Rx bandwidth while AFC

        WriteByte(0xad, RH_RF95_REG_08_FRF_LSB);       // 868 MHz
        WriteByte(0x08, RH_RF95_REG_40_DIO_MAPPING1);  // Rx ready on DIO2
        WriteByte(0xC0, RH_RF95_REG_41_DIO_MAPPING2);  // Rssi/Preamble on DIO4
        WriteTxPowerMin();

        return 0;
    }

    void ContinousModeTest(void) const {
        uint8_t Old;

        WriteMode(RH_RF95_MODE_STDBY);            // change mode to STBY
        WriteByte(0x49, RH_RF95_REG_0A_PA_RAMP);  // 50us ramp, gaussian BT 0.5

        // WriteTxPowerMin();
        WriteTxPower(23, 0);

        ClearIrqFlags();
        Old = ReadByte(RH_RF95_REG_31_PACKET_CONFIG_2);
        Old |= (0x01 << 6);  // enable continuous mode
        WriteByte(Old, RH_RF95_REG_31_PACKET_CONFIG_2);

        WriteByte(0xad, RH_RF95_REG_08_FRF_LSB);  // 868 MHz

        WriteByte(0x20, RH_RF95_REG_41_DIO_MAPPING2);  // enable data mode on pin DIO5
        WriteByte(0xb, RH_RF95_REG_01_OP_MODE);
    }

    uint8_t ReadVersion(void) const { return ReadByte(RH_RF95_REG_42_VERSION); }  // normally returns: 0x24

    void TriggerTemp(void) const { WriteByte(0x08, RH_RF95_REG_3C_TEMP); }            // trigger measurement
    uint8_t RunningTemp(void) const { return ReadByte(RH_RF95_REG_3C_TEMP) & 0x04; }  // still running ?
    uint8_t ReadTemp(void) const { return ReadByte(RH_RF95_REG_3C_TEMP); }            // read value: -1 deg/LSB
};
#endif
