#include <stdint.h>
#include <stdlib.h>

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"

#include "tim.h"

#include "nmea.h"
#include "ubx.h"

#include "ogn.h"

#include "uart1.h"
#include "uart2.h"

#include "parameters.h"

#include "gps.h"

// ----------------------------------------------------------------------------

inline void GPS_DISABLE(void) {
    GPIO_ResetBits(GPIOB, GPIO_Pin_4);
}
inline void GPS_ENABLE(void) {
    GPIO_SetBits(GPIOB, GPIO_Pin_4);
}

inline int GPS_PPS_isOn(void) {
    return (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == Bit_SET);
}

void GPS_Configuration(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;  // Configure PB4 as output: GPS Enable(HIGH) / Shutdown(LOW)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  // Configure PB3 as input: PPS from GPS
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPS_ENABLE();
}

// void Debug_Print(uint8_t Byte) { while(!UART1_TxEmpty()) taskYIELD(); UART1_TxChar(Byte); }

static NMEA_RxMsg NMEA;  // NMEA sentences catcher
static UBX_RxMsg UBX;    // UBX messages catcher

static OgnPosition Position[4];  // four GPS position pipe
static uint8_t PosIdx = 0;

// static TickType_t Burst_TickCount;       // [msec] TickCount when the data burst from GPS started
//
//         TickType_t PPS_TickCount;       // [msec] TickCount of the most recent PPS pulse
uint32_t GPS_TimeSinceLock;    // [sec] time since the GPS has a lock
volatile uint8_t GPS_Sec = 0;  // [sec] UTC time: second

uint32_t GPS_UnixTime = 0;  // [sec] UTC date/time in Unix format
uint32_t GPS_FatTime = 0;   // [sec] UTC date/time in FAT format
int32_t GPS_Altitude = 0;   // [0.1m] last valid altitude
int32_t GPS_Latitude = 0;
int32_t GPS_Longitude = 0;
int16_t GPS_GeoidSepar = 0;  // [0.1m]

// static char Line[64];                  // for console output formating

// UART2 as 'STDOUT'
static void (*STDOUT)(char) = UART2_Write;

uint16_t PPS_Phase(void) {
    // TickType_t Phase = xTaskGetTickCount() - PPS_TickCount;
    // if (Phase < 1000) return Phase;
    return 0;  // Phase % 1000;
}  // [0...999 ms] current time-phase in respect to PPS

// ----------------------------------------------------------------------------

static void GPS_PPS_On(void)  // called on rising edge of PPS
{
    // LED_PCB_Flash(50);
    // PPS_TickCount = xTaskGetTickCount();
    uint8_t Sec = GPS_Sec;
    Sec++;
    if (Sec >= 60) Sec = 0;
    GPS_Sec = Sec;
    GPS_UnixTime++;
}

static void GPS_PPS_Off(void)  // called on falling edge of PPS
{}

static void GPS_LockStart(void)  // called when GPS catches a lock
{}

static void GPS_LockEnd(void)  // called when GPS looses a lock
{}

static void GPS_BurstStart(void)  // when GPS starts sending the data on the serial port
{
    // Burst_TickCount = xTaskGetTickCount();
}

static void GPS_BurstEnd(void)  // when GPS stops sending data on the serial port
{
    if (Position[PosIdx].isComplete())  // position data complete
    {
        if (Position[PosIdx].isTimeValid()) {
            GPS_Sec = Position[PosIdx].Sec;
            if (Position[PosIdx].isDateValid()) {
                GPS_UnixTime = Position[PosIdx].getUnixTime();
                GPS_FatTime = Position[PosIdx].getFatTime();
            }
        }
        if (Position[PosIdx].isValid())  // position is complete and locked
        {
            GPS_TimeSinceLock++;
            GPS_Altitude = Position[PosIdx].Altitude;
            GPS_Latitude = Position[PosIdx].Latitude;
            GPS_Longitude = Position[PosIdx].Longitude;
            GPS_GeoidSepar = Position[PosIdx].GeoidSeparation;
            if (GPS_TimeSinceLock == 1) {
                GPS_LockStart();
            }
            if (GPS_TimeSinceLock > 2) {
                uint8_t PrevIdx = (PosIdx + 2) & 3;
                /* int Delta = */ Position[PosIdx].calcDifferences(Position[PrevIdx]);
                // LED_PCB_Flash(100);
            }
        } else  // complete but not valid lock
        {
            if (GPS_TimeSinceLock) {
                GPS_LockEnd();
                GPS_TimeSinceLock = 0;
            }
        }
    } else  // position not complete, no GPS lock
    {
        if (GPS_TimeSinceLock) {
            GPS_LockEnd();
            GPS_TimeSinceLock = 0;
        }
    }
    uint8_t NextPosIdx = (PosIdx + 1) & 3;  // next position to be recorded
    Position[NextPosIdx].Clear();
    int8_t Sec = Position[PosIdx].Sec;
    Sec++;
    if (Sec >= 60) Sec = 0;
    Position[NextPosIdx].Sec = Sec;
    // Position[NextPosIdx].copyTime(Position[PosIdx]);                        // copy time from current position
    // Position[NextPosIdx].incrTime();                                        // increment time by 1 sec
    PosIdx = NextPosIdx;  // advance the index
}

OgnPosition *GPS_getPosition(void) {
    uint8_t PrevIdx = PosIdx;
    OgnPosition *PrevPos = Position + PrevIdx;
    if (PrevPos->isComplete()) return PrevPos;
    PrevIdx = (PrevIdx + 3) & 3;
    PrevPos = Position + PrevIdx;
    if (PrevPos->isComplete()) return PrevPos;
    return 0;
}

OgnPosition *GPS_getPosition(int8_t Sec) {
    for (uint8_t Idx = 0; Idx < 4; Idx++) {
        if (Sec == Position[Idx].Sec) return Position + Idx;
    }
    return 0;
}

// ----------------------------------------------------------------------------

static void GPS_NMEA(void)  // when GPS gets a correct NMEA sentence
{
    // LED_PCB_Flash(2);                 // Flash the LED for 2 ms
    Position[PosIdx].ReadNMEA(NMEA);  // read position elements from NMEA
    GPS_Altitude = Position[PosIdx].Altitude;
    GPS_Latitude = Position[PosIdx].Latitude;
    GPS_Longitude = Position[PosIdx].Longitude;
    if (NMEA.isGxRMC() || NMEA.isGxGGA() || NMEA.isGxGSA() || NMEA.isGPTXT()) {
        static char CRNL[3] = "\r\n";
        // xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
        Format_String(STDOUT, "GPS DATA: ");
        Format_Bytes(STDOUT, NMEA.Data, NMEA.Len);
        Format_String(STDOUT, " -lat=");
        Format_String(STDOUT, " -lon=");
        // #TODO print -lat -lon correctly
        Format_Bytes(STDOUT, (const uint8_t *)CRNL, 2);
        // xSemaphoreGive(UART1_Mutex);
    }
}

static void GPS_UBX(void)  // when GPS gets an UBX packet
{}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------

int GPS_ReadCycle(void) {
    uint16_t delay_ms = 1;  // Delay between serial FIFO polls - at 9600baud FIFO of len 64 fills in roughly 5ms
    uint8_t bytesRead = 0;
    int n = 2000;  // Minimum read cycle duration - n*delay_ms = 2sec

    NMEA.Clear();
    while (n) {
        for (;;) {
            uint8_t Byte;
            int Err = UART1_Read(Byte);  // get Byte from serial port, if present
            if (Err <= 0) break;
            ++bytesRead;
            NMEA.ProcessByte(Byte);                // process current byte through the NMEA interpreter
            if (NMEA.isComplete()) {               // NMEA completely received ?
                if (NMEA.isChecked()) GPS_NMEA();  // NMEA check sum is correct ?
                NMEA.Clear();
            }
        }
        TIM3_DelayMS(delay_ms);
        --n;
    }
    return bytesRead;
}

#ifdef __cplusplus
extern "C"
#endif
    void
    vTaskGPS(void *pvParameters) {
    // PPS_TickCount = 0;
    // Burst_TickCount = 0;

    // UART2_Configuration(Parameters.GPSbaud);
    // GPS_Configuration();

    // vTaskDelay(5);

    // xSemaphoreTake(UART1_Mutex, portMAX_DELAY);
    Format_String(STDOUT, "TaskGPS:");
    Format_String(STDOUT, "\n");
    // xSemaphoreGive(UART1_Mutex);

    int Burst = (-1);  // GPS transmission ongoing or line is idle ?
                       //{ // why new scope {} without if / loop statement?!
    int LineIdle = 0;  // counts idle time for the GPS data
    int PPS = 0;
    NMEA.Clear();
    UBX.Clear();  // scans GPS input for NMEA and UBX frames
    for (uint8_t Idx = 0; Idx < 4; Idx++)
        Position[Idx].Clear();
    PosIdx = 0;
    // PktIdx=0;

    for (;;)  //
    {
        // vTaskDelay(1);  // wait for the next time tick

        if (GPS_PPS_isOn()) {
            if (!PPS) {
                PPS = 1;
                GPS_PPS_On();
            }
        }  // monitor GPS PPS signal
        else {
            if (PPS) {
                PPS = 0;
                GPS_PPS_Off();
            }
        }

        LineIdle++;  // count idle time
        for (;;) {
            uint8_t Byte;
            int Err = UART1_Read(Byte);  // get Byte from serial port, if present
            if (Err <= 0) break;
            LineIdle = 0;            // if there was a byte: restart idle counting
            NMEA.ProcessByte(Byte);  // process current byte through the NMEA interpreter
            UBX.ProcessByte(Byte);   // process current byte through the UBX interpreter
            if (NMEA.isComplete())   // NMEA completely received ?
            {
                if (NMEA.isChecked()) GPS_NMEA();  // NMEA check sum is correct ?
                NMEA.Clear();
            }
            if (UBX.isComplete()) {
                GPS_UBX();
                UBX.Clear();
            }
        }

        if (LineIdle == 0)  // if any bytes were received ?
        {
            if (Burst == 0) GPS_BurstStart();  // burst started
            Burst = 1;
        } else if (LineIdle > 10)  // if GPS sends no more data for 10 time ticks
        {
            if (Burst > 0)  // if still in burst
            {
                GPS_BurstEnd();          // burst just ended
            } else if (LineIdle > 1000)  // if idle for more than 1 sec
            {
            }
            Burst = 0;
        }
    }
    //}
}
