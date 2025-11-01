/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "platform.h"

#ifdef USE_SERIALRX_CRSF

#include "build/build_config.h"
#include "build/debug.h"

#include "common/crc.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/config.h"

#include "pg/rx.h"

#include "drivers/persistent.h"
#include "drivers/serial.h"
#include "drivers/serial_uart.h"
#include "drivers/system.h"
#include "drivers/time.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/crsf.h"

#include "telemetry/crsf.h"

#define CRSF_TIME_NEEDED_PER_FRAME_US   1750 // a maximally sized 64byte payload will take ~1550us, round up to 1750.
#define CRSF_TIME_BETWEEN_FRAMES_US     6667 // At fastest, frames are sent by the transmitter every 6.667 milliseconds, 150 Hz

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

#define CRSF_PAYLOAD_OFFSET offsetof(crsfFrameDef_t, type)

#define CRSF_LINK_STATUS_UPDATE_TIMEOUT_US  250000 // 250ms, 4 Hz mode 1 telemetry

#define CRSF_FRAME_ERROR_COUNT_THRESHOLD    3

typedef struct crsfRuntimeState_s {
    uint8_t         portID;
    bool            crsfFrameDone;
    crsfFrame_t     crsfFrame;
    crsfFrame_t     crsfChannelDataFrame;
    uint32_t        crsfChannelData[CRSF_MAX_CHANNEL];
    timeUs_t        crsfFrameStartAtUs;
    uint8_t         crsfFramePosition;
#if defined(USE_CRSF_V3)
    uint8_t         crsfFrameErrorCnt;
#endif
    float           channelScale;
} crsfRuntimeState_t;

STATIC_UNIT_TESTED crsfRuntimeState_t crsfRuntimeStates[RX_SERIAL_COUNT];


static serialPort_t *serialPort;
static uint8_t telemetryBuf[CRSF_FRAME_SIZE_MAX];
static uint8_t telemetryBufLen = 0;

#ifdef USE_RX_LINK_UPLINK_POWER
#define CRSF_UPLINK_POWER_LEVEL_MW_ITEMS_COUNT 9
// Uplink power levels by uplinkTXPower expressed in mW (250 mW is from ver >=4.00, 50 mW in a future version and for ExpressLRS)
const uint16_t uplinkTXPowerStatesMw[CRSF_UPLINK_POWER_LEVEL_MW_ITEMS_COUNT] = {0, 10, 25, 100, 500, 1000, 2000, 250, 50};
#endif

/*
 * CRSF protocol
 *
 * CRSF protocol uses a single wire half duplex uart connection.
 * The master sends one frame every 4ms and the slave replies between two frames from the master.
 *
 * 420000 baud
 * not inverted
 * 8 Bit
 * 1 Stop bit
 * Big endian
 * 420000 bit/s = 46667 byte/s (including stop bit) = 21.43us per byte
 * Max frame size is 64 bytes
 * A 64 byte frame plus 1 sync byte can be transmitted in 1393 microseconds.
 *
 * CRSF_TIME_NEEDED_PER_FRAME_US is set conservatively at 1500 microseconds
 *
 * Every frame has the structure:
 * <Device address><Frame length><Type><Payload><CRC>
 *
 * Device address: (uint8_t)
 * Frame length:   length in  bytes including Type (uint8_t)
 * Type:           (uint8_t)
 * CRC:            (uint8_t)
 *
 */

struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));

typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

/*
* SUBSET RC FRAME 0x17
*
* The structure of 0x17 frame consists of 8-bit configuration data & variable length packed channel data.
*
* definition of the configuration byte
* bits 0-4: number of first channel packed
* bits 5-6: resolution configuration of the channel data (00 -> 10 bits, 01 -> 11 bits, 10 -> 12 bits, 11 -> 13 bits)
* bit 7:    reserved

* data structure of the channel data
*  - first channel packed with specified resolution
*  - second channel packed with specified resolution
*  - third channel packed with specified resolution
*                       ...
*  - last channel packed with specified resolution
*/


#if defined(USE_CRSF_LINK_STATISTICS)
/*
 * 0x14 Link statistics
 * Payload:
 *
 * uint8_t Uplink RSSI Ant. 1 ( dBm * -1 )
 * uint8_t Uplink RSSI Ant. 2 ( dBm * -1 )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
 * uint8_t RF Mode ( enum 4fps = 0 , 50fps, 150hz)
 * uint8_t Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 250mW )
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * Uplink is the connection from the ground to the UAV and downlink the opposite direction.
 */

/*
 * 0x1C Link statistics RX
 * Payload:
 *
 * uint8_t Downlink RSSI ( dBm * -1 )
 * uint8_t Downlink RSSI ( % )
 * uint8_t Downlink Package success rate / Link quality ( % )
 * int8_t Downlink SNR ( db )
 * uint8_t Uplink RF Power ( db )
 */

/*
 * 0x1D Link statistics TX
 * Payload:
 *
 * uint8_t Uplink RSSI ( dBm * -1 )
 * uint8_t Uplink RSSI ( % )
 * uint8_t Uplink Package success rate / Link quality ( % )
 * int8_t Uplink SNR ( db )
 * uint8_t Downlink RF Power ( db )
 * uint8_t Uplink FPS ( FPS / 10 )
 */

typedef struct crsfPayloadLinkstatistics_s {
    uint8_t uplink_RSSI_1;
    uint8_t uplink_RSSI_2;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t active_antenna;
    uint8_t rf_Mode;
    uint8_t uplink_TX_Power;
    uint8_t downlink_RSSI;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
} crsfLinkStatistics_t;

#if defined(USE_CRSF_V3)
typedef struct crsfPayloadLinkstatisticsRx_s {
    uint8_t downlink_RSSI_1;
    uint8_t downlink_RSSI_1_percentage;
    uint8_t downlink_Link_quality;
    int8_t downlink_SNR;
    uint8_t uplink_power;
} crsfLinkStatisticsRx_t; // this struct is currently not used

typedef struct crsfPayloadLinkstatisticsTx_s {
    uint8_t uplink_RSSI;
    uint8_t uplink_RSSI_percentage;
    uint8_t uplink_Link_quality;
    int8_t uplink_SNR;
    uint8_t downlink_power; // currently not used
    uint8_t uplink_FPS; // currently not used
} crsfLinkStatisticsTx_t;
#endif

static timeUs_t lastLinkStatisticsFrameUs;

static int port_id_to_rssi_id(int port_id)
{
    int pid[] = {
        /*SERIAL_PORT_USART1,*/
        /*SERIAL_PORT_USART2,*/
        SERIAL_PORT_USART3,
        SERIAL_PORT_UART4,
        SERIAL_PORT_UART5,
        SERIAL_PORT_USART6,
    };
    int rssi_id = -1;

    for (unsigned int i = 0; i < sizeof(pid) / sizeof(pid[0]); i++) {
        if (pid[i] == port_id) {
            rssi_id = i;
            break;
        }
    }

    return rssi_id;
}

static void handleCrsfLinkStatisticsFrame(const crsfLinkStatistics_t* statsPtr, timeUs_t currentTimeUs, int portID)
{
    const crsfLinkStatistics_t stats = *statsPtr;
    lastLinkStatisticsFrameUs = currentTimeUs;
    int16_t rssiDbm = -1 * (stats.active_antenna ? stats.uplink_RSSI_2 : stats.uplink_RSSI_1);
    if (rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) {
        const uint16_t rssiPercentScaled = scaleRange(rssiDbm, CRSF_RSSI_MIN, CRSF_RSSI_MAX, 0, RSSI_MAX_VALUE);
        int rssi_id = port_id_to_rssi_id(portID);
        if (rssi_id < 0)
            setRssi(rssiPercentScaled, RSSI_SOURCE_RX_PROTOCOL_CRSF);
        else
            set_rssi_val(rssiPercentScaled, RSSI_SOURCE_RX_PROTOCOL_CRSF, rssi_id);
    }
#ifdef USE_RX_RSSI_DBM
    setRssiDbm(rssiDbm, RSSI_SOURCE_RX_PROTOCOL_CRSF);
    setActiveAntenna(stats.active_antenna);
#endif

#ifdef USE_RX_RSNR
    setRsnr(stats.uplink_SNR);
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) {
        setLinkQualityDirect(stats.uplink_Link_quality);
        rxSetRfMode(stats.rf_Mode);
    }
#endif

#ifdef USE_RX_LINK_UPLINK_POWER
    const uint8_t crsfUplinkPowerStatesItemIndex = (stats.uplink_TX_Power < CRSF_UPLINK_POWER_LEVEL_MW_ITEMS_COUNT) ? stats.uplink_TX_Power : 0;
    rxSetUplinkTxPwrMw(uplinkTXPowerStatesMw[crsfUplinkPowerStatesItemIndex]);
#endif

    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 0, stats.uplink_RSSI_1);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 1, stats.uplink_RSSI_2);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 2, stats.uplink_Link_quality);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 3, stats.rf_Mode);

    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 0, stats.active_antenna);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 1, stats.uplink_SNR);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_PWR, 2, stats.uplink_TX_Power);

    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 0, stats.downlink_RSSI);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 1, stats.downlink_Link_quality);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_DOWN, 2, stats.downlink_SNR);
}

#if defined(USE_CRSF_V3)
static void handleCrsfLinkStatisticsTxFrame(const crsfLinkStatisticsTx_t* statsPtr, timeUs_t currentTimeUs)
{
    const crsfLinkStatisticsTx_t stats = *statsPtr;
    lastLinkStatisticsFrameUs = currentTimeUs;
    if (rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) {
        const uint16_t rssiPercentScaled = scaleRange(stats.uplink_RSSI_percentage, 0, 100, 0, RSSI_MAX_VALUE);
        setRssi(rssiPercentScaled, RSSI_SOURCE_RX_PROTOCOL_CRSF);
    }
#ifdef USE_RX_RSSI_DBM
    int16_t rssiDbm = -1 * stats.uplink_RSSI;
    setRssiDbm(rssiDbm, RSSI_SOURCE_RX_PROTOCOL_CRSF);
#endif

#ifdef USE_RX_RSNR
    setRsnr(stats.uplink_SNR);
#endif

#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) {
        setLinkQualityDirect(stats.uplink_Link_quality);
    }
#endif

    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 0, stats.uplink_RSSI);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 1, stats.uplink_SNR);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 2, stats.uplink_Link_quality);
    DEBUG_SET(DEBUG_CRSF_LINK_STATISTICS_UPLINK, 3, stats.uplink_RSSI_percentage);
}
#endif
#endif

#if defined(USE_CRSF_LINK_STATISTICS)
static void crsfCheckRssi(uint32_t currentTimeUs)
{

    if (cmpTimeUs(currentTimeUs, lastLinkStatisticsFrameUs) > CRSF_LINK_STATUS_UPDATE_TIMEOUT_US) {
        if (rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) {
            setRssiDirect(0, RSSI_SOURCE_RX_PROTOCOL_CRSF);
#ifdef USE_RX_RSSI_DBM
            setRssiDbmDirect(CRSF_RSSI_MIN, RSSI_SOURCE_RX_PROTOCOL_CRSF);
#endif
#ifdef USE_RX_RSNR
            setRsnrDirect(CRSF_SNR_MIN);
#endif
        }
#ifdef USE_RX_LINK_QUALITY_INFO
        if (linkQualitySource == LQ_SOURCE_RX_PROTOCOL_CRSF) {
            setLinkQualityDirect(0);
        }
#endif
    }
}
#endif

STATIC_UNIT_TESTED uint8_t crsfFrameCRC(crsfFrame_t *crsfFrame)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, crsfFrame->frame.type);
    for (int ii = 0; ii < crsfFrame->frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crsfFrame->frame.payload[ii]);
    }
    return crc;
}

#if defined(USE_CRSF_V3) || defined(UNIT_TEST)
STATIC_UNIT_TESTED uint8_t crsfFrameCmdCRC(crsfFrame_t *crsfFrame)
{
    // CRC includes type and payload
    uint8_t crc = crc8_poly_0xba(0, crsfFrame->frame.type);
    for (int ii = 0; ii < crsfFrame->frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC - 1; ++ii) {
        crc = crc8_poly_0xba(crc, crsfFrame->frame.payload[ii]);
    }
    return crc;
}
#endif

// Receive ISR callback, called back from serial port
STATIC_UNIT_TESTED void crsfDataReceive(uint16_t c, void *data)
{
    rxRuntimeState_t *const rxRuntimeState = (rxRuntimeState_t *const)data;
    crsfRuntimeState_t *const crsfRuntimeState = (crsfRuntimeState_t *const)rxRuntimeState->priv;

    const timeUs_t currentTimeUs = microsISR();

#ifdef DEBUG_CRSF_PACKETS
    debug[2] = currentTimeUs - crsfRuntimeState->crsfFrameStartAtUs;
#endif

    if (cmpTimeUs(currentTimeUs, crsfRuntimeState->crsfFrameStartAtUs) > CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
#if defined(USE_CRSF_V3)
        if (crsfRuntimeState->crsfFramePosition > 0) {
            // count an error if full valid frame not received within the allowed time.
            crsfRuntimeState->crsfFrameErrorCnt++;
        }
#endif
        crsfRuntimeState->crsfFramePosition = 0;
    }

    if (crsfRuntimeState->crsfFramePosition == 0) {
        crsfRuntimeState->crsfFrameStartAtUs = currentTimeUs;
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    // sometimes we can receive some garbage data. So, we need to check max size for preventing buffer overrun.
    const int fullFrameLength = crsfRuntimeState->crsfFramePosition < 3 ? 5 : MIN(crsfRuntimeState->crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH, CRSF_FRAME_SIZE_MAX);

    if (crsfRuntimeState->crsfFramePosition < fullFrameLength) {
        crsfRuntimeState->crsfFrame.bytes[crsfRuntimeState->crsfFramePosition++] = (uint8_t)c;
        if (crsfRuntimeState->crsfFramePosition >= fullFrameLength) {
            crsfRuntimeState->crsfFramePosition = 0;
            const uint8_t crc = crsfFrameCRC(&crsfRuntimeState->crsfFrame);
            if (crc == crsfRuntimeState->crsfFrame.bytes[fullFrameLength - 1]) {
#if defined(USE_CRSF_V3)
                crsfRuntimeState->crsfFrameErrorCnt = 0;
#endif
                switch (crsfRuntimeState->crsfFrame.frame.type) {
                case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
                case CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED:
                    if (crsfRuntimeState->crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) {
                        rxRuntimeState->lastRcFrameTimeUs = currentTimeUs;
                        crsfRuntimeState->crsfFrameDone = true;
                        memcpy(&crsfRuntimeState->crsfChannelDataFrame, &crsfRuntimeState->crsfFrame, sizeof(crsfRuntimeState->crsfFrame));
                    }
                    break;

#if defined(USE_TELEMETRY_CRSF) && defined(USE_MSP_OVER_TELEMETRY)
                case CRSF_FRAMETYPE_MSP_REQ:
                case CRSF_FRAMETYPE_MSP_WRITE: {
                    uint8_t *frameStart = (uint8_t *)&crsfRuntimeState->crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                    if (bufferCrsfMspFrame(frameStart, crsfRuntimeState->crsfFrame.frame.frameLength - 4)) {
                        crsfScheduleMspResponse(crsfRuntimeState->crsfFrame.frame.payload[1]);
                    }
                    break;
                }
#endif
#if defined(USE_CRSF_CMS_TELEMETRY)
                case CRSF_FRAMETYPE_DEVICE_PING:
                    crsfScheduleDeviceInfoResponse();
                    break;
                case CRSF_FRAMETYPE_DEVICE_INFO:
                    crsfHandleDeviceInfoResponse(crsfRuntimeState->crsfFrame.frame.payload);
                    break;
                case CRSF_FRAMETYPE_DISPLAYPORT_CMD: {
                    uint8_t *frameStart = (uint8_t *)&crsfRuntimeState->crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE;
                    crsfProcessDisplayPortCmd(frameStart);
                    break;
                }
#endif
#if defined(USE_CRSF_LINK_STATISTICS)

                case CRSF_FRAMETYPE_LINK_STATISTICS: {
                    // if to FC and 10 bytes + CRSF_FRAME_ORIGIN_DEST_SIZE
                    if ((rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) &&
                        (crsfRuntimeState->crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                        (crsfRuntimeState->crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE)) {
                        const crsfLinkStatistics_t* statsFrame = (const crsfLinkStatistics_t*)&crsfRuntimeState->crsfFrame.frame.payload;
                        handleCrsfLinkStatisticsFrame(statsFrame, currentTimeUs, crsfRuntimeState->portID);
                    }
                    break;
                }
#if defined(USE_CRSF_V3)
                case CRSF_FRAMETYPE_LINK_STATISTICS_RX: {
                    break;
                }
                case CRSF_FRAMETYPE_LINK_STATISTICS_TX: {
                    if ((rssiSource == RSSI_SOURCE_RX_PROTOCOL_CRSF) &&
                        (crsfRuntimeState->crsfFrame.frame.deviceAddress == CRSF_ADDRESS_FLIGHT_CONTROLLER) &&
                        (crsfRuntimeState->crsfFrame.frame.frameLength == CRSF_FRAME_ORIGIN_DEST_SIZE + CRSF_FRAME_LINK_STATISTICS_TX_PAYLOAD_SIZE)) {
                        const crsfLinkStatisticsTx_t* statsFrame = (const crsfLinkStatisticsTx_t*)&crsfRuntimeState->crsfFrame.frame.payload;
                        handleCrsfLinkStatisticsTxFrame(statsFrame, currentTimeUs);
                    }
                    break;
                }
#endif
#endif
#if defined(USE_CRSF_V3)
                case CRSF_FRAMETYPE_COMMAND:
                    if ((crsfRuntimeState->crsfFrame.bytes[fullFrameLength - 2] == crsfFrameCmdCRC(&crsfRuntimeState->crsfFrame)) &&
                        (crsfRuntimeState->crsfFrame.bytes[3] == CRSF_ADDRESS_FLIGHT_CONTROLLER)) {
                        crsfProcessCommand(crsfRuntimeState->crsfFrame.frame.payload + CRSF_FRAME_ORIGIN_DEST_SIZE);
                    }
                    break;
#endif
                default:
                    break;
                }
            } else {
#if defined(USE_CRSF_V3)
                if (crsfRuntimeState->crsfFrameErrorCnt < CRSF_FRAME_ERROR_COUNT_THRESHOLD)
                    crsfRuntimeState->crsfFrameErrorCnt++;
#endif
            }
        }
#if defined(USE_CRSF_V3)
        if (crsfBaudNegotiationInProgress() || isEepromWriteInProgress()) {
            // don't count errors when negotiation or eeprom write is in progress
            crsfRuntimeState->crsfFrameErrorCnt = 0;
        } else if (crsfRuntimeState->crsfFrameErrorCnt >= CRSF_FRAME_ERROR_COUNT_THRESHOLD) {
            // fall back to default speed if speed mismatch detected
            setCrsfDefaultSpeed();
            crsfRuntimeState->crsfFrameErrorCnt = 0;
        }
#endif
    }
}

STATIC_UNIT_TESTED uint8_t crsfFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    crsfRuntimeState_t *crsfRuntimeState = (crsfRuntimeState_t *)rxRuntimeState->priv;

#if defined(USE_CRSF_LINK_STATISTICS)
    crsfCheckRssi(micros());
#endif
    if (crsfRuntimeState->crsfFrameDone) {
        crsfRuntimeState->crsfFrameDone = false;

        // unpack the RC channels
        if (crsfRuntimeState->crsfChannelDataFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            // use ordinary RC frame structure (0x16)
            const crsfPayloadRcChannelsPacked_t* const rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfRuntimeState->crsfChannelDataFrame.frame.payload;
            crsfRuntimeState->channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;
            crsfRuntimeState->crsfChannelData[0] = rcChannels->chan0;
            crsfRuntimeState->crsfChannelData[1] = rcChannels->chan1;
            crsfRuntimeState->crsfChannelData[2] = rcChannels->chan2;
            crsfRuntimeState->crsfChannelData[3] = rcChannels->chan3;
            crsfRuntimeState->crsfChannelData[4] = rcChannels->chan4;
            crsfRuntimeState->crsfChannelData[5] = rcChannels->chan5;
            crsfRuntimeState->crsfChannelData[6] = rcChannels->chan6;
            crsfRuntimeState->crsfChannelData[7] = rcChannels->chan7;
            crsfRuntimeState->crsfChannelData[8] = rcChannels->chan8;
            crsfRuntimeState->crsfChannelData[9] = rcChannels->chan9;
            crsfRuntimeState->crsfChannelData[10] = rcChannels->chan10;
            crsfRuntimeState->crsfChannelData[11] = rcChannels->chan11;
            crsfRuntimeState->crsfChannelData[12] = rcChannels->chan12;
            crsfRuntimeState->crsfChannelData[13] = rcChannels->chan13;
            crsfRuntimeState->crsfChannelData[14] = rcChannels->chan14;
            crsfRuntimeState->crsfChannelData[15] = rcChannels->chan15;
        } else {
            // use subset RC frame structure (0x17)
            uint8_t readByteIndex = 0;
            const uint8_t *payload = crsfRuntimeState->crsfChannelDataFrame.frame.payload;

            // get the configuration byte
            uint8_t configByte = payload[readByteIndex++];

            // get the channel number of start channel
            uint8_t startChannel = configByte & CRSF_SUBSET_RC_STARTING_CHANNEL_MASK;
            configByte >>= CRSF_SUBSET_RC_STARTING_CHANNEL_BITS;

            // get the channel resolution settings
            uint8_t channelBits;
            uint16_t channelMask;
            uint8_t channelRes = configByte & CRSF_SUBSET_RC_RES_CONFIGURATION_MASK;
            configByte >>= CRSF_SUBSET_RC_RES_CONFIGURATION_BITS;
            switch (channelRes) {
            case CRSF_SUBSET_RC_RES_CONF_10B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_10B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_10B;
                crsfRuntimeState->channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_10B;
                break;
            default:
            case CRSF_SUBSET_RC_RES_CONF_11B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_11B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_11B;
                crsfRuntimeState->channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_11B;
                break;
            case CRSF_SUBSET_RC_RES_CONF_12B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_12B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_12B;
                crsfRuntimeState->channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_12B;
                break;
            case CRSF_SUBSET_RC_RES_CONF_13B:
                channelBits = CRSF_SUBSET_RC_RES_BITS_13B;
                channelMask = CRSF_SUBSET_RC_RES_MASK_13B;
                crsfRuntimeState->channelScale = CRSF_SUBSET_RC_CHANNEL_SCALE_13B;
                break;
            }

            // do nothing for the reserved configuration bit
            configByte >>= CRSF_SUBSET_RC_RESERVED_CONFIGURATION_BITS;

            // calculate the number of channels packed
            uint8_t numOfChannels = ((crsfRuntimeState->crsfChannelDataFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC - 1) * 8) / channelBits;

            // unpack the channel data
            uint8_t bitsMerged = 0;
            uint32_t readValue = 0;
            for (uint8_t n = 0; n < numOfChannels; n++) {
                while (bitsMerged < channelBits) {
                    uint8_t readByte = payload[readByteIndex++];
                    readValue |= ((uint32_t) readByte) << bitsMerged;
                    bitsMerged += 8;
                }
                crsfRuntimeState->crsfChannelData[startChannel + n] = readValue & channelMask;
                readValue >>= channelBits;
                bitsMerged -= channelBits;
            }
        }
        return RX_FRAME_COMPLETE;
    }
    return RX_FRAME_PENDING;
}

STATIC_UNIT_TESTED float crsfReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t chan)
{
    crsfRuntimeState_t *crsfRuntimeState = (crsfRuntimeState_t *)rxRuntimeState->priv;
    if (crsfRuntimeState->channelScale == CRSF_RC_CHANNEL_SCALE_LEGACY) {
        /* conversion from RC value to PWM
        * for 0x16 RC frame
        *       RC     PWM
        * min  172 ->  988us
        * mid  992 -> 1500us
        * max 1811 -> 2012us
        * scale factor = (2012-988) / (1811-172) = 0.62477120195241
        * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
        */
        return (crsfRuntimeState->channelScale * (float)crsfRuntimeState->crsfChannelData[chan]) + 881;
    } else {
        /* conversion from RC value to PWM
        * for 0x17 Subset RC frame
        */
        return (crsfRuntimeState->channelScale * (float)crsfRuntimeState->crsfChannelData[chan]) + 988;
    }
}

void crsfRxWriteTelemetryData(const void *data, int len)
{
    len = MIN(len, (int)sizeof(telemetryBuf));
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}

void crsfRxSendTelemetryData(void)
{
    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        if (serialPort != NULL) {
            serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);
        }
        telemetryBufLen = 0; // reset telemetry buffer
    }
}

bool crsfRxIsTelemetryBufEmpty(void)
{
    return telemetryBufLen == 0;
}

bool crsfRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, int id)
{
    crsfRuntimeState_t *crsfRuntimeState;
    serialPort_t *sPort;

    rxRuntimeState->channelCount = CRSF_MAX_CHANNEL;
    rxRuntimeState->rcReadRawFn = crsfReadRawRC;
    rxRuntimeState->rcFrameStatusFn = crsfFrameStatus;
    rxRuntimeState->rcFrameTimeUsFn = rxFrameTimeUs;
    rxRuntimeState->priv = &crsfRuntimeStates[id];

    crsfRuntimeState = (crsfRuntimeState_t *)rxRuntimeState->priv;

    for (int ii = 0; ii < CRSF_MAX_CHANNEL; ++ii) {
        crsfRuntimeState->crsfChannelData[ii] = (16 * rxConfig->midrc) / 10 - 1408;
    }
    crsfRuntimeState->channelScale = CRSF_RC_CHANNEL_SCALE_LEGACY;

    const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    if (!portConfig) {
        return false;
    }
    crsfRuntimeState->portID = portConfig->identifier;

    uint32_t crsfBaudrate = CRSF_BAUDRATE;

#if defined(USE_CRSF_V3)
    crsfBaudrate = rxConfig->crsf_use_negotiated_baud ? getCrsfCachedBaudrate() : CRSF_BAUDRATE;
#endif

    sPort = openSerialPort(portConfig->identifier,
        FUNCTION_RX_SERIAL,
        crsfDataReceive,
        rxRuntimeState,
        crsfBaudrate,
        CRSF_PORT_MODE,
        CRSF_PORT_OPTIONS | (rxConfig->serialrx_inverted ? SERIAL_INVERTED : 0)
        );

    if (!serialPort)
        serialPort = sPort;

    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_RX_PROTOCOL_CRSF;
    }
#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource == LQ_SOURCE_NONE) {
        linkQualitySource = LQ_SOURCE_RX_PROTOCOL_CRSF;
    }
#endif

    return sPort != NULL;
}

#if defined(USE_CRSF_V3)
void crsfRxUpdateBaudrate(uint32_t baudrate)
{
    serialSetBaudRate(serialPort, baudrate);
    persistentObjectWrite(PERSISTENT_OBJECT_SERIALRX_BAUD, baudrate);
}

bool crsfRxUseNegotiatedBaud(void)
{
    return rxConfig()->crsf_use_negotiated_baud;
}
#endif

bool crsfRxIsActive(void)
{
    return serialPort != NULL;
}

void crsfRxBind(void)
{
    if (serialPort != NULL) {
        uint8_t bindFrame[] = {
            CRSF_SYNC_BYTE,
            0x07,  // frame length
            CRSF_FRAMETYPE_COMMAND,
            CRSF_ADDRESS_CRSF_RECEIVER,
            CRSF_ADDRESS_FLIGHT_CONTROLLER,
            CRSF_COMMAND_SUBCMD_RX,
            CRSF_COMMAND_SUBCMD_RX_BIND,
            0x9E,  // Command CRC8
            0xE8,  // Packet CRC8
        };
        serialWriteBuf(serialPort, bindFrame, 9);
    }
}
#endif
