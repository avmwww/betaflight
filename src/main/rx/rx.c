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

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"
#include "common/filter.h"

#include "config/config.h"
#include "config/config_reset.h"
#include "config/feature.h"

#include "drivers/adc.h"
#include "drivers/rx/rx_pwm.h"
#include "drivers/time.h"

#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "fc/tasks.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"
#include "pg/rx.h"

#include "rx/rx.h"
#include "rx/pwm.h"
#include "rx/fport.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/srxl2.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/msp.h"
#include "rx/xbus.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/crsf.h"
#include "rx/ghst.h"
#include "rx/rx_spi.h"
#include "rx/targetcustomserial.h"
#include "rx/msp_override.h"


const char rcChannelLetters[] = "AERT12345678abcdefgh";

static timeUs_t lastRssiSmoothingUs = 0; // may use on all rx sources
#ifdef USE_RX_RSNR
static int16_t rsnr = CRSF_SNR_MIN;        // range: [-30,20]
static int16_t rsnrRaw = CRSF_SNR_MIN;     // range: [-30,20]
#endif //USE_RX_RSNR
static timeUs_t lastMspRssiUpdateUs = 0;

#ifdef USE_RX_RSNR
static pt1Filter_t rsnrFilter;
#endif //USE_RX_RSNR

#ifdef USE_RX_LINK_UPLINK_POWER
static uint16_t uplinkTxPwrMw = 0;  //Uplink Tx power in mW
#endif

#define RSSI_ADC_DIVISOR (4096 / 1024)
#define RSSI_OFFSET_SCALING (1024 / 100.0f)

rssiSource_e rssiSource;
linkQualitySource_e linkQualitySource;


static uint8_t rxChannelCount;

static timeUs_t suspendRxSignalUntil = 0;
static uint8_t  skipRxSamples = 0;

static float rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // last received raw value, as it comes
uint32_t validRxSignalTimeout[MAX_SUPPORTED_RC_CHANNEL_COUNT];

#define MAX_INVALID_PULSE_TIME_MS 300                   // hold time in milliseconds after bad channel or Rx link loss
// will not be actioned until the nearest multiple of 100ms
#define PPM_AND_PWM_SAMPLE_COUNT 3

#define DELAY_20_MS (20 * 1000)                         // 20ms in us
#define DELAY_100_MS (100 * 1000)                       // 100ms in us
#define DELAY_1500_MS (1500 * 1000)                     // 1.5 seconds in us
#define SKIP_RC_SAMPLES_ON_RESUME  2                    // flush 2 samples to drop wrong measurements (timing independent)

static rxRuntimeState_t rxRuntimeStates[RX_SERIAL_COUNT];
static uint8_t rcSampleIndex = 0;

rxRuntimeState_t *getRxRuntimeState(int id)
{
    if (id > RX_SERIAL_COUNT - 1)
        return NULL;
    return &rxRuntimeStates[id];
}

float *getRcData(int id)
{
    return rxRuntimeStates[id].rcData;
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);
void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
        rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxFailsafeChannelConfig_t, MAX_SUPPORTED_RC_CHANNEL_COUNT, rxFailsafeChannelConfigs, PG_RX_FAILSAFE_CHANNEL_CONFIG, 0);
void pgResetFn_rxFailsafeChannelConfigs(rxFailsafeChannelConfig_t *rxFailsafeChannelConfigs)
{
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxFailsafeChannelConfigs[i].mode = (i < NON_AUX_CHANNEL_COUNT) ? RX_FAILSAFE_MODE_AUTO : RX_FAILSAFE_MODE_HOLD;
        rxFailsafeChannelConfigs[i].step = (i == THROTTLE)
            ? CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MIN_USEC)
            : CHANNEL_VALUE_TO_RXFAIL_STEP(RX_MID_USEC);
    }
}

void resetAllRxChannelRangeConfigurations(rxChannelRangeConfig_t *rxChannelRangeConfig)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfig->min = PWM_RANGE_MIN;
        rxChannelRangeConfig->max = PWM_RANGE_MAX;
        rxChannelRangeConfig++;
    }
}

static float nullReadRawRC(const rxRuntimeState_t *rxRuntimeState, uint8_t channel)
{
    UNUSED(rxRuntimeState);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return RX_FRAME_PENDING;
}

static bool nullProcessFrame(const rxRuntimeState_t *rxRuntimeState)
{
    UNUSED(rxRuntimeState);

    return true;
}

STATIC_UNIT_TESTED bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

#ifdef USE_SERIALRX
static bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeState_t *rxRuntimeState, int id)
{
    bool enabled = false;
    switch (rxRuntimeState->serialrxProvider) {
#ifdef USE_SERIALRX_SRXL2
    case SERIALRX_SRXL2:
        enabled = srxl2RxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SRXL:
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        enabled = spektrumInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        enabled = sbusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        enabled = sumdInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        enabled = sumhInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        enabled = xBusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        enabled = ibusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        enabled = jetiExBusInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_CRSF
    case SERIALRX_CRSF:
        enabled = crsfRxInit(rxConfig, rxRuntimeState, id);
        break;
#endif
#ifdef USE_SERIALRX_GHST
    case SERIALRX_GHST:
        enabled = ghstRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_TARGET_CUSTOM
    case SERIALRX_TARGET_CUSTOM:
        enabled = targetCustomSerialRxInit(rxConfig, rxRuntimeState);
        break;
#endif
#ifdef USE_SERIALRX_FPORT
    case SERIALRX_FPORT:
        enabled = fportRxInit(rxConfig, rxRuntimeState);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

static void rxInitID(int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);

    if (featureIsEnabled(FEATURE_RX_PARALLEL_PWM)) {
        rxRuntimeState->rxProvider = RX_PROVIDER_PARALLEL_PWM;
    } else if (featureIsEnabled(FEATURE_RX_PPM)) {
        rxRuntimeState->rxProvider = RX_PROVIDER_PPM;
    } else if (featureIsEnabled(FEATURE_RX_SERIAL)) {
        rxRuntimeState->rxProvider = RX_PROVIDER_SERIAL;
    } else if (featureIsEnabled(FEATURE_RX_MSP)) {
        rxRuntimeState->rxProvider = RX_PROVIDER_MSP;
    } else if (featureIsEnabled(FEATURE_RX_SPI)) {
        rxRuntimeState->rxProvider = RX_PROVIDER_SPI;
    } else {
        rxRuntimeState->rxProvider = RX_PROVIDER_NONE;
    }
    rxRuntimeState->serialrxProvider = rxConfig()->serialrx_provider;
    rxRuntimeState->rcReadRawFn = nullReadRawRC;
    rxRuntimeState->rcFrameStatusFn = nullFrameStatus;
    rxRuntimeState->rcProcessFrameFn = nullProcessFrame;
    rxRuntimeState->lastRcFrameTimeUs = 0;
    rxRuntimeState->rssiDbm = CRSF_RSSI_MIN;
    rxRuntimeState->rssiDbmRaw = CRSF_RSSI_MIN;
    rcSampleIndex = 0;

    uint32_t now = millis();
    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rxRuntimeState->rcData[i] = rxConfig()->midrc;
        validRxSignalTimeout[i] = now + MAX_INVALID_PULSE_TIME_MS;
    }

    rxRuntimeState->rcData[THROTTLE] = (featureIsEnabled(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

    // Initialize ARM switch to OFF position when arming via switch is defined
    // TODO - move to rc_mode.c
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        const modeActivationCondition_t *modeActivationCondition = modeActivationConditions(i);
        if (modeActivationCondition->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationCondition->range)) {
            // ARM switch is defined, determine an OFF value
            float value;
            if (modeActivationCondition->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationCondition->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rxRuntimeState->rcData[modeActivationCondition->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

    switch (rxRuntimeState->rxProvider) {
    default:

        break;
#ifdef USE_SERIALRX
    case RX_PROVIDER_SERIAL:
        {
            const bool enabled = serialRxInit(rxConfig(), rxRuntimeState, id);
            if (!enabled) {
                rxRuntimeState->rcReadRawFn = nullReadRawRC;
                rxRuntimeState->rcFrameStatusFn = nullFrameStatus;
            }
        }

        break;
#endif

#ifdef USE_RX_MSP
    case RX_PROVIDER_MSP:
        rxMspInit(rxConfig(), rxRuntimeState);

        break;
#endif

#ifdef USE_RX_SPI
    case RX_PROVIDER_SPI:
        {
            const bool enabled = rxSpiInit(rxSpiConfig(), rxRuntimeState);
            if (!enabled) {
                rxRuntimeState->rcReadRawFn = nullReadRawRC;
                rxRuntimeState->rcFrameStatusFn = nullFrameStatus;
            }
        }

        break;
#endif

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    case RX_PROVIDER_PPM:
    case RX_PROVIDER_PARALLEL_PWM:
        rxPwmInit(rxConfig(), rxRuntimeState);

        break;
#endif
    }

#if defined(USE_ADC)
    if (featureIsEnabled(FEATURE_RSSI_ADC)) {
        rssiSource = RSSI_SOURCE_ADC;
    } else
#endif
    if (rxConfig()->rssi_channel > 0) {
        rssiSource = RSSI_SOURCE_RX_CHANNEL;
    }

    // Setup source frame RSSI filtering to take averaged values every FRAME_ERR_RESAMPLE_US
    pt1FilterInit(&rxRuntimeState->frameErrFilter, pt1FilterGain(GET_FRAME_ERR_LPF_FREQUENCY(rxConfig()->rssi_src_frame_lpf_period), FRAME_ERR_RESAMPLE_US * 1e-6f));

    // Configurable amount of filtering to remove excessive jumpiness of the values on the osd
    float k = (256.0f - rxConfig()->rssi_smoothing) / 256.0f;

    pt1FilterInit(&rxRuntimeState->rssiFilter, k);

#ifdef USE_RX_RSSI_DBM
    pt1FilterInit(&rxRuntimeState->rssiDbmFilter, k);
#endif //USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
    pt1FilterInit(&rsnrFilter, k);
#endif //USE_RX_RSNR

    rxChannelCount = MIN(rxConfig()->max_aux_channel + NON_AUX_CHANNEL_COUNT, rxRuntimeState->channelCount);
}

void rxInit(void)
{
    int id;

    for (id = 0; id < RX_SERIAL_COUNT; id++)
        rxInitID(id);
}

bool rxIsReceivingSignal(void)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);

    if (!rxRuntimeState)
        return false;
    return rxRuntimeState->rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);

    if (!rxRuntimeState)
        return false;
    return rxRuntimeState->rxFlightChannelsValid;
}

void suspendRxSignal(void)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    if (rxRuntimeState->rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState->rxProvider == RX_PROVIDER_PPM) {
        suspendRxSignalUntil = micros() + DELAY_1500_MS;  // 1.5s
        skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
    }
#endif
    failsafeOnRxSuspend(DELAY_1500_MS);  // 1.5s
}

void resumeRxSignal(void)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    if (rxRuntimeState->rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState->rxProvider == RX_PROVIDER_PPM) {
        suspendRxSignalUntil = micros();
        skipRxSamples = SKIP_RC_SAMPLES_ON_RESUME;
    }
#endif
    failsafeOnRxResume();
}

#ifdef USE_RX_LINK_QUALITY_INFO

void rx_set_rfmode(uint8_t rfModeValue, int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

    rxRuntimeState->rfMode = rfModeValue;
}

STATIC_UNIT_TESTED uint16_t updateLinkQualitySamples(uint16_t value, int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return 0;

    rxRuntimeState->sum += value - rxRuntimeState->samples[rxRuntimeState->sampleIndex];
    rxRuntimeState->samples[rxRuntimeState->sampleIndex] = value;
    rxRuntimeState->sampleIndex = (rxRuntimeState->sampleIndex + 1) % LINK_QUALITY_SAMPLE_COUNT;
    return rxRuntimeState->sum / LINK_QUALITY_SAMPLE_COUNT;
}

void rxSetRfMode(uint8_t rfModeValue)
{
    rx_set_rfmode(rfModeValue, 0);
}
#endif

void set_link_quality_direct(uint16_t linkqualityValue, int id)
{
#ifdef USE_RX_LINK_QUALITY_INFO
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

    rxRuntimeState->linkQuality = linkqualityValue;
#else
    UNUSED(linkqualityValue);
    UNUSED(id);
#endif
}

static void set_link_quality(bool validFrame, timeDelta_t currentDeltaTimeUs, int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

#ifdef USE_RX_LINK_QUALITY_INFO
    if (linkQualitySource == LQ_SOURCE_NONE) {
        // calculate new sample mean
        rxRuntimeState->linkQuality = updateLinkQualitySamples(validFrame ? LINK_QUALITY_MAX_VALUE : 0, id);
    }
#endif

    if (rssiSource == RSSI_SOURCE_FRAME_ERRORS) {
        rxRuntimeState->resampleTimeUs += currentDeltaTimeUs;
        rxRuntimeState->rssiSum += validFrame ? RSSI_MAX_VALUE : 0;
        rxRuntimeState->rssiCount++;

        if (rxRuntimeState->resampleTimeUs >= FRAME_ERR_RESAMPLE_US) {
            set_rssi_val(rxRuntimeState->rssiSum / rxRuntimeState->rssiCount, rssiSource, id);
            rxRuntimeState->rssiSum = 0;
            rxRuntimeState->rssiCount = 0;
            rxRuntimeState->resampleTimeUs -= FRAME_ERR_RESAMPLE_US;
        }
    }
}

void setLinkQualityDirect(uint16_t linkqualityValue)
{
    set_link_quality_direct(linkqualityValue, 0);
}

#ifdef USE_RX_LINK_UPLINK_POWER
void rxSetUplinkTxPwrMw(uint16_t uplinkTxPwrMwValue)
{
    uplinkTxPwrMw = uplinkTxPwrMwValue;
}
#endif

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    UNUSED(currentTimeUs);
    UNUSED(currentDeltaTimeUs);

    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
    if (!rxRuntimeState)
        return false;


    return taskUpdateRxMainInProgress() || rxRuntimeState->rxDataProcessingRequired || rxRuntimeState->auxiliaryProcessingRequired;
}

static void rxFrameCheckInternal(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs, int id)
{
    bool signalReceived = false;
    bool useDataDrivenProcessing = true;
    timeDelta_t needRxSignalMaxDelayUs = DELAY_100_MS;

    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

    switch (rxRuntimeState->rxProvider) {
    default:

        break;
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
    case RX_PROVIDER_PPM:
        if (isPPMDataBeingReceived()) {
            signalReceived = true;
            resetPPMDataReceivedState();
        }

        break;
    case RX_PROVIDER_PARALLEL_PWM:
        if (isPWMDataBeingReceived()) {
            signalReceived = true;
            useDataDrivenProcessing = false;
        }

        break;
#endif
    case RX_PROVIDER_SERIAL:
    case RX_PROVIDER_MSP:
    case RX_PROVIDER_SPI:
    case RX_PROVIDER_UDP:
        {
            const uint8_t frameStatus = rxRuntimeState->rcFrameStatusFn(rxRuntimeState);
            DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 1, (frameStatus & RX_FRAME_FAILSAFE));
            signalReceived = (frameStatus & RX_FRAME_COMPLETE) && !(frameStatus & (RX_FRAME_FAILSAFE | RX_FRAME_DROPPED));
            set_link_quality(signalReceived, currentDeltaTimeUs, id);
            rxRuntimeState->auxiliaryProcessingRequired |= (frameStatus & RX_FRAME_PROCESSING_REQUIRED);
        }

        break;
    }

    if (signalReceived) {
        //  true only when a new packet arrives
        rxRuntimeState->needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        rxRuntimeState->rxSignalReceived = true; // immediately process packet data
        if (useDataDrivenProcessing) {
            rxRuntimeState->rxDataProcessingRequired = true;
            //  process the new Rx packet when it arrives
        }
    } else {
        //  watch for next packet
        if (cmpTimeUs(currentTimeUs, rxRuntimeState->needRxSignalBefore) > 0) {
            //  initial time to signalReceived failure is 100ms, then we check every 100ms
            rxRuntimeState->rxSignalReceived = false;
            rxRuntimeState->needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
            //  review and process rcData values every 100ms in case failsafe changed them
            rxRuntimeState->rxDataProcessingRequired = true;
        }
    }

#if defined(USE_RX_MSP_OVERRIDE)
    if (IS_RC_MODE_ACTIVE(BOXMSPOVERRIDE) && rxConfig()->msp_override_channels_mask && rxConfig()->msp_override_failsafe) {
        if (rxMspOverrideFrameStatus() & RX_FRAME_COMPLETE) {
            rxRuntimeState->rxSignalReceived = true;
            rxRuntimeState->rxDataProcessingRequired = true;
            rxRuntimeState->needRxSignalBefore = currentTimeUs + needRxSignalMaxDelayUs;
        }
    }
#endif
    
    rxRuntimeState = getRxRuntimeState(0);
    if (!rxRuntimeState)
        return;

    DEBUG_SET(DEBUG_FAILSAFE, 1, rxRuntimeState->rxSignalReceived);
    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 0, rxRuntimeState->rxSignalReceived);
}

FAST_CODE_NOINLINE void rxFrameCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTimeUs)
{
    for (int id = 0; id < RX_SERIAL_COUNT; id++)
        rxFrameCheckInternal(currentTimeUs, currentDeltaTimeUs, id);
}

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
static uint16_t calculateChannelMovingAverage(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT][PPM_AND_PWM_SAMPLE_COUNT];
    static int16_t rcDataMean[MAX_SUPPORTED_RX_PARALLEL_PWM_OR_PPM_CHANNEL_COUNT];
    static bool rxSamplesCollected = false;

    const uint8_t currentSampleIndex = rcSampleIndex % PPM_AND_PWM_SAMPLE_COUNT;

    // update the recent samples and compute the average of them
    rcSamples[chan][currentSampleIndex] = sample;

    // avoid returning an incorrect average which would otherwise occur before enough samples
    if (!rxSamplesCollected) {
        if (rcSampleIndex < PPM_AND_PWM_SAMPLE_COUNT) {
            return sample;
        }
        rxSamplesCollected = true;
    }

    rcDataMean[chan] = 0;
    for (int sampleIndex = 0; sampleIndex < PPM_AND_PWM_SAMPLE_COUNT; sampleIndex++) {
        rcDataMean[chan] += rcSamples[chan][sampleIndex];
    }
    return rcDataMean[chan] / PPM_AND_PWM_SAMPLE_COUNT;
}
#endif

static uint16_t getRxfailValue(uint8_t channel)
{
    const rxFailsafeChannelConfig_t *channelFailsafeConfig = rxFailsafeChannelConfigs(channel);
    const bool boxFailsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);

    switch (channelFailsafeConfig->mode) {
    case RX_FAILSAFE_MODE_AUTO:
        switch (channel) {
        case ROLL:
        case PITCH:
        case YAW:
            return rxConfig()->midrc;
        case THROTTLE:
            if (featureIsEnabled(FEATURE_3D) && !IS_RC_MODE_ACTIVE(BOX3D) && !flight3DConfig()->switched_mode3d) {
                return rxConfig()->midrc;
            } else {
                return rxConfig()->rx_min_usec;
            }
        }

    FALLTHROUGH;
    default:
    case RX_FAILSAFE_MODE_INVALID:
    case RX_FAILSAFE_MODE_HOLD:
        if (boxFailsafeSwitchIsOn) {
            return rcRaw[channel]; // current values are allowed through on held channels with switch induced failsafe
        } else {
            rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
            return rxRuntimeState->rcData[channel]; // last good value
        }
    case RX_FAILSAFE_MODE_SET:
        return RXFAIL_STEP_TO_CHANNEL_VALUE(channelFailsafeConfig->step);
    }
}

STATIC_UNIT_TESTED float applyRxChannelRangeConfiguraton(float sample, const rxChannelRangeConfig_t *range)
{
    // Avoid corruption of channel with a value of PPM_RCVR_TIMEOUT
    if (sample == PPM_RCVR_TIMEOUT) {
        return PPM_RCVR_TIMEOUT;
    }

    sample = scaleRangef(sample, range->min, range->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
    // out of range channel values are now constrained after the validity check in detectAndApplySignalLossBehaviour()
    return sample;
}

static void readRxChannelsApplyRanges(void)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
    for (int channel = 0; channel < rxChannelCount; channel++) {

        const uint8_t rawChannel = channel < RX_MAPPABLE_CHANNEL_COUNT ? rxConfig()->rcmap[channel] : channel;

        // sample the channel
        float sample;
#if defined(USE_RX_MSP_OVERRIDE)
        if (rxConfig()->msp_override_channels_mask) {
            sample = rxMspOverrideReadRawRc(rxRuntimeState, rxConfig(), rawChannel);
        } else
#endif
        {
            sample = rxRuntimeState->rcReadRawFn(rxRuntimeState, rawChannel);
        }

        // apply the rx calibration
        if (channel < NON_AUX_CHANNEL_COUNT) {
            sample = applyRxChannelRangeConfiguraton(sample, rxChannelRangeConfigs(channel));
        }

        rcRaw[channel] = sample;
    }
}

void detectAndApplySignalLossBehaviour(void)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
    const uint32_t currentTimeMs = millis();
    const bool boxFailsafeSwitchIsOn = IS_RC_MODE_ACTIVE(BOXFAILSAFE);
    rxRuntimeState->rxFlightChannelsValid = rxRuntimeState->rxSignalReceived && !boxFailsafeSwitchIsOn;
    // rxFlightChannelsValid is false after 100ms of no packets, or as soon as use the BOXFAILSAFE switch is actioned
    // rxFlightChannelsValid is true the instant we get a good packet or the BOXFAILSAFE switch is reverted
    // can also go false with good packets but where one flight channel is bad > 300ms (PPM type receiver error)

    for (int channel = 0; channel < rxChannelCount; channel++) {
        float sample = rcRaw[channel]; // sample has latest RC value, rcData has last 'accepted valid' value
        const bool thisChannelValid = rxRuntimeState->rxFlightChannelsValid && isPulseValid(sample);
        // if the whole packet is bad, or BOXFAILSAFE switch is actioned, consider all channels bad
        if (thisChannelValid) {
            //  reset the invalid pulse period timer for every good channel
            validRxSignalTimeout[channel] = currentTimeMs + MAX_INVALID_PULSE_TIME_MS;
        }

        if (failsafeIsActive()) {
            // we are in failsafe Stage 2, whether Rx loss or BOXFAILSAFE induced
            // pass valid incoming flight channel values to FC,
            // so that GPS Rescue can get the 30% requirement for termination of the rescue
            if (channel < NON_AUX_CHANNEL_COUNT) {
                if (!thisChannelValid) {
                    if (channel == THROTTLE ) {
                        sample = failsafeConfig()->failsafe_throttle;
                        // stage 2 failsafe throttle value. In GPS Rescue Flight mode, gpsRescueGetThrottle overrides, late in mixer.c
                    } else {
                        sample = rxConfig()->midrc;
                    }
                }
            } else {
                // set aux channels as per Stage 1 failsafe hold/set values, allow all for Failsafe and GPS rescue MODE switches
                sample = getRxfailValue(channel);
            }
        } else {
            // we are normal, or in failsafe stage 1
            if (boxFailsafeSwitchIsOn) {
                // BOXFAILSAFE active, but not in stage 2 yet, use stage 1 values
                sample = getRxfailValue(channel);
                //  set channels to Stage 1 values immediately failsafe switch is activated
            } else if (!thisChannelValid) {
                // everything is normal but this channel was invalid
                if (cmp32(currentTimeMs, validRxSignalTimeout[channel]) < 0) {
                    // first 300ms of Stage 1 failsafe
                    sample = rxRuntimeState->rcData[channel];
                    //  HOLD last valid value on bad channel/s for MAX_INVALID_PULSE_TIME_MS (300ms)
                } else {
                    // remaining Stage 1 failsafe period after 300ms
                    if (channel < NON_AUX_CHANNEL_COUNT) {
                        rxRuntimeState->rxFlightChannelsValid = false;
                        //  declare signal lost after 300ms of any one bad flight channel
                    }
                    sample = getRxfailValue(channel);
                    // set channels that are invalid for more than 300ms to Stage 1 values
                }
            }
            // everything is normal, ie rcData[channel] will be set to rcRaw(channel) via 'sample'
        }

        sample = constrainf(sample, PWM_PULSE_MIN, PWM_PULSE_MAX);

#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
        if (rxRuntimeState->rxProvider == RX_PROVIDER_PARALLEL_PWM || rxRuntimeState->rxProvider == RX_PROVIDER_PPM) {
            //  smooth output for PWM and PPM using moving average
            rxRuntimeState->rcData[channel] = calculateChannelMovingAverage(channel, sample);
        } else
#endif

        {
            //  set rcData to either validated incoming values, or failsafe-modified values
            rxRuntimeState->rcData[channel] = sample;
        }
    }

    if (rxRuntimeState->rxFlightChannelsValid) {
        failsafeOnValidDataReceived();
        //  --> start the timer to exit stage 2 failsafe 100ms after losing all packets or the BOXFAILSAFE switch is actioned
    } else {
        failsafeOnValidDataFailed();
        //  -> start stage 1 timer to enter stage2 failsafe the instant we get a good packet or the BOXFAILSAFE switch is reverted
    }

    DEBUG_SET(DEBUG_RX_SIGNAL_LOSS, 3, rxRuntimeState->rcData[THROTTLE]);
}

bool calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
    if (rxRuntimeState->auxiliaryProcessingRequired) {
        rxRuntimeState->rcProcessFrameFn(rxRuntimeState);
        rxRuntimeState->auxiliaryProcessingRequired = false;
    }

    if (!rxRuntimeState->rxDataProcessingRequired) {
        return false;
    }

    rxRuntimeState->rxDataProcessingRequired = false;

    // only proceed when no more samples to skip and suspend period is over
    if (skipRxSamples || currentTimeUs <= suspendRxSignalUntil) {
        if (currentTimeUs > suspendRxSignalUntil) {
            skipRxSamples--;
        }

        return true;
    }

    readRxChannelsApplyRanges();            // returns rcRaw
    detectAndApplySignalLossBehaviour();    // returns rcData

    rcSampleIndex++;

    return true;
}

void parseRcChannels(const char *input, rxConfig_t *rxConfig)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + RX_MAPPABLE_CHANNEL_COUNT)) {
            rxConfig->rcmap[s - rcChannelLetters] = c - input;
        }
    }
}

/* multi rssi */
void set_rssi_val_direct(uint16_t newRssi, rssiSource_e source, int id)
{
    if (source != rssiSource) {
        return;
    }

    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

    rxRuntimeState->rssi = newRssi;
    rxRuntimeState->rssiRaw = newRssi;
}

void set_rssi_val(uint16_t rssiValue, rssiSource_e source, int id)
{
    if (source != rssiSource) {
        return;
    }

    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

    // Filter RSSI value
    if (source == RSSI_SOURCE_FRAME_ERRORS) {
        rxRuntimeState->rssiRaw = pt1FilterApply(&rxRuntimeState->frameErrFilter, rssiValue);
    } else {
        rxRuntimeState->rssiRaw = rssiValue;
    }
}

uint16_t get_rssi_val(int id)
{
    uint16_t rssiValue = 0;

    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (rxRuntimeState)
        rssiValue = rxRuntimeState->rssi;

    // RSSI_Invert option
    if (rxConfig()->rssi_invert) {
        rssiValue = RSSI_MAX_VALUE - rssiValue;
    }

    return rxConfig()->rssi_scale / 100.0f * rssiValue + rxConfig()->rssi_offset * RSSI_OFFSET_SCALING;
}

uint8_t get_rssi_val_percent(int id)
{
    return scaleRange(get_rssi_val(id), 0, RSSI_MAX_VALUE, 0, 100);
}

void setRssiDirect(uint16_t newRssi, rssiSource_e source)
{
    set_rssi_val_direct(newRssi, source, 0);
}

void setRssi(uint16_t rssiValue, rssiSource_e source)
{
    set_rssi_val(rssiValue, source, 0);
}

void setRssiMsp(uint8_t newMspRssi)
{
    if (rssiSource == RSSI_SOURCE_NONE) {
        rssiSource = RSSI_SOURCE_MSP;
    }

    if (rssiSource == RSSI_SOURCE_MSP) {
        rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
        if (!rxRuntimeState)
            return;

        rxRuntimeState->rssi = ((uint16_t)newMspRssi) << 2;
        lastMspRssiUpdateUs = micros();
    }
}

static void updateRSSIPWM(void)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
    // Read value of AUX channel as rssi
    int16_t pwmRssi = rxRuntimeState->rcData[rxConfig()->rssi_channel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    setRssiDirect(scaleRange(constrain(pwmRssi, PWM_RANGE_MIN, PWM_RANGE_MAX), PWM_RANGE_MIN, PWM_RANGE_MAX, 0, RSSI_MAX_VALUE), RSSI_SOURCE_RX_CHANNEL);
}

static void updateRSSIADC(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
#else
    static uint32_t rssiUpdateAt = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAt) < 0) {
        return;
    }
    rssiUpdateAt = currentTimeUs + DELAY_20_MS;

    const uint16_t adcRssiSample = adcGetChannel(ADC_RSSI);
    uint16_t rssiValue = adcRssiSample / RSSI_ADC_DIVISOR;

    setRssi(rssiValue, RSSI_SOURCE_ADC);
#endif
}

static void update_rssi_val(float k2)
{
    rxRuntimeState_t *rxRuntimeState;
    int i = 0;

    while ((rxRuntimeState = getRxRuntimeState(i)) != NULL) {
        if (rxRuntimeState->rssi != rxRuntimeState->rssiRaw) {
            pt1FilterUpdateCutoff(&rxRuntimeState->rssiFilter, k2);
            rxRuntimeState->rssi = pt1FilterApply(&rxRuntimeState->rssiFilter, rxRuntimeState->rssiRaw);
        }
        i++;
    }
}

#ifdef USE_RX_RSSI_DBM
static void update_rssi_dbm_val(float k2)
{
    rxRuntimeState_t *rxRuntimeState;
    int i = 0;

    while ((rxRuntimeState = getRxRuntimeState(i)) != NULL) {
        if (rxRuntimeState->rssiDbm != rxRuntimeState->rssiDbmRaw) {
            pt1FilterUpdateCutoff(&rxRuntimeState->rssiDbmFilter, k2);
            rxRuntimeState->rssiDbm = pt1FilterApply(&rxRuntimeState->rssiDbmFilter, rxRuntimeState->rssiDbmRaw);
        }
        i++;
    }
}
#endif

void updateRSSI(timeUs_t currentTimeUs)
{
    switch (rssiSource) {
    case RSSI_SOURCE_RX_CHANNEL:
        updateRSSIPWM();
        break;
    case RSSI_SOURCE_ADC:
        updateRSSIADC(currentTimeUs);
        break;
    case RSSI_SOURCE_MSP:
        if (cmpTimeUs(micros(), lastMspRssiUpdateUs) > DELAY_1500_MS) {  // 1.5s
            rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
            if (rxRuntimeState)
                rxRuntimeState->rssi = 0;
        }
        break;
    default:
        break;
    }

    if (cmpTimeUs(currentTimeUs, lastRssiSmoothingUs) > 250000) { // 0.25s
        lastRssiSmoothingUs = currentTimeUs;
    } else {
        if (lastRssiSmoothingUs != currentTimeUs) { // avoid div by 0
            float k = (256.0f - rxConfig()->rssi_smoothing) / 256.0f;
            float factor = ((currentTimeUs - lastRssiSmoothingUs) / 1000000.0f) / (1.0f / 4.0f);
            float k2  = (k * factor) / ((k * factor) - k + 1);

            update_rssi_val(k2);

#ifdef USE_RX_RSSI_DBM
            update_rssi_dbm_val(k2);
#endif //USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
            if (rsnr != rsnrRaw) {
                pt1FilterUpdateCutoff(&rsnrFilter, k2);
                rsnr = pt1FilterApply(&rsnrFilter, rsnrRaw);
            }
#endif //USE_RX_RSNR

            lastRssiSmoothingUs = currentTimeUs;
        }
    }
}

uint16_t getRssi(void)
{
    return get_rssi_val(0);
}

uint8_t getRssiPercent(void)
{
    return get_rssi_val_percent(0);
}

#ifdef USE_RX_RSSI_DBM
uint16_t get_rssi_dbm_val(int id)
{
    int16_t rssiValue = 0;

    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (rxRuntimeState)
        rssiValue = rxRuntimeState->rssiDbm;
    return rssiValue;
}

void set_rssi_dbm_val(int16_t rssiDbmValue, rssiSource_e source, int id)
{
    if (source != rssiSource) {
        return;
    }

    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

    rxRuntimeState->rssiDbmRaw = rssiDbmValue;
}

void set_rssi_dbm_val_direct(int16_t newRssiDbm, rssiSource_e source, int id)
{
    if (source != rssiSource) {
        return;
    }

    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

    rxRuntimeState->rssiDbm = newRssiDbm;
    rxRuntimeState->rssiDbmRaw = newRssiDbm;
}

int8_t get_active_antenna(int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return 0;

    return rxRuntimeState->activeAntenna;
}

void set_active_antenna(int8_t antenna, int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return;

    rxRuntimeState->activeAntenna = antenna;
}

int16_t getRssiDbm(void)
{
    return get_rssi_dbm_val(0);
}

void setRssiDbm(int16_t rssiDbmValue, rssiSource_e source)
{
    set_rssi_dbm_val(rssiDbmValue, source, 0);
}

void setRssiDbmDirect(int16_t newRssiDbm, rssiSource_e source)
{
    set_rssi_dbm_val_direct(newRssiDbm, source, 0);
}

int8_t getActiveAntenna(void)
{
    return get_active_antenna(0);
}

void setActiveAntenna(int8_t antenna)
{
    set_active_antenna(antenna, 0);
}

#endif //USE_RX_RSSI_DBM

#ifdef USE_RX_RSNR
int16_t getRsnr(void)
{
    return rsnr;
}

void setRsnr(int16_t rsnrValue)
{
    rsnrRaw = rsnrValue;
}

void setRsnrDirect(int16_t newRsnr)
{
    rsnr = newRsnr;
    rsnrRaw = newRsnr;
}
#endif //USE_RX_RSNR

#ifdef USE_RX_LINK_QUALITY_INFO
uint16_t rx_get_link_quality(int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return 0;

    return rxRuntimeState->linkQuality;
}

uint8_t rx_get_rfmode(int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return 0;

    return rxRuntimeState->rfMode;
}

uint16_t rx_get_link_quality_percent(int id)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(id);
    if (!rxRuntimeState)
        return 0;

    return (linkQualitySource == LQ_SOURCE_NONE) ? scaleRange(rxRuntimeState->linkQuality,
                                                              0, LINK_QUALITY_MAX_VALUE,
                                                              0, 100) : rxRuntimeState->linkQuality;
}

uint16_t rxGetLinkQuality(void)
{
    return rx_get_link_quality(0);
}

uint8_t rxGetRfMode(void)
{
    return rx_get_rfmode(0);
}

uint16_t rxGetLinkQualityPercent(void)
{
    return rx_get_link_quality_percent(0);
}
#endif

#ifdef USE_RX_LINK_UPLINK_POWER
uint16_t rxGetUplinkTxPwrMw(void)
{
    return uplinkTxPwrMw;
}
#endif

bool isRssiConfigured(void)
{
    return rssiSource != RSSI_SOURCE_NONE;
}

timeDelta_t rxGetFrameDelta(timeDelta_t *frameAgeUs)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
    static timeUs_t previousFrameTimeUs = 0;
    static timeDelta_t frameTimeDeltaUs = 0;

    if (rxRuntimeState->rcFrameTimeUsFn) {
        const timeUs_t frameTimeUs = rxRuntimeState->rcFrameTimeUsFn();

        *frameAgeUs = cmpTimeUs(micros(), frameTimeUs);

        const timeDelta_t deltaUs = cmpTimeUs(frameTimeUs, previousFrameTimeUs);
        if (deltaUs) {
            frameTimeDeltaUs = deltaUs;
            previousFrameTimeUs = frameTimeUs;
        }
    }

    return frameTimeDeltaUs;
}

timeUs_t rxFrameTimeUs(void)
{
    rxRuntimeState_t *rxRuntimeState = getRxRuntimeState(0);
    return rxRuntimeState->lastRcFrameTimeUs;
}


