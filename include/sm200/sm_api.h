// Copyright (c).2017-2020, Signal Hound, Inc.
// For licensing information, please see the API license in the software_licenses folder

#ifndef SM_API_H
#define SM_API_H

#if defined(_WIN32) // Windows
    #ifdef SM_EXPORTS
        #define SM_API __declspec(dllexport)
    #else 
        #define SM_API 
    #endif

    // bare minimum stdint typedef support
    #if _MSC_VER < 1700 // For VS2010 or earlier
        typedef signed char        int8_t;
        typedef short              int16_t;
        typedef int                int32_t;
        typedef long long          int64_t;
        typedef unsigned char      uint8_t;
        typedef unsigned short     uint16_t;
        typedef unsigned int       uint32_t;
        typedef unsigned long long uint64_t;
    #else
        #include <stdint.h>
    #endif

    #define SM_DEPRECATED(comment) __declspec(deprecated(comment))
#else // Linux 
    #include <stdint.h>
    #define SM_API __attribute__((visibility("default")))

    #if defined(__GNUC__)
        #define SM_DEPRECATED(comment) __attribute__((deprecated))
    #else
        #define SM_DEPRECATED(comment) comment
    #endif
#endif

#define SM_INVALID_HANDLE (-1)

#define SM_TRUE (1)
#define SM_FALSE (0)

#define SM_MAX_DEVICES (9)

// For networked (10GbE devices)
#define SM200_ADDR_ANY ("0.0.0.0")
#define SM200_DEFAULT_ADDR ("192.168.2.10")
#define SM200_DEFAULT_PORT (51665)

#define SM200A_AUTO_ATTEN (-1)
// Valid atten values [0,6] or -1 for auto
#define SM200A_MAX_ATTEN (6) 

// Maximum number of sweeps that can be queued up
// Sweep indices [0,15]
#define SM200A_MAX_SWEEP_QUEUE_SZ (16)

// Device is only calibrated to 100 kHz
#define SM200A_MIN_FREQ (100.0e3)
// Device is only calibrated to 20 GHz
#define SM200A_MAX_FREQ (20.6e9)
#define SM200A_MAX_REF_LEVEL (20.0)
#define SM200A_MAX_IQ_DECIMATION (4096)

// The frequency at which the manually controlled preselector filters end. 
// Past this frequency, the preselector filters are always enabled.
#define SM200A_PRESELECTOR_MAX_FREQ (645.0e6)

// Minimum RBW for fast sweep with Nuttall window
#define SM200A_FAST_SWEEP_MIN_RBW (30.0e3)

// Min/max span for device configured in RTSA measurement mode
#define SM200A_RTSA_MIN_SPAN (200.0e3)
#define SM200A_RTSA_MAX_SPAN (160.0e6)

// Sweep time range [1us, 100s]
#define SM200A_MIN_SWEEP_TIME (1.0e-6)
#define SM200A_MAX_SWEEP_TIME (100.0)

// Max number of bytes per SPI transfer
#define SM200A_SPI_MAX_BYTES (4)

// For GPIO sweeps
#define SM200A_GPIO_SWEEP_MAX_STEPS (64)

// For IQ GPIO switching
#define SM200A_GPIO_SWITCH_MAX_STEPS (64)
#define SM200A_GPIO_SWITCH_MIN_COUNT (2)
#define SM200A_GPIO_SWITCH_MAX_COUNT (4194303 - 1) // 2^22 - 1

// FPGA internal temperature (Celsius)
// Returned from smGetDeviceDiagnostics()
#define SM200A_TEMP_WARNING (95.0)
#define SM200A_TEMP_MAX (102.0)

// SM200B segmented I/Q captures
#define SM200B_MAX_SEGMENTED_IQ_SEGMENTS (250)
#define SM200B_MAX_SEGMENTED_IQ_SAMPLES (520e6)

typedef enum SmStatus {
    smCalErr = -1003, // Internal use
    smMeasErr = -1002, // Internal use
    smErrorIOErr = -1001, // Internal use

    // Calibration file unable to be used with the API
    smInvalidCalibrationFileErr = -200,
    // Invalid center frequency specified
    smInvalidCenterFreqErr = -101,
    // IQ decimation value provided not a valid value
    smInvalidIQDecimationErr = -100,

    // Socket/network error
    smNetworkErr = -53,
    // If the core FX3 program fails to run
    smFx3RunErr = -52,
    // Only can connect up to SM_MAX_DEVICES receivers
    smMaxDevicesConnectedErr = -51,
    // FPGA boot error
    smFPGABootErr = -50,
    // Boot error
    smBootErr = -49,

    // Requesting GPS information when the GPS is not locked
    smGpsNotLockedErr = -16,
    // Invalid API version for target device, TBD
    smVersionMismatchErr = -14,
    // Unable to allocate resources needed to configure the measurement mode
    smAllocationErr = -13,

    // Returned when the device detects framing issue on measurement data
    // Measurement results are likely invalid. Device should be preset/power cycled
    smSyncErr = -11,
    // Invalid or already active sweep position
    smInvalidSweepPosition = -10,
    // Attempting to perform an operation that cannot currently be performed.
    // Often the result of trying to do something while the device is currently
    //   making measurements or not in an idle state.
    smInvalidConfigurationErr = -8,
    // Device disconnected, likely USB error detected
    smConnectionLostErr = -6,
    // Required parameter found to have invalid value
    smInvalidParameterErr = -5, 
    // One or more required pointer parameters were null
    smNullPtrErr = -4, 
    // User specified invalid device index
    smInvalidDeviceErr = -3, 
    // Unable to open device
    smDeviceNotFoundErr = -2, 

    // Function returned successfully
    smNoError = 0,

    // One or more of the provided settings were adjusted
    smSettingClamped = 1,
    // Measurement includes data which caused an ADC overload (clipping/compression)
    smAdcOverflow = 2,
    // Measurement is uncalibrated, overrides ADC overflow
    smUncalData = 3,
    // Temperature drift occured, measurements uncalibrated, reconfigure the device
    smTempDriftWarning = 4,
    // Warning when the preselector span is smaller than the user selected span
    smSpanExceedsPreselector = 5,
    // Warning when the internal temperature gets too hot. The device is close to shutting down
    smTempHighWarning = 6,
    // Returned when the API was unable to keep up with the necessary processing
    smCpuLimited = 7,
    // Returned when the API detects a device with newer features than what was available
    //   when this version of the API was released. Suggested fix, update the API.
    smUpdateAPI = 8,
} SmStatus;

typedef enum SmDataType {
    smDataType32fc,
    smDataType16sc
} SmDataType;

typedef enum SmMode {
    smModeIdle = 0,
    smModeSweeping = 1,
    smModeRealTime = 2,
    smModeIQ = 3, // Use smModeIQStreaming
    smModeIQStreaming = 3,
    smModeIQSegmentedCapture = 5,
    smModeAudio = 4,
} SmMode;

typedef enum SmSweepSpeed {
    smSweepSpeedAuto = 0,
    smSweepSpeedNormal = 1,
    smSweepSpeedFast = 2
} SmSweepSpeed;

typedef enum SmIQStreamSampleRate {
    smIQStreamSampleRateNative = 0,
    smIQStreamSampleRateLTE = 1,
} SmIQStreamSampleRate;

typedef enum SmPowerState {
    smPowerStateOn = 0,
    smPowerStateStandby = 1
} SmPowerState;

typedef enum SmDetector {
    smDetectorAverage = 0,
    smDetectorMinMax = 1
} SmDetector;

typedef enum SmScale {
    smScaleLog = 0, // Sweep in dBm
    smScaleLin = 1, // Sweep in mV
    smScaleFullScale = 2 // N/A
} SmScale;

typedef enum SmVideoUnits {
    smVideoLog = 0,
    smVideoVoltage = 1,
    smVideoPower = 2,
    smVideoSample = 3
} SmVideoUnits;

typedef enum SmWindowType {
    smWindowFlatTop = 0,
    // 1 (N/A)
    smWindowNutall = 2,
    smWindowBlackman = 3,
    smWindowHamming = 4,
    smWindowGaussian6dB = 5,
    smWindowRect = 6
} SmWindowType;

typedef enum SmTriggerType {
    smTriggerTypeImm = 0,
    smTriggerTypeVideo = 1, 
    smTriggerTypeExt = 2,
    smTriggerTypeFMT = 3
} SmTriggerType;

typedef enum SmTriggerEdge {
    smTriggerEdgeRising = 0,
    smTriggerEdgeFalling = 1
} SmTriggerEdge;

typedef enum SmBool {
    smFalse = 0,
    smTrue = 1
} SmBool;

typedef enum SmGPIOState {
    smGPIOStateOutput = 0,
    smGPIOStateInput = 1
} SmGPIOState;

typedef enum SmReference {
    smReferenceUseInternal = 0,
    smReferenceUseExternal = 1
} SmReference;

typedef enum SmDeviceType {
    smDeviceTypeSM200A = 0,
    smDeviceTypeSM200B = 1,
    smDeviceTypeSM200C = 2
} SmDeviceType;

typedef enum SmAudioType {
    smAudioTypeAM = 0,
    smAudioTypeFM = 1,
    smAudioTypeUSB = 2,
    smAudioTypeLSB = 3,
    smAudioTypeCW = 4
} SmAudioType;

typedef enum SmGPSState {
    smGPSStateNotPresent = 0,
    smGPSStateLocked = 1,
    smGPSStateDisciplined = 2
} SmGPSState;

typedef struct SmGPIOStep {
    double freq;
    uint8_t mask; // gpio bits
} SmGPIOStep;

// For troubleshooting purposes.
// (For standard diagnostics, use smGetDeviceDiagnostics)
typedef struct SmDeviceDiagnostics {
    float voltage;
    float currentInput;
    float currentOCXO;
    float current58;
    float tempFPGAInternal;
    float tempFPGANear;
    float tempOCXO;
    float tempVCO;
    float tempRFBoardLO;
    float tempPowerSupply;
} SmDeviceDiagnostics;

#ifdef __cplusplus
extern "C" {
#endif 

// This function is for USB devices only.
SM_API SmStatus smGetDeviceList(int *serials, int *deviceCount);

SM_API SmStatus smBroadcastNetworkConfig(const char *hostAddr, const char *deviceAddr, uint16_t port, SmBool nonVolatile);

// USB devices only (SM200A/B)
SM_API SmStatus smOpenDevice(int *device);
SM_API SmStatus smOpenDeviceBySerial(int *device, int serialNumber);
// Networked devices only (SM200C)
SM_API SmStatus smOpenNetworkedDevice(int *device, const char *hostAddr, const char *deviceAddr, uint16_t port);

SM_API SmStatus smCloseDevice(int device);
SM_API SmStatus smPreset(int device);
// Preset a device that has not been opened with the smOpenDevice functions
SM_API SmStatus smPresetSerial(int serialNumber);

SM_API SmStatus smNetworkedSpeedTest(int device, double durationSeconds, double *bytesPerSecond);

SM_API SmStatus smGetDeviceInfo(int device, SmDeviceType *deviceType, int *serialNumber);
SM_API SmStatus smGetFirmwareVersion(int device, int *major, int *minor, int *revision);

SM_API SmStatus smGetDeviceDiagnostics(int device, float *voltage, float *current, float *temperature);
SM_API SmStatus smGetFullDeviceDiagnostics(int device, SmDeviceDiagnostics *diagnostics);
// SM200C only
SM_API SmStatus smGetSFPDiagnostics(int device, float *temp, float *voltage, float *txPower, float *rxPower);

SM_API SmStatus smSetPowerState(int device, SmPowerState powerState);
SM_API SmStatus smGetPowerState(int device, SmPowerState *powerState);

// Overrides reference level when set to non-auto values
SM_API SmStatus smSetAttenuator(int device, int atten);
SM_API SmStatus smGetAttenuator(int device, int *atten);

// Uses this when attenuation is automatic
SM_API SmStatus smSetRefLevel(int device, double refLevel);
SM_API SmStatus smGetRefLevel(int device, double *refLevel);

// Set preselector state for all measurement modes
SM_API SmStatus smSetPreselector(int device, SmBool enabled);
SM_API SmStatus smGetPreselector(int device, SmBool *enabled);

// Configure IO routines
SM_API SmStatus smSetGPIOState(int device, SmGPIOState lowerState, SmGPIOState upperState);
SM_API SmStatus smGetGPIOState(int device, SmGPIOState *lowerState, SmGPIOState *upperState);
SM_API SmStatus smWriteGPIOImm(int device, uint8_t data);
SM_API SmStatus smReadGPIOImm(int device, uint8_t *data);
SM_API SmStatus smWriteSPI(int device, uint32_t data, int byteCount);
// For standard sweeps only
SM_API SmStatus smSetGPIOSweepDisabled(int device);
SM_API SmStatus smSetGPIOSweep(int device, SmGPIOStep *steps, int stepCount);
// For IQ streaming only
SM_API SmStatus smSetGPIOSwitchingDisabled(int device);
SM_API SmStatus smSetGPIOSwitching(int device, uint8_t *gpio, uint32_t *counts, int gpioSteps); 

// Enable the external reference out port
SM_API SmStatus smSetExternalReference(int device, SmBool enabled);
SM_API SmStatus smGetExternalReference(int device, SmBool *enabled);
// Specify whether to use the internal reference or reference on the ref in port
SM_API SmStatus smSetReference(int device, SmReference reference);
SM_API SmStatus smGetReference(int device, SmReference *reference);

// Enable whether or not the API auto updates the timebase calibration
// value when a valid GPS lock is acquired.
SM_API SmStatus smSetGPSTimebaseUpdate(int device, SmBool enabled);
SM_API SmStatus smGetGPSTimebaseUpdate(int device, SmBool *enabled);
SM_API SmStatus smGetGPSHoldoverInfo(int device, SmBool *usingGPSHoldover, uint64_t *lastHoldoverTime);

// Returns whether the GPS is locked, can be called anytime
SM_API SmStatus smGetGPSState(int device, SmGPSState *GPSState);

SM_API SmStatus smSetSweepSpeed(int device, SmSweepSpeed sweepSpeed);
SM_API SmStatus smSetSweepCenterSpan(int device, double centerFreqHz, double spanHz);
SM_API SmStatus smSetSweepStartStop(int device, double startFreqHz, double stopFreqHz);
SM_API SmStatus smSetSweepCoupling(int device, double rbw, double vbw, double sweepTime);
SM_API SmStatus smSetSweepDetector(int device, SmDetector detector, SmVideoUnits videoUnits);
SM_API SmStatus smSetSweepScale(int device, SmScale scale);
SM_API SmStatus smSetSweepWindow(int device, SmWindowType window);
SM_API SmStatus smSetSweepSpurReject(int device, SmBool spurRejectEnabled);

SM_API SmStatus smSetRealTimeCenterSpan(int device, double centerFreqHz, double spanHz);
SM_API SmStatus smSetRealTimeRBW(int device, double rbw);
SM_API SmStatus smSetRealTimeDetector(int device, SmDetector detector);
SM_API SmStatus smSetRealTimeScale(int device, SmScale scale, double frameRef, double frameScale);
SM_API SmStatus smSetRealTimeWindow(int device, SmWindowType window);

SM_API SmStatus smSetIQBaseSampleRate(int device, SmIQStreamSampleRate sampleRate);
SM_API SmStatus smSetIQDataType(int device, SmDataType dataType);
SM_API SmStatus smSetIQCenterFreq(int device, double centerFreqHz);
SM_API SmStatus smGetIQCenterFreq(int device, double *centerFreqHz);
SM_API SmStatus smSetIQSampleRate(int device, int decimation);
SM_API SmStatus smSetIQBandwidth(int device, SmBool enableSoftwareFilter, double bandwidth);
SM_API SmStatus smSetIQExtTriggerEdge(int device, SmTriggerEdge edge);
// Generally this function does not need to be called. 
// Please read the API manual before using this function.
SM_API SmStatus smSetIQQueueSize(int device, float ms);

// Begin Segmented I/Q configuration, SM200B only
SM_API SmStatus smSetSegIQDataType(int device, SmDataType dataType);
SM_API SmStatus smSetSegIQCenterFreq(int device, double centerFreqHz);
SM_API SmStatus smSetSegIQVideoTrigger(int device, double triggerLevel, SmTriggerEdge triggerEdge);
SM_API SmStatus smSetSegIQExtTrigger(int device, SmTriggerEdge extTriggerEdge);
SM_API SmStatus smSetSegIQFMTParams(int device, int fftSize, const double *frequencies, const double *ampls, int count);
SM_API SmStatus smSetSegIQSegmentCount(int device, int segmentCount);
SM_API SmStatus smSetSegIQSegment(int device, int segment, SmTriggerType triggerType,
                                  int preTrigger, int captureSize, double timeoutSeconds);
// End Segmented I/Q configuration

SM_API SmStatus smSetAudioCenterFreq(int device, double centerFreqHz);
SM_API SmStatus smSetAudioType(int device, SmAudioType audioType);
SM_API SmStatus smSetAudioFilters(int device, double ifBandwidth, double audioLpf, double audioHpf);
SM_API SmStatus smSetAudioFMDeemphasis(int device, double deemphasis);

SM_API SmStatus smConfigure(int device, SmMode mode);
SM_API SmStatus smGetCurrentMode(int device, SmMode *mode);
SM_API SmStatus smAbort(int device);

SM_API SmStatus smGetSweepParameters(int device, double *actualRBW, double *actualVBW,
                                     double *actualStartFreq, double *binSize, int *sweepSize);
SM_API SmStatus smGetRealTimeParameters(int device, double *actualRBW, int *sweepSize, double *actualStartFreq,
                                        double *binSize, int *frameWidth, int *frameHeight, double *poi);
// Retrieve I/Q parameters for streaming and segmented I/Q captures.
SM_API SmStatus smGetIQParameters(int device, double *sampleRate, double *bandwidth);
// Retrieve the correction factor/scalar for streaming and segmented I/Q captures.
SM_API SmStatus smGetIQCorrection(int device, float *scale);

SM_API SmStatus smSegIQGetMaxCaptures(int device, int *maxCaptures);

// Performs a single sweep, blocking function
SM_API SmStatus smGetSweep(int device, float *sweepMin, float *sweepMax, int64_t *nsSinceEpoch);

// Queued sweep mechanisms
SM_API SmStatus smStartSweep(int device, int pos);
SM_API SmStatus smFinishSweep(int device, int pos, float *sweepMin, float *sweepMax, int64_t *nsSinceEpoch);

SM_API SmStatus smGetRealTimeFrame(int device, float *colorFrame, float *alphaFrame, float *sweepMin,
                                   float *sweepMax, int *frameCount, int64_t *nsSinceEpoch);

SM_API SmStatus smGetIQ(int device, void *iqBuf, int iqBufSize, double *triggers, int triggerBufSize, 
                        int64_t *nsSinceEpoch, SmBool purge, int *sampleLoss, int *samplesRemaining);

// Begin Segmented I/Q acquisition functions, SM200B only
SM_API SmStatus smSegIQCaptureStart(int device, int capture);
SM_API SmStatus smSegIQCaptureWait(int device, int capture);
SM_API SmStatus smSegIQCaptureWaitAsync(int device, int capture, SmBool *completed);
SM_API SmStatus smSegIQCaptureTimeout(int device, int capture, int segment, SmBool *timedOut);
SM_API SmStatus smSegIQCaptureTime(int device, int capture, int segment, int64_t *nsSinceEpoch);
SM_API SmStatus smSegIQCaptureRead(int device, int capture, int segment, void *iq, int offset, int len);
SM_API SmStatus smSegIQCaptureFinish(int device, int capture);
// Convenience function for a single segment capture.
SM_API SmStatus smSegIQCaptureFull(int device, int capture, void *iq, int offset, int len, 
                                   int64_t *nsSinceEpoch, SmBool *timedOut);
// Convenience function to resample a 250 MS/s capture to the LTE rate of 245.76 MS/s
SM_API SmStatus smSegIQLTEResample(float *input, int inputLen, float *output, int *outputLen, bool clearDelayLine);
// End Segmented I/Q acquisition functions

SM_API SmStatus smGetAudio(int device, float *audio);

SM_API SmStatus smGetGPSInfo(int device, SmBool refresh, SmBool *updated, int64_t *secSinceEpoch,
                             double *latitude, double *longitude, double *altitude, char *nmea, int *nmeaLen);

// Accepts values between [10-90] as the temp threshold for when the fan turns on
SM_API SmStatus smSetFanThreshold(int device, int temp);
SM_API SmStatus smGetFanThreshold(int device, int *temp);

SM_API SmStatus smGetCalDate(int device, uint64_t *lastCalDate);

SM_API const char* smGetAPIVersion();
SM_API const char* smGetErrorString(SmStatus status);
SM_API const char* smGetProductID();

SM_DEPRECATED("smSetIQUSBQueueSize has been deprecated, use smSetIQQueueSize")
SM_API SmStatus smSetIQUSBQueueSize(int device, float ms);

#ifdef __cplusplus
} // Extern "C"
#endif

#endif // SM_API_H
