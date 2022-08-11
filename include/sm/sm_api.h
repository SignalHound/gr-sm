// Copyright (c).2022, Signal Hound, Inc.
// For licensing information, please see the API license in the software_licenses folder

/*!
 * \file sm_api.h
 * \brief API functions for the SM435/SM200 spectrum analyzers.
 *
 * This is the main file for user accessible functions for controlling the
 * SM435/SM200 spectrum analyzers.
 *
 */

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

/** Used for boolean true when integer parameters are being used. Also see #SmBool. */
#define SM_TRUE (1)
/** Used for boolean false when integer parameters are being used. Also see #SmBool. */
#define SM_FALSE (0)

/** Max number of devices that can be interfaced in the API. */
#define SM_MAX_DEVICES (9)

/** Convenience host address for connecting networked devices. */
#define SM_ADDR_ANY ("0.0.0.0")
/** Default device IP address for networked devices. */
#define SM_DEFAULT_ADDR ("192.168.2.10")
/** Default port number for networked devices. */
#define SM_DEFAULT_PORT (51665)

/** Tells the API to automatically choose attenuation based on reference level. */
#define SM_AUTO_ATTEN (-1)
/** Valid atten values [0,6] or -1 for auto */
#define SM_MAX_ATTEN (6)
/** Maximum reference level in dBm */
#define SM_MAX_REF_LEVEL (20.0)

/** Maximum number of sweeps that can be queued up. Valid sweep indices between [0,15] */
#define SM_MAX_SWEEP_QUEUE_SZ (16)

/** Min frequency for sweeps, and min center frequency for I/Q measurements for SM200 devices. */
#define SM200_MIN_FREQ (100.0e3)
/** Max frequency for sweeps, and max center frequency for I/Q measurements for SM200 devices. */
#define SM200_MAX_FREQ (20.6e9)
/** Min frequency for sweeps, and min center frequency for I/Q measurements for SM435 devices. */
#define SM435_MIN_FREQ (100.0e3)
/** Max frequency for sweeps, and max center frequency for I/Q measurements for SM435 devices. */
#define SM435_MAX_FREQ (44.2e9)
/** Max frequency for sweeps, and max center frequency for I/Q measurements for SM435 devices with the IF output option. */
#define SM435_MAX_FREQ_IF_OPT (40.9e9)

/** Max decimation for I/Q streaming. */
#define SM_MAX_IQ_DECIMATION (4096)

/**
 * The frequency at which the manually controlled preselector filters end.
 * Past this frequency, the preselector filters are always enabled.
 */
#define SM_PRESELECTOR_MAX_FREQ (645.0e6)

/** Minimum RBW in Hz for fast sweep with Nuttall window. */
#define SM_FAST_SWEEP_MIN_RBW (30.0e3)

/** Min span for device configured in real-time measurement mode */
#define SM_REAL_TIME_MIN_SPAN (200.0e3)
/** Max span for device configured in real-time measurement mode */
#define SM_REAL_TIME_MAX_SPAN (160.0e6)

/** Min sweep time in seconds. See #smSetSweepCoupling. */
#define SM_MIN_SWEEP_TIME (1.0e-6)
/** Max sweep time in seconds. See #smSetSweepCoupling. */
#define SM_MAX_SWEEP_TIME (100.0)

/** Max number of bytes per SPI transfer. */
#define SM_SPI_MAX_BYTES (4)

/** Max number of freq/state pairs for GPIO sweeps. */
#define SM_GPIO_SWEEP_MAX_STEPS (64)

/** Max number of GPIO states for I/Q streaming. */
#define SM_GPIO_SWITCH_MAX_STEPS (64)
/** Min length for GPIO state for I/Q streaming, in counts. */
#define SM_GPIO_SWITCH_MIN_COUNT (2)
/** Max length for GPIO state for I/Q streaming, in counts. */
#define SM_GPIO_SWITCH_MAX_COUNT (4194303 - 1)

/** FPGA core temp should not exceed this value, in C. */
#define SM_TEMP_WARNING (95.0)
/** FPGA shutdown temp, in C. */
#define SM_TEMP_MAX (102.0)

/** Segmented I/Q captures, max segments. */
#define SM_MAX_SEGMENTED_IQ_SEGMENTS (250)
/** Segmented I/Q captures, max samples for all segments combined. */
#define SM_MAX_SEGMENTED_IQ_SAMPLES (520e6)

/** IF output, output frequency. IF output option devices only. */
#define SM435_IF_OUTPUT_FREQ (1.5e9)
/** Min IF output, input frequency. IF output option devices only. */
#define SM435_IF_OUTPUT_MIN_FREQ (24.0e9)
/** Max IF output, input frequency. IF output option devices only. */
#define SM435_IF_OUTPUT_MAX_FREQ (43.5e9)

/**
 * Status code returned from all SM API functions.
 */
typedef enum SmStatus {
    // Internal use
    smCalErr = -1003,
    // Internal use
    smMeasErr = -1002,
    // Internal use
    smErrorIOErr = -1001,

    /** Calibration file unable to be used with the API */
    smInvalidCalibrationFileErr = -200,
    /** Invalid center frequency specified */
    smInvalidCenterFreqErr = -101,
    /** I/Q decimation value provided not a valid value */
    smInvalidIQDecimationErr = -100,

    /** FPGA/initialization error */
    smJESDErr = -54,
    /** Socket/network error */
    smNetworkErr = -53,
    /** If the core FX3 program fails to run */
    smFx3RunErr = -52,
    /** Only can connect up to SM_MAX_DEVICES receivers */
    smMaxDevicesConnectedErr = -51,
    /** FPGA boot error */
    smFPGABootErr = -50,
    /** Boot error */
    smBootErr = -49,

    /** Requesting GPS information when the GPS is not locked */
    smGpsNotLockedErr = -16,
    /** Invalid API version for target device, update API */
    smVersionMismatchErr = -14,
    /** Unable to allocate resources needed to configure the measurement mode */
    smAllocationErr = -13,

    /**
     * Returned when the device detects framing issue on measurement data
     * Measurement results are likely invalid. Device should be preset/power
     * cycled
     */
    smSyncErr = -11,
    /** Invalid or already active sweep position */
    smInvalidSweepPosition = -10,
    /**
     * Attempting to perform an operation that cannot currently be performed.
     * Often the result of trying to do something while the device is currently
     * making measurements or not in an idle state.
     */
    smInvalidConfigurationErr = -8,
    /** Device disconnected, likely USB error detected */
    smConnectionLostErr = -6,
    /** Required parameter found to have invalid value */
    smInvalidParameterErr = -5,
    /** One or more required pointer parameters were null */
    smNullPtrErr = -4,
    /** User specified invalid device index */
    smInvalidDeviceErr = -3,
    /** Unable to open device */
    smDeviceNotFoundErr = -2,

    /** Function returned successfully */
    smNoError = 0,

    /** One or more of the provided settings were adjusted */
    smSettingClamped = 1,
    /** Measurement includes data which caused an ADC overload (clipping/compression) */
    smAdcOverflow = 2,
    /** Measurement is uncalibrated, overrides ADC overflow */
    smUncalData = 3,
    /** Temperature drift occured, measurements uncalibrated, reconfigure the device */
    smTempDriftWarning = 4,
    /** Warning when the preselector span is smaller than the user selected span */
    smSpanExceedsPreselector = 5,
    /** Warning when the internal temperature gets too hot. The device is close to shutting down */
    smTempHighWarning = 6,
    /** Returned when the API was unable to keep up with the necessary processing */
    smCpuLimited = 7,
    /**
     * Returned when the API detects a device with newer features than what was
     * available when this version of the API was released. Suggested fix,
     * update the API.
     */
    smUpdateAPI = 8,
    /** Calibration data potentially corrupt */
    smInvalidCalData = 9,
} SmStatus;

/**
 * Specifies a data type for data returned from the API
 */
typedef enum SmDataType {
    /** 32-bit complex floats */
    smDataType32fc,
    /** 16-bit complex shorts */
    smDataType16sc
} SmDataType;

/**
 * Measurement mode
 */
typedef enum SmMode {
    /** Idle, no measurement */
    smModeIdle = 0,
    /** Swept spectrum analysis */
    smModeSweeping = 1,
    /** Real-time spectrum analysis */
    smModeRealTime = 2,
    /** I/Q streaming */
    smModeIQStreaming = 3,
    /** SM200B/SM435B wide band I/Q capture */
    smModeIQSegmentedCapture = 5,
    /** I/Q sweep list / frequency hopping */
    smModeIQSweepList = 6,
    /** Audio demod */
    smModeAudio = 4,

    // Deprecated, use smModeIQStreaming
    smModeIQ = 3,
} SmMode;

/**
 * Sweep speed
 */
typedef enum SmSweepSpeed {
    /** Automatically choose the fastest sweep speed while maintaining customer requested settings */
    smSweepSpeedAuto = 0,
    /** Use standard sweep speed, always available */
    smSweepSpeedNormal = 1,
    /** Choose fast sweep speed whenever possible, possibly ignoring some requested settings */
    smSweepSpeedFast = 2
} SmSweepSpeed;

/**
 * Base sample rate used for I/Q streaming. See @ref iqAcquisition for more information.
 */
typedef enum SmIQStreamSampleRate {
    /** Use device native sample rate */
    smIQStreamSampleRateNative = 0,
    /** Use LTE sample rates */
    smIQStreamSampleRateLTE = 1,
} SmIQStreamSampleRate;

/**
 * Specifies device power state. See @ref powerStates for more information.
 */
typedef enum SmPowerState {
    /** On */
    smPowerStateOn = 0,
    /** Standby */
    smPowerStateStandby = 1
} SmPowerState;

/**
 * Detector used for sweep and real-time spectrum analysis.
 */
typedef enum SmDetector {
    /** Average */
    smDetectorAverage = 0,
    /** Min/Max */
    smDetectorMinMax = 1
} SmDetector;

/**
 * Specifies units of sweep and real-time spectrum analysis measurements.
 */
typedef enum SmScale {
    /** dBm */
    smScaleLog = 0,
    /** mV */
    smScaleLin = 1,
    /** Log scale, no corrections */
    smScaleFullScale = 2
} SmScale;

/**
 * Specifies units in which VBW processing occurs.
 */
typedef enum SmVideoUnits {
    /** dBm */
    smVideoLog = 0,
    /** Linear voltage */
    smVideoVoltage = 1,
    /** Linear power */
    smVideoPower = 2,
    /** No VBW processing */
    smVideoSample = 3
} SmVideoUnits;

/**
 * Specifies the window used for sweep and real-time analysis.
 */
typedef enum SmWindowType {
    /** SRS flattop */
    smWindowFlatTop = 0,
    /** Nutall */
    smWindowNutall = 2,
    /** Blackman */
    smWindowBlackman = 3,
    /** Hamming */
    smWindowHamming = 4,
    /** Gaussian 6dB BW window for EMC measurements and CISPR compatibility */
    smWindowGaussian6dB = 5,
    /** Rectangular (no) window */
    smWindowRect = 6
} SmWindowType;

/**
 * Trigger type for specific I/Q capture modes.
 */
typedef enum SmTriggerType {
    /** Immediate/no trigger */
    smTriggerTypeImm = 0,
    /** Video/level trigger */
    smTriggerTypeVideo = 1,
    /** External trigger */
    smTriggerTypeExt = 2,
    /** Frequency mask trigger */
    smTriggerTypeFMT = 3
} SmTriggerType;

/**
 * Trigger edge for video and external triggers.
 */
typedef enum SmTriggerEdge {
    /** Rising edge */
    smTriggerEdgeRising = 0,
    /** Falling edge */
    smTriggerEdgeFalling = 1
} SmTriggerEdge;

/**
 * Boolean type. Used in public facing functions instead of `bool` to improve
 * API use from different programming languages.
 */
typedef enum SmBool {
    /** False */
    smFalse = 0,
    /** True */
    smTrue = 1
} SmBool;

/**
 * Used to set the 8 configurable GPIO pins to inputs/outputs.
 */
typedef enum SmGPIOState {
    /** Output */
    smGPIOStateOutput = 0,
    /** Input */
    smGPIOStateInput = 1
} SmGPIOState;

/**
 * Used to indicate the source of the timebase reference for the device.
 */
typedef enum SmReference {
    /** Use the internal 10MHz timebase. */
    smReferenceUseInternal = 0,
    /** Use an external 10MHz timebase on the `10 MHz In` port. */
    smReferenceUseExternal = 1
} SmReference;

/**
 * Device type
 */
typedef enum SmDeviceType {
    /** SM200A */
    smDeviceTypeSM200A = 0,
    /** SM200B */
    smDeviceTypeSM200B = 1,
    /** SM200C */
    smDeviceTypeSM200C = 2,
    /** SM435B */
    smDeviceTypeSM435B = 3,
    /** SM435C */
    smDeviceTypeSM435C = 4
} SmDeviceType;

/**
 * Audio demodulation type.
 */
typedef enum SmAudioType {
    /** AM */
    smAudioTypeAM = 0,
    /** FM */
    smAudioTypeFM = 1,
    /** Upper side band */
    smAudioTypeUSB = 2,
    /** Lower side band */
    smAudioTypeLSB = 3,
    /** CW */
    smAudioTypeCW = 4
} SmAudioType;

/**
 * Internal GPS state
 */
typedef enum SmGPSState {
    /** GPS is not locked */
    smGPSStateNotPresent = 0,
    /** GPS is locked, NMEA data is valid, but the timebase is not being disciplined by the GPS */
    smGPSStateLocked = 1,
    /** GPS is locked, NMEA data is valid, timebase is being disciplined by the GPS */
    smGPSStateDisciplined = 2
} SmGPSState;

/**
 * Used to set the GPIO sweep. See @ref gpio for more information.
 */
typedef struct SmGPIOStep {
    /** Frequency threshold */
    double freq;
    /** GPIO setting for the given threshold */
    uint8_t mask;
} SmGPIOStep;

/**
 * For troubleshooting purposes. For standard diagnostics use #smGetDeviceDiagnostics
 */
typedef struct SmDeviceDiagnostics {
    /** Device voltage */
    float voltage;
    /** Input current */
    float currentInput;
    /** OCXO current */
    float currentOCXO;
    /** TODO */
    float current58;
    /** FPGA core/internal temp */
    float tempFPGAInternal;
    /** Temp near FPGA */
    float tempFPGANear;
    /** OCXO temperature */
    float tempOCXO;
    /** VCO temperature */
    float tempVCO;
    /** Temperature on RF board LO */
    float tempRFBoardLO;
    /** Power supply temperature */
    float tempPowerSupply;
} SmDeviceDiagnostics;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * This function is for USB SM devices only. This function is used to retrieve
 * the serial numbers of all unopened USB SM devices connected to the PC. The
 * maximum number of serial numbers that can be returned is 9. The serial
 * numbers returned can then be used to open specific devices with the
 * smOpenDeviceBySerial function. When the function returns successfully, the
 * serials array will contain deviceCount number of unique SM serial numbers.
 * Only deviceCount values will be modified. This function will not return the
 * serial numbers of any connected networked devices.
 *
 * @param[out] serials Pointer to an array of integers. The array must be larger
 * than the number of USB SM devices connected to the PC.
 *
 * @param[out] deviceCount If the function returns successfully deviceCount
 * will be set to the number devices found on the system.
 *
 * @return
 */

SM_API SmStatus smGetDeviceList(int *serials, int *deviceCount);

/**
 * This function is for USB SM devices only. This function is used to retrieve
 * the serial numbers and device types of all unopened USB SM devices connected
 * to the PC. The maximum number of serial numbers that can be returned is 9.
 * The serial numbers returned can then be used to open specific devices with
 * the smOpenDeviceBySerial function. When the function returns successfully,
 * the serials and deviceCount array will contain deviceCount number of unique
 * SM serial numbers and deviceTypes. Only deviceCount values will be modified.
 * This function will not return the serial numbers of any connected networked
 * devices.
 *
 * @param[out] serials Pointer to an array of integers. The array must be
 * larger than the number of USB SM devices connected to the PC.
 *
 * @param[out] deviceTypes Pointer to an array of SmDeviceType enums. The array
 * must be larger than the number of USB SM devices connected to the PC.
 *
 * @param[out] deviceCount Pointer to integer. If the function returns
 * successfully deviceCount will be set to the number devices found on the
 * system.
 *
 * @return
 */
SM_API SmStatus smGetDeviceList2(int *serials, SmDeviceType *deviceTypes, int *deviceCount);

/**
 * This function is for USB SM devices only. Claim the first unopened USB SM
 * device detected on the system. If the device is opened successfully, a
 * handle to the function will be returned through the device pointer. This
 * handle can then be used to refer to this device for all future API calls.
 * This function has the same effect as calling smGetDeviceList and using the
 * first device found to call smOpenDeviceBySerial.
 *
 * @param[out] device Returns handle that can be used to interface the device.
 *
 * @return
 */
SM_API SmStatus smOpenDevice(int *device);

/**
 * This function is similar to #smOpenDevice except it allows you to specify the
 * device you wish to open. This function is often used in conjunction with
 * #smGetDeviceList when managing several SM devices on on PC.
 *
 * @param[out] device Returns handle that can be used to interface the device.
 *
 * @param[in] serialNumber Serial number of the device you wish to open.
 *
 * @return
 */
SM_API SmStatus smOpenDeviceBySerial(int *device, int serialNumber);

/**
 * This function is for networked (10GbE) devices only. Attempts to connect to
 * a networked device. If the device is opened successfully, a handle to the
 * function will be returned through the device pointer. This handle can then
 * be used to refer to this device for all future API calls. The device takes
 * approximately 12 seconds to boot up after applying power. Until the device
 * is booted, this function will return device not found. The SM API does not
 * set the SO_REUSEADDR socket option. For customers connecting multiple
 * networked devices, we recommend specifying the hostAddr explicitly instead
 * of using “0.0.0.0”. Especially for configurations that involve multiple
 * subnets. If not done, devices beyond the first will likely not be found and
 * this function will return an error.
 *
 * @param[out] device Returns handle that can be used to interface the device.
 *
 * @param[in] hostAddr Host interface IP on which the networked device is
 * connected, provided as a string. Can be “0.0.0.0”. An example parameter is
 * “192.168.2.2”.
 *
 * @param[in] deviceAddr Target device IP provided as a string. If more than
 * one device with this IP is connected to the host interface, the behavior is
 * undefined.
 *
 * @param[in] port Target device port.
 *
 * @return
 */
SM_API SmStatus smOpenNetworkedDevice(int *device,
                                      const char *hostAddr,
                                      const char *deviceAddr,
                                      uint16_t port);

/**
 * This function should be called when you want to release the resources for a
 * device. All resources (memory, etc.) will be released, and the device will
 * become available again for use in the current process. The device handle
 * specified will no longer point to a valid device and the device must be
 * re-opened again to be used. This function should be called before the
 * process exits, but it is not strictly required.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
SM_API SmStatus smCloseDevice(int device);

/**
 * Performs a full device preset. When this function returns, the hardware will
 * have performed a full reset, the device handle will no longer be valid, the
 * #smCloseDevice function will have been called for the device handle, and the
 * device will need to be re-opened again. For USB devices, the full 20 seconds
 * open cycle will occur when re-opening the device. For networked devices,
 * this function blocks for an additional 15 seconds to ensure the device has
 * fully power cycled and can be opened. This function can be used to recover
 * from an undesirable device state.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
SM_API SmStatus smPreset(int device);

/**
 * Performs a full device preset for a device that has not been opened with the
 * #smOpenDevice function. This function will open and then preset the device.
 * This function does not check if the device is already opened. Calling this
 * function on a device that is already open through the API is undefined
 * behavior.
 *
 * @param[in] serialNumber Serial number of the device to preset.
 *
 * @return
 */
SM_API SmStatus smPresetSerial(int serialNumber);

/**
 * This function is for networked (10GbE) devices only. Measure the network
 * throughput between the device and the PC. Useful for troubleshooting network
 * throughput issues.
 *
 * @param[in] device Device handle.
 *
 * @param[in] durationSeconds The duration of the test specified in seconds.
 * Can be values between 16ms and 100s. Recommended value of 1 second minimum
 * to produce good averaging and reduce startup overhead.
 *
 * @param[out] bytesPerSecond Pointer to double which when finished, will
 * contain the measured bytes per second throughput between the device and PC.
 *
 * @return
 */
SM_API SmStatus smNetworkedSpeedTest(int device, double durationSeconds, double *bytesPerSecond);

/**
 * This function returns basic information about a specific open device. Also
 * see #smGetDeviceDiagnostics.
 *
 * @param[in] device Device handle.
 *
 * @param[out] deviceType Pointer to #SmDeviceType to contain the device model
 * number. Can be NULL.
 *
 * @param[out] serialNumber Returns device serial number. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smGetDeviceInfo(int device, SmDeviceType *deviceType, int *serialNumber);

/**
 * Get the firmware version of the device. The firmware version is of the form
 * `major.minor.revision`.
 *
 * @param[in] device Device handle.
 *
 * @param[out] major Pointer to int. Can be NULL.
 *
 * @param[out] minor Pointer to int. Can be NULL.
 *
 * @param[out] revision Pointer to int. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smGetFirmwareVersion(int device, int *major, int *minor, int *revision);

/**
 * Returns whethe the SM435 device has the IF output option. See @ref
 * sm435IFOutputOption for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[out] present Set to #smTrue if the device has the IF output option.
 *
 * @return
 */
SM_API SmStatus smHasIFOutput(int device, SmBool *present);

/**
 * Returns operational information about a device.
 *
 * @param[in] device Device handle.
 *
 * @param[out] voltage Pointer to float, to contain device voltage. Can be
 * `NULL`.
 *
 * @param[out] current Pointer to float, to contain device current. Can be
 * `NULL`.
 *
 * @param[out] temperature Pointer to float, to contain device temperature. Can
 * be `NULL`.
 *
 * @return
 */
SM_API SmStatus smGetDeviceDiagnostics(int device, float *voltage, float *current, float *temperature);

/**
 * Returns operational information about a device. If any temperature sensors
 * are unpopulated, the temperature returned for that sensor will be 240C.
 * Should always be able to retrieve the FPGA core and RF board temperatures.
 *
 * @param[in] device Device handle.
 *
 * @param[out] diagnostics Pointer to struct.
 *
 * @return
 */
SM_API SmStatus smGetFullDeviceDiagnostics(int device, SmDeviceDiagnostics *diagnostics);

/**
 * For networked (10GbE) devices only. Returns a number of diagnostic
 * information for the SFP+ transceiver attached to the device. If either the
 * device is not a networked device or the SFP+ does not communicate diagnostic
 * information, the values returned will be zero.
 *
 * @param[in] device Device handle.
 *
 * @param[out] temp Reported SFP+ temperature in C. Can be `NULL`.
 *
 * @param[out] voltage Reported SFP+ voltage in V. Can be `NULL`.
 *
 * @param[out] txPower Reported transmit power in mW. Can be `NULL`.
 *
 * @param[out] rxPower Reported receive power in mW. Can be `NULL`.
 *
 * @return
 */
SM_API SmStatus smGetSFPDiagnostics(int device,
                                    float *temp,
                                    float *voltage,
                                    float *txPower,
                                    float *rxPower);

/**
 * Change the power state of the device. The power state controls the power
 * consumption of the device. See @ref powerStates for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] powerState New power state.
 *
 * @return
 */
SM_API SmStatus smSetPowerState(int device, SmPowerState powerState);

/**
 * Retrieves the current power state. See @ref powerStates for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[out] powerState Pointer to #SmPowerState.
 *
 * @return
 */
SM_API SmStatus smGetPowerState(int device, SmPowerState *powerState);

/**
 * Set the device attenuation. See @ref refLevelAndSensitivity for more
 * information. Valid values for attenuation are between [0,6] representing
 * between [0,30] dB of attenuation (5dB steps). Setting the attenuation to -1
 * tells the receiver to automatically choose the best attenuation value for
 * the specified reference level selected. Setting attenuation to a non-auto
 * value overrides the reference level selection. The header file provides the
 * SM_AUTO_ATTEN macro for -1.
 *
 * @param[in] device Device handle.
 *
 * @param[in] atten Attenuation value between [-1,6].
 *
 * @return
 */
SM_API SmStatus smSetAttenuator(int device, int atten);

/**
 * Get the device attenuation. See @ref refLevelAndSensitivity for more
 * information.
 *
 * @param[in] device Device handle.
 *
 * @param[out] atten Returns current attenuation value.
 *
 * @return
 */
SM_API SmStatus smGetAttenuator(int device, int *atten);

/**
 * The reference level controls the sensitivity of the receiver by setting the
 * attenuation of the receiver to optimize measurements for signals at or below
 * the reference level. See @ref refLevelAndSensitivity for more information.
 * Attenuation must be set to automatic (-1) to set reference level.
 *
 * @param[in] device Device handle.
 *
 * @param[in] refLevel Set the reference level of the receiver in dBm.
 *
 * @return
 */
SM_API SmStatus smSetRefLevel(int device, double refLevel);

/**
 * Retreive the current device reference level.
 *
 * @param[in] device Device handle.
 *
 * @param[out] refLevel Reference level returned in dBm.
 *
 * @return
 */
SM_API SmStatus smGetRefLevel(int device, double *refLevel);

/**
 * Enable/disable the RF preselector. This setting controls the preselector for
 * all measurement modes.
 *
 * @param[in] device Device handle.
 *
 * @param[in] enabled Set to smTrue to enable the preselector.
 *
 * @return
 */
SM_API SmStatus smSetPreselector(int device, SmBool enabled);

/**
 * Retrieve the current preselector setting.
 *
 * @param[in] device Device handle.
 *
 * @param[out] enabled Returns smTrue if the preselector is enabled.
 *
 * @return
 */
SM_API SmStatus smGetPreselector(int device, SmBool *enabled);

/**
 * Configure whether the GPIO pins are read/write. This affects the pins
 * immediately. See the @ref gpio section for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] lowerState Sets the direction of the lower 4 GPIO pins.
 *
 * @param[in] upperState Sets the direction of the upper 4 GPIO pins.
 *
 * @return
 */
SM_API SmStatus smSetGPIOState(int device, SmGPIOState lowerState, SmGPIOState upperState);

/**
 * Get the direction (read/write) of the GPIO pins. See the @ref gpio section for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[out] lowerState Returns the direction of the lower 4 GPIO pins.
 *
 * @param[out] upperState Returns the direction of the upper 4 GPIO pins.
 *
 * @return
 */
SM_API SmStatus smGetGPIOState(int device, SmGPIOState *lowerState, SmGPIOState *upperState);

/**
 * Set the GPIO output levels. Will only affect GPIO pins configured as
 * outputs. The bits in the data parameter that correspond with GPIO pins that
 * have been set as inputs are ignored.
 *
 * @param[in] device Device handle.
 *
 * @param[in] data Data used to set the GPIO. Each bit corresponds to the 8
 * GPIO pins.
 *
 * @return
 */
SM_API SmStatus smWriteGPIOImm(int device, uint8_t data);

/**
 * Retrieve the values of the GPIO pins. GPIO pins that are configured as
 * outputs will return the set output logic level. If the device is currently
 * idle, the GPIO logic levels are sampled. If the device is configured in a
 * measurement mode, the values returned are those reported from the last
 * measurement taken. For example, if the device is configured for sweeping,
 * each sweep performed will update the GPIO. To retrieve the most current
 * values, either perform another sweep and re-request the GPIO state or put
 * the device in an idle mode and query the GPIO.
 *
 * @param[in] device Device handle.
 *
 * @param[out] data Pointer to byte. Each bit corresponds to the 8 GPIO pins.
 *
 * @return
 */
SM_API SmStatus smReadGPIOImm(int device, uint8_t *data);

/**
 * Output up to 4 bytes on the SPI data pins. See the @ref spi section for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] data Up to 4 bytes of data to transfer.
 *
 * @param[in] byteCount Number of bytes to transfer.
 *
 * @return
 */
SM_API SmStatus smWriteSPI(int device, uint32_t data, int byteCount);

/**
 * Disables and clears the current GPIO sweep setup. The effect of this
 * function will be seen the next time the device is configured. See the @ref
 * gpio section for more information.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
SM_API SmStatus smSetGPIOSweepDisabled(int device);

/**
 * This function is used to set the frequency cross over points for the GPIO
 * sweep functionality and the associated GPIO output logic levels for each
 * frequency. See @ref gpio for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] steps Array of #SmGPIOStep structs. The array must be `stepCount`
 * in length.
 *
 * @param[in] stepCount The number of steps in the steps array.
 *
 * @return
 */
SM_API SmStatus smSetGPIOSweep(int device, SmGPIOStep *steps, int stepCount);

/**
 * Disables any GPIO switching setup. The effect of this function will be seen
 * the next time the device is configured for I/Q streaming. If the device is
 * actively in a GPIO switching loop (and I/Q streaming) the GPIO switching is
 * not disabled until the device is reconfigured. This function can be called
 * at any time. See @ref gpio for more information.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
SM_API SmStatus smSetGPIOSwitchingDisabled(int device);

/**
 * Configures the GPIO switching functionality. See @ref gpio for more
 * information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] gpio Array of GPIO output settings.
 *
 * @param[in] counts Array of dwell times (in 20ns counts). The maximum count
 * value for a given state/step is (2^22 - 1).
 *
 * @param[in] gpioSteps Number of GPIO steps.
 *
 * @return
 */
SM_API SmStatus smSetGPIOSwitching(int device, uint8_t *gpio, uint32_t *counts, int gpioSteps);

/**
 * Enable or disable the 10MHz reference out port. If enabled, the current
 * reference being used by the SM (as specified by #smSetReference) will be
 * output on the 10MHz out port.
 *
 * @param[in] device Device handle.
 *
 * @param[in] enabled Set to smTrue to enable the 10MHz reference out port.
 *
 * @return
 */
SM_API SmStatus smSetExternalReference(int device, SmBool enabled);

/**
 * Return whether the 10MHz reference out port is enabled.
 *
 * @param[in] device Device handle.
 *
 * @param[out] enabled Returns smTrue if the ref out port is enabled.
 *
 * @return
 */
SM_API SmStatus smGetExternalReference(int device, SmBool *enabled);

/**
 * Tell the receiver to use either the internal time base reference or use a
 * 10MHz reference present on the 10MHz in port. The device must be in the idle
 * state (call #smAbort) for this function to take effect. If the function
 * returns successfully, verify the new state with the #smGetReference function.
 *
 * @param[in] device Device handle.
 *
 * @param[in] reference New reference state.
 *
 * @return
 */
SM_API SmStatus smSetReference(int device, SmReference reference);

/**
 * Get the current reference state.
 *
 * @param[in] device Device handle.
 *
 * @param[out] reference Returns current reference configuration.
 *
 * @return
 */
SM_API SmStatus smGetReference(int device, SmReference *reference);

/**
 * Enable whether or not the API auto updates the timebase calibration value
 * when a valid GPS lock is acquired. This function must be called in an idle
 * state. See @ref autoGPSTimebaseDiscipline for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] enabled Send smTrue to enable.
 *
 * @return
 */
SM_API SmStatus smSetGPSTimebaseUpdate(int device, SmBool enabled);

/**
 * Get auto GPS timebase update status. See @ref autoGPSTimebaseDiscipline for
 * more information.
 *
 * @param[in] device Device handle.
 *
 * @param[out] enabled Returns smTrue if auto GPS timebase update is enabled.
 *
 * @return
 */
SM_API SmStatus smGetGPSTimebaseUpdate(int device, SmBool *enabled);

/**
 * Return information about the GPS holdover correction. Determine if a
 * correction exists and when it was generated.
 *
 * @param[in] device Device handle.
 *
 * @param[out] usingGPSHoldover Returns whether the GPS holdover value is newer
 * than the factory calibration value. To determine whether the holdover value
 * is actively in use, you will need to use this function in combination with
 * smGetGPSState. This parameter can be NULL.
 *
 * @param[out] lastHoldoverTime If a GPS holdover value exists on the system,
 * return the timestamp of the value. Value is seconds since epoch. This
 * parameter can be NULL.
 *
 * @return
 */
SM_API SmStatus smGetGPSHoldoverInfo(int device, SmBool *usingGPSHoldover, uint64_t *lastHoldoverTime);

/**
 * Determine the lock and discipline status of the GPS. See the @ref gpsLock section
 * for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[out] GPSState Pointer to #SmGPSState.
 *
 * @return
 */
SM_API SmStatus smGetGPSState(int device, SmGPSState *GPSState);

/**
 * Set sweep speed.
 *
 * @param[in] device Device handle.
 *
 * @param[in] sweepSpeed New sweep speed.
 *
 * @return
 */
SM_API SmStatus smSetSweepSpeed(int device, SmSweepSpeed sweepSpeed);

/**
 * Set sweep center/span.
 *
 * @param[in] device Device handle.
 *
 * @param[in] centerFreqHz New center frequency in Hz.
 *
 * @param[in] spanHz New span in Hz.
 *
 * @return
 */
SM_API SmStatus smSetSweepCenterSpan(int device, double centerFreqHz, double spanHz);

/**
 * Set sweep start/stop frequency.
 *
 * @param[in] device Device handle.
 *
 * @param[in] startFreqHz Start frequency in Hz.
 *
 * @param[in] stopFreqHz Stop frequency in Hz.
 *
 * @return
 */
SM_API SmStatus smSetSweepStartStop(int device, double startFreqHz, double stopFreqHz);

/**
 *Set sweep RBW/VBW parameters.
 *
 * @param[in] device Device handle.
 *
 * @param[in] rbw Resolution bandwidth in Hz.
 *
 * @param[in] vbw Video bandwidth in Hz. Cannot be greater than RBW.
 *
 * @param[in] sweepTime Suggest the total acquisition time of the sweep.
 * Specified in seconds. This parameter is a suggestion and will ensure RBW and
 * VBW are first met before increasing sweep time.
 *
 * @return
 */
SM_API SmStatus smSetSweepCoupling(int device, double rbw, double vbw, double sweepTime);

/**
 * Set sweep detector.
 *
 * @param[in] device Device handle.
 *
 * @param[in] detector New sweep detector.
 *
 * @param[in] videoUnits New video processing units.
 *
 * @return
 */
SM_API SmStatus smSetSweepDetector(int device, SmDetector detector, SmVideoUnits videoUnits);

/**
 * Set the sweep mode output unit type.
 *
 * @param[in] device Device handle.
 *
 * @param[in] scale New sweep mode units.
 *
 * @return
 */
SM_API SmStatus smSetSweepScale(int device, SmScale scale);

/**
 * Set sweep mode window function.
 *
 * @param[in] device Device handle.
 *
 * @param[in] window New window function.
 *
 * @return
 */
SM_API SmStatus smSetSweepWindow(int device, SmWindowType window);

/**
 * Set sweep mode spur rejection enable/disable.
 *
 * @param[in] device Device handle.
 *
 * @param[in] spurRejectEnabled Enable/disable.
 *
 * @return
 */
SM_API SmStatus smSetSweepSpurReject(int device, SmBool spurRejectEnabled);

/**
 * Set the center frequency and span for real-time spectrum analysis.
 *
 * @param[in] device Device handle.
 *
 * @param[in] centerFreqHz Center frequency in Hz.
 *
 * @param[in] spanHz Span in Hz.
 *
 * @return
 */
SM_API SmStatus smSetRealTimeCenterSpan(int device, double centerFreqHz, double spanHz);

/**
 * Set the resolution bandwidth for real-time spectrum analysis.
 *
 * @param[in] device Device handle.
 *
 * @param[in] rbw Resolution bandwidth in Hz.
 *
 * @return
 */
SM_API SmStatus smSetRealTimeRBW(int device, double rbw);

/**
 * Set the detector for real-time spectrum analysis.
 *
 * @param[in] device Device handle.
 *
 * @param[in] detector New detector.
 *
 * @return
 */
SM_API SmStatus smSetRealTimeDetector(int device, SmDetector detector);

/**
 * Set the sweep and frame units used in real-time spectrum analysis.
 *
 * @param[in] device Device handle.
 *
 * @param[in] scale Scale for the returned sweeps.
 *
 * @param[in] frameRef Sets the reference level of the real-time frame, or, the
 * amplitude of the highest pixel in the frame.
 *
 * @param[in] frameScale Specify the height of the frame in dB. A common value
 * is 100dB.
 *
 * @return
 */
SM_API SmStatus smSetRealTimeScale(int device, SmScale scale, double frameRef, double frameScale);

/**
 * Specify the window function used for real-time spectrum analysis.
 *
 * @param[in] device Device handle.
 *
 * @param[in] window New window function.
 *
 * @return
 */
SM_API SmStatus smSetRealTimeWindow(int device, SmWindowType window);

/**
 * Set the base sample rate for I/Q streaming.
 *
 * @param[in] device Device handle.
 *
 * @param[in] sampleRate Base sample rate. Any decimation selected occurs on
 * this sample rate. See @ref iqSampleRates for more information.
 *
 * @return
 */
SM_API SmStatus smSetIQBaseSampleRate(int device, SmIQStreamSampleRate sampleRate);

/**
 * Set the I/Q data type of the samples returned for I/Q streaming.
 *
 * @param[in] device Device handle.
 *
 * @param[in] dataType Data type. See @ref iqDataTypes for more information.
 *
 * @return
 */
SM_API SmStatus smSetIQDataType(int device, SmDataType dataType);

/**
 * Set the center frequency for I/Q streaming.
 *
 * @param[in] device Device handle.
 *
 * @param[in] centerFreqHz Center frequency in Hz.
 *
 * @return
 */
SM_API SmStatus smSetIQCenterFreq(int device, double centerFreqHz);

/**
 * Get the I/Q streaming center frequency.
 *
 * @param[in] device Device handle.
 *
 * @param[in] centerFreqHz Pointer to double.
 *
 * @return
 */
SM_API SmStatus smGetIQCenterFreq(int device, double *centerFreqHz);

/**
 * Set sample rate for I/Q streaming.
 *
 * @param[in] device Device handle.
 *
 * @param[in] decimation Decimation of the I/Q data as a power of 2. See @ref
 * iqSampleRates for more information.
 *
 * @return
 */
SM_API SmStatus smSetIQSampleRate(int device, int decimation);

/**
 * Specify the software filter bandwidth in I/Q streaming. See @ref
 * iqSampleRates for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] enableSoftwareFilter Set to true to enable software filtering
 * (USB devices only). This values is ignored for 10GbE devices.
 *
 * @param[in] bandwidth The bandwidth in Hz.
 *
 * @return
 */
SM_API SmStatus smSetIQBandwidth(int device, SmBool enableSoftwareFilter, double bandwidth);

/**
 * Configure the external trigger edge detect in I/Q streaming.
 *
 * @param[in] device Device handle.
 *
 * @param[in] edge Set the external trigger edge.
 *
 * @return
 */
SM_API SmStatus smSetIQExtTriggerEdge(int device, SmTriggerEdge edge);

/**
 * Configure how external triggers are reported for I/Q streaming.
 *
 * @param[in] sentinelValue Value used to fill the remainder of the trigger
 * buffer when the trigger buffer provided is larger than the number of
 * triggers returned. The default sentinel value is zero. See the @ref
 * iqStreaming section for more information on triggering.
 *
 * @return
 */
SM_API SmStatus smSetIQTriggerSentinel(double sentinelValue);

/**
 * Controls the size of the queue of data that is being actively requested by
 * the API. For example, a queue size of 20ms means the API keeps up to 20ms of
 * data requests active. A larger queue size means a greater tolerance to data
 * loss in the event of an interruption. Because once data is requested, it’s
 * transfer must be completed, a smaller queue size can give you faster
 * reconfiguration times. For instance, if you wanted to change frequencies
 * quickly, a smaller queue size would allow this. A default is chosen for the
 * best resistance to data loss for both Linux and Windows. If you are on Linux
 * and you are using multiple devices, please see @ref linuxNotes.
 *
 * @param[in] device Device handle
 *
 * @param[in] ms Queue size in ms. Will be clamped to multiples of 2.62ms
 * between 2 * 2.62ms and 16 * 2.62ms.
 *
 * @return
 */
SM_API SmStatus smSetIQQueueSize(int device, float ms);

/**
 * Set the data type for data returned for I/Q sweep list measurements.
 *
 * @param[in] device Device handle.
 *
 * @param[in] dataType See @ref iqDataTypes for more information.
 *
 * @return
 */
SM_API SmStatus smSetIQSweepListDataType(int device, SmDataType dataType);

/**
 * Set whether the data returns for I/Q sweep list meausurements is full-scale
 * or corrected.
 *
 * @param[in] device Device handle.
 *
 * @param[in] corrected Set to false for the data to be returned as full scale,
 * and true to be returned amplitude corrected. See @ref iqDataTypes for more
 * information on how to perform these conversions.
 *
 * @return
 */
SM_API SmStatus smSetIQSweepListCorrected(int device, SmBool corrected);

/**
 * Set the number frequency steps for I/Q sweep list measurements.
 *
 * @param[in] device Device handle.
 *
 * @param[in] steps Number of frequency steps in I/Q sweep.
 *
 * @return
 */
SM_API SmStatus smSetIQSweepListSteps(int device, int steps);

/**
 * Get the number steps in the I/Q sweep list measurement.
 *
 * @param[in] device Device handle.
 *
 * @param[out] steps Pointer to int.
 *
 * @return
 */
SM_API SmStatus smGetIQSweepListSteps(int device, int *steps);

/**
 * Set the center frequency of the acquisition at a given step for the I/Q
 * sweep list measurement.
 *
 * @param[in] device Device handle.
 *
 * @param[in] step Step at which to configure the center frequency. Should be
 * between [0, steps-1] where steps is set in the #smSetIQSweepListSteps
 * function.
 *
 * @param[in] freq Center frequency in Hz.
 *
 * @return
 */
SM_API SmStatus smSetIQSweepListFreq(int device, int step, double freq);

/**
 * Set the reference level for a step for the I/Q sweep list measurement.
 *
 * @param[in] device Device handle.
 *
 * @param[in] step Step at which to configure the center frequency. Should be
 * between [0, steps-1] where steps is set in the #smSetIQSweepListSteps
 * function.
 *
 * @param[in] level Reference level in dBm. If this is set, attenuation is set
 * to automatic for this step.
 *
 * @return
 */
SM_API SmStatus smSetIQSweepListRef(int device, int step, double level);

/**
 * Set the attenuation for a step for the I/Q sweep list measurement.
 *
 * @param[in] device Device handle.
 *
 * @param[in] step Step at which to configure the center frequency. Should be
 * between [0, steps-1] where steps is set in the #smSetIQSweepListSteps
 * function.

 * @param[in] atten Attenuation value between [0,6] representing [0,30] dB of
 * attenuation (5dB steps). Setting the attenuation to -1 forces the
 * attenuation to auto, at which time the reference level is used to control
 * the attenuator instead.
 *
 * @return
 */
SM_API SmStatus smSetIQSweepListAtten(int device, int step, int atten);

/**
 * Set the number of I/Q samples to be collected at each step.
 *
 * @param[in] device Device handle.
 *
 * @param[in] step Step at which to configure the center frequency. Should be
 * between [0, steps-1] where steps is set in the #smSetIQSweepListSteps
 * function.
 *
 * @param[in] samples Number of samples. Must be greater than 0. There is no
 * upper limit, but keep in mind contiguous memory must be allocated for the
 * capture. Memory allocation for the capture is the responsibility of the user
 * program.
 *
 * @return
 */
SM_API SmStatus smSetIQSweepListSampleCount(int device, int step, uint32_t samples);

/**
 * Set the data type for the data returned for segmented I/Q captures.
 *
 * @param[in] device Device handle.
 *
 * @param[in] dataType New data type.
 *
 * @return
 */
SM_API SmStatus smSetSegIQDataType(int device, SmDataType dataType);

/**
 * Set the center frequency for segmeneted I/Q captures.
 *
 * @param[in] device Device handle.
 *
 * @param[in] centerFreqHz Center frequency in Hz.
 *
 * @return
 */
SM_API SmStatus smSetSegIQCenterFreq(int device, double centerFreqHz);

/**
 * Configure the video trigger available in segmented I/Q captures. Only 1
 * video trigger configuration can be set.
 *
 * @param[in] device Device handle.
 *
 * @param[in] triggerLevel Trigger level in dBm.
 *
 * @param[in] triggerEdge Video trigger edge type.
 *
 * @return
 */
SM_API SmStatus smSetSegIQVideoTrigger(int device, double triggerLevel, SmTriggerEdge triggerEdge);

/**
 * Configure the external trigger available in segmented I/Q captures. Only 1
 * external trigger configuration can be set.
 *
 * @param[in] device Device handle.
 *
 * @param[in] extTriggerEdge External trigger edge type.
 *
 * @return
 */
SM_API SmStatus smSetSegIQExtTrigger(int device, SmTriggerEdge extTriggerEdge);

/**
 * Configure the frequency mask trigger available in segmented I/Q captures.
 * Only 1 frequency mask trigger configuration can be set.
 *
 * @param[in] device Device handle.
 *
 * @param[in] fftSize Size of the FFT used for FMT triggering. This value must
 * be a power of two between 512 and 16384. The frequency/amplitude mask
 * provided by the user is linearly interpolated and tested at each of the FFT
 * result bins. Smaller FFT sizes provide more time resolution at the expense
 * of frequency resolution, while larger FFT sizes improve frequency resolution
 * at the expense of time resolution. The complex FFT is performed at the
 * 250MS/s I/Q samples with a 50% overlap.
 *
 * @param[in] frequencies Array of count frequencies, specified as Hz,
 * specifying the frequency points of the FMT mask.
 *
 * @param[in] ampls Array of count amplitudes, specified as dBm, specifying the
 * amplitude threshold limits of the FMT mask.
 *
 * @param[in] count Number of FMT points in the frequencies and ampls arrays.
 *
 * @return
 */
SM_API SmStatus smSetSegIQFMTParams(int device,
                                    int fftSize,
                                    const double *frequencies,
                                    const double *ampls,
                                    int count);

/**
 * Set the number of segments in the segmented I/Q captures.
 *
 * @param[in] device Device handle.
 *
 * @param[in] segmentCount Number of segments. Must be set before configuring
 * each segment.
 *
 * @return
 */
SM_API SmStatus smSetSegIQSegmentCount(int device, int segmentCount);

/**
 * Configure a segment for segmented I/Q captures.
 *
 * @param[in] device Device handle.
 *
 * @param[in] segment Segment to configure. Must be between [0,segmentCount-1]
 * where segmentCount is set in #smSetSegIQSegmentCount.
 *
 * @param[in] triggerType Specify the trigger used for this segment.
 *
 * @param[in] preTrigger The number of samples to capture before the trigger event.
 * This is in addition to the capture size. For immediate trigger, pretrigger
 * is added to capture size and then set to zero.
 *
 * @param[in] captureSize The number of sample to capture after the trigger event.
 * For immediate triggers, pretrigger is added to this value and pretrigger is
 * set to zero.
 *
 * @param[in] timeoutSeconds The amount of time to wait for the trigger before
 * returning. If a timeout occurs, a capture still occurs at the moment of the
 * timeout and the API will report a timeout condition.
 *
 * @return
 */
SM_API SmStatus smSetSegIQSegment(int device,
                                  int segment,
                                  SmTriggerType triggerType,
                                  int preTrigger,
                                  int captureSize,
                                  double timeoutSeconds);

/**
 * Set the center frequency for audio demodulation.
 *
 * @param[in] device Device handle.
 *
 * @param[in] centerFreqHz Center frequency in Hz.
 *
 * @return
 */
SM_API SmStatus smSetAudioCenterFreq(int device, double centerFreqHz);

/**
 * Set the audio demodulator for audio demodulation.
 *
 * @param[in] device Device handle.
 *
 * @param[in] audioType Demodulator.
 *
 * @return
 */
SM_API SmStatus smSetAudioType(int device, SmAudioType audioType);

/**
 * Set the audio demodulation filters for audio demodulation.
 *
 * @param[in] device Device handle.
 *
 * @param[in] ifBandwidth IF bandwidth (RBW) in Hz.
 *
 * @param[in] audioLpf Audio low pass frequency in Hz.
 *
 * @param[in] audioHpf Audio high pass frequency in Hz.
 *
 * @return
 */
SM_API SmStatus smSetAudioFilters(int device,
                                  double ifBandwidth,
                                  double audioLpf,
                                  double audioHpf);

/**
 * Set the FM deemphasis for audio demodulation.
 *
 * @param[in] device Device handle.
 *
 * @param[in] deemphasis Deemphasis in us.
 *
 * @return
 */
SM_API SmStatus smSetAudioFMDeemphasis(int device, double deemphasis);

/**
 * This function configures the receiver into a state determined by the mode
 * parameter. All relevant configuration routines must have already been
 * called. This function calls #smAbort to end the previous measurement mode
 * before attempting to configure the receiver. If any error occurs attempting
 * to configure the new measurement state, the previous measurement mode will
 * no longer be active.
 *
 * @param[in] device Device handle.
 *
 * @param[in] mode New measurement mode.
 *
 * @return
 */
SM_API SmStatus smConfigure(int device, SmMode mode);

/**
 * Retrieve the current device measurement mode.
 *
 * @param[in] device Device handle.
 *
 * @param[in] mode Pointer to SmMode.
 *
 * @return
 */
SM_API SmStatus smGetCurrentMode(int device, SmMode *mode);

/**
 * This function ends the current measurement mode and puts the device into the
 * idle state. Any current measurements are completed and discarded and will
 * not be accessible after this function returns.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
SM_API SmStatus smAbort(int device);

/**
 * Retrieves the sweep parameters for an active sweep measurement mode. This
 * function should be called after a successful device configuration to
 * retrieve the sweep characteristics.
 *
 * @param[in] device Device handle.
 *
 * @param[out] actualRBW Returns the RBW being used in Hz. Can be NULL.
 *
 * @param[out] actualVBW Returns the VBW being used in Hz. Can be NULL.
 *
 * @param[out] actualStartFreq Returns the frequency of the first
 * bin in Hz. Can be NULL.
 *
 * @param[out] binSize Returns the frequency spacing between each frequency bin
 * in the sweep in Hz.
 *
 * @param[out] sweepSize Returns the length of the sweep (number of frequency
 * bins). Can be NULL.
 *
 * @return
 */
SM_API SmStatus smGetSweepParameters(int device,
                                     double *actualRBW,
                                     double *actualVBW,
                                     double *actualStartFreq,
                                     double *binSize,
                                     int *sweepSize);

/**
 * Retrieve the real-time measurement mode parameters for an active real-time
 * configuration. This function is typically called after a successful device
 * configuration to retrieve the real-time sweep and frame characteristics.
 *
 * @param[in] device Device handle.
 *
 * @param[out] actualRBW Returns the RBW used in Hz. Can be NULL.
 *
 * @param[out] sweepSize Returns the number of frequency bins in the sweep. Can
 * be NULL.
 *
 * @param[out] actualStartFreq Returns the frequency of the first bin in the
 * sweep in Hz. Can be NULL.
 *
 * @param[out] binSize Frequency bin spacing in Hz. Can be NULL.
 *
 * @param[out] frameWidth The width of the real-time frame. Can be NULL.
 *
 * @param[out] frameHeight The height of the real-time frame. Can be NULL.
 *
 * @param[out] poi 100% probability of intercept of a signal given the current
 * configuration. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smGetRealTimeParameters(int device,
                                        double *actualRBW,
                                        int *sweepSize,
                                        double *actualStartFreq,
                                        double *binSize,
                                        int *frameWidth,
                                        int *frameHeight,
                                        double *poi);

/**
 * Retrieve the I/Q measurement mode parameters for an active I/Q stream or
 * segmented I/Q capture configuration. This function is called after a
 * successful device configuration.
 *
 * @param[in] device Device handle.
 *
 * @param[out] sampleRate The sample rate in Hz. Can be NULL.
 *
 * @param bandwidth The bandwidth of the I/Q capture in Hz. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smGetIQParameters(int device, double *sampleRate, double *bandwidth);

/**
 * Retrieve the I/Q correction factor for an active I/Q stream or segmented I/Q
 * capture. This function is called after a successful device configuration.
 *
 * @param[in] device Device handle.
 *
 * @param[out] scale Amplitude correction used by the API to convert from full
 * scale I/Q to amplitude corrected I/Q. The formulas for these conversions are
 * in the @ref iqDataTypes section.
 *
 * @return
 */
SM_API SmStatus smGetIQCorrection(int device, float *scale);

/**
 * Retrieve the correctsions used to convert full scale I/Q values to amplitude
 * corrected I/Q values for the I/Q sweep list measurement. A correction is
 * returned for each step configured. The device must be configured for I/Q
 * sweep list measurements before calling this function.
 *
 *
 * @param[in] device Device handle.
 *
 * @param[out] corrections Pointer to an array. Array should length >= number
 * of steps configured for the I/Q sweep list measurement. A correction value
 * will be returned for each step configured.
 *
 * @return
 */
SM_API SmStatus smIQSweepListGetCorrections(int device, float *corrections);

/**
 * This function is called after the device is successfully configured for
 * segmented I/Q acquisition. Returns the maximum number of queued captures
 * that can be active. This is calculated with the formula (250 / # of segments
 * in each capture). See @ref iqSegmented for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[out] maxCaptures The maximum number of queued segmented acquisitions
 * that can be active at any time.
 *
 * @return
 */
SM_API SmStatus smSegIQGetMaxCaptures(int device, int *maxCaptures);

/**
 * Perform a single sweep. Block until the sweep completes. Internally, this
 * function is implemented as calling #smStartSweep followed by #smFinishSweep
 * with a sweep position of zero (0). This means that if you want to mix the
 * blocking and queue sweep acquisitions, avoid using index zero for queued
 * sweeps.
 *
 * @param[in] device Device handle.
 *
 * @param[out] sweepMin Can be NULL.
 *
 * @param[out] sweepMax Can be NULL.
 *
 * @param[out] nsSinceEpoch Nanoseconds since epoch. Timestamp representing the
 * end of the sweep. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smGetSweep(int device, float *sweepMin, float *sweepMax, int64_t *nsSinceEpoch);

/**
 * Set the GPIO setting to use for a queued sweep. The next time this sweep is
 * started, the GPIO will change to this value just prior to the sweep
 * starting.
 *
 * @param[in] device Device handle.
 *
 * @param[in] pos Sweep queue position.
 *
 * @param[in] data Data used to set the GPIO pins. Each bit represents a single
 * GPIO pin.
 *
 * @return
 */
SM_API SmStatus smSetSweepGPIO(int device, int pos, uint8_t data);

/**
 * Start a sweep at the queue position. If successful, this function returns
 * immediately.
 *
 * @param[in] device Device handle.
 *
 * @param[in] pos Sweep queue position.
 *
 * @return
 */
SM_API SmStatus smStartSweep(int device, int pos);

/**
 * Finish a previously started queued sweep. Blocks until the sweep completes.
 *
 * @param[in] device Device handle.
 *
 * @param[in] pos Sweep queue position.
 *
 * @param[out] sweepMin Can be NULL.
 *
 * @param[out] sweepMax Can be NULL.
 *
 * @param[out] nsSinceEpoch Nanoseconds since epoch. Timestamp representing the
 * end of the sweep. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smFinishSweep(int device, int pos, float *sweepMin, float *sweepMax, int64_t *nsSinceEpoch);

/**
 * Retrieve a single real-time frame. See @ref realTime for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[out] colorFrame Pointer to memory for the frame.
 * Must be (frameWidth * frameHeight) floats in size. Can be NULL.
 *
 * @param[out] alphaFrame Pointer to memory for the frame.
 * Must be (frameWidth * frameHeight) floats in size. Can be NULL.
 *
 * @param[out] sweepMin Can be NULL.
 *
 * @param[out] sweepMax Can be NULL.
 *
 * @param[out] frameCount Unique integer which refers to a real-time frame and
 * sweep. The frame count starts at zero following a device reconfigure and
 * increments by one for each frame.
 *
 * @param[out] nsSinceEpoch Nanoseconds since epoch for the returned frame. For
 * real-time mode, this value represents the time at the end of the real-time
 * acquisition and processing of this given frame. It is approximate. Can be
 * NULL.
 *
 * @return
 */
SM_API SmStatus smGetRealTimeFrame(int device,
                                   float *colorFrame,
                                   float *alphaFrame,
                                   float *sweepMin,
                                   float *sweepMax,
                                   int *frameCount,
                                   int64_t *nsSinceEpoch);

/**
 * Retrieve one block of I/Q data as specified by the user. This function
 * blocks until the data requested is available.
 *
 * @param[in] device Device handle.
 *
 * @param[out] iqBuf Pointer to user allocated buffer of complex values. The
 * buffer size must be at least (iqBufSize * 2 * sizeof(dataTypeSelected)).
 * Cannot be NULL. Data is returned as interleaved contiguous complex samples.
 * For more information on the data returned and the selectable data types, see
 * @ref iqDataTypes.
 *
 * @param[in] iqBufSize Specifies the number of I/Q samples to be retrieves.
 * Must be greater than zero.
 *
 * @param[out] triggers Pointer to user allocated array of doubles. The buffer
 * must be at least triggerBufSize contiguous doubles. The pointer can also be
 * NULL to indicate you do not wish to receive external trigger information.
 * See @ref iqStreaming section for more information on triggers.
 *
 * @param[in] triggerBufSize Specifies the size of the triggersr array. If the
 * triggers array is NULL, this value should be zero.
 *
 * @param[out] nsSinceEpoch Nanoseconds since epoch. The time of the first I/Q
 * sample returned. Can be NULL. See @ref gpsAndTimestamps for more information.
 *
 * @param[in] purge When set to smTrue, any buffered I/Q data in the API is
 * purged before returned beginning the I/Q block acquisition.
 *
 * @param[out] sampleLoss Set by the API when a sample loss condition occurs.
 * If enough I/Q data has accumulated in the internal API circular buffer, the
 * buffer is cleared and the sample loss flag is set. If purge is set to true,
 * the sample flag will always be set to SM_FALSE. Can be NULL.
 *
 * @param[out] samplesRemaining Set by the API, returns the number of samples
 * remaining in the I/Q circular buffer. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smGetIQ(int device,
                        void *iqBuf,
                        int iqBufSize,
                        double *triggers,
                        int triggerBufSize,
                        int64_t *nsSinceEpoch,
                        SmBool purge,
                        int *sampleLoss,
                        int *samplesRemaining);

/**
 * Perform an I/Q sweep. Blocks until the sweep is complete. Can
 * only be called if no sweeps are in the queue.
 *
 * @param[in] device Device handle.
 *
 * @param[out] dst Pointer to memory allocated for sweep. The user must
 * allocate this memory before calling this function. Must be large enough to
 * contain all samples for all steps in a sweep. The memory must be contiguous.
 * The samples in the sweep are placed contiguously into the array (step 1
 * samples follow step 0, step 2 follows step 1, etc). Samples are tightly
 * packed. It is the responsibility of the user to properly index the arrays
 * when finished. The array will be cast to the user selected data type
 * internally in the API.
 *
 * @param[out] timestamps Pointer to memory allocated for timestamps. The user
 * must allocate this memory before calling these functions. Must be an array
 * of steps int64_t’s, where steps are the number of frequency steps in the
 * sweep. When the sweep completes each timestamp in the array represents the
 * time of the first sample at that frequency in the sweep. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smIQSweepListGetSweep(int device, void *dst, int64_t *timestamps);

/**
 * Starts an I/Q sweep at the given queue position. Up to 16 sweeps can be
 * queue.
 *
 * @param[in] device Device handle.
 *
 * @param[in] pos Sweep queue position. Must be between [0,15].
 *
 * @param[out] dst Pointer to memory allocated for sweep. The user must
 * allocate this memory before calling this function. Must be large enough to
 * contain all samples for all steps in a sweep. The memory must be contiguous.
 * The samples in the sweep are placed contiguously into the array (step 1
 * samples follow step 0, step 2 follows step 1, etc). Samples are tightly
 * packed. It is the responsibility of the user to properly index the arrays
 * when finished. The array will be cast to the user selected data type
 * internally in the API.
 *
 * @param[out] timestamps Pointer to memory allocated for timestamps. The user
 * must allocate this memory before calling these functions. Must be an array
 * of steps int64_t’s, where steps are the number of frequency steps in the
 * sweep. When the sweep completes each timestamp in the array represents the
 * time of the first sample at that frequency in the sweep. Can be NULL.
 *
 * @return
 */
SM_API SmStatus smIQSweepListStartSweep(int device, int pos, void *dst, int64_t *timestamps);

/**
 * Finishes an I/Q sweep at the given queue position. Blocks until the sweep is
 * finished.
 *
 * @param[in] device Device handle.
 *
 * @param[in] pos Sweep queue position. Must be betwee [0,15].
 *
 * @return
 */
SM_API SmStatus smIQSweepListFinishSweep(int device, int pos);

/**
 * Initializes a segmented I/Q capture with the given capture index. If no
 * other captures are active, this capture begins immediately.
 *
 * @param[in] device Device handle.
 *
 * @param[in] capture Capture index, must be between [0, maxCaptures-1].
 *
 * @return
 */
SM_API SmStatus smSegIQCaptureStart(int device, int capture);

/**
 * Waits for a capture to complete. This is a blocking function. To determine
 * if a capture is complete without blocking, use the #smSegIQCaptureWaitAsync
 * function.
 *
 * @param[in] device Device index.
 *
 * @param[in] capture Capture index, must be between [0, maxCaptures-1].
 *
 * @return
 */
SM_API SmStatus smSegIQCaptureWait(int device, int capture);

/**
 * Queries whether the capture is completed. This is a non-blocking function.
 *
 * @param[in] device Device handle.
 *
 * @param[in] capture Capture index, must be between [0, maxCaptures-1].
 *
 * @param[out] completed Returns true if the capture is completed.
 *
 * @return
 */
SM_API SmStatus smSegIQCaptureWaitAsync(int device, int capture, SmBool *completed);

/**
 * Determines if the capture timed out.
 *
 * @param[in] device Device handle.
 *
 * @param[in] capture Capture index, must be between [0, maxCaptures-1].
 *
 * @param[in] segment Segment index within capture. Must be between
 * [0,segmentCount-1].
 *
 * @param[out] timedOut Returns true if the segment was not triggered in time
 * according to the configuration and a timeout occurred.
 *
 * @return
 */
SM_API SmStatus smSegIQCaptureTimeout(int device, int capture, int segment, SmBool *timedOut);

/**
 * Retrieve the timestamp of the capture. The capture should be completed
 * before calling this function.
 *
 * @param[in] device Device handle.
 *
 * @param[in] capture Capture index, must be between [0, maxCaptures-1].
 *
 * @param[in] segment Segment index within capture. Must be between
 * [0,segmentCount-1].
 *
 * @param[out] nsSinceEpoch Nanoseconds since epoch of first sample in capture.
 * If the GPS is locked, this time is synchronized to GPS, otherwise the time
 * is synchronized to the system clock and the system sample rate. When using
 * the system clock, the PC system clock is cached for the first time returned,
 * and all subsequent timings are extrapolated from the first clock using the
 * system clock. If over 16 seconds pass between segment acquisitions, a new
 * CPU system clock is cached. This ensures very accurate relative timings for
 * closely spaced acquisitions when a GPS is not present.
 *
 * @return
 */
SM_API SmStatus smSegIQCaptureTime(int device, int capture, int segment, int64_t *nsSinceEpoch);

/**
 * Retrieves the I/Q sampes of the capture. The capture should be completed
 * before calling this function.
 *
 * @param[in] device Device handle.
 *
 * @param[in] capture Capture index, must be between [0, maxCaptures-1].
 *
 * @param[in] segment Segment index within capture. Must be between
 * [0,segmentCount-1].
 *
 * @param[out] iq User provided I/Q buffer of len complex samples. Should be
 * large enough to accommodate 32-bit complex floats or 16-bit complex shorts
 * depending on the data type selected by smSegIQSetDataType.
 *
 * @param[in] offset I/Q sample offset into segment to retrieve.
 *
 * @param[in] len Number of samples after the offset to retrive.
 *
 * @return
 */
SM_API SmStatus smSegIQCaptureRead(int device, int capture, int segment, void *iq, int offset, int len);

/**
 * Frees the capture so that it can be started again.
 *
 * @param[in] device Device handle.
 *
 * @param[in] capture Capture index, must be between [0, maxCaptures-1].
 *
 * @return
 */
SM_API SmStatus smSegIQCaptureFinish(int device, int capture);

/**
 * Convenience function for captures that only have 1 segment. Performs the
 * full start/wait/time/read/finish sequences.
 *
 * @param[in] device Device handle.
 *
 * @param[in] capture Capture index, must be between [0, maxCaptures-1].
 *
 * @param[out] iq User provided I/Q buffer of len complex samples. Should be
 * large enough to accommodate 32-bit complex floats or 16-bit complex shorts
 * depending on the data type selected by smSegIQSetDataType.
 *
 * @param[in] offset I/Q sample offset into segment to retrieve.
 *
 * @param[in] len Number of samples after the offset to retrive.
 *
 * @param[out] nsSinceEpoch Nanoseconds since epoch of first sample in capture.
 *
 * @param[out] timedOut Returns true if the segment was not triggered in time
 * according to the configuration and a timeout occurred.
 *
 * @return
 */
SM_API SmStatus smSegIQCaptureFull(int device,
                                   int capture,
                                   void *iq,
                                   int offset,
                                   int len,
                                   int64_t *nsSinceEpoch,
                                   SmBool *timedOut);

/**
 * This function is a convenience function for resampling the 250MS/s I/Q
 * output of the segmented I/Q captures to a 245.76MS/s rate required for LTE
 * demodulation. This is a complex to complex resample using a polyphase
 * resample filter with resample fraction 3072/3125. Filter performance is ~24M
 * samples per second. For example, if you provided a 200M sample input, this
 * function would take approximately 8.3 seconds to complete.
 *
 * @param[in] input Pointer to input array. Input array should be interleaved
 * I/Q samples retrieved from the segmented I/Q capture functions.
 *
 * @param[in] inputLen Number of complex I/Q samples in input.
 *
 * @param[out] output Pointer to destination buffer. Should be large enough to
 * accept a resampled input. To guarantee this, a simple approach would be to
 * ensure the output buffer is the same size as the input buffer.
 *
 * @param[inout] outputLen The integer pointed to by outputLen should initially be
 * the size of the output buffer. If the function returns successfully, the
 * integer pointed to by outputLen will contain the number of I/Q samples in
 * the output buffer.
 *
 * @param[in] clearDelayLine Set to true to clear the filter delay line. Set to
 * true when providing the first set of samples in a capture. If the samples
 * provided are a continuation of a capture, set this to false.
 *
 * @return
 */
SM_API SmStatus smSegIQLTEResample(float *input,
                                   int inputLen,
                                   float *output,
                                   int *outputLen,
                                   bool clearDelayLine);

/**
 * Configure the attenuation for the full band I/Q measurement.
 *
 * @param[in] device Device handle.
 *
 * @param[in] atten Value between [0,6]. Sets the attenuator in 5dB steps
 * between [0,30]dB. Cannot be set to auto (-1).
 *
 * @return
 */
SM_API SmStatus smSetIQFullBandAtten(int device, int atten);

/**
 * Enable/disable the I/Q flatness and imbalance corrections for full band I/Q
 * measurements.
 *
 * @param[in] device Device handle.
 *
 * @param[in] corrected When set to smTrue, the IF image and flatness response
 * is corrected. When set to smFalse, no corrections are applied. RF leveling
 * corrections are not applied at any point. Data returned is full scale.
 *
 * @return
 */
SM_API SmStatus smSetIQFullBandCorrected(int device, SmBool corrected);

/**
 * Set the number of samples to be collected in full band I/Q measurements.
 *
 * @param[in] device Device handle.
 *
 * @param[in] samples Number of samples between [2048, 32768]. For full band
 * I/Q sweeps, this is the number of samples to be collected at each freuqency.
 *
 * @return
 */
SM_API SmStatus smSetIQFullBandSamples(int device, int samples);

/**
 * Configure the I/Q full band trigger type.
 *
 * @param[in] device Device handle.
 *
 * @param[in] triggerType Can be set to immediate, video, or external only.
 * Video is only available for SM200C devices with FW version 7.7.5 or newer.
 *
 * @return
 */
SM_API SmStatus smSetIQFullBandTriggerType(int device, SmTriggerType triggerType);

/**
 * Configure the video trigger level for full band I/Q measurements. Video
 * triggering in full band I/Q mode is only available for SM200C devices with
 * FW version 7.7.5 or newer.
 *
 * @param[in] device Device handle.
 *
 * @param[in] triggerLevel Trigger level in dBFS.
 *
 * @return
 */
SM_API SmStatus smSetIQFullBandVideoTrigger(int device, double triggerLevel);

/**
 * Specify the video/external trigger timeout for full band I/Q captures. This
 * is how long the device will wait for a trigger. This setting can only be
 * changed for SM200C devices with FW version 7.7.5 or newer. If not, this
 * value is fixed at 1 second.
 *
 * @param[in] device Device handle.
 *
 * @param[in] triggerTimeout Timeout in seconds. Can be between [0,1].
 *
 * @return
 */
SM_API SmStatus smSetIQFullBandTriggerTimeout(int device, double triggerTimeout);

/**
 * The device must be idle to call this function. When this function returns
 * the device is left in the idle state. This function fully configures and
 * performs this measurement before returning. Calling smConfigure is not
 * required.
 *
 * See smSetIQFullBand*** functions for all configuration parameters associated
 * with this capture.
 *
 * This function acquires I/Q samples at the baseband 500MS/s sample rate at a
 * single frequency. While the IF flatness and image corrections can be
 * applied, RF leveling corrections are not applied. The I/Q data is in full
 * scale. This function can be useful for measuring short transients or fast
 * rise times.
 *
 * When external or video trigger is selected, the SM device will wait up to
 * the configured timeout period before capturing. The capture will
 * automatically trigger after this wait period and no trigger has occurred. If
 * the trigger is detected, the capture will return immediately.
 *
 * There is approximately 48ns of pre-trigger for any video or external trigger
 * capture. This is ~24 samples of pre-trigger.
 *
 * There is no indication that the trigger occurred. The customer will need to
 * inspect the data to verify if a trigger occurred. One possible way to detect
 * whether a trigger occurred is to time the duration of this function call
 * with a long trigger timeout (for example, 1 second). If the function returns
 * much sooner than the timeout period, a trigger likely occurred.
 *
 * See the SDK for an example of using this function.
 *
 * @param[in] device Device handle.
 *
 * @param[out] iq Pointer to array of interleaved I/Q values. The size of the
 * array should be equal to the number of samples set in the
 * smSetIQFullBandSamples function. When the function returns successfully,
 * this array will contain the captured data.
 *
 * @param[in] freq Sets the frequency in 39.0625MHz steps. The center frequency
 * of the capture is equal to (freq + 1) * 39.0625MHz.
 *
 * @return
 */
SM_API SmStatus smGetIQFullBand(int device, float *iq, int freq);

/**
 * The device must be idle to call this function. When this function returns
 * the device is left in the idle state. This function fully configures and
 * performs this measurement before returning. Calling smConfigure is not
 * required.
 *
 * See smSetIQFullBand*** functions for all configuration parameters associated
 * with this capture.
 *
 * This function acquires I/Q samples at the baseband 500MS/s sample rate at
 * several different frequencies.
 *
 * The frequency indices that data are collected at are equal to
 *
 * startIndex + N * stepSize
 *
 * where N is in the range of [0, steps-1]. The center frequency of any given
 * frequency index is,
 *
 * (index + 1) * 39.0625MHz.
 *
 * For example, with a startIndex = 26, step = 4, and stepSize = 9, the
 * frequency indices at which data is collected are
 *
 * (26, 35, 44, 53)
 *
 * which correspond to the center frequencies,
 *
 * (1054.6875 MHz, 1406.25 MHz, 1757.8125 MHz, 2070.3125 MHz).
 *
 * Center frequencies below 650MHz have reduced bandwidth. Above 650MHz, the
 * maximum step size while still maintaining full frequency coverage is 9 (or
 * 351.5625 MHz per step). Reasonable parameters for sweeping from 600MHz to
 * 20GHz with full frequency coverage are startIndex = 16, stepSize = 9, and
 * steps = 56.
 *
 * While the IF flatness and image corrections can be applied, RF leveling
 * corrections are not applied. The amplitude is in full scale. This function
 * can be useful for measuring short transients or fast rise times.
 *
 * Full band sweeps cannot be used in conjunction with triggering. For full
 * band sweeps, immediate triggering is used.
 *
 * See the SDK for an example of using this function.
 *
 * @param[in] device Device handle.
 *
 * @param[out] iq Pointer to an array of steps * samplesPerStep number of
 * interleaved complex values, or 2 * steps * samplesPerStep floating point
 * values. When this function returns data will be stored contiguously in this
 * array. The data at step N is in the index range of [N*samplesPerStep,
 * (N+1)*samplesPerStep-1]. (zero-based indexing) Samples per step is
 * determined by the smSetIQFullBandSamples function.
 *
 * @param[in] startIndex The frequency index of the first acquisition. See the
 * description for how the frequencies are determined.
 *
 * @param[in] stepSize Determines the frequency index step size between each
 * acquisition. Can be zero or negative. See the description for more
 * information.
 *
 * @param[in] steps Determines the number of steps at which I/Q data is
 * collected. Must be in the range of [1, 64]. See the description for more
 * information.
 *
 * @return
 */
SM_API SmStatus smGetIQFullBandSweep(int device, float *iq, int startIndex, int stepSize, int steps);

/**
 * If the device is configured to audio demodulation, use this function to
 * retrieve the next 1000 audio samples. This function will block until the
 * data is ready. Minor buffering of audio data is performed in the API, so it
 * is necessary this function is called repeatedly if contiguous audio data is
 * required. The values returned range between [-1.0, 1.0] representing
 * full-scale audio. In FM mode, the audio values will scale with a change in
 * IF bandwidth.
 *
 * @param[in] device Device handle.
 *
 * @param[out] audio Pointer to array of 1000 32-bit floats.
 *
 * @return
 */
SM_API SmStatus smGetAudio(int device, float *audio);

/**
 * Acquire the latest GPS information which includes a time stamp, location
 * information, and NMEA sentences. The GPS info is updated once per second at
 * the PPS interval. This function can be called while measurements are active.
 * For devices with GPS write capability (see Writing Messages to the GPS) this
 * function has slightly modified behavior. The nmea data will update once per
 * second even when GPS lock is not present. This allows users to retrieve msg
 * responses as a result of sending a message with the smWriteToGPS function.
 * NMEA data can contain null values. When parsing, do not use the null
 * delimiter to mark the end of the message, use the returned nmeaLen.
 *
 * @param[in] device Device handle.
 *
 * @param[in] refresh When set to true and the device is not in a streaming
 * mode, the API will request the latest GPS information. Otherwise the last
 * retrieved data is returned.
 *
 * @param[out] updated Will be set to true if the NMEA data has been updated
 * since the last time the user called this function. Can be set to NULL.
 *
 * @param[out] secSinceEpoch Number of seconds since epoch as reported by the
 * GPS NMEA sentences. Last reported value by the GPS. If the GPS is not
 * locked, this value will be set to zero. Can be NULL.
 *
 * @param[out] latitude Latitude in decimal degrees. If the GPS is not locked,
 * this value will be set to zero. Can be NULL.
 *
 * @param[out] longitude Longitude in decimal degrees. If the GPS is not
 * locked, this value will be set to zero. Can be NULL.
 *
 * @param[out] altitude Altitude in meters. If the GPS is not locked, this
 * value will be set to zero. Can be NULL.
 *
 * @param[out] nmea Pointer to user allocated array of char. The length of this
 * array is specified by the nmeaLen parameter. Can be set to NULL.
 *
 * @param[inout] nmeaLen Pointer to an integer. The integer will initially
 * specify the length of the nmea buffer. If the nmea buffer is shorter than
 * the NMEA sentences to be returned, the API will only copy over nmeaLen
 * characters, including the null terminator. After the function returns,
 * nmeaLen will be the length of the copied nmea data, including the null
 * terminator. Can be set to NULL. If NULL, the nmea parameter is ignored.
 *
 * @return
 */
SM_API SmStatus smGetGPSInfo(int device,
                             SmBool refresh,
                             SmBool *updated,
                             int64_t *secSinceEpoch,
                             double *latitude,
                             double *longitude,
                             double *altitude,
                             char *nmea,
                             int *nmeaLen);

/**
 * Receivers must have GPS write capability to use this function. See @ref
 * writingToGPS. Use this function to send messages to the internal u-blox M8
 * GPS. Messages provided are rounded/padded up to the next multiple of 4
 * bytes. The padded bytes are set to zero.
 *
 * @param[in] device Device handle.
 *
 * @param[in] mem The message to send to the GPS.

 * @param[in] len The length of the message in bytes.
 *
 * @return
 */
SM_API SmStatus smWriteToGPS(int device, const uint8_t *mem, int len);

/**
 * Specify the temperature at which the fan should be enabled. This function
 * has no effect if the optional fan assembly is not installed. The available
 * temperature range is between [10-90] degrees. This function must be called
 * when the device is idle (no measurement mode active).
 *
 * @param[in] device Device handle.
 *
 * @param[in] temp Temperature in C.
 *
 * @return
 */
SM_API SmStatus smSetFanThreshold(int device, int temp);

/**
 * Get current fan temperature threshold.
 *
 * @param[in] device Device handle.
 *
 * @param[out] temp Temperature in C.
 *
 * @return
 */
SM_API SmStatus smGetFanThreshold(int device, int *temp);

/**
 * This command configures the down converted IF output. The device must be
 * idle before calling this function. When this function returns successfully,
 * the RF input will be down converted from the specified frequency to 1.5GHz,
 * output on the 10MHz output port. Use #smSetAttenuator and #smSetRefLevel to
 * adjust device sensitivity. See @ref sm435IFOutputOption for more information.
 *
 * @param[in] device Device handle.
 *
 * @param[in] frequency Input frequency between [24GHz, 43.5GHz], as Hz.
 *
 * @return
 */
SM_API SmStatus smSetIFOutput(int device, double frequency);

/**
 * Returns the last device adjustment date.
 *
 * @param[in] device Device handle.
 *
 * @param[out] lastCalDate Last adjustment data as seconds since epoch.
 *
 * @return
 */
SM_API SmStatus smGetCalDate(int device, uint64_t *lastCalDate);

/**
 * This function is for networked devices only. This function broadcasts a
 * configuration UDP packet on the host network interface with the IP specified
 * by hostAddr. The device will take on the IP address and port specified by
 * deviceAddr and port.
 *
 * @param[in] hostAddr This is the host IP address the broadcast message will
 * be sent on.
 *
 * @param[in] deviceAddr This is the address the device will use if it receives
 * the broadcast message.
 *
 * @param[in] port This is the port the device will use if it receives the
 * broadcast message.
 *
 * @param[in] nonVolatile If set to true, the device will use the address and
 * port on future power ups. As this requires a flash erase/write, setting this
 * value to true reduces the life of the flash memory on the device. We
 * recommend either setting this value to false and broadcasting the
 * configuration before each connect, or only setting the device once up front
 * with the nonvolatile flag set to true.
 *
 * @return
 */
SM_API SmStatus smBroadcastNetworkConfig(const char *hostAddr,
                                         const char *deviceAddr,
                                         uint16_t port,
                                         SmBool nonVolatile);

/**
 * This function is part of a group of functions used to configure the network
 * settings of a networked SM device over the USB 2.0 port. The handle used for
 * these functions can only be used with the other network config functions.
 *
 * @param[out] serials Array of ints. Must be as big as deviceCount.
 *
 * @param[inout] deviceCount Point to int that contains the size of the serials
 * array. When the function returns it will contain the number of devices
 * returned.
 *
 * @return
 */
SM_API SmStatus smNetworkConfigGetDeviceList(int *serials, int *deviceCount);

/**
 * This function is part of a group of functions used to configure the network
 * settings of a networked SM device over the USB 2.0 port. The handle used for
 * these functions can only be used with the other network config functions.
 *
 * @param[out] device If successful, device will point to an integer that can be used to
 *
 * @param[in] serialNumber Serial number of the device to open. A list of
 * connected serial numbers can be retrieved from the
 * #smNetworkConfigGetDeviceList function.
 *
 * @return
 */
SM_API SmStatus smNetworkConfigOpenDevice(int *device, int serialNumber);

/**
 * This function is part of a group of functions used to configure the network
 * settings of a networked SM device over the USB 2.0 port. The handle used for
 * these functions can only be used with the other network config functions.
 *
 * Closes the device and frees any resources. The handle should be closed
 * before interfacing the device through the main API interface.
 *
 * @param[in] device Device handle.
 *
 * @return
 */
SM_API SmStatus smNetworkConfigCloseDevice(int device);

/**
 * This function is part of a group of functions used to configure the network
 * settings of a networked SM device over the USB 2.0 port. The handle used for
 * these functions can only be used with the other network config functions.
 *
 * Retrieve the device MAC address.
 *
 * @param[in] device Device handle.
 *
 * @param[out] mac Pointer to char buffer, to contain the null terminated MAC
 * address string of the unit, with the following format “XX-XX-XX-XX-XX-XX”.
 * Must be large enough to accommodate this string including null termination.
 *
 * @return
 */
SM_API SmStatus smNetworkConfigGetMAC(int device, char *mac);

/**
 * This function is part of a group of functions used to configure the network
 * settings of a networked SM device over the USB 2.0 port. The handle used for
 * these functions can only be used with the other network config functions.
 *
 * Set device IP address.
 *
 * @param[in] device Device handle.
 *
 * @param[in] addr pointer to char buffer, to contain the null terminated IP
 * address string of the form “xxx.xxx.xxx.xxx”. For functions retrieving the
 * IP address, the buffer must be large enough to hold this string including
 * null termination.
 *
 * @param[in] nonVolatile When set to smTrue, setting applied will be written
 * to internal flash, which will persist through a device power cycle.
 *
 * @return
 */
SM_API SmStatus smNetworkConfigSetIP(int device, const char *addr, SmBool nonVolatile);

/**
 * This function is part of a group of functions used to configure the network
 * settings of a networked SM device over the USB 2.0 port. The handle used for
 * these functions can only be used with the other network config functions.
 *
 * Get device IP address.
 *
 * @param[in] device Device handle.
 *
 * @param[out] addr pointer to char buffer, to contain the null terminated IP
 * address string of the form “xxx.xxx.xxx.xxx”. For functions retrieving the
 * IP address, the buffer must be large enough to hold this string including
 * null termination.
 *
 * @return
 */
SM_API SmStatus smNetworkConfigGetIP(int device, char *addr);

/**
 * This function is part of a group of functions used to configure the network
 * settings of a networked SM device over the USB 2.0 port. The handle used for
 * these functions can only be used with the other network config functions.
 *
 * Set the device IP port.
 *
 * @param[in] device Device handle.
 *
 * @param[in] port Port number.
 *
 * @param[in] nonVolatile When set to smTrue, setting applied will be written
 * to internal flash, which will persist through a device power cycle.
 *
 * @return
 */
SM_API SmStatus smNetworkConfigSetPort(int device, int port, SmBool nonVolatile);

/**
 * This function is part of a group of functions used to configure the network
 * settings of a networked SM device over the USB 2.0 port. The handle used for
 * these functions can only be used with the other network config functions.
 *
 * Get the device IP port.
 *
 * @param[in] device Device handle.
 *
 * @param[out] port Port number.
 *
 * @return
 */
SM_API SmStatus smNetworkConfigGetPort(int device, int *port);

/**
 * Get the API version.
 *
 * @return
 * The returned string is of the form
 *
 * major.minor.revision
 *
 * Ascii periods ('.') separate positive integers. Major/minor/revision are not
 * guaranteed to be a single decimal digit. The string is null terminated. The
 * string should not be modified or freed by the user. An example string is
 * below…
 *
 * ['3' | '.' | '0' | '.' | '1' | '1' | '\0'] = "3.0.11"
 */
SM_API const char* smGetAPIVersion();

/**
 * Retrieve a descriptive string of a SmStatus enumeration. Useful for
 * debugging and diagnostic purposes.
 *
 * @param[in] status Status code returned from any API function.
 *
 * @return
 */
SM_API const char* smGetErrorString(SmStatus status);

SM_DEPRECATED("smSetIQUSBQueueSize has been deprecated, use smSetIQQueueSize")
SM_API SmStatus smSetIQUSBQueueSize(int device, float ms);

#ifdef __cplusplus
} // Extern "C"
#endif

// Deprecated macros
#define SM200A_AUTO_ATTEN (SM_AUTO_ATTEN)
#define SM200A_MAX_ATTEN (SM_MAX_ATTEN)
#define SM200A_MAX_REF_LEVEL (SM_MAX_REF_LEVEL)
#define SM200A_MAX_SWEEP_QUEUE_SZ (SM_MAX_SWEEP_QUEUE_SZ)
#define SM200A_MIN_FREQ (SM200_MIN_FREQ)
#define SM200A_MAX_FREQ (SM200_MAX_FREQ)
#define SM200A_MAX_IQ_DECIMATION (SM_MAX_IQ_DECIMATION)
#define SM200A_PRESELECTOR_MAX_FREQ (SM_PRESELECTOR_MAX_FREQ)
#define SM200A_FAST_SWEEP_MIN_RBW (SM_FAST_SWEEP_MIN_RBW)
#define SM200A_RTSA_MIN_SPAN (SM_REAL_TIME_MIN_SPAN)
#define SM200A_RTSA_MAX_SPAN (SM_REAL_TIME_MAX_SPAN)
#define SM200A_MIN_SWEEP_TIME (SM_MIN_SWEEP_TIME)
#define SM200A_MAX_SWEEP_TIME (SM_MAX_SWEEP_TIME)
#define SM200A_SPI_MAX_BYTES (SM_SPI_MAX_BYTES)
#define SM200A_GPIO_SWEEP_MAX_STEPS (SM_GPIO_SWEEP_MAX_STEPS)
#define SM200A_GPIO_SWITCH_MAX_STEPS (SM_GPIO_SWITCH_MAX_STEPS)
#define SM200A_GPIO_SWITCH_MIN_COUNT (SM_GPIO_SWITCH_MIN_COUNT)
#define SM200A_GPIO_SWITCH_MAX_COUNT (SM_GPIO_SWITCH_MAX_COUNT)
#define SM200A_TEMP_WARNING (SM_TEMP_WARNING)
#define SM200A_TEMP_MAX (SM_TEMP_MAX)
#define SM200B_MAX_SEGMENTED_IQ_SEGMENTS (SM_MAX_SEGMENTED_IQ_SEGMENTS)
#define SM200B_MAX_SEGMENTED_IQ_SAMPLES (SM_MAX_SEGMENTED_IQ_SAMPLES)
#define SM200_ADDR_ANY (SM_ADDR_ANY)
#define SM200_DEFAULT_ADDR (SM_DEFAULT_ADDR)
#define SM200_DEFAULT_PORT (SM_DEFAULT_PORT)

#endif // SM_API_H
