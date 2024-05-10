/*
  Copyright Cepton Technologies Inc., All rights reserved.

  Cepton Sensor SDK C interface.
*/
#ifndef CEPTON_SDK2_H
#define CEPTON_SDK2_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// API compatibility version (Not the same as SDK version)
#define CEPTON_API_VERSION 203

//------------------------------------------------------------------------------
// Macros
//------------------------------------------------------------------------------
// CEPTON_EXPORT
#ifndef CEPTON_EXPORT
#ifdef _MSC_VER
#define CEPTON_EXPORT __declspec(dllimport)
#else
#define CEPTON_EXPORT
#endif
#endif

//------------------------------------------------------------------------------
// Errors
//------------------------------------------------------------------------------
/// Error code returned by most library functions.
#ifndef CEPTON_SDK_H
enum {
  /// No error.
  CEPTON_SUCCESS = 0,
  /// Generic error.
  CEPTON_ERROR_GENERIC = -1,
  /// Failed to allocate heap memory.
  CEPTON_ERROR_OUT_OF_MEMORY = -2,
  /// Could not find sensor.
  CEPTON_ERROR_SENSOR_NOT_FOUND = -4,
  /// SDK version mismatch.
  CEPTON_ERROR_SDK_VERSION_MISMATCH = -5,
  /// Networking error.
  CEPTON_ERROR_COMMUNICATION = -6,
  /// Callback already set.
  CEPTON_ERROR_TOO_MANY_CALLBACKS = -7,
  /// Invalid value or uninitialized struct.
  CEPTON_ERROR_INVALID_ARGUMENTS = -8,
  /// Already initialized.
  CEPTON_ERROR_ALREADY_INITIALIZED = -9,
  /// Not initialized.
  CEPTON_ERROR_NOT_INITIALIZED = -10,
  /// Invalid file type.
  CEPTON_ERROR_INVALID_FILE_TYPE = -11,
  /// File IO error.
  CEPTON_ERROR_FILE_IO = -12,
  /// Corrupt/invalid file.
  CEPTON_ERROR_CORRUPT_FILE = -13,
  /// Not open.
  CEPTON_ERROR_NOT_OPEN = -14,
  /// End of file.
  CEPTON_ERROR_EOF = -15,
  /// Functionality not supported by device
  CEPTON_ERROR_NOT_SUPPORTED = -16,
  /// Device response invalid
  CEPTON_ERROR_INVALID_RESPONSE = -17,
  /// Software state invalid
  CEPTON_ERROR_INVALID_STATE = -18,
  /// Feature is not enabled
  CEPTON_ERROR_NOT_ENABLED = -19,
  /// Get data timeout
  CEPTON_ERROR_TIMEOUT = -20,

  /// Internal sensor parameter out of range.
  CEPTON_FAULT_INTERNAL = -1000,
  /// Extreme sensor temperature fault.
  CEPTON_FAULT_EXTREME_TEMPERATURE = -1001,
  /// Extreme sensor humidity fault.
  CEPTON_FAULT_EXTREME_HUMIDITY = -1002,
  /// Extreme sensor acceleration fault.
  CEPTON_FAULT_EXTREME_ACCELERATION = -1003,
  /// Abnormal sensor FOV fault.
  CEPTON_FAULT_ABNORMAL_FOV = -1004,
  /// Abnormal sensor frame rate fault.
  CEPTON_FAULT_ABNORMAL_FRAME_RATE = -1005,
  /// Sensor motor malfunction fault.
  CEPTON_FAULT_MOTOR_MALFUNCTION = -1006,
  /// Sensor laser malfunction fault.
  CEPTON_FAULT_LASER_MALFUNCTION = -1007,
  /// Sensor detector malfunction fault.
  CEPTON_FAULT_DETECTOR_MALFUNCTION = -1008,
};
#endif
//------------------------------------------------------------------------------
// Static functions that can be called before initialization
//------------------------------------------------------------------------------

/**
 * Returns empty string if error code is invalid.
 *
 * @return Error code name string converted from int. "" if unrecognized.
 */
CEPTON_EXPORT const char *CeptonGetErrorCodeName(int error_code);

/**
 * Get the SDK version (not the same as API version)
 * @return major/minor/build/patch in 4 bytes (lower endian 32bit integer).
           e.g. 2.1.15.0 will be 0x000F0102
 */
CEPTON_EXPORT uint32_t CeptonGetSdkVersion(void);

//------------------------------------------------------------------------------
// Types
//------------------------------------------------------------------------------
/// Sensor identifier.
/**
 * Generated from sensor IP address.
 */
typedef uint64_t CeptonSensorHandle;

/// Sensor model.
typedef uint16_t CeptonSensorModel;
enum {
  CEPTON_MODEL_VISTA_860_GEN2 = 7,
  CEPTON_MODEL_VISTA_X120 = 10,
  CEPTON_MODEL_SORA_P60 = 11,
  CEPTON_MODEL_VISTA_P60 = 12,
  // 13 is reserved
  CEPTON_MODEL_VISTA_P90 = 14,
  CEPTON_MODEL_SORA_P90 = 15,
  CEPTON_MODEL_VISTA_P61 = 16,
  CEPTON_MODEL_SORA_P61 = 17,
  CEPTON_MODEL_NOVA_A = 18,
  // 19 is reserved for VISTA_P60 Rev2 firmware releases
  // 20 is reserved
  // 21 is reserved
  CEPTON_MODEL_APEX90 = 22,
  // 23 is reserved
  CEPTON_MODEL_VISTA_X90 = 24,
  CEPTON_NOVA_B1 = 25,
};

//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------

/**
 * Callback for receiving SDK and sensor errors.
 * This is a global function (without user_data)
 * @param handle sensor handle.
 * @param error_code Error code.
 * @param error_msg Error message string. Owned by SDK.
 * @param error_data Error payload. Specific to the error reported
 * @param error_data_size Error payload size.
 */
typedef void (*CeptonSensorErrorCallback)(CeptonSensorHandle handle,
                                          int error_code, const char *error_msg,
                                          const void *error_data,
                                          size_t error_data_size);

/// Returns 1 if sdk is initialized. 0 if not.
CEPTON_EXPORT int CeptonIsInitialized(void);

/**
 * Initialize SDK.
 * Must be called before any other sdk functions.
 *
 * @param api_version `CEPTON_API_VERSION`, to ensure header and library match
 * @param cb Global error and fault callback.
 */
CEPTON_EXPORT int CeptonInitialize(int api_version,
                                   CeptonSensorErrorCallback cb);

/**
 * Deinitialize SDK.
 */
CEPTON_EXPORT int CeptonDeinitialize(void);

/**
 * Start networking. Use default port
 */
CEPTON_EXPORT int CeptonStartNetworking(void);  // Use default port

/**
 * Start networking on a custom port
 * @param port UDP port to connect to sensor
 */
CEPTON_EXPORT int CeptonStartNetworkingOnPort(uint16_t port);  // custom port

/**
 * Start networking by joining a multicast group
 * @param targetMcastGroup
 * @param localIfAddress
 * @param port
 */
CEPTON_EXPORT int CeptonStartNetworkingMulticast(
    const char *targetMcastGroup, const char *localIfAddress,
    uint16_t port);  // sensor is in multicast group

/**
 * Stop networking
 */
CEPTON_EXPORT int CeptonStopNetworking(void);

//------------------------------------------------------------------------------
/**
 * A simple set of APIs to replay pcap files.
 *
 * See samples/capture_replay on how to implement pause/resume/seek and
 * multiple concurrent replays functionalities
 *
 * NOTE: CeptonReplayHandle refers to a capture being replayed. A capture can
 * have multiple sensors, therefore it is not corresponding to any SensorHandle
 */
typedef void *CeptonReplayHandle;
enum CeptonReplayFlags {
  CEPTON_REPLAY_FLAG_LOAD_WITHOUT_INDEX = 1,
  CEPTON_REPLAY_FLAG_PLAY_LOOPED = 2,
  CEPTON_REPLAY_FLAG_LOAD_PAUSED = 4,
};

/**
 * Load pcap. By default, it will start indexing and plays async. Will need to
 * be unloaded in the end.
 * @param pcapFileName pcap file path for replay.
 * @param flags see enum CeptonReplayFlags
 * @param pHandle this will need to be passed in other replay calls.
 * @return CEPTON_SUCCESS or error code
 *  CEPTON_ERROR_INVALID_STATE  too many loaded pcaps
 *  CEPTON_ERROR_FILE_IO        load failed, file IO or format error
 */
CEPTON_EXPORT int CeptonReplayLoadPcap(const char *pcapFileName, uint32_t flags,
                                       CeptonReplayHandle *pHandle);

/**
 * Unload the pcap.
 * @param {CeptonReplayHandle} pcap handle
 */
CEPTON_EXPORT int CeptonReplayUnloadPcap(CeptonReplayHandle);

/**
 * Start playing. Resume from a paused state.
 * @param {CeptonReplayHandle} pcap handle
 */
CEPTON_EXPORT int CeptonReplayPlay(CeptonReplayHandle);

/**
 * Play until the whole pcap is finished. API will fail if auto_replay is set.
 * This API can be called while playing. It will still block until whole
 * pcap is finished. If it is called from the player thread (e.g. points
 * callback), the API will fail.
 * @param {CeptonReplayHandle} pcap handle
 */
CEPTON_EXPORT int CeptonReplayPlayToFinish(CeptonReplayHandle);

/**
 * Pause. Does nothing if already paused.
 * Listener Callback will not be called after return from this function.
 * @param {CeptonReplayHandle} pcap handle
 */
CEPTON_EXPORT int CeptonReplayPause(CeptonReplayHandle);

/**
 * Get length of the pcap file.
 * NOTE: If indexing is not done this call will block until it is.
 * It starts indexing if pcap was loaded with flag to not do indexing.
 * @param {CeptonReplayHandle} pcap handle
 * @return length in microseconds of the pcap file. Error code if
 * return value is negative.
 */
CEPTON_EXPORT int64_t CeptonReplayGetLength(CeptonReplayHandle);

/**
 * Seek to position.
 * NOTE: This call will block until indexing has been completed up to the
 * required seek position. The poistion is based on the recorded file and not
 * affected by the current playing speed.
 * @param {CeptonReplayHandle} pcap handle
 * @param position microseconds from starting of the pcap
 * @return Error code
 */
CEPTON_EXPORT int CeptonReplaySeek(CeptonReplayHandle, int64_t position);

/**
 * Get the seek position.
 * @return microseconds from the start of the pcap if positive. Error code if
 * return value is negative.
 */
CEPTON_EXPORT int64_t CeptonReplayGetSeekPosition(CeptonReplayHandle);

/**
 * Play until next frame.
 * State will always be paused after this function returns.
 * @param {CeptonReplayHandle} pcap handle
 * @return Error code
 */
CEPTON_EXPORT int CeptonReplayNextFrame(CeptonReplayHandle);

/**
 * Sets the speed for pcap replay.
 * @param {CeptonReplayHandle} pcap handle
 * @param speed_percent 100 means 1x speed. Special value 0 for as fast as
 * possible
 * @return Error code
 */
CEPTON_EXPORT int CeptonReplaySetSpeed(CeptonReplayHandle, int speed);

/**
 * Sets whether the pcap should auto-replay when reaching the end.
 * @param {CeptonReplayHandle} pcap handle
 * @param {int} autoReplay 1 to replay automatically, 0 to not replay
 */
CEPTON_EXPORT int CeptonReplaySetAutoReplay(CeptonReplayHandle, int autoReplay);

/**
 * Gets replay speed.
 * @param {CeptonReplayHandle} pcap handle
 * @return replay speed (100 mean 1x). Error code if return value is negative.
 */
CEPTON_EXPORT int CeptonReplayGetSpeed(CeptonReplayHandle);

/**
 * Gets last indexed position in microseconds from starting of pcap.
 * @param {CeptonReplayHandle}
 * @return last indexed position in microseconds. Error code if
 * return value is negative.
 */
CEPTON_EXPORT int64_t CeptonReplayGetIndexPosition(CeptonReplayHandle handle);

/**
 * Checks if async playback has finished.
 * @return 1 if playback is finished, 0 if not. Negative for error code.
 */
CEPTON_EXPORT int CeptonReplayIsFinished(CeptonReplayHandle handle);

/**
 * Check if playback is currently paused.
 * @return 1 if replay is paused, 0 if not paused, negative for error code.
 */
CEPTON_EXPORT int CeptonReplayIsPaused(CeptonReplayHandle handle);

//------------------------------------------------------------------------------
/**
 * Async relay create a thread and allocates count*size buffer space.
 * The goal is to decouple the network receiving thread so the packets are
 * always received timely. This is esp. important when timestamp is only set by
 * the network receiving time.
 *
 * buffer_count must be between 2 and 4096, typically 16 or 32
 *
 * NOTE: Async relay covers all data going through CeptonReceiveData
 */
CEPTON_EXPORT int CeptonStartAsyncRelay(int buffer_count);

/**
 * Stop async relay.
 */
CEPTON_EXPORT int CeptonStopAsyncRelay(void);
//------------------------------------------------------------------------------
// Framework/Mechanism
//------------------------------------------------------------------------------
/**
 * Blocks while processing, and calls parser callbacks synchronously before
 * returning.
 *
 * @param handle Sensor handle, arbitrary value to indicate same sensor.
 * Networking uses IPv4
 * @param timestamp Unix timestamp [microseconds].
 * @param buffer Packet bytes.
 * @param buffer_size Packet size.
 */
CEPTON_EXPORT int CeptonReceiveData(CeptonSensorHandle handle,
                                    int64_t timestamp, const uint8_t *buffer,
                                    size_t buffer_size);

/**
 * Callback to handle incoming data. Return 0 to indicate the data is handled.
 * There can be multiple callbacks
 * @param handle Sensor handle
 * @param timestamp Time data is received
 * @param data Pointer to the data buffer
 * @param data_size Size of the data buffer
 * @param user_data Pass back the user data entered at register time.
 */
typedef int (*CeptonParserCallback)(CeptonSensorHandle handle,
                                    int64_t timestamp, const uint8_t *data,
                                    size_t data_size, void *user_data);

/**
 * Register a parser. Unique parsers are determined by the pair of callback and
 * user_data. Save callback with different user_data are considered different
 * parsers
 */
CEPTON_EXPORT int CeptonRegisterParser(CeptonParserCallback callback,
                                       void *user_data);

/**
 * Remove a parser. Only remove when both callback and user_data matches
 */
CEPTON_EXPORT int CeptonUnregisterParser(CeptonParserCallback callback,
                                         void *user_data);

//------------------------------------------------------------------------------
// Points and Sensors structure

enum {
  CEPTON_POINT_SATURATED = 1 << 0,
  CEPTON_POINT_LOW_SNR = 1 << 1,
  CEPTON_POINT_FRAME_PARITY = 1 << 2,
  CEPTON_POINT_FRAME_BOUNDARY = 1 << 3,
  CEPTON_POINT_SECOND_RETURN = 1 << 4,
  CEPTON_POINT_NO_RETURN = 1 << 5,
  CEPTON_POINT_NOISE = 1 << 6,
};

#pragma pack(push, 1)
struct CeptonPoint {
  int16_t x;
  uint16_t y;
  int16_t z;
  uint8_t reflectivity;
  uint8_t relative_timestamp;
  uint8_t channel_id;
  uint8_t flags;
};

struct CeptonSensor {
  /// Size of the CeptonSensor struct
  /// plus any consecutive sensor info blocks
  uint32_t info_size;

  // per sensor info
  uint32_t serial_number;
  CeptonSensorHandle handle;

  // Model
  char model_name[28];
  uint16_t model;
  uint16_t model_reserved;
  uint32_t part_number;

  // FW
  uint32_t firmware_version;  // LSB->MSB major/minor/build/patch

  // Time
  int64_t power_up_timestamp;
  int64_t time_sync_offset;
  int64_t time_sync_drift;

  /// Config
  uint8_t return_count;
  uint8_t channel_count;
  uint8_t reserved[2];
  uint32_t status_flags;

  // Unit in 0.01 Kelvin
  uint16_t temperature;
};
#pragma pack(pop)

enum _CeptonSensorStatusFlags {
  CEPTON_SENSOR_PTP_CONNECTED = 1,
  CEPTON_SENSOR_PPS_CONNECTED = 2,
  CEPTON_SENSOR_NMEA_CONNECTED = 4,
};

//------------------------------------------------------------------------------
// Sensors
//------------------------------------------------------------------------------
/**
 * Get number of sensors attached.
 * Use to check for new sensors. Sensors are not deleted until deinitialization.
 */
CEPTON_EXPORT size_t CeptonGetSensorCount(void);

/// Returns sensor information by sensor index.
/**
 * Useful for getting information for all sensors.
 * Valid indices are in range [0, `cepton_sdk_get_n_sensors()`).
 *
 * Returns error if index invalid.
 *
 * @param idx Sensor index. Returns error if invalid.
 * @param info Sensor information.
 */
CEPTON_EXPORT int CeptonGetSensorInformationByIndex(size_t idx,
                                                    struct CeptonSensor *info);

/**
 * Get sensor information by the handle
 * @param handle
 * @param info
 */
CEPTON_EXPORT int CeptonGetSensorInformation(CeptonSensorHandle handle,
                                             struct CeptonSensor *info);

//------------------------------------------------------------------------------
// Point Cloud Data
//------------------------------------------------------------------------------
typedef void (*CeptonPointsCallback)(CeptonSensorHandle handle,
                                     int64_t start_timestamp, size_t n_points,
                                     size_t stride, const uint8_t *points,
                                     void *user_data);

CEPTON_EXPORT int CeptonListenPoints(CeptonPointsCallback callback,
                                     void *user_data);

/// Clears image frame callback.
CEPTON_EXPORT int CeptonUnlistenPoints(CeptonPointsCallback callback,
                                       void *user_data);

//------------------------------------------------------------------------------
// Frame Aggregator
//------------------------------------------------------------------------------

/**
 * aggregateMode definition:
 *  0 (default) for "natural" frames as defined by the sensor
 *  Positive number enables timed aggregation with number of microseconds
 * NOTE: Any positive number above 1000 is allowed, but when frame exceeds
 * maximum number of points per frame, callback will be triggered even before
 * time limit has arrived.
 */
enum {
  CEPTON_AGGREGATION_MODE_NATURAL = 0,
  CEPTON_AGGREGATION_MODE_FIXED_10Hz = 180000,
  // CEPTON_AGGREGATION_MODE_FIXED_20Hz = 50000,
};

/**
 * CeptonListenFrames
 * @return
 *  CEPTON_ERROR_INVALID_STATE if existing listeners are using a different
 *     aggregation mode.
 * NOTE: For FIXED modes, every frame is complete. For NATURAL mode, the first
 * frame right after the first listener is registered collects only points up to
 * the first frame boundary and should not be considered a complete frame.
 */
CEPTON_EXPORT int CeptonListenFrames(int aggregationMode,
                                     CeptonPointsCallback callback,
                                     void *user_data);

/**
 * CeptonUnlistenFrames
 *
 * NOTE: The last unlisten call will destroy all frame buffers and stop frame
 * aggregation altogether. After that aggregationMode can be changed.
 */
CEPTON_EXPORT int CeptonUnlistenFrames(CeptonPointsCallback callback,
                                       void *user_data);
//------------------------------------------------------------------------------
// Serial
//------------------------------------------------------------------------------
/// Callback for receiving serial data (e.g. NMEA).
/**
 * @param handle Sensor handle.
 * @param str Serial line string. Owned by SDK.
 */
typedef void (*CeptonSerialReceiveCallback)(CeptonSensorHandle handle,
                                            const char *str, void *user_data);

/// Sets serial line callback.
/**
 * Useful for listening to NMEA data from GPS attached to sensor.
 * Each callback contains 1 line of serial data (including newline characters).
 *
 * Returns error if callback already registered.
 *
 * @param cb Callback function
 */
CEPTON_EXPORT int CeptonListenSerialLines(CeptonSerialReceiveCallback callback,
                                          void *user_data);

/// Clears serial line callback.
CEPTON_EXPORT int CeptonUnlistenSerialLines(
    CeptonSerialReceiveCallback callback, void *user_data);

//------------------------------------------------------------------------------
// Sensor Info Packet
//------------------------------------------------------------------------------
typedef void (*CeptonSensorInfoCallback)(CeptonSensorHandle handle,
                                         const struct CeptonSensor *info,
                                         void *user_data);

/// Sets Sensor Info Packet callback.
/**
 * Useful for listening info packet coming from sensor.
 * @param cb Callback.
 */
CEPTON_EXPORT int CeptonListenSensorInfo(CeptonSensorInfoCallback callback,
                                         void *user_data);

/// Clears Sensor Info callback.
CEPTON_EXPORT int CeptonUnlistenSensorInfo(CeptonSensorInfoCallback callback,
                                           void *user_data);

//------------------------------------------------------------------------------
// Legacy support (experimental)
//------------------------------------------------------------------------------

/**
 * CeptonEnableLegacyTranslation() performs translation of legacy point cloud
 * to new format as a parser plugin. It loads the dynamic library of cepton_sdk
 * explicitly by symbols and starts a "parse only" legacy SDK by passing input
 * through cepton_sdk::cepton_sdk_mock_network_receive() and getting output
 * with a callback through cepton_sdk::cepton_sdk_listen_image_frames()
 *
 * Internally a map of handle to sensor is also maintained and updated whenever
 * new handles are detected, with
 * cepton_sdk::cepton_sdk_get_sensor_information()
 *
 * @return Error code
 *   CEPTON_ERROR_FILE_IO: Failed to find or load the legacy cepton_sdk library
 *   CEPTON_ERROR_INVALID_RESPONSE: Failed to load some method entries
 *
 *   cepton_sdk::cepton_sdk_initialize() return value if that fails
 *   cepton_sdk::cepton_sdk_listen_image_frames() return value if that fails
 *
 *   CEPTON_SUCCESS if everything works
 */
CEPTON_EXPORT int CeptonEnableLegacyTranslation();

/**
 * CeptonDisableLegacyTranslation() unloads cepton_sdk module dynamically. After
 * this call the library of the legacy cepton_sdk is no longer referenced.
 */
CEPTON_EXPORT int CeptonDisableLegacyTranslation();

//------------------------------------------------------------------------------
// Frame FIFO Support (experimental)
//
// Frame FIFO is a standalone facility that listens to frames and keep them in
// a pre-allocated FIFO. With a frame FIFO in place, the application doesn't
// need to register callbacks to process frames asynchronously anymore. Instead
// application can have a simple structure to get frames one at a time. This is
// particularly suitable for off-line processing of captured data.
//
// - Frame FIFO is standalone. Implemented with callbacks internally.
// - Frame FIFO works only with frames, not point streams
// - Frame FIFO queues frames from all connected sensors into a single FIFO.
//------------------------------------------------------------------------------

/**
 * CeptonEnableFrameFifo() enables a FIFO
 * @return Error code
 *  CEPTON_ERROR_INVALID_STATE: If frame FIFO is already enabled.
 *  CEPTON_ERROR_INVALID_STATE if existing listeners are using a different
 *     aggregation mode.
 */
CEPTON_EXPORT int CeptonEnableFrameFifo(int aggregationMode, int nFrames);

/**
 * CeptonDisableFrameFifo() disables the frame FIFO and discards all the
 * buffered frames already in FIFO.
 *
 * @return Error code
 *  CEPTON_ERROR_NOT_ENABLED: If the frame aggregation is not enabled, or
 *    if the current frame buffer is not released.
 */
CEPTON_EXPORT int CeptonDisableFrameFifo();

struct CeptonPointData {
  CeptonSensorHandle handle;
  int64_t start_timestamp;
  size_t n_points;
  size_t stride;
  const uint8_t *points;
};

/**
 * CeptonFrameFifoGet() Get the pointer to the frame in FIFO.
 * NOTE: This is only getting the pointer to the internal buffer. The buffers
 * are not copied. You must call CeptonFrameFifoRelease() to indicate to the
 * SDK the buffer is no longer needed. Before releasing the current buffer
 * a call to get the next buffer will fail. If there is no data to get in
 * certain amount of time, it will return a timeout error.
 * @param pPointData Pointer of a CeptonPointData instance.
 * @param maxWaitTime Maximum time to spend on waiting to get a frame before
 * returning an error code: CEPTON_ERROR_TIMEOUT. 0 will be interpreted as
 * waiting forever. Larger than 0 will be interpreted as the maximum waiting
 * time in millisecond.
 * @return Error code
 *   CEPTON_ERROR_NOT_ENABLED: If the frame FIFO is not enabled.
 *   CEPTON_ERROR_INVALID_STATE: If last frame is not released.
 *   CEPTON_ERROR_TIMEOUT: If get frame timeout.
 */
CEPTON_EXPORT int CeptonFrameFifoGet(struct CeptonPointData *pPointData,
                                     unsigned maxWaitTime);

/**
 * CeptonFrameFifoPutBack() will put a frame back to fifo after getting it.
 * @return Error code
 *   CEPTON_ERROR_NOT_ENABLED: If the frame FIFO is not enabled.
 *   CEPTON_ERROR_INVALID_STATE: If no frame buffer is acquired.
 */
CEPTON_EXPORT int CeptonFrameFifoPutBack();

/**
 * @return Error code
 *   CEPTON_ERROR_NOT_ENABLED: If the frame FIFO is not enabled.
 *   CEPTON_ERROR_INVALID_STATE: If no frame buffer is acquired.
 */
CEPTON_EXPORT int CeptonFrameFifoRelease();

/**
 * @return 1 if FIFO is empty. 0 if not empty. Negative for error
 *   CEPTON_ERROR_NOT_ENABLED: If frame FIFO is not enabled.
 */
CEPTON_EXPORT int CeptonFrameFifoEmpty();

/**
 * @return Larger or equal to 0 for fifo usage. Negative for error
 *   CEPTON_ERROR_NOT_ENABLED: If frame FIFO is not enabled.
 */
CEPTON_EXPORT int CeptonFrameFifoSize();

/**
 * @return 1 if FIFO is full. 0 if not full. Negative for error
 *   CEPTON_ERROR_NOT_ENABLED: If frame FIFO is not enabled.
 */
CEPTON_EXPORT int CeptonFrameFifoFull();

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // CEPTON_SDK2_H
