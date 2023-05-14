Header header

uint64 handle
uint64 serial_number
string model_name
uint16 model
uint16 model_reserved

string firmware_version
            
int64 power_up_timestamp
int64 time_sync_offset
int64 time_sync_drift

uint8 return_count
uint8 segment_count

bool is_pps_connected
bool is_nmea_connected
bool is_ptp_connected

uint8[] data # `cepton_sdk::SensorInformation` bytes