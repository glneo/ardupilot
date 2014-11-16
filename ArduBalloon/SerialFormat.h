#include <stdint.h>

struct __attribute__((__packed__)) serial_struct
{
	uint32_t time_stamp;
	double pressure;
	int32_t alt;			///< param 2 - Altitude in centimeters (meters * 100)
	int32_t lat;			///< param 3 - Lattitude * 10**7
	int32_t lng;			///< param 4 - Longitude * 10**7
	uint32_t ground_speed_cm;
	int32_t ground_course_cd;
	uint8_t num_sats;
	uint32_t time_week_ms;
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
	float mag_x;
	float mag_y;
	float mag_z;
	uint8_t crc;
};
