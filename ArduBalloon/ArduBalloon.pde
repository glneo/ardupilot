/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
 *       Flight data logging for Purdue Balloon Team
 *       By Andrew F. Davis
 */

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_Param.h>
#include <Filter.h>
#include <AP_Math.h>
#include <AP_ADC.h>
#include <AP_Baro.h>
#include <AP_InertialSensor.h>
#include <AP_GPS.h>
#include <GCS_MAVLink.h>
#include <AP_Notify.h>
#include <DataFlash.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Terrain.h>
#include <AP_Mission.h>
#include <AP_Relay.h>
#include <StorageManager.h>

#include "SerialFormat.h"

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

/* Declare instances of our sensors */
static AP_Baro_MS5611       baro(&AP_Baro_MS5611::spi);
static AP_GPS               gps;
AP_InertialSensor_MPU6000   ins;
static AP_Compass_HMC5843   compass;
static AP_Relay             relay;

void setup()
{
    /* Initialize sensors */
    baro.init();
    baro.calibrate();
    compass.init();
    relay.init();
    hal.uartB->begin(38400, 256, 16); // GPS uart baud 38400
    //gps.init(hal.uartB, AP_GPS::GPS_ENGINE_AIRBORNE_4G);
    ins.init(AP_InertialSensor::COLD_START, AP_InertialSensor::RATE_50HZ);
    ins.init_accel();
    hal.uartC->begin(115200, 256, 16); // logging uart baud 115200
}

void loop()
{
    hal.scheduler->delay(10); // Recored data at 1Hz
//    struct serial_struct serial_packet;
    /* Update sensors */
    baro.read();
    gps.update();
    ins.update();
    compass.read();
    const Vector3f &gyro = ins.get_gyro(0);
    const Vector3f &accel = ins.get_accel(0);
    const Vector3f &mag = compass.get_field();
    const Location &loc = gps.location();
    /* Send sensor values to logging uart */
    hal.uartC->printf( "%lu %f %f %ld %ld %ld %lu %ld %d %lu %f %f %f %f %f %f %f %f %f\n",
                            hal.scheduler->millis(), (double)baro.get_pressure(),
                            (double)baro.get_altitude(), loc.lat,
                            loc.lng, loc.alt,
                            gps.ground_speed_cm(), gps.ground_course_cd(),
                            gps.num_sats(), gps.time_week_ms(),
                            (double)accel.x, (double)accel.y, (double)accel.z,
                            (double)gyro.x, (double)gyro.y, (double)gyro.z,
                            (double)mag.x, (double)mag.y, (double)mag.z);
    /* Send sensor values to radio */
//    serial_packet.time_stamp = hal.scheduler->millis();
//    serial_packet.pressure = baro.get_pressure();
//    serial_packet.alt = loc.alt;			///< param 2 - Altitude in centimeters (meters * 100)
//    serial_packet.lat = loc.lat;			///< param 3 - Lattitude * 10**7
//    serial_packet.lng = loc.lng;			///< param 4 - Longitude * 10**7
//    serial_packet.ground_speed_cm = gps.ground_speed_cm();
//    serial_packet.ground_course_cd = gps.ground_course_cd();
//    serial_packet.num_sats = gps.num_sats();
//    serial_packet.time_week_ms = gps.time_week_ms();
//    serial_packet.accel_x = accel.x;
//    serial_packet.accel_y = accel.y;
//    serial_packet.accel_z = accel.z;
//    serial_packet.gyro_x = gyro.x;
//    serial_packet.gyro_y = gyro.y;
//    serial_packet.gyro_z = gyro.z;
//    serial_packet.mag_x = mag.x;
//    serial_packet.mag_y = mag.y;
//    serial_packet.mag_z = mag.z;
//    serial_packet.crc = 42; // FIXME: Do real crc
    
//    hal.uartA->write( (uint8_t *)&serial_packet, sizeof( struct serial_struct ) );
//    uint8_t buf[] = "\n";
//    hal.uartA->write( buf, 1 );
}

AP_HAL_MAIN();
