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
#include <AP_Mission.h>
#include <AP_Relay.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

/* Declare instances of our sensors */
static AP_Baro_MS5611       baro(&AP_Baro_MS5611::spi);
static AP_GPS               gps;
AP_InertialSensor_MPU6000   ins;
static AP_Compass_HMC5843 compass;
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
    hal.scheduler->delay(100); // Recored data at 10Hz
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
}

AP_HAL_MAIN();
