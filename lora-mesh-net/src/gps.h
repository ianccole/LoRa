#ifndef GPS_L80_h
#define GPS_L80_h

#include <Arduino.h>

#if defined(NODE_HAVE_GPS)
#include "TinyGPS.h"
TinyGPS gps;

class GPSModule
{
public:
    enum class gps_state
    {
        gps_on,
        gps_on_first_fix,
        gps_on_fix,
        gps_off
    };

    /// Constructor.
    GPSModule() : 
        ttff(0),
        gpsFix(false),
        lat(0),
        lon(0),
        fix_age(0),
        date(0),
        time(0),
        _state(gps_state::gps_off)
        
    {}

    void setup()
    {
        Serial1.begin(9600);
        pinMode(PIN_GPS_POWER, OUTPUT);
        powerOn();
    }

    bool checkGPS()
    {
        // _gpsFix = false;
        while (Serial1.available())
        {
            char ch;
            ch = Serial1.read();
            // SerialUSB.print(ch);

            if (gps.encode(ch))
            {
                gpsFix = true;

                switch(state())
                {
                    case GPSModule::gps_state::gps_on:
                        _state = gps_state::gps_on_first_fix;
                        ttff = millis() - _start;
                        SerialUSB.print(ttff);
                        SerialUSB.println(F(" TTFF"));
                        break;

                    case GPSModule::gps_state::gps_on_first_fix:
                        _state = gps_state::gps_on_fix;
                        break;

                    case GPSModule::gps_state::gps_off:
                    case GPSModule::gps_state::gps_on_fix:
                        break;
                }
            }
        }
        return gpsFix;
    }

    void getFixStr(char* buffer)
    {
        if (buffer)
        {
            gps.get_position(&lat, &lon, &fix_age);
            gps.get_datetime(&date, &time, &fix_age);
            // SerialUSB.println(fix_age);

            if (fix_age == TinyGPS::GPS_INVALID_AGE)
            {
                SerialUSB.println(F("No fix detected"));
            }
            else if (fix_age > 5000)
            {
                SerialUSB.println(F("Warning: possible stale data!"));
            }
            else
            {
                sprintf(buffer, "Date: %lu, Time: %lu, LAT: %ld, LON: %ld\n", date, time, lat, lon);
                // SerialUSB.print(buffer);
            }
        }
    }

    void getPosition(long *latitude, long *longitude, unsigned long *fix_age)
    {
        gps.get_position(latitude, longitude, fix_age);
    }

    void getPosition(float *latitude, float *longitude, unsigned long *fix_age)
    {
        gps.f_get_position(latitude, longitude, fix_age);
    }

    int32_t getAltitude()
    {
        return gps.altitude();
    }

    int32_t getLatitude()
    {
        return gps.latitude();
    }

    int32_t getLongitude()
    {
        return gps.longitude();
    }

    void getDateTime(uint32_t *date, uint32_t *time, uint32_t *time_age)
    {
        gps.get_datetime(date, time, time_age);
    }

    void powerOn()
    {
        ttff = 0;
        gpsFix = false;
        digitalWrite(PIN_GPS_POWER, GPS_ON);    // GPS power ON
        _state = gps_state::gps_on;
        _start = millis();
    }

    void powerOff()
    {
        digitalWrite(PIN_GPS_POWER, GPS_OFF);   // GPS power OFF
        _state = gps_state::gps_off;
    }

    gps_state state()
    {
        return _state;
    }

    unsigned long ttff;
    bool gpsFix;
    long lat;
    long lon;
    unsigned long fix_age;
    unsigned long date;
    unsigned long time;

private:
    gps_state _state;
    unsigned long _start;

};

static GPSModule gpsModule;

#endif


#endif