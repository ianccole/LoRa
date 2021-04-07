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
    GPSModule() : _state(gps_state::gps_off), _gpsFix(false)
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
                _gpsFix = true;

                switch(state())
                {
                    case GPSModule::gps_state::gps_on:
                        _state = gps_state::gps_on_first_fix;
                        _ttff = millis() - _start;
                        SerialUSB.print(_ttff);
                        SerialUSB.println(" TTFF");
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
        return _gpsFix;
    }

    void getFixStr(char* buffer)
    {
        if (buffer)
        {

            long lat, lon;
            unsigned long fix_age; // returns +- latitude/longitude in degrees
            unsigned long date, time;
            gps.get_position(&lat, &lon, &fix_age);
            gps.get_datetime(&date, &time, &fix_age);
            // SerialUSB.println(fix_age);
            if (fix_age == TinyGPS::GPS_INVALID_AGE)
            {
                SerialUSB.println("No fix detected");
            }
            else if (fix_age > 5000)
            {
                SerialUSB.println("Warning: possible stale data!");
            }
            else
            {
                sprintf(buffer, "Date: %lu, Time: %lu, LAT: %ld, LON: %ld\n", date, time, lat, lon);
                // SerialUSB.print(buffer);
            }
        }
    }

    void powerOn()
    {
        digitalWrite(PIN_GPS_POWER, GPS_ON);    // GPS power ON
        _state = gps_state::gps_on;
        _start = millis();
        _ttff = 0;
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

    unsigned long ttff()
    {
        return _ttff;
    }

    bool gpsFix()
    {
        return _gpsFix;
    }

private:
    gps_state _state;
    bool _gpsFix;
    unsigned long _start;
    unsigned long _ttff;

};
#else

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
    GPSModule() : _state(gps_state::gps_off), _gpsFix(false)
    {}

    void setup()
    {
    }

    bool checkGPS()
    {
        return _gpsFix;
    }

    void getFixStr(char* buffer)
    {
    }

    void powerOn()
    {
    }

    void powerOff()
    {
    }

    gps_state state()
    {
        return _state;
    }

    unsigned long ttff()
    {
        return _ttff;
    }

    bool gpsFix()
    {
        return _gpsFix;
    }

private:
    gps_state _state;
    bool _gpsFix;
    unsigned long _start;
    unsigned long _ttff;
};

#endif


static GPSModule gpsModule;

#endif