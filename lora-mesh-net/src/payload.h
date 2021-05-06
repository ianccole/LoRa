#include <Arduino.h>

class Payload
{

public:
    Payload(uint8_t *buffer, uint8_t size) : _buffer(buffer), _maxsize(size)
    {
        _cursor = 0;
    }

    uint32_t getValue32(uint8_t &channel, uint8_t &type, uint8_t &size)
    {
        channel = _buffer[_cursor++];
        type = _buffer[_cursor++];
        size = _buffer[_cursor++];

        uint32_t value = 0;
        for (uint8_t ii=0; ii<size; ii++) 
        {
            value = (value << 8) + _buffer[_cursor];
            _cursor++;
        }

        return value;
    }

    uint8_t addValue32(uint8_t channel, uint8_t type, uint8_t size, uint32_t value) 
    {
        _buffer[_cursor++] = channel;
        _buffer[_cursor++] = type;
        _buffer[_cursor++] = size;

        for (uint8_t i=1; i<=size; i++) 
        {
            _buffer[_cursor + size - i] = (value & 0xFF);
            value >>= 8;
        }
        _cursor += size;

        return _cursor;
    }

    uint8_t getSize(void) 
    {
        return _cursor;
    }

protected:
    uint8_t *_buffer;
    uint8_t _maxsize;
    uint8_t _cursor;

};
