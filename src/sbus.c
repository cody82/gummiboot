#include "sbus.h"


static uint8_t idx;
static uint8_t buffer[25];
uint16_t _channels[18];
uint32_t _lostFrames;
uint32_t frames;
uint8_t _failsafe = 1;

static uint8_t sbus_frame()
{
            if (buffer[0] != 0x0f || buffer[24] != 0)
                return 0;

frames++;

            _channels[0] = ((buffer[1] | buffer[2] << 8) & 0x07FF);
            _channels[1] = ((buffer[2] >> 3 | buffer[3] << 5) & 0x07FF);
            _channels[2] = ((buffer[3] >> 6 | buffer[4] << 2 | buffer[5] << 10) & 0x07FF);
            _channels[3] = ((buffer[5] >> 1 | buffer[6] << 7) & 0x07FF);
            _channels[4] = ((buffer[6] >> 4 | buffer[7] << 4) & 0x07FF);
            _channels[5] = ((buffer[7] >> 7 | buffer[8] << 1 | buffer[9] << 9) & 0x07FF);
            _channels[6] = ((buffer[9] >> 2 | buffer[10] << 6) & 0x07FF);
            _channels[7] = ((buffer[10] >> 5 | buffer[11] << 3) & 0x07FF);
            _channels[8] = ((buffer[12] | buffer[13] << 8) & 0x07FF);
            _channels[9] = ((buffer[13] >> 3 | buffer[14] << 5) & 0x07FF);
            _channels[10] = ((buffer[14] >> 6 | buffer[15] << 2 | buffer[16] << 10) & 0x07FF);
            _channels[11] = ((buffer[16] >> 1 | buffer[17] << 7) & 0x07FF);
            _channels[12] = ((buffer[17] >> 4 | buffer[18] << 4) & 0x07FF);
            _channels[13] = ((buffer[18] >> 7 | buffer[19] << 1 | buffer[20] << 9) & 0x07FF);
            _channels[14] = ((buffer[20] >> 2 | buffer[21] << 6) & 0x07FF);
            _channels[15] = ((buffer[21] >> 5 | buffer[22] << 3) & 0x07FF);

            //((buffer[23]) & 0x0001) ? _channels[16] = 2047 : _channels[16] = 0;
            //((buffer[23] >> 1) & 0x0001) ? _channels[17] = 2047 : _channels[17] = 0;
            if ((buffer[23] & 0x80)!=0) _channels[16] = 2047; else _channels[16] = 0;
            if ((buffer[23] & 0x40)!=0) _channels[17] = 2047; else _channels[17] = 0;


            if ((buffer[23] & 0x10)!=0)
            {
                _failsafe = 1;
            }
            else
            {
                _failsafe = 0;
            }

            if ((buffer[23] & 0x20)!=0)
            {
                _lostFrames++;
            }

    return 1;
}

uint8_t sbus_receive(uint8_t b)
{
    if (idx == 0)
    {
        if (b != 0x0f)
            return 0;
    }

    buffer[idx++] = b;

    if(idx == 25)
    {
        idx = 0;
        return sbus_frame();
    }

    return 0;
}
