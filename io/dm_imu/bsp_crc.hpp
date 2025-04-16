#ifndef IO__BSP_CRC_HPP
#define IO__BSP_CRC_HPP
#include <iostream>

namespace io
{

uint8_t Get_CRC8(uint8_t init_value, uint8_t * ptr, uint8_t len);
uint16_t Get_CRC16(uint8_t * ptr, uint16_t len);

}  // namespace io

#endif