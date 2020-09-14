#ifndef _ROS_pwm_radio_arduino_tripple_range_h
#define _ROS_pwm_radio_arduino_tripple_range_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pwm_radio_arduino
{

  class tripple_range : public ros::Msg
  {
    public:
      typedef float _low_type;
      _low_type low;
      typedef float _zero_type;
      _zero_type zero;
      typedef float _high_type;
      _high_type high;

    tripple_range():
      low(0),
      zero(0),
      high(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_low;
      u_low.real = this->low;
      *(outbuffer + offset + 0) = (u_low.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_low.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_low.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_low.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->low);
      union {
        float real;
        uint32_t base;
      } u_zero;
      u_zero.real = this->zero;
      *(outbuffer + offset + 0) = (u_zero.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_zero.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_zero.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_zero.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->zero);
      union {
        float real;
        uint32_t base;
      } u_high;
      u_high.real = this->high;
      *(outbuffer + offset + 0) = (u_high.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_high.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_high.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_high.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->high);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_low;
      u_low.base = 0;
      u_low.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_low.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_low.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_low.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->low = u_low.real;
      offset += sizeof(this->low);
      union {
        float real;
        uint32_t base;
      } u_zero;
      u_zero.base = 0;
      u_zero.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_zero.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_zero.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_zero.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->zero = u_zero.real;
      offset += sizeof(this->zero);
      union {
        float real;
        uint32_t base;
      } u_high;
      u_high.base = 0;
      u_high.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_high.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_high.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_high.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->high = u_high.real;
      offset += sizeof(this->high);
     return offset;
    }

    const char * getType(){ return "pwm_radio_arduino/tripple_range"; };
    const char * getMD5(){ return "5ec04308d6f41344d29549afb365c4a2"; };

  };

}
#endif
