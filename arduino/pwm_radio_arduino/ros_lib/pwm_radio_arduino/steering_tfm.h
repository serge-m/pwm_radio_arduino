#ifndef _ROS_pwm_radio_arduino_steering_tfm_h
#define _ROS_pwm_radio_arduino_steering_tfm_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pwm_radio_arduino
{

  class steering_tfm : public ros::Msg
  {
    public:
      typedef float _angle_in_low_type;
      _angle_in_low_type angle_in_low;
      typedef float _angle_in_zero_type;
      _angle_in_zero_type angle_in_zero;
      typedef float _angle_in_high_type;
      _angle_in_high_type angle_in_high;
      typedef float _angle_out_low_type;
      _angle_out_low_type angle_out_low;
      typedef float _angle_out_zero_type;
      _angle_out_zero_type angle_out_zero;
      typedef float _angle_out_high_type;
      _angle_out_high_type angle_out_high;
      typedef float _speed_in_low_type;
      _speed_in_low_type speed_in_low;
      typedef float _speed_in_zero_type;
      _speed_in_zero_type speed_in_zero;
      typedef float _speed_in_high_type;
      _speed_in_high_type speed_in_high;
      typedef float _speed_out_low_type;
      _speed_out_low_type speed_out_low;
      typedef float _speed_out_zero_type;
      _speed_out_zero_type speed_out_zero;
      typedef float _speed_out_high_type;
      _speed_out_high_type speed_out_high;

    steering_tfm():
      angle_in_low(0),
      angle_in_zero(0),
      angle_in_high(0),
      angle_out_low(0),
      angle_out_zero(0),
      angle_out_high(0),
      speed_in_low(0),
      speed_in_zero(0),
      speed_in_high(0),
      speed_out_low(0),
      speed_out_zero(0),
      speed_out_high(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angle_in_low;
      u_angle_in_low.real = this->angle_in_low;
      *(outbuffer + offset + 0) = (u_angle_in_low.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_in_low.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_in_low.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_in_low.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_in_low);
      union {
        float real;
        uint32_t base;
      } u_angle_in_zero;
      u_angle_in_zero.real = this->angle_in_zero;
      *(outbuffer + offset + 0) = (u_angle_in_zero.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_in_zero.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_in_zero.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_in_zero.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_in_zero);
      union {
        float real;
        uint32_t base;
      } u_angle_in_high;
      u_angle_in_high.real = this->angle_in_high;
      *(outbuffer + offset + 0) = (u_angle_in_high.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_in_high.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_in_high.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_in_high.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_in_high);
      union {
        float real;
        uint32_t base;
      } u_angle_out_low;
      u_angle_out_low.real = this->angle_out_low;
      *(outbuffer + offset + 0) = (u_angle_out_low.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_out_low.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_out_low.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_out_low.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_out_low);
      union {
        float real;
        uint32_t base;
      } u_angle_out_zero;
      u_angle_out_zero.real = this->angle_out_zero;
      *(outbuffer + offset + 0) = (u_angle_out_zero.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_out_zero.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_out_zero.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_out_zero.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_out_zero);
      union {
        float real;
        uint32_t base;
      } u_angle_out_high;
      u_angle_out_high.real = this->angle_out_high;
      *(outbuffer + offset + 0) = (u_angle_out_high.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_out_high.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_out_high.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_out_high.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_out_high);
      union {
        float real;
        uint32_t base;
      } u_speed_in_low;
      u_speed_in_low.real = this->speed_in_low;
      *(outbuffer + offset + 0) = (u_speed_in_low.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_in_low.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_in_low.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_in_low.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_in_low);
      union {
        float real;
        uint32_t base;
      } u_speed_in_zero;
      u_speed_in_zero.real = this->speed_in_zero;
      *(outbuffer + offset + 0) = (u_speed_in_zero.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_in_zero.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_in_zero.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_in_zero.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_in_zero);
      union {
        float real;
        uint32_t base;
      } u_speed_in_high;
      u_speed_in_high.real = this->speed_in_high;
      *(outbuffer + offset + 0) = (u_speed_in_high.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_in_high.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_in_high.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_in_high.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_in_high);
      union {
        float real;
        uint32_t base;
      } u_speed_out_low;
      u_speed_out_low.real = this->speed_out_low;
      *(outbuffer + offset + 0) = (u_speed_out_low.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_out_low.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_out_low.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_out_low.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_out_low);
      union {
        float real;
        uint32_t base;
      } u_speed_out_zero;
      u_speed_out_zero.real = this->speed_out_zero;
      *(outbuffer + offset + 0) = (u_speed_out_zero.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_out_zero.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_out_zero.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_out_zero.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_out_zero);
      union {
        float real;
        uint32_t base;
      } u_speed_out_high;
      u_speed_out_high.real = this->speed_out_high;
      *(outbuffer + offset + 0) = (u_speed_out_high.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_out_high.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_out_high.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_out_high.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_out_high);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_angle_in_low;
      u_angle_in_low.base = 0;
      u_angle_in_low.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_in_low.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_in_low.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_in_low.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_in_low = u_angle_in_low.real;
      offset += sizeof(this->angle_in_low);
      union {
        float real;
        uint32_t base;
      } u_angle_in_zero;
      u_angle_in_zero.base = 0;
      u_angle_in_zero.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_in_zero.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_in_zero.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_in_zero.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_in_zero = u_angle_in_zero.real;
      offset += sizeof(this->angle_in_zero);
      union {
        float real;
        uint32_t base;
      } u_angle_in_high;
      u_angle_in_high.base = 0;
      u_angle_in_high.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_in_high.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_in_high.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_in_high.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_in_high = u_angle_in_high.real;
      offset += sizeof(this->angle_in_high);
      union {
        float real;
        uint32_t base;
      } u_angle_out_low;
      u_angle_out_low.base = 0;
      u_angle_out_low.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_out_low.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_out_low.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_out_low.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_out_low = u_angle_out_low.real;
      offset += sizeof(this->angle_out_low);
      union {
        float real;
        uint32_t base;
      } u_angle_out_zero;
      u_angle_out_zero.base = 0;
      u_angle_out_zero.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_out_zero.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_out_zero.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_out_zero.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_out_zero = u_angle_out_zero.real;
      offset += sizeof(this->angle_out_zero);
      union {
        float real;
        uint32_t base;
      } u_angle_out_high;
      u_angle_out_high.base = 0;
      u_angle_out_high.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_out_high.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_out_high.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_out_high.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_out_high = u_angle_out_high.real;
      offset += sizeof(this->angle_out_high);
      union {
        float real;
        uint32_t base;
      } u_speed_in_low;
      u_speed_in_low.base = 0;
      u_speed_in_low.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_in_low.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_in_low.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_in_low.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_in_low = u_speed_in_low.real;
      offset += sizeof(this->speed_in_low);
      union {
        float real;
        uint32_t base;
      } u_speed_in_zero;
      u_speed_in_zero.base = 0;
      u_speed_in_zero.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_in_zero.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_in_zero.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_in_zero.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_in_zero = u_speed_in_zero.real;
      offset += sizeof(this->speed_in_zero);
      union {
        float real;
        uint32_t base;
      } u_speed_in_high;
      u_speed_in_high.base = 0;
      u_speed_in_high.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_in_high.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_in_high.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_in_high.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_in_high = u_speed_in_high.real;
      offset += sizeof(this->speed_in_high);
      union {
        float real;
        uint32_t base;
      } u_speed_out_low;
      u_speed_out_low.base = 0;
      u_speed_out_low.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_out_low.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_out_low.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_out_low.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_out_low = u_speed_out_low.real;
      offset += sizeof(this->speed_out_low);
      union {
        float real;
        uint32_t base;
      } u_speed_out_zero;
      u_speed_out_zero.base = 0;
      u_speed_out_zero.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_out_zero.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_out_zero.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_out_zero.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_out_zero = u_speed_out_zero.real;
      offset += sizeof(this->speed_out_zero);
      union {
        float real;
        uint32_t base;
      } u_speed_out_high;
      u_speed_out_high.base = 0;
      u_speed_out_high.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_out_high.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_out_high.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_out_high.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_out_high = u_speed_out_high.real;
      offset += sizeof(this->speed_out_high);
     return offset;
    }

    const char * getType(){ return "pwm_radio_arduino/steering_tfm"; };
    const char * getMD5(){ return "bcf494045d25c6e3f0b2e6e226af87b3"; };

  };

}
#endif
