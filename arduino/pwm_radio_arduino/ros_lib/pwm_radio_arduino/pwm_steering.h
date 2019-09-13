#ifndef _ROS_pwm_radio_arduino_pwm_steering_h
#define _ROS_pwm_radio_arduino_pwm_steering_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pwm_radio_arduino
{

  class pwm_steering : public ros::Msg
  {
    public:
      typedef int32_t _steering_type;
      _steering_type steering;
      typedef int32_t _throttle_type;
      _throttle_type throttle;

    pwm_steering():
      steering(0),
      throttle(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_steering;
      u_steering.real = this->steering;
      *(outbuffer + offset + 0) = (u_steering.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_steering.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_steering.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_steering.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->steering);
      union {
        int32_t real;
        uint32_t base;
      } u_throttle;
      u_throttle.real = this->throttle;
      *(outbuffer + offset + 0) = (u_throttle.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_throttle.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_throttle.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_throttle.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->throttle);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_steering;
      u_steering.base = 0;
      u_steering.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_steering.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_steering.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_steering.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->steering = u_steering.real;
      offset += sizeof(this->steering);
      union {
        int32_t real;
        uint32_t base;
      } u_throttle;
      u_throttle.base = 0;
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_throttle.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->throttle = u_throttle.real;
      offset += sizeof(this->throttle);
     return offset;
    }

    const char * getType(){ return "pwm_radio_arduino/pwm_steering"; };
    const char * getMD5(){ return "c0f7d339bfa66e00b092d090a4518c5a"; };

  };

}
#endif
