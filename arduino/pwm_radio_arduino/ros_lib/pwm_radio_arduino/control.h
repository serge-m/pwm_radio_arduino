#ifndef _ROS_pwm_radio_arduino_control_h
#define _ROS_pwm_radio_arduino_control_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pwm_radio_arduino
{

  class control : public ros::Msg
  {
    public:
      typedef int32_t _speed_control_usec_type;
      _speed_control_usec_type speed_control_usec;
      typedef int32_t _angle_control_usec_type;
      _angle_control_usec_type angle_control_usec;

    control():
      speed_control_usec(0),
      angle_control_usec(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_speed_control_usec;
      u_speed_control_usec.real = this->speed_control_usec;
      *(outbuffer + offset + 0) = (u_speed_control_usec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_speed_control_usec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_speed_control_usec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_speed_control_usec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->speed_control_usec);
      union {
        int32_t real;
        uint32_t base;
      } u_angle_control_usec;
      u_angle_control_usec.real = this->angle_control_usec;
      *(outbuffer + offset + 0) = (u_angle_control_usec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_angle_control_usec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_angle_control_usec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_angle_control_usec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->angle_control_usec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_speed_control_usec;
      u_speed_control_usec.base = 0;
      u_speed_control_usec.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_speed_control_usec.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_speed_control_usec.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_speed_control_usec.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->speed_control_usec = u_speed_control_usec.real;
      offset += sizeof(this->speed_control_usec);
      union {
        int32_t real;
        uint32_t base;
      } u_angle_control_usec;
      u_angle_control_usec.base = 0;
      u_angle_control_usec.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_angle_control_usec.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_angle_control_usec.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_angle_control_usec.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->angle_control_usec = u_angle_control_usec.real;
      offset += sizeof(this->angle_control_usec);
     return offset;
    }

    const char * getType(){ return "pwm_radio_arduino/control"; };
    const char * getMD5(){ return "c7f76211b6619c21d358e72e4ee393fb"; };

  };

}
#endif
