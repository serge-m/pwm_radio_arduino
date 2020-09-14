#ifndef _ROS_pwm_radio_arduino_control_range_h
#define _ROS_pwm_radio_arduino_control_range_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "tripple_range.h"

namespace pwm_radio_arduino
{

  class control_range : public ros::Msg
  {
    public:
      typedef pwm_radio_arduino::tripple_range _angle_type;
      _angle_type angle;
      typedef pwm_radio_arduino::tripple_range _speed_type;
      _speed_type speed;

    control_range():
      angle(),
      speed()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->angle.serialize(outbuffer + offset);
      offset += this->speed.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->angle.deserialize(inbuffer + offset);
      offset += this->speed.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "pwm_radio_arduino/control_range"; };
    const char * getMD5(){ return "2c649efb10444b8c3e78c519d039751c"; };

  };

}
#endif
