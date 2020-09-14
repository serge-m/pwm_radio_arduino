#ifndef _ROS_pwm_radio_arduino_conversion_settings_h
#define _ROS_pwm_radio_arduino_conversion_settings_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "control_range.h"

namespace pwm_radio_arduino
{

  class conversion_settings : public ros::Msg
  {
    public:
      typedef pwm_radio_arduino::control_range _driver_in_type;
      _driver_in_type driver_in;
      typedef pwm_radio_arduino::control_range _radio_in_type;
      _radio_in_type radio_in;
      typedef pwm_radio_arduino::control_range _out_type;
      _out_type out;

    conversion_settings():
      driver_in(),
      radio_in(),
      out()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->driver_in.serialize(outbuffer + offset);
      offset += this->radio_in.serialize(outbuffer + offset);
      offset += this->out.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->driver_in.deserialize(inbuffer + offset);
      offset += this->radio_in.deserialize(inbuffer + offset);
      offset += this->out.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "pwm_radio_arduino/conversion_settings"; };
    const char * getMD5(){ return "a7617b6de5d646e533426d15b9b4b0aa"; };

  };

}
#endif
