#ifndef _ROS_SERVICE_control_lights_h
#define _ROS_SERVICE_control_lights_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace arena_control
{

static const char CONTROL_LIGHTS[] = "arena_control/control_lights";

  class control_lightsRequest : public ros::Msg
  {
    public:
      int8_t light_states[2];

    control_lightsRequest():
      light_states()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_light_statesi;
      u_light_statesi.real = this->light_states[i];
      *(outbuffer + offset + 0) = (u_light_statesi.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->light_states[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      for( uint32_t i = 0; i < 2; i++){
      union {
        int8_t real;
        uint8_t base;
      } u_light_statesi;
      u_light_statesi.base = 0;
      u_light_statesi.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->light_states[i] = u_light_statesi.real;
      offset += sizeof(this->light_states[i]);
      }
     return offset;
    }

    virtual const char * getType() override { return CONTROL_LIGHTS; };
    virtual const char * getMD5() override { return "f8b46c4506ab8909b232348a2053af8e"; };

  };

  class control_lightsResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;

    control_lightsResponse():
      success(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
     return offset;
    }

    virtual const char * getType() override { return CONTROL_LIGHTS; };
    virtual const char * getMD5() override { return "358e233cde0c8a8bcfea4ce193f8fc15"; };

  };

  class control_lights {
    public:
    typedef control_lightsRequest Request;
    typedef control_lightsResponse Response;
  };

}
#endif
