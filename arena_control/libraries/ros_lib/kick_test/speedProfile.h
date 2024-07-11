#ifndef _ROS_SERVICE_speedProfile_h
#define _ROS_SERVICE_speedProfile_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32MultiArray.h"

namespace kick_test
{

static const char SPEEDPROFILE[] = "kick_test/speedProfile";

  class speedProfileRequest : public ros::Msg
  {
    public:
      typedef float _dt_type;
      _dt_type dt;
      typedef float _tf_type;
      _tf_type tf;
      typedef float _p0_type;
      _p0_type p0;
      typedef float _pf_type;
      _pf_type pf;

    speedProfileRequest():
      dt(0),
      tf(0),
      p0(0),
      pf(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_dt;
      u_dt.real = this->dt;
      *(outbuffer + offset + 0) = (u_dt.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_dt.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_dt.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_dt.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dt);
      union {
        float real;
        uint32_t base;
      } u_tf;
      u_tf.real = this->tf;
      *(outbuffer + offset + 0) = (u_tf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tf);
      union {
        float real;
        uint32_t base;
      } u_p0;
      u_p0.real = this->p0;
      *(outbuffer + offset + 0) = (u_p0.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_p0.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_p0.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_p0.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->p0);
      union {
        float real;
        uint32_t base;
      } u_pf;
      u_pf.real = this->pf;
      *(outbuffer + offset + 0) = (u_pf.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_pf.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_pf.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_pf.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->pf);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_dt;
      u_dt.base = 0;
      u_dt.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_dt.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_dt.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_dt.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->dt = u_dt.real;
      offset += sizeof(this->dt);
      union {
        float real;
        uint32_t base;
      } u_tf;
      u_tf.base = 0;
      u_tf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tf = u_tf.real;
      offset += sizeof(this->tf);
      union {
        float real;
        uint32_t base;
      } u_p0;
      u_p0.base = 0;
      u_p0.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_p0.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_p0.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_p0.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->p0 = u_p0.real;
      offset += sizeof(this->p0);
      union {
        float real;
        uint32_t base;
      } u_pf;
      u_pf.base = 0;
      u_pf.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_pf.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_pf.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_pf.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->pf = u_pf.real;
      offset += sizeof(this->pf);
     return offset;
    }

    const char * getType(){ return SPEEDPROFILE; };
    const char * getMD5(){ return "7118cdefd10ac9ba6a22559201a99722"; };

  };

  class speedProfileResponse : public ros::Msg
  {
    public:
      typedef std_msgs::Float32MultiArray _profiled_positions_type;
      _profiled_positions_type profiled_positions;

    speedProfileResponse():
      profiled_positions()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->profiled_positions.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->profiled_positions.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SPEEDPROFILE; };
    const char * getMD5(){ return "c9819f60bcf874e58dd00a24318e3006"; };

  };

  class speedProfile {
    public:
    typedef speedProfileRequest Request;
    typedef speedProfileResponse Response;
  };

}
#endif
