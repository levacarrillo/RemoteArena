#ifndef _ROS_SERVICE_getPose_h
#define _ROS_SERVICE_getPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Float32MultiArray.h"

namespace kick_test
{

static const char GETPOSE[] = "kick_test/getPose";

  class getPoseRequest : public ros::Msg
  {
    public:
      typedef const char* _kick_mode_type;
      _kick_mode_type kick_mode;
      typedef int64_t _robot_pose_type;
      _robot_pose_type robot_pose;

    getPoseRequest():
      kick_mode(""),
      robot_pose(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_kick_mode = strlen(this->kick_mode);
      varToArr(outbuffer + offset, length_kick_mode);
      offset += 4;
      memcpy(outbuffer + offset, this->kick_mode, length_kick_mode);
      offset += length_kick_mode;
      union {
        int64_t real;
        uint64_t base;
      } u_robot_pose;
      u_robot_pose.real = this->robot_pose;
      *(outbuffer + offset + 0) = (u_robot_pose.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_robot_pose.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_robot_pose.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_robot_pose.base >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (u_robot_pose.base >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (u_robot_pose.base >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (u_robot_pose.base >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (u_robot_pose.base >> (8 * 7)) & 0xFF;
      offset += sizeof(this->robot_pose);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_kick_mode;
      arrToVar(length_kick_mode, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_kick_mode; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_kick_mode-1]=0;
      this->kick_mode = (char *)(inbuffer + offset-1);
      offset += length_kick_mode;
      union {
        int64_t real;
        uint64_t base;
      } u_robot_pose;
      u_robot_pose.base = 0;
      u_robot_pose.base |= ((uint64_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_robot_pose.base |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_robot_pose.base |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_robot_pose.base |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      u_robot_pose.base |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      u_robot_pose.base |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      u_robot_pose.base |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      u_robot_pose.base |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      this->robot_pose = u_robot_pose.real;
      offset += sizeof(this->robot_pose);
     return offset;
    }

    const char * getType(){ return GETPOSE; };
    const char * getMD5(){ return "8e6ab8ce6d67b4496ca3e128af7cc109"; };

  };

  class getPoseResponse : public ros::Msg
  {
    public:
      typedef float _delay_type;
      _delay_type delay;
      typedef int32_t _number_poses_type;
      _number_poses_type number_poses;
      uint32_t joint_name_length;
      typedef char* _joint_name_type;
      _joint_name_type st_joint_name;
      _joint_name_type * joint_name;
      typedef std_msgs::Float32MultiArray _joint_goal_position_type;
      _joint_goal_position_type joint_goal_position;

    getPoseResponse():
      delay(0),
      number_poses(0),
      joint_name_length(0), joint_name(NULL),
      joint_goal_position()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_delay;
      u_delay.real = this->delay;
      *(outbuffer + offset + 0) = (u_delay.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_delay.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_delay.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_delay.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->delay);
      union {
        int32_t real;
        uint32_t base;
      } u_number_poses;
      u_number_poses.real = this->number_poses;
      *(outbuffer + offset + 0) = (u_number_poses.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_number_poses.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_number_poses.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_number_poses.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->number_poses);
      *(outbuffer + offset + 0) = (this->joint_name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->joint_name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->joint_name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->joint_name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->joint_name_length);
      for( uint32_t i = 0; i < joint_name_length; i++){
      uint32_t length_joint_namei = strlen(this->joint_name[i]);
      varToArr(outbuffer + offset, length_joint_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->joint_name[i], length_joint_namei);
      offset += length_joint_namei;
      }
      offset += this->joint_goal_position.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_delay;
      u_delay.base = 0;
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_delay.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->delay = u_delay.real;
      offset += sizeof(this->delay);
      union {
        int32_t real;
        uint32_t base;
      } u_number_poses;
      u_number_poses.base = 0;
      u_number_poses.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_number_poses.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_number_poses.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_number_poses.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->number_poses = u_number_poses.real;
      offset += sizeof(this->number_poses);
      uint32_t joint_name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      joint_name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->joint_name_length);
      if(joint_name_lengthT > joint_name_length)
        this->joint_name = (char**)realloc(this->joint_name, joint_name_lengthT * sizeof(char*));
      joint_name_length = joint_name_lengthT;
      for( uint32_t i = 0; i < joint_name_length; i++){
      uint32_t length_st_joint_name;
      arrToVar(length_st_joint_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_joint_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_joint_name-1]=0;
      this->st_joint_name = (char *)(inbuffer + offset-1);
      offset += length_st_joint_name;
        memcpy( &(this->joint_name[i]), &(this->st_joint_name), sizeof(char*));
      }
      offset += this->joint_goal_position.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETPOSE; };
    const char * getMD5(){ return "001a5e367375996299bc5e120ec01044"; };

  };

  class getPose {
    public:
    typedef getPoseRequest Request;
    typedef getPoseResponse Response;
  };

}
#endif
