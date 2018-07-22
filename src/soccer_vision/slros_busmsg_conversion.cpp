#include "slros_busmsg_conversion.h"

<<<<<<< HEAD
=======

// Conversions between SL_Bus_soccer_vision_ros_time_Time and ros::Time

void convertFromBus(ros::Time* msgPtr, SL_Bus_soccer_vision_ros_time_Time const* busPtr)
{
  const std::string rosMessageType("ros_time/Time");

  msgPtr->sec =  busPtr->Sec;
  msgPtr->nsec =  busPtr->Nsec;
}

void convertToBus(SL_Bus_soccer_vision_ros_time_Time* busPtr, ros::Time const* msgPtr)
{
  const std::string rosMessageType("ros_time/Time");

  busPtr->Sec =  msgPtr->sec;
  busPtr->Nsec =  msgPtr->nsec;
}


// Conversions between SL_Bus_soccer_vision_sensor_msgs_Image and sensor_msgs::Image

void convertFromBus(sensor_msgs::Image* msgPtr, SL_Bus_soccer_vision_sensor_msgs_Image const* busPtr)
{
  const std::string rosMessageType("sensor_msgs/Image");

  convertFromBusVariablePrimitiveArray(msgPtr->data, busPtr->Data, busPtr->Data_SL_Info);
  convertFromBusVariablePrimitiveArray(msgPtr->encoding, busPtr->Encoding, busPtr->Encoding_SL_Info);
  convertFromBus(&msgPtr->header, &busPtr->Header);
  msgPtr->height =  busPtr->Height;
  msgPtr->is_bigendian =  busPtr->IsBigendian;
  msgPtr->step =  busPtr->Step;
  msgPtr->width =  busPtr->Width;
}

void convertToBus(SL_Bus_soccer_vision_sensor_msgs_Image* busPtr, sensor_msgs::Image const* msgPtr)
{
  const std::string rosMessageType("sensor_msgs/Image");

  convertToBusVariablePrimitiveArray(busPtr->Data, busPtr->Data_SL_Info, msgPtr->data, slros::EnabledWarning(rosMessageType, "data"));
  convertToBusVariablePrimitiveArray(busPtr->Encoding, busPtr->Encoding_SL_Info, msgPtr->encoding, slros::EnabledWarning(rosMessageType, "encoding"));
  convertToBus(&busPtr->Header, &msgPtr->header);
  busPtr->Height =  msgPtr->height;
  busPtr->IsBigendian =  msgPtr->is_bigendian;
  busPtr->Step =  msgPtr->step;
  busPtr->Width =  msgPtr->width;
}


// Conversions between SL_Bus_soccer_vision_std_msgs_Header and std_msgs::Header

void convertFromBus(std_msgs::Header* msgPtr, SL_Bus_soccer_vision_std_msgs_Header const* busPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertFromBusVariablePrimitiveArray(msgPtr->frame_id, busPtr->FrameId, busPtr->FrameId_SL_Info);
  msgPtr->seq =  busPtr->Seq;
  convertFromBus(&msgPtr->stamp, &busPtr->Stamp);
}

void convertToBus(SL_Bus_soccer_vision_std_msgs_Header* busPtr, std_msgs::Header const* msgPtr)
{
  const std::string rosMessageType("std_msgs/Header");

  convertToBusVariablePrimitiveArray(busPtr->FrameId, busPtr->FrameId_SL_Info, msgPtr->frame_id, slros::EnabledWarning(rosMessageType, "frame_id"));
  busPtr->Seq =  msgPtr->seq;
  convertToBus(&busPtr->Stamp, &msgPtr->stamp);
}

>>>>>>> f034160bf346457d529040a5ab836b42c9bd8806
