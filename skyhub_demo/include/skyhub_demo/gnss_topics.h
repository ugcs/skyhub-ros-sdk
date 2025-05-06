#pragma once

#include <ugcs_skyhub/topics.hpp>
#include <skyhub_msgs/msg/gnss_coordinates.hpp>

///Namespace of SDK example
namespace skyhub_demo
{
  namespace topics ///Topic defined by SDK example GNSS demo driver
  {
    static const char TOPIC_GNSS_COORDINATES[] = "GnssCoordinatesDemo";

    using GnssCoordinatesTopic =
      ugcs_skyhub::topics::TopicBase_<skyhub_msgs::msg::GnssCoordinates, TOPIC_GNSS_COORDINATES>;
  }
}
