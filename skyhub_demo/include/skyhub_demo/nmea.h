#pragma once

#include <skyhub_demo/gnss_topics.h>
#include <memory>

namespace skyhub_demo
{
class NmeaProcessor
{
  public:
    NmeaProcessor();

    void processNmeaData(const std::vector<char>& data, const int length, const rclcpp::Time& ts);
    void processNmeaField(const int msgType, const int fieldNum, const std::vector<char>& data) const;

    std::shared_ptr<topics::GnssCoordinatesTopic::MessageType> data_sample;
    bool ds_coords_ready;
    bool ds_quality_ready;

  private:
    enum Stage {Idle, Address, Data, CheckSum, CR, LF};
    enum NmeaMessageTypes {RMC=1, GGA=2};
};
}
