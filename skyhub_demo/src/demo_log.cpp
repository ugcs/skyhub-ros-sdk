#include <ugcs_skyhub/processor_node.h>
#include <ugcs_skyhub/datalog_service.h>
#include <skyhub_demo/timer.hpp>

/**
 * Import full names of classes we are going to use:
 */

using ugcs_skyhub::payloads::Address;
using ugcs_skyhub::payloads::ProcessorNode;
using ugcs_skyhub::services::ParamSet;
using skyhub_demo::time::SkyhubTimer;
using ugcs_skyhub::datalogs::StandardFields;

using namespace std::chrono_literals;

namespace {

struct Params {
    constexpr static char
        INTERVAL[] = "INTERVAL_MS";
};

} /* anonymous namespace */


class DemoLog : public ProcessorNode
{

public:

  DemoLog()
  : ProcessorNode("demolog")
  {
  }

  std::string getType() const override
  {
    return "demolog";
  }

  std::string getModel() const override
  {
    return "test";
  }

  ParamSet getParameters()
  {
      return {
          {Params::INTERVAL, "1000"},
      };
  }

  virtual Address
  getPayloadMask() const override
  {
    return Address("v1:demolog:test:0");
  }

  virtual void configureResources(const Address& instance, const ugcs_skyhub::services::ParamSet& ps) override
  {
    //Check to be the first instance:
    if(instance.getInstance() != 0)
      return;
    m_interval = getParam<int>(ps, Params::INTERVAL);

    m_timer = std::make_unique<SkyhubTimer>(getRosNode(), std::chrono::milliseconds(m_interval),
      std::bind(&DemoLog::timer_callback, this));
  }

  virtual void
  activatePayload(const ugcs_skyhub::payloads::Address& payload) override
  {
    auto instance = payload.getInstance().value_or(0);
    if(instance != 0)
      return;
    //Connect to datalog:
    m_log = getDatalogService()->getPositionDataLog();
    //Add single custom field
    m_log->addField("Counter", "DEMO");
    //Add multiple custom fields
    m_log->addFields({"CounterNext", "CounterPrev"}, "DEMO");

    m_simple_log = getDatalogService()->getCsvDataLog("simple_demo");
    //Add two standard fields
    m_simple_log->addStandardFields({StandardFields::TS_TIME, StandardFields::VELOCITY});
    //Add Yaw value to "Heading" column
    m_simple_log->addStandardField(StandardFields::ATTITUDE_YAW, "Yaw");
    //Add single custom field
    m_simple_log->addFields({"Counter"});
    m_timer->start();
  }

private:
  SkyhubTimer::UniquePtr m_timer;
  int m_interval = 1000;
  ugcs_skyhub::datalogs::CsvDataLog::SharedPtr m_log;
  ugcs_skyhub::datalogs::CsvDataLog::SharedPtr m_simple_log;

  void
  timer_callback()
  {
    static int val = 0;
    //Send to Position log:
    std::vector<std::any> to_log;
    val++;
    //Add monotonic example value:
    to_log.push_back(val);
    m_simple_log->writeRecord(to_log);

    //Additional custom fields for position log:
    to_log.push_back(val+1);
    to_log.push_back(val-1);
    m_log->writeRecord(to_log);
    printf("Sent value: %d\n", val);
  }
};

int main(int argc, char * argv[])
{
  ugcs_skyhub::sdkInit(argc, argv);
  DemoLog node;
  node.execute();
  ugcs_skyhub::sdkRelease();
  return 0;

}


