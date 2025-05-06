/**
 * This node implements Payload to merge position data from Autopilot, provided by Skyhub Core
 * and GNSS coordinates provided by GNSS demo driver. All these data aligned at time line and stored
 * into configured CSV log.
 */

/**
 * This node based on ProcessorNode interface. It do no operate with specific hardware, instead it
 * knows how to process data from other sources.
 */
#include <ugcs_skyhub/processor_node.h>
/**
 * This file is provided by sample GNSS receiver and contains topics definitions specific for this
 * payload type
 */
#include <skyhub_demo/gnss_topics.h>
/**
 * This file contains topics defined for autopilot, as provided by Skyhub Core and supplied as
 * a part of Skyhub SDK
 */
#include <ugcs_skyhub/autopilot_topics.h>
/**
 * We use Payload interface to work with remote PayloadNodes: GNSS driver and Autopilot
 */
#include <ugcs_skyhub/payload.h>
/**
* Datalog services used to store result data in pre-configured log file
*/
#include <ugcs_skyhub/datalog_service.h>

/**
 * Import full names of classes we are going to use:
 */
using ugcs_skyhub::topics::autopilot::AttitudeTopic;
using skyhub_demo::topics::GnssCoordinatesTopic;
using ugcs_skyhub::payloads::Address;
using ugcs_skyhub::payloads::ProcessorNode;
using ugcs_skyhub::payloads::Payload;

using ugcs_skyhub::services::ParamSet;
using ugcs_skyhub::datalogs::DataLog;

/**
 * Main class which implements application logic to align data.
 * It's based on ProcessorNode interface:
 */
class Observer : public ProcessorNode
{
public:
  /**
  * Primary constructor does nothing, but sets default node name for ROS node.
  * This is how the node will be listed by "ros2 node list" command
  */
  Observer()
  : ProcessorNode("observer")
  {
  }
  /**
   * Every node defines payload type it supports. This name is used by configuration
   * storage and ConfigurationService::findPayload method to search for specific nodes. Supported type should
   * always be documented and available for driver's user.
   */
  std::string getType() const override
  {
    return "observer";
  }
  /**
   * The default payload model this node supports. It's possible to implement node
   * which may work with any or several models. This method defines default value if model
   * specification omitted. For example, if no parameters are found in configuration for
   * this node, default parameter set will be stored for instance 0 of "getType" type and
   * "getModel" model.
   */
  std::string getModel() const override
  {
    return "demo";
  }
  /**
   * %Payload mask defines which parameter sets will be retrieved from configuration storage
   * to try to detect payloads. For this example, sample driver will support only one (0-indexed)
   * instance.
   */
  virtual Address
  getPayloadMask() const override
  {
    return Address("v1:observer:demo:0");
  }

  /**
   * On first launch there is no stored parameters for this driver. We need one CSV datalog to
   * store merged data, so demo processor has one parameter: log name.
   * It's assumed datalog resource was configured manually before node starts working.
   */
  ParamSet getParameters()
  {
    std::string log = getConfigService()->addDataLog();
    ParamSet ps;
    ps[DATALOG_NAME] = log;
    return ps;
  }

  /**
   * At configuration stage we have to claim resources to work with. These resources are links
   * to other payloads (Autopilot and GNSS receiver) and two services.
   */
  virtual void configureResources(const Address& instance, const ugcs_skyhub::services::ParamSet& ps) override
  {
    //Check to be the first instance:
    if(instance.getInstance() != 0)
      return;

    /**
     * Make link to autopilot (Flight Controller) payload. This payload is provided by Skyhub Core and supported
     * by gateway node of Skyhub SDK.
     */
    m_fc = getConfigService()->findPayload(Address("v1:autopilot::0"));
    /**
     * Make link to the single instance of GNSS demo driver from this tutorial.
     */
    m_gnss = getConfigService()->findPayload(Address("v1:gnss:demo:0"));

    /**
     * Search for datalog name in parameters. We expect it was configured at "register payload" step.
     */
    std::string log_name = getParam<std::string>(ps, DATALOG_NAME);

    /**
     * Get instance of named log. This log is manager by DataLog Service and accessible as pre-configured
     * resource by it's name
     */
    m_log = getDatalogService()->getCsvDataLog(log_name);

    /**
     * Create shared structure for data of two sources:
     */
    m_currentAngles = std::make_shared<CurrentAngles>();
    /**
     * Create attitude event handler. It will save angles into shared structure
     */
    auto storeAngles = [angles = m_currentAngles](const AttitudeTopic::MessageType& msg) -> void
    {
      //enter critical section
      std::lock_guard<std::mutex> guard(angles->protector);
      //Update angle values:
      angles->pitch = msg.pitch;
      angles->roll = msg.roll;
      angles->yaw = msg.yaw;
    };
    /**
     * Create topic to get attutude data from Flight Controller:
     */
    m_attitudeTopic = m_fc->createTopic<AttitudeTopic>();
    //Register handler for topic:
    m_attitudeTopic->subscribe(storeAngles);

    /**
     * Do the same for GNSS topic: create instance with data receiving handler.
     * Received coordinates are merged with angles stored in m_current Angles and
     * sent to datalog
     */
    auto mergeData = [angles = m_currentAngles, log = m_log](const GnssCoordinatesTopic::MessageType& msg) -> void
    {
      //enter critical section
      std::lock_guard<std::mutex> guard(angles->protector);
      //make record of arbitrary fields:
      std::vector<std::any> record;
      //Put GNSS coordinates into it:
      record.push_back(msg.latitude);
      record.push_back(msg.longitude);
      record.push_back(angles->pitch);
      record.push_back(angles->roll);
      record.push_back(angles->yaw);
      //Send to log
      log->writeRecord(record);
    };
    m_coordTopic = m_gnss->createTopic<GnssCoordinatesTopic>();
    //and register it with payload
    m_coordTopic->subscribe(mergeData);
  }

  /**
   * Use this structure to store all three angles together
   */
  struct CurrentAngles
  {
      //Angle values:
      float pitch = 0;
      float roll = 0;
      float yaw = 0;
      //To guard parallel access:
      std::mutex protector;
  };

private:

  //Here is Datalog link:
  std::shared_ptr<DataLog> m_log;
  //Here is flight controller payload:
  std::shared_ptr<Payload> m_fc;
  //Here is GNSS demo driver:
  std::shared_ptr<Payload> m_gnss;
  //Shared storage for current Euler angle values:
  std::shared_ptr<CurrentAngles> m_currentAngles;
  //Here is attitude data topic:
  AttitudeTopic::SharedPtr m_attitudeTopic;
  //Here is GNSS data topic:
  GnssCoordinatesTopic::SharedPtr m_coordTopic;

  //Parameter name Observer expects from configuration. This is name of pre-configured datalog resource.
  static constexpr const char* DATALOG_NAME = "LOG_NAME";
};

/**
 * Entry point of node binary:
 */
int main(int argc, char * argv[])
{
  /**
   * Initialize SDK:
   */
  ugcs_skyhub::sdkInit(argc, argv);

  /**
   * Make instance of observer node:
   */
  Observer observer;
  RCLCPP_INFO(observer.getRosNode().get_logger(), "Observer started");

  /**
   * Launch ROS2 event loop for this node. This is blocking method, until node will be shutdown.
   */
  observer.execute();
  /**
   * Cleanup SDK resources:
   */
  ugcs_skyhub::sdkRelease();
  return 0;
}
