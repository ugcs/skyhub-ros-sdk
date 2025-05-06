/**
 * Implementation of sample GNSS driver based on Skyhub SDK framework
 */

/**
 * This node is based in DriverNode class of SDK, since it devoted to interact with
 * hardware and send read data to any connected consumer. So, we include DriverNod
 * interface
 */
#include <ugcs_skyhub/driver_node.h>
/**
 * This node will provide it's own topic to publish GNSS coordinates. This topic
 * is defined in separate header to make it available to any consumer.
 */
#include <skyhub_demo/gnss_topics.h>
/**
 * GNSS receiver provides data in NMEA format. To parse NMEA messages we are
 * interested in, we include simple nmea parser. This parser is not a part of SDK
 * and included only for demonstration proposes.
 */
#include <skyhub_demo/nmea.h>

/**
 * Import full names of classes we are going to use:
 */
using ugcs_skyhub::payloads::DriverNode;
using ugcs_skyhub::payloads::Address;
using ugcs_skyhub::connections::UartConnection;

/**
 * To implement driver's port registration, we'll need Configuration Service,
 * so import services namespace.
 */
using namespace ugcs_skyhub::services;

/**
 * Place our classes to separate namespace
 */
namespace skyhub_demo
{

/**
 * Main class which implements GNSS driver. It's based on DriverNode interface:
 */
class GNSSdriver : public DriverNode
{
public:
  /**
   * Primay constructor does nothing, but sets default node name for ROS node.
   * This is how this node will be listed by "ros2 node list" command
   */
  GNSSdriver(): DriverNode("gnss_driver")
  {
  }

  /**
   * Every node defines payload type it supports. This name is used by configuration
   * storage and findPayload method to search for specific nodes. Supported type should
   * always be documented and available for driver's user.
   */
  std::string getType() const override
  {
    return "gnss";
  }

  /**
   * The default payload model this payload supports. It's possible to implement payload
   * which may work with any or several models, but this method defines default model if
   * specification ommited. For example, if no parameters are found in configuration for
   * this node, default parameter set will be stored for instance 0 of "getType" type and
   * "getModel" model.
   */
  std::string getModel() const override
  {
    return "demo";
  }

  /**
   * Payload mask defines which parameter sets will be retrived from configuration storage
   * to try to detect payloads. For this example, sample driver will support only one (0) instance
   * of GNSS receiver of model "demo". If consumers will search for any "gnss" drivers, this driver
   * may be used. To distinct this driver from any other, we define model as "demo" to let consumers
   * know what to choose.
   */
  Address
  getPayloadMask() const override
  {
    return Address("v1:gnss:demo:0");
  }

  /**
   * On first launch there is no stored parameters for this driver. We need one UART port for
   * communications with GNSS receiver, so demo driver has one parameter: port name
   */
  ParamSet getParameters() override
  {
    /*First, we ask Configuration service to add default UART port definition to configuration
    * On successful call serivce return port name. This name may be used with Communication
    * service to open pre-configured resources.
    */
    std::string uart_id = getConfigService()->addUartPort();

    //Parameter set for our driver
    ParamSet ps;
    /*Add received port name from configuration as a value of driver specific parameter:
    * name of UART device. This is not OS-specific name,  but ID of configured resource
    */
    ps["DEV_UART"] = uart_id;
    return ps;
  }

  /**
   * \brief This is the point there driver should claim the resources it will need to work.
   * \param instance Zero-based index of payload instance to configure
   * \param params Parameters set for this payload instance
   */
  void configureResources(const Address& instance, const ParamSet& params) override
  {
    //Again, we work only for single instance of this payload:
    if(instance.getInstance() != 0)
      return;

    //Obtain port name:
    std::string port = getParam<std::string>(params, "DEV_UART");

    /* Get UART connection object and store it to internal variable
     * Port is not opened at this step.
     */
    m_connect = getConnectionService()->connectUart(port,
            /*
             * We use default error handling for async reading, provided by Connection Service.
             * This routine will catch exceptions at reading thread and write them to log and
             * send to CPM.
             */
            std::bind(&ConnectionService::OnAsyncError, getConnectionService(), std::placeholders::_1));

    //Instance the topic class this driver implemented:
    m_gnss = createTopic<topics::GnssCoordinatesTopic>(instance);
    //Define this topic to be ready for publishing by this node:
    m_gnss->install();
  }

  /**
   * Then payload node reaches activation payload step it should begin data processing and
   * any other activities it's developed for. Every payload instance activated independently.
   */
  void activatePayload(const Address& instance) override
  {
    //Activate only the first instance:
    if(instance.getInstance() != 0)
      return;

    //Now, open the UART port there GNSS driver we detected:
    m_connect->connect();
    /**
     * Launch infinite reading loop. This call is non-blocking and always return as loop is
     * started on separate thread. This loop is supported by SDK implementation and user must
     * only to define what to do with obtained data. See the data processing implementation below.
     *
     * onData will get from 1 to READ_BUFFER_SIZE bytes of data to parse or to process in any other way.
     *
     * Developer may stop this loop by stopAsyncRead. Only one async loop is support for every opened
     * connection.
     */
    m_connect->startAsyncReadLoop(
          [this](std::vector<char> data, const size_t len){
            this->onData(data, len);
          },
          READ_BUFFER_SIZE);
  }

  /**
   * Then instance must be deactivated this method is called. Every instance must be deactivated
   * independent of other.
   */
  void deactivatePayload(const Address& instance) override
  {
    //Work for single instance only:
    if(instance.getInstance() != 0)
      return;
    /*
     * Stop infinite loop if it was started
     */
    m_connect->stopAsyncReadLoop();
    /*Just close any opened connections. Any asyncronous operations will be stoped automatically.
     * The finishing of running operations may take some time and this method will wait for their
     * compleation.
     */
    m_connect->disconnect();
  }

private:
  /**
   * This routine defines what to do with data we received while reading port.
   * The data parameter is a byte array of received data and the len is actual size of stored bytes.
   * len <= data.size() for always
   */
  void onData(std::vector<char> data, const size_t len)
  {
    //If any data was received send it to simple NMEA parser
    if(len > 0)
      nmea.processNmeaData(data, len, getRosNode().now());

    /*Just internal logic: NMEA processor parses to types of message: coordinates and their quality.
     * Then both message are available and up-to-date, we are ready to post them on topic
     */
    if(nmea.ds_coords_ready && nmea.ds_quality_ready) {
      nmea.ds_coords_ready = false;
      nmea.ds_quality_ready = false;
      /**
       * Access configured topic of "GnssCoordinates" type for instance 0 and post parsed data to it:
       */
      m_gnss->publish(nmea.data_sample);
    }
  }

  //Here is topic to publish data:
  topics::GnssCoordinatesTopic::SharedPtr m_gnss;
  //Here we store UART connection resource:
  std::shared_ptr<ugcs_skyhub::connections::UartConnection> m_connect;
  //Maximum size of parsing buffer:
  static const size_t READ_BUFFER_SIZE = 256;
  //Simple NMEA parser instance:
  NmeaProcessor nmea;
};

}

/**
 * Every node defines entry point for it's start: the regular main function
 */
int main(int argc, char * argv[])
{
  /**
   * Standart SDK initialization. At this step ROS2 framework may override default node names, logging levels
   * and so on.
   */
  ugcs_skyhub::sdkInit(argc, argv);
  /**
   * Make instance of main driver class. At this moment Payload Node gets "Unconfigured" stage.
   */
  skyhub_demo::GNSSdriver gnss;

  RCLCPP_INFO(gnss.getRosNode().get_logger(), "GNSS demo driver started");

  /**
   * Enter node event loop. This method will block until node will not be shutdowned.
   */
  gnss.execute();
  /**
   * Clean up SDK resources after GNSS driver node finished it's life cycle.
   */
  ugcs_skyhub::sdkRelease();
  return 0;
}
