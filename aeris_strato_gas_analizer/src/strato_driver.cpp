/**
 * Implementation of Aeris Strato gas analizer driver based on Skyhub SDK framework
 */

/**
 * This node is based on DriverNode class of SDK, since it devoted to interact with
 * hardware and send read data to any connected consumer. So, we include DriverNod
 * interface
 */
#include <ugcs_skyhub/driver_node.h>
/**
* Datalog services used to store result data in pre-configured log file
*/
#include <ugcs_skyhub/datalog_service.h>
/**
 * Will use ground connection to show data in CPM
 */
#include <ugcs_skyhub/connection_ground.h>
/**
 * Use Altimeter widget to display data:
 */
#include <skyhub_msgs/msg/monitor_message.hpp>

/**
 * Import full names of classes we are going to use:
 */
using ugcs_skyhub::payloads::DriverNode;
using ugcs_skyhub::payloads::Address;
using ugcs_skyhub::connections::UartConnection;
using ugcs_skyhub::datalogs::DataLog;
using skyhub_msgs::msg::MonitorMessage;

/**
 * To implement driver's port registration, we'll need Configuration Service,
 * so import services namespace.
 */
using namespace ugcs_skyhub::services;

/**
 * Place our classes to separate namespace
 */
namespace skyhub_aeris_strato
{

/**
 * Main class which implements gas analyzer driver. It's based on DriverNode interface:
 */
class GasAnalyzerDriver : public DriverNode
{
public:
  /**
   * Primay constructor does nothing, but sets default node name for ROS node.
   * This is how this node will be listed by "ros2 node list" command
   */
    GasAnalyzerDriver(): DriverNode("aeris_strato_driver")
  {
  }

  /**
   * Every node defines payload type it supports. This name is used by configuration
   * storage and findPayload method to search for specific nodes. Supported type should
   * always be documented and available for driver's user.
   */
  std::string getType() const override
  {
    return "gasanalyzer";
  }

  /**
   * The default payload model this payload supports.
   */
  std::string getModel() const override
  {
    return "aerisstrato";
  }

  /**
   * Payload mask defines which parameter sets will be retrived from configuration storage
   * to try to detect payloads. This driver will support only one (0) instance.
   */
  Address
  getPayloadMask() const override
  {
    return Address("v1:gasanalyzer:aerisstrato:0");
  }

  /**
   * On first launch there is no stored parameters for this driver. We need one UART port for
   * communications with device, so driver has one parameter: port name
   */
  ParamSet getParameters() override
  {
    /*First, we ask Configuration service to add default UART port definition to configuration
    * On successful call serivce return port name.
    */
    std::string uart_id = getConfigService()->addUartPort();

    std::string log = getConfigService()->addDataLog();

    //Parameter set for our driver
    ParamSet ps;
    /*Add received port name from configuration as a value of driver specific parameter:
    * name of UART device. This is not OS-specific name,  but ID of configured resource
    */
    ps["DEV_UART"] = uart_id;
    /* Add received log into param set:
     */
    ps["DATALOG_NAME"] = log;
    return ps;
  }

  /**
   * This is the point there driver should claim the resources it will need to work.
   */
  void configureResources(long instance, const ParamSet& params)
  {
    //Again, we work only for single instance of this payload:
    if(instance != 0)
      return;

    //Obtain log instance:
    /**
     * Search for datalog name in parameters. We expect if was configured at "register payload" step.
     */
    auto log_name = params.find("DATALOG_NAME");
    //Report error if no log name found
    if(log_name == params.end())
      throw std::invalid_argument("Datalog name was not configured");

    /**
     * Get instance of named log. This log is manager by DataLog Service and accessible as pre-configured
     * resource by it's name
     */
    m_log = getDatalogService()->getDataLog(log_name->second);

    /**
     * Get connection to the Ground software: show data with CPM
     */
    m_ground = getConnectionService()->connectGround();

    //Obtain port name:
    std::string port;
    auto uart_dev = params.find("DEV_UART");
    if(uart_dev != params.end())
      port = uart_dev->second;
    else
      throw std::invalid_argument("Port name not found");

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
  }

  /**
   * Then payload node reaches activation payload step it should begin data processing and
   * any other activities it's developed for. Every payload instance activated independently.
   */
  void activatePayload(long instance)
  {
    //Activate only the first instance:
    if(instance != 0)
      return;

    //Now, open the UART port there gas analyzer fos configured:
    m_connect->connect();
    /**
     * Launch infinite reading loop. This call is non-blocking and always return as loop is
     * started on separate thread. This loop is supported by SDK implementation and user must
     * only define what to do with obtained data.
     *
     * onData will get from 1 to READ_BUFFER_SIZE bytes of data to parse or to process in any other way.
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
  void deactivatePayload(long instance)
  {
    //Work for single instance only:
    if(instance != 0)
      return;
    /*
     * Stop infinite loop if it was started
     */
    m_connect->stopAsyncReadLoop();
    /*Just close any opened connections. Any asynchronous operations will be stopped automatically.
     * The finishing of running operations may take some time and this method will wait for their
     * completion.
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
      //Parse incoming data:
      parseData(data, len);
  }

  /**
   * Parse byte vector (data) and real data len.
   * Call publish data routine every time \r is reached
   */
  void parseData(const std::vector<char>& data, size_t len)
  {
      //Possible parser states
      enum States {Error, CR, Ready};
      //Start with Error state to seek for line start
      static States state = Error;
      //Storage for field data between calls
      static std::vector<char> field;
      //Store parsed values to be send in scope
      static float temp = 0, ch4 = 0, h2o = 0, c2h6 = 0;
      //Field index we process right now
      static int field_num = 0;

      for(size_t idx = 0; idx < len; idx++) {
        const char symbol = data[idx];
        switch(state) {
        case Error:
            if(symbol == '\n') {
                field_num = 0;
                field.clear();
                state = Ready;
            }
            continue;
        case CR:
            if(symbol == '\n') {
                field_num = 0;
                field.clear();
                state = Ready;
            }
            else
                state = Error; //Only \n may follow \r
            continue;
        case Ready://New line started
            if(symbol == '\r') {
                publishData(temp, ch4, h2o, c2h6);
                state = CR;
                continue;
            }
            if(symbol == '\n') {
                state = Error; //Error \r must pretend \n
                continue;
            }
            if(symbol == ',') {
                field.push_back('\0'); //Add tail for proper C-string
                switch(field_num) {
                case 3:
                    temp = parseField(field);
                    RCLCPP_DEBUG(get_logger(), "Parsed temp: %f", temp);
                    break;
                case 4:
                    ch4 = parseField(field);
                    RCLCPP_DEBUG(get_logger(), "Parsed CH4: %f", ch4);
                    break;
                case 5:
                    h2o = parseField(field);
                    RCLCPP_DEBUG(get_logger(), "Parsed H2O: %f", h2o);
                    break;
                case 6:
                    c2h6 = parseField(field);
                    RCLCPP_DEBUG(get_logger(), "Parsed H2O: %f", c2h6);
                    break;
                default:
                    //No interest in this field
                    break;
                }
                field.clear();
                field_num++;
            }
            else {
                //Add symbol into buffer:
                field.push_back(symbol);
            }
            continue;
        default:
            //Never reaches this branch
            break;
        }
      }
  }

  /**
   * Performs post to log when full set collected
   */
  void publishData(float temp, float ch4, float h2o, float c2h6)
  {
      //make record of arbitrary fields:
      std::vector<std::any> record;
      //Put analizer data into it:
      record.push_back(ch4);
      record.push_back(temp);
      record.push_back(h2o);
      record.push_back(c2h6);
      //Send to log
      m_log->WriteRecord(record);
      //Send to CPM on Ground:
      m_ground->sendScalarValue(temp,
                                MonitorMessage::PAYLOAD_ID_USER_PAYLOAD_FIRST,
                                0xC0, //Place this ID into Payload Example widget settings
                                MonitorMessage::STATE_GOOD);
      RCLCPP_INFO(get_logger(), "- CH4: %f - Temp: %f - H2O: %f - C2H6: %f", ch4, temp, h2o, c2h6);
  }

  //Convert bytes of string representation into float
  float parseField(const std::vector<char>& data)
  {
      return std::atof(data.data());
  }

  //Here we store UART connection resource:
  std::shared_ptr<ugcs_skyhub::connections::UartConnection> m_connect;
  //Link to common log:
  std::shared_ptr<DataLog> m_log;
  //Channel to the ground:
  std::shared_ptr<ugcs_skyhub::connections::GroundConnection> m_ground;
  //Maximum size of parsing buffer:
  static const size_t READ_BUFFER_SIZE = 256;
};

}

/**
 * Every node defines entry point for it's start: the regular main function
 */
int main(int argc, char * argv[])
{
  /**
   * Standart ROS2 initialization. At this step ROS2 framework may override default node names, logging levels
   * and so on.
   */
  rclcpp::init(argc, argv);
  /**
   * Make instance of main driver class. At this moment Payload Node gets "Unconfigured" stage.
   */
  std::shared_ptr<skyhub_aeris_strato::GasAnalyzerDriver> analyzer =
          std::make_shared<skyhub_aeris_strato::GasAnalyzerDriver>();

  RCLCPP_INFO(analyzer->get_logger(), "Aeris Strato driver started");

  /**
   * Manuallu trigger configuration process: transition Unconfigured->Configuring->Inactive.
   */
  analyzer->configure();
  /**
   * Enter node event loop. This method will block until node will not be shutdowned.
   */
  rclcpp::spin(analyzer->get_node_base_interface());
  /**
   * Clean up ROS2 framework resources after driver node finished it's life cycle.
   */
  rclcpp::shutdown();
  return 0;
}
