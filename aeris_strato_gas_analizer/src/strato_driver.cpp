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
 * Topics for CPM communication:
 */
#include <ugcs_skyhub/autopilot_topics.h>

/**
 * Import full names of classes we are going to use:
 */
using ugcs_skyhub::payloads::DriverNode;
using ugcs_skyhub::payloads::Address;
using ugcs_skyhub::connections::UartConnection;
using ugcs_skyhub::datalogs::DataLog;
using ugcs_skyhub::topics::autopilot::MonitorScalarTopic;

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
   * This is the point there driver should claim the resources it will need to work.
   */
  void configureResources(long instance, const ParamSet& params)
  {
    //Again, we work only for single instance of this payload:
    if(instance != 0)
      return;

    /**
     * Get instance of named log. This log is manager by DataLog Service:
     */
    m_log = getDatalogService()->getCsvDataLog("strato");

    /**
     * Get connection to the Ground software: show data with CPM
     */
    m_ground = getConnectionService()->connectGround();

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
  void deactivatePayload(long instance)
  {
    //Work for single instance only:
    if(instance != 0)
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
                    LOG.debug() <<  "Parsed temp: " << temp;
                    break;
                case 4:
                    ch4 = parseField(field);
                    LOG.debug() << "Parsed CH4: " << ch4;
                    break;
                case 5:
                    h2o = parseField(field);
                    LOG.debug() << "Parsed H2O: " << h2o;
                    break;
                case 6:
                    c2h6 = parseField(field);
                    LOG.debug() << "Parsed H2O: " << c2h6;
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
      m_log->writeRecord(record);
      //Send to CPM widget. Use Scalar Widget to see the value.
      m_ground->sendScalarValue(ch4,
              MonitorScalarTopic::MessageType::PAYLOAD_ID_ROS_CUSTOM,
              MonitorScalarTopic::MessageType::MESSAGE_ID_CUSTOM_PAYLOAD_FIRST,
              MonitorScalarTopic::MessageType::STATE_GOOD);
      //Send text string to CPM. It will be shown on the console:
      LOG.ui(true).info() << "- CH4: " << ch4 << " - Temp: " << temp << " - H2O: "<< h2o << " - C2H6: " << c2h6;
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
   * ROS2 and Skyhub SDK initialization. At this step ROS2 framework may override default node names, logging levels
   * and so on.
   */
    ugcs_skyhub::sdkInit(argc, argv);
    /**
     * Make instance of main driver class. At this moment Payload Node gets "Unconfigured" stage.
     */
    skyhub_aeris_strato::GasAnalyzerDriver node;
    /**
     * Enter node event loop. This method will block until node will not be shutdowned.
     */
    node.execute();
    /**
     * Clean up ROS2 framework resources after driver node finished it's life cycle.
     */
    ugcs_skyhub::sdkRelease();


  rclcpp::init(argc, argv);

  return 0;
}
