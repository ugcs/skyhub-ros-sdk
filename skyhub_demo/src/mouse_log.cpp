#include <ugcs_skyhub/driver_node.h>
#include <ugcs_skyhub/datalog_service.h>
#include <ugcs_skyhub/autopilot_topics.h>
#include <skyhub_demo/timer.hpp>
#include <fcntl.h>
#include <unistd.h>

using ugcs_skyhub::payloads::Address;
using ugcs_skyhub::payloads::DriverNode;
using ugcs_skyhub::services::ParamSet;

using ugcs_skyhub::topics::autopilot::MonitorScalarTopic;
using skyhub_demo::time::SkyhubTimer;
using ugcs_skyhub::datalogs::StandardFields;

using namespace std::chrono_literals;
using namespace std;

struct Params {
        constexpr static char INTERVAL[] = "INTERVAL_MS";
        constexpr static char DEVICE[] = "MICE_DEVICE";
};

class MouseLog : public DriverNode
{
public:
    MouseLog() : DriverNode("mouse_log")
    {
    }
    ~MouseLog()
    {
        for (int fd : descriptors)
        {
            close(fd);
        }
    }

    string getType() const override
    {
        return "mouse_log";
    }

    string getModel() const override
    {
        return "test";
    }

    ParamSet getParameters() override  // setting up parameters for the node
    {
        return {
            {Params::INTERVAL, "1000"},             // duration of one node iteration
            {Params::DEVICE, "/dev/input/mouse0"}     // path to the device
        };
    }

    Address getPayloadMask() const override  // setting up the payload mask
    {
        return Address(getType(), getModel(), nullopt);
    }

    void configureResources(const Address&, const ugcs_skyhub::services::ParamSet& ps) override
    {        
        // Reading parameters
        m_interval = getParam<int>(ps, Params::INTERVAL);
        mice_device = getParam<string>(ps, Params::DEVICE);
        mice_devices.push_back(mice_device);

        // Initializing timer with callback
        timer = std::make_unique<SkyhubTimer>(this->getRosNode(),
                                                chrono::milliseconds(m_interval),
                                                bind(&MouseLog::timer_callback, this));

    }

    void activatePayload(const ugcs_skyhub::payloads::Address& payload) override
    {
        auto instance = payload.getInstance().value_or(0);

        if (instance == 0)
        {
            // Connecting to the common log
            position_log = getDatalogService()->getPositionDataLog();

            // Connecting to the node specific log
            mouse_log = getDatalogService()->getCsvDataLog("mouse_log");
            // Adding the standard data and time fields (no manual update required)
            mouse_log->addStandardFields({StandardFields::TS_DATE, StandardFields::TS_TIME});
        }
        
        string coord_x = "Coord_X" + to_string(instance);
        string coord_y = "Coord_Y" + to_string(instance);
        string velocity = "Velocity" + to_string(instance);
        // Adding mouse coordinates fields and position change velocity field
        position_log->addFields({coord_x, coord_y, velocity});
        // Adding mouse coordinates fields and position change velocity field
        mouse_log->addFields({coord_x, coord_y, velocity});

        // Openning device file
        int mouse_fd = open(mice_devices[instance].c_str(), O_RDONLY | O_NONBLOCK);

        if (mouse_fd == -1)
        {
            LOG.error().format("Error opening mouse device");
            return;
        }
        else
        {
            descriptors.push_back(mouse_fd);
        }

        // Connecting to the ground
        m_ground = getConnectionService()->connectGround();

        // Starting timer (launching the main loop)
        timer->start();
    }
private:
    SkyhubTimer::UniquePtr timer;   // node timer
    int m_interval;                 // duration between two timer iterations
    string mice_device;             // path to the device file
    ugcs_skyhub::datalogs::CsvDataLog::SharedPtr position_log;          // common log
    ugcs_skyhub::datalogs::CsvDataLog::SharedPtr mouse_log;             // node specific log
    shared_ptr<ugcs_skyhub::connections::GroundConnection> m_ground;    // connection with ground
    vector<int> descriptors;
    vector<string> mice_devices;

    char mouse_buffer[3];   // buffer for device data

    void timer_callback()
    {
        vector<std::any> mouse_data;    // container for log data
        int mouse_x = 0;
        int mouse_y = 0;
        vector<float> xy_velocity(descriptors.size(), 0);
        static vector<int> x_integral((int)descriptors.size(), 0);
        static vector<int> y_integral((int)descriptors.size(), 0);

        for (int i = 0; i < (int)descriptors.size(); i ++)
        {
            // Reading the device file
            if (read(descriptors[i], mouse_buffer, sizeof(mouse_buffer)) > 0)
            {
                // Writing the file data to mouse coordinates (relative to the previous position)
                mouse_x = mouse_buffer[1];
                mouse_y = mouse_buffer[2];

                // Need to integrate data to calculate absolute mouse coordinates
                x_integral[i] += mouse_x;
                y_integral[i] += mouse_y;

                // Calculating the position change velocity
                float transition = sqrtf(pow(mouse_x, 2) + pow(mouse_y, 2));
                xy_velocity[i] = transition / (float)m_interval * 1000.0f;
            }

            mouse_data.push_back(x_integral[i]);
            mouse_data.push_back(y_integral[i]);
            mouse_data.push_back(xy_velocity[i]);
        }

        // Sending the mouse velocity to the ground
        m_ground->sendScalarValue(xy_velocity[0],
                                  MonitorScalarTopic::MessageType::PAYLOAD_ID_ROS_CUSTOM,
                                  MonitorScalarTopic::MessageType::MESSAGE_ID_CUSTOM_PAYLOAD_FIRST,
                                  MonitorScalarTopic::MessageType::STATE_GOOD);

        // Writing the data to logs
        position_log->writeRecord(mouse_data);
        mouse_log->writeRecord(mouse_data);
    };
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MouseLog>();
    node->execute();
    node = nullptr;
    rclcpp::shutdown();

    return 0;
}
