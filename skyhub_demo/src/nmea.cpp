#include <rclcpp/rclcpp.hpp>
#include <skyhub_demo/nmea.h>

namespace NMEA {
    const char START = '$'; //Start of sentecnce
    const char CR = 0x0D; // Carriage return
    const char LF = 0x0A; // Line feed
    const char FIELD_SEPARATOR = ','; //Fields separator
    const char CHECK_SEPARATOR = '*'; //Checksum separator

    const int AddressLen = 5; //Length of address field (symbols)
    const int CheckSumLen = 2; //Length of check sum field (symbols)
}

namespace skyhub_demo
{

NmeaProcessor::NmeaProcessor() :
        data_sample(std::make_shared<topics::GnssCoordinatesTopic::MessageType>()),
        ds_coords_ready(false),
        ds_quality_ready(false)
{
}

void NmeaProcessor::processNmeaData(const std::vector<char>& rawData, const int length, const rclcpp::Time& ts)
{
    static Stage stage = Idle;
    static int dataCounter = 0;
    static int msgType = 0;
    static int fieldNumber = 0;
    static std::vector<char> rawValue;

    for (int i = 0; i < length; i++) {
        switch (stage) {
        case Idle:
            if (rawData[i] == NMEA::START) {
                stage = Address;
                rawValue.clear();
                dataCounter = 0;
            }
            break;
        case Address:
            if(dataCounter < NMEA::AddressLen) { //read out address bytes for fixed lenght
                rawValue.push_back(rawData[i]);
                dataCounter++;
            }
            else {
                //Address read complete. Check for expected message type:
                if(memcmp(rawValue.data(), "GPRMC", 5) == 0) {
                  //Timestamp is determined at the moment of RMC receiving:
                  data_sample->header.stamp = ts;
                  msgType = NmeaMessageTypes::RMC;
                }
                else if(memcmp(rawValue.data(), "GPGGA", 5) == 0) {
                  msgType = NmeaMessageTypes::GGA;
                }
                else {
                    stage = Idle;
                    continue; //Goto cycle begining
                }

                if(rawData[i] == NMEA::FIELD_SEPARATOR) { //switch to read out data
                   stage = Data;
                   rawValue.clear();
                   dataCounter = 0;
                   fieldNumber = 0;
                }
                else { //comma must be always after address: error if no separator found
                    RCLCPP_ERROR(rclcpp::get_logger("NMEA"),
                                "Unexpected sybol:[%d] was read\n", rawData[i]);
                    stage = Idle;
                }
            }
            break;
         case Data:
            switch(rawData[i]) {
            case NMEA::CHECK_SEPARATOR: //all data field are read. switch to checksum control
                stage = CheckSum;

                rawValue.push_back('\0');
                processNmeaField(msgType, fieldNumber, rawValue);

                rawValue.clear();
                dataCounter = 0;
                fieldNumber = 0;

                break;
            case NMEA::FIELD_SEPARATOR: //field was read: next field expected
                rawValue.push_back('\0');
                processNmeaField(msgType, fieldNumber, rawValue);

                rawValue.clear();
                dataCounter = 0;
                fieldNumber++;

                break;
            default: //new data byte available
                rawValue.push_back(rawData[i]);
                dataCounter++;
            }
            break;
         case CheckSum:
            if(dataCounter < NMEA::CheckSumLen-1) { //reading checksum bytes
                dataCounter++;
            }
            else {
                //TODO: add check sum calculation and validation

                //If checksum correct it's ready to publish received values:
                switch(msgType) {
                case NmeaMessageTypes::RMC:
                    ds_coords_ready = true;
                    //TODO: fill real status. GPS fix is always for now:
                    data_sample->status = 2;
                    break;
                case NmeaMessageTypes::GGA:
                    ds_quality_ready = true;
                    break;
                default:
                    break;
                }

                stage = CR;
            }
            break;
         case CR:
            if(rawData[i] == NMEA::CR) {
                stage = LF;
            }
            else {
                RCLCPP_ERROR(rclcpp::get_logger("NMEA"),
                        "CR expected after checksum, but:[%d] was read\n", rawData[i]);
                stage = Idle;
            }
            break;
         case LF:
           if(rawData[i] != NMEA::LF)
               RCLCPP_ERROR(rclcpp::get_logger("NMEA"),
                       "LF expected after checksum, but:[%d] was read\n", rawData[i]);
           stage = Idle;
           break;
        }
    }
}

void NmeaProcessor::processNmeaField(const int msgType, const int fieldNum, const std::vector<char>& data) const
{
    switch(msgType)
    {
        case 1: //RMC
            switch(fieldNum)
            {
                case 2: //lat
                data_sample->latitude = std::atof(data.data());
                break;
                case 3: //lat direction
                if(data[0] == 'S')
                  data_sample->latitude *= -1;
                break;
                case 4: //lon
                data_sample->longitude = std::atof(data.data());
                break;
                case 5: //lon direction
                if(data[0] == 'W')
                  data_sample->longitude *= -1;
                break;
                default:
                break;
            }
        break;
        case 2: //GGA
            switch(fieldNum)
            {
                case 6: //nsv
                data_sample->nsv = std::atoi(data.data());
                break;
                case 7: //hdop
                data_sample->hdop = std::atof(data.data());
                break;
                default:
                break;
            }
        break;
        default:
        break;
    }
}
}
