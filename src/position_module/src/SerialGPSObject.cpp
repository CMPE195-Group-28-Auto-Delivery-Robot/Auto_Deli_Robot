#include "SerialGPSObject.h"

int SerialGPSObject::openSerial(){
    struct termios ttyCnf;
    serialFp = open(portPath.c_str(), O_RDWR);
    if (serialFp < 0) {
        ROS_ERROR("Error %i from open: %s\n", errno, strerror(errno));
        return -1;
    }
    if(tcgetattr(serialFp, &ttyCnf) != 0) {
        ROS_ERROR("Error %i from tcgetattr: %s\n", errno, strerror(errno));
        close(serialFp);
        return -1;
    }
    if(parity){
        ttyCnf.c_cflag |= PARENB; 
    }else{
        ttyCnf.c_cflag &= ~PARENB;
    }
    if(stopBitEven){
        ttyCnf.c_cflag |= CSTOPB;; 
    }else{
        ttyCnf.c_cflag &= ~CSTOPB;
    }
    if(flowCtrl){
        ttyCnf.c_cflag |= CRTSCTS; 
    }else{
        ttyCnf.c_cflag &= ~CRTSCTS;
    }
    switch(dataBit){
        case 5:
            ttyCnf.c_cflag |= CS5; 
            break;
        case 6:
            ttyCnf.c_cflag |= CS6; 
            break;
        case 7:
            ttyCnf.c_cflag |= CS7; 
            break;
        case 8:
            ttyCnf.c_cflag |= CS8; 
            break;
        default:
            ROS_ERROR("Data Bit set invalid");
            close(serialFp);
            return -1;
    }
    ttyCnf.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
    ttyCnf.c_lflag &= ~ICANON; // Disabling Canonical Mode
    ttyCnf.c_lflag &= ~ECHO; // Disable echo
    ttyCnf.c_lflag &= ~ECHOE; // Disable erasure
    ttyCnf.c_lflag &= ~ECHONL; // Disable new-line echo
    ttyCnf.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    ttyCnf.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    ttyCnf.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes
    ttyCnf.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    ttyCnf.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    ttyCnf.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    ttyCnf.c_cc[VMIN] = 0;
    switch(baudrate){
        case 1200:
            cfsetispeed(&ttyCnf, B1200);
            break;
        case 1800:
            cfsetispeed(&ttyCnf, B1800);
            break;
        case 2400:
            cfsetispeed(&ttyCnf, B2400);
            break;
        case 4800:
            cfsetispeed(&ttyCnf, B4800);
            break;
        case 9600:
            cfsetispeed(&ttyCnf, B9600);
            break;
        case 19200:
            cfsetispeed(&ttyCnf, B19200);
            break;
        case 38400:
            cfsetispeed(&ttyCnf, B38400);
            break;
        case 57600:
            cfsetispeed(&ttyCnf, B57600);
            break;
        case 115200:
            cfsetispeed(&ttyCnf, B115200);
            break;
        default:
            ROS_ERROR("Unsupported Baud Rate");
            close(serialFp);
            return -1;
    }
    // Save tty settings, also checking for error
    if (tcsetattr(serialFp, TCSANOW, &ttyCnf) != 0) {
        ROS_ERROR("Error %i from tcsetattr: %s\n", errno, strerror(errno));
        close(serialFp);
        return -1;
    }
    return serialFp;
} // Code Above Partially From https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/#configuration-setup

uint8_t SerialGPSObject::stringCheckSum(std::string inStr){
    uint8_t checkSum = inStr[0];
    for(int i=1; i<inStr.size();i++){
        checkSum ^= inStr[i];
    }
    return checkSum;
}

void SerialGPSObject::sendCommand(std::string content){
    char numStr[3];
    char readStr[32];
    int commandNum, result;
    sprintf(numStr, "%X", stringCheckSum(content));
    content = "$"+content+"*"+numStr+"\r\n";
    // ROS_INFO("Send: %s",content.c_str());
    write(serialFp, content.c_str(), content.size());
}

void SerialGPSObject::RebootGPS(){
    switch (chipType)
    {
    case 1:
        ROS_INFO("Chip: MT3339");
        sendCommand("PMTK102");
        ros::Duration(3).sleep();
        ROS_INFO("Boot Complete");
        sendCommand("PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0");
        sendCommand("PMTK220,100");
        break;

    case 2:
        ROS_INFO("Chip: UBlox M8");
        //TODO: Ublox Command
        break;
    
    default:
        ROS_INFO("Chip: Unknown");
        break;
    }
}

bool SerialGPSObject::RebootGPS(robot_msgs::RebootGPS::Request &req,
                                robot_msgs::RebootGPS::Response &res){
    RebootGPS();
}