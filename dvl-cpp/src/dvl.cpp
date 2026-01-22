#include "dvl.hpp"

namespace dvl {

    // Constructor
   DVL::DVL(const std::string& port, unsigned long baudrate /*= 115200*/) {
        // Error config definition
        error_config.speed_of_sound = 0.0;
        error_config.mounting_rotation_offset = 0.0;
        error_config.acoustic_enabled = 'x';
        error_config.dark_mode_enabled = 'x';
        error_config.range_mode = 'x';
        error_config.periodic_cycling_enabled = 'x';
        config = error_config; // will get overwritten by first successful readConfig

        // Open serial port
        fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd < 0) {
            throw std::runtime_error("Failed to open serial port: " + std::string(strerror(errno)));
        }

        // Configure port
        struct termios tty;
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            throw std::runtime_error("Failed to get terminal attributes: " + std::string(strerror(errno)));
        }

        // Set baud rate
        speed_t speed;
        switch (baudrate) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            default:
                close(fd);
                throw std::invalid_argument("Unsupported baudrate");
        }
        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        // Configure 8N1, no flow control
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits
        tty.c_cflag &= ~PARENB; // no parity
        tty.c_cflag &= ~CSTOPB; // 1 stop bit
        tty.c_cflag &= ~CRTSCTS; // no hardware flow control
        tty.c_cflag |= CLOCAL | CREAD; // enable receiver

        tty.c_lflag = 0; // non-canonical mode
        tty.c_oflag = 0; // no remapping, no delays
        tty.c_iflag = 0; // no special handling

        tty.c_cc[VMIN] = 0;  // non-blocking read
        tty.c_cc[VTIME] = 10; // 1 second timeout (VTIME is in deciseconds)

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            throw std::runtime_error("Failed to set terminal attributes: " + std::string(strerror(errno)));
        }
    }

    // Public Reads

    VR DVL::readVelocityReport(){
        if(holdForResponse(REC_VR)){
            return vr;
        } else {
            return error_vr;
        }
        
    }

    DRR DVL::readDRReport(){
        if(holdForResponse(REC_DRR)){
            return drr;
        } else {
            return error_drr;
        }
    }

    std::string DVL::readVersion(){

        sendCommand(CMD_GET_VERSION); 
        
        if(holdForResponse(CMD_GET_VERSION)){
            return version;
        } else {
            version = "x.x.x";
            return version;
        }

    }

    std::string DVL::readDetails(){
        sendCommand(CMD_GET_PRODUCT_DETAIL); 
        
        if(holdForResponse(CMD_GET_PRODUCT_DETAIL)){
            return product_details;
        } else {
            return "x,x,x,x";
        }
    }

    Config DVL::readConfig(){
        sendCommand(CMD_GET_SETTINGS); 
        
        if(holdForResponse(CMD_GET_SETTINGS)){
            return config;
        } else {
            return error_config;
        }
    }


    // Public Writes
    bool DVL::setConfig(float speed_of_sound, float mounting_rotation_offset, char acoustic_enabled, char dark_mode_enabled, std::string range_mode, bool periodic_cycling_enabled){
        //to do: add setting args
        return sendCommand(CMD_SET_SETTINGS, {std::to_string(speed_of_sound),std::to_string(mounting_rotation_offset),std::string(1, acoustic_enabled),std::string(1, dark_mode_enabled),range_mode,std::to_string(periodic_cycling_enabled)
        });    
    }


    bool DVL::resetDRR(){
        sendCommand(CMD_RESET_DR);
        if(holdForResponse(ACK)){
            return true;
        } else{
            return false;
        }
    }

    bool DVL::resetGyro(){
        sendCommand(CMD_CALIBRATE_GYRO);
        if(holdForResponse(ACK)){
            return true;
        } else{
            return false;
        }
    }

    bool DVL::triggerPing(){
        sendCommand(CMD_TRIGGER_PING);
        if(holdForResponse(ACK)){
            return true;
        } else {
            return false;
        }

    }

    bool DVL::setSerialProtocol(int protocol){
        sendCommand(CMD_CHANGE_SER_OUTPUT,{std::to_string(protocol)}); //currently only can be used to start serial output
        if(holdForResponse(ACK)){
            return true;
        } else{
            return false;
        }
    }

    //Private Methods
    bool DVL::holdForResponse(const char expected_response) {
        /*
        Waits until either 10 ms have elapsed or the expected response is received.
        Inputs:
            const std::string& expected_response -- string containing the expected response to the command such as "wrx" or "wra"
        Outputs:
            true or false depending on whether the expected response was found.
        */
        
        using clock = std::chrono::steady_clock;
        constexpr auto TIMEOUT = std::chrono::milliseconds(10);

        auto start = clock::now();
        std::string complete_line;

        while (clock::now() - start < TIMEOUT) {

            complete_line.clear(); //clear the complete line each time a line complete line is parsed 

            while (clock::now() - start < TIMEOUT) { //read until a complete line is found

                std::string partial_line = "";
                char c;
                ssize_t n = ::read(fd, &c, 1); // read 1 byte from the serial port
                if (n == 1) {
                    partial_line += c; // append to the end of the existing string
                } else if (n < 0) {
                    throw std::runtime_error("Serial read error: " + std::string(strerror(errno)));
                }
                // n == 0: no data available (non-blocking read)

                if (partial_line.empty()) {
                    continue; //loop again if the the partial line is empty
                }

                complete_line += partial_line; //add the partial line to the complete line

                if (partial_line == "\n" || partial_line == "\r") {
                    break; //break out of the reading loop if an end-of-line char is detected
                }
            }

            //if inner loop timed out without EOL, keep looping
            if (complete_line.empty()) {
                continue; 
            }

            parseResponse(complete_line);

            if (complete_line[2] == 2) {
                return true; //if the expected response was received as the command field of the response
            }
        }

        return false; //return false if the code timed out
    }

    bool DVL::parseResponse(std::string& complete_line){

        //identify the command
        char cmd = complete_line[2];

        //strip whitespace and newlines
        complete_line.erase(0, complete_line.find_first_not_of(" \t\r\n"));
        complete_line.erase(complete_line.find_last_not_of(" \t\r\n") + 1);

        // strip checksum "*xx"
        if (complete_line.size() >= 3 &&
            complete_line[complete_line.size() - 3] == '*') {
            complete_line.erase(complete_line.size() - 3);
        }

        //check cmd against each possibility
        switch (cmd) {
            case 'v': //protocol version
                version = complete_line.substr(4);
                return true;
                break;
            case 'w': //product details
                product_details = complete_line.substr(4);
                return true;
                break;
            case 'a': //acknowledge
                return true;
                break;
            case 'c': {//configuration
                std::stringstream ss(complete_line);

                //split up the line into a string array
                std::string field;
                std::vector<std::string> fields;
                while(std::getline(ss, field,',')) {
                    fields.push_back(field);
                }

                //unpack into config struct
                config.speed_of_sound = std::stof(fields[1]);
                config.mounting_rotation_offset = std::stof(fields[2]);
                config.acoustic_enabled = fields[3][0];
                config.dark_mode_enabled = fields[4][0];
                config.range_mode = fields[5];
                config.periodic_cycling_enabled = fields[6][0];

                return true;
                break;
                }
            case 'z': {//velocity report
                std::stringstream ss(complete_line);
                std::string field;
                std::vector<std::string> fields;

                while (std::getline(ss, field, ',')) {
                    fields.push_back(field);
                }

                vr.vx = std::stof(fields[1]);
                vr.vy = std::stof(fields[2]);
                vr.vz = std::stof(fields[3]);
                vr.valid = fields[4][0];
                vr.altitude = std::stof(fields[5]);
                vr.fom = std::stof(fields[6]);

                for (int i = 0; i < 9; ++i) {
                    vr.covariance[i] = std::stof(fields[7 + i]);
                }

                vr.time_of_validity = std::stoll(fields[16]);
                vr.time_of_transmission = std::stoll(fields[17]);
                vr.time = std::stof(fields[18]);
                vr.status = static_cast<uint8_t>(std::stoul(fields[19], nullptr, 10));
                return true;
                break;
                }
            case 'p': {//dead reckoning report
                std::stringstream ss(complete_line);
                std::string field;
                std::vector<std::string> fields;
                while(std::getline(ss, field, ',')){
                    fields.push_back(field);
                }

                drr.x = std::stof(fields[1]);
                drr.y = std::stof(fields[2]);
                drr.z = std::stof(fields[3]);
                drr.pos_std = std::stof(fields[4]);
                drr.roll = std::stof(fields[5]);
                drr.pitch = std::stof(fields[6]);
                drr.yaw = std::stof(fields[7]);
                drr.status = static_cast<uint8_t>(std::stoul(fields[8], nullptr, 10));
                return true;
                break;
                 }
            case '?': //malformed request
                return false;
                break;
            case '!': //bad checksum
                return false;
                break;
            case 'n': //not acknowledged
                return false;
                break;
            default:
                return false;
                break;
        }

    }

    bool DVL::sendCommand(uint8_t cmd, const std::vector<std::string>& options) {

        std::stringstream msg;

        // Build message
        msg << SOP << DIR_CMD << static_cast<int>(cmd);  // add start character, direction, and command

        for (const auto& opt : options) {               // add options as comma-separated
            msg << "," << opt;
        }

        // Compute checksum (CRC-8)
        uint8_t crc = 0x00;
        std::string body = msg.str();
        for (char c : body) {
            crc = CRC8_TABLE[crc ^ static_cast<uint8_t>(c)];
        }

        msg << CS << std::hex << std::setw(2) << std::setfill('0') << (int)crc << "\n";

        // Write to serial port using POSIX write
        std::string data = msg.str();
        size_t total_written = 0;
        while (total_written < data.size()) {
            ssize_t n = ::write(fd, data.c_str() + total_written, data.size() - total_written);
            if (n < 0) {
                throw std::runtime_error("Serial write error: " + std::string(strerror(errno)));
            }
            total_written += n;
        }

        return true;
}


} //namespace



