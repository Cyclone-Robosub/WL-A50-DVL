#include "dvl.hpp"
#include <chrono>
#include <thread>
#include <iostream>

int main() {

    dvl::DVL dvl("/dev/ttyUSB0");
    
    using namespace std::chrono;

    const auto loop_period = milliseconds(1000);

    while (true) {
        auto loop_start = steady_clock::now();

        dvl::Config config = dvl.readConfig();
        dvl::VR vr = dvl.readVelocityReport();
        dvl::DRR drr = dvl.readDRReport();

        std::cout << "\nSETTINGS:\nSpeed of Sound: " << std::to_string(config.speed_of_sound) <<
        "\nMounting Rotation Offset: " << std::to_string(config.mounting_rotation_offset) <<
        "\nAcoustic Enabled: " << config.acoustic_enabled <<
        "\nDark Mode Enabled: " << config.dark_mode_enabled <<
        "\nRange Mode: " << config.range_mode << "\nPeriodic Cycling Enabled: " << config.periodic_cycling_enabled << std::endl;

        std::cout << "Version: " << dvl.readVersion() << "\nProduct Details: " << dvl.readDetails() << std::endl;

        std::cout << "\nVELOCITY REPORT: " << "v = [" << std::to_string(vr.vx) << ", " <<std::to_string(vr.vy) << ", " << std::to_string(vr.vz) << "]" <<
        "\nFigure of Merit = " << std::to_string(vr.fom) << "\nAltitude = " << std::to_string(vr.altitude) << std::endl;

        std::cout << "\nDEAD RECKONING REPORT: " << "\nR = [" << std::to_string(drr.x) << ", " << std::to_string(drr.y) << ", " << std::to_string(drr.z) << "]" <<
        "\nEul = [" << std::to_string(drr.roll) << ", " << std::to_string(drr.pitch) << ", " << std::to_string(drr.yaw) << "]" << std::endl;
        


        auto elapsed = steady_clock::now() - loop_start;
        if(elapsed < loop_period) {
            std::this_thread::sleep_for(loop_period - elapsed);
        }
    }
}