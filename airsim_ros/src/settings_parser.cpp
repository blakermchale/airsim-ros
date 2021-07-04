#include "airsim_ros/settings_parser.hpp"

namespace airsim_ros
{

SettingsParser::SettingsParser(const std::string& host_ip)
    : host_ip_(host_ip)
{
    success_ = initializeSettings();
}

bool SettingsParser::success()
{
    return success_;
}

bool SettingsParser::getSettingsText(std::string& settings_text) const
{
    msr::airlib::RpcLibClientBase airsim_client(host_ip_);
    airsim_client.confirmConnection();

    settings_text = airsim_client.getSettingsString();

    return !settings_text.empty();
}

std::string SettingsParser::getSimMode()
{
    Settings& settings_json = Settings::loadJSonString(settings_text_);
    return settings_json.getString("SimMode", "");
}

// mimics void ASimHUD::initializeSettings()
bool SettingsParser::initializeSettings()
{
    if (getSettingsText(settings_text_)) {
        AirSimSettings::initializeSettings(settings_text_);

        AirSimSettings::singleton().load(std::bind(&SettingsParser::getSimMode, this));
        std::cout << "SimMode: " << AirSimSettings::singleton().simmode_name << std::endl;

        return true;
    }

    return false;
}

}  // namespace airsim_ros
