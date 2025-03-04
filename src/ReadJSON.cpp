#include "ReadJSON.hpp"


void from_json(const json& j, OdoBroadcastStatus& o) {
    j.at("angleBroadcast").get_to(o.angleBroadcast);
    j.at("speedBroadcast").get_to(o.speedBroadcast);
    j.at("pwmBroadcast").get_to(o.pwmBroadcast);
}

void from_json(const json& j, PIDConstants& p) {
    j.at("p").get_to(p.p);
    j.at("i").get_to(p.i);
    j.at("d").get_to(p.d);
}

void from_json(const json& j, PWMLimits& p) {
    j.at("minPWM").get_to(p.minPWM);
    j.at("maxPWM").get_to(p.maxPWM);
}

void from_json(const json& j, MotorConnections& m) {
    j.at("dirPin").get_to(m.dirPin);
    j.at("pwmPin").get_to(m.pwmPin);
    j.at("encPinA").get_to(m.encPinA);
    j.at("encPinB").get_to(m.encPinB);
}

void from_json(const json& j, OdometryData& o) {
    j.at("angle").get_to(o.angle);
    j.at("rpm").get_to(o.rpm);
}

void from_json(const json& j, Setpoint& s) {
    j.at("angle").get_to(s.angle);
    j.at("rpm").get_to(s.rpm);
}

void from_json(const json& j, UpdateFrequenciesWheel& u) {
    j.at("pwm").get_to(u.pwm);
    j.at("anglePID").get_to(u.anglePID);
    j.at("speedPID").get_to(u.speedPID);
}

void from_json(const json& j, MotorData& m) {
    j.at("motorID").get_to(m.motorID);
    j.at("controlMode").get_to(m.controlMode);
    j.at("anglePIDConstants").get_to(m.anglePIDConstants);
    j.at("speedPIDConstants").get_to(m.speedPIDConstants);
    j.at("pwmLimits").get_to(m.pwmLimits);
    j.at("motorConnections").get_to(m.motorConnections);
    j.at("odometryData").get_to(m.odometryData);
    j.at("setpoint").get_to(m.setpoint);
    j.at("odoBroadcastStatus").get_to(m.odoBroadcastStatus);
    j.at("updateFrequenciesWheel").get_to(m.updateFrequenciesWheel);
    j.at("pwmValue").get_to(m.pwmValue);
}

void from_json(const json& j, UpdateFrequencies& u) {
    j.at("interfaceRun").get_to(u.interfaceRun);
}

void from_json(const json& j, ControllerProperties& c) {
    j.at("run").get_to(c.run);
    j.at("numMotors").get_to(c.numMotors);
    j.at("odoBroadcastStatus").get_to(c.odoBroadcastStatus);
    j.at("odoBroadcastFrequency").get_to(c.odoBroadcastFrequency);
    j.at("updateFrequencies").get_to(c.updateFrequencies);
}

void from_json(const json& j, ControllerData& c) {
    if (c.motorData) {
        delete[] c.motorData;
    }
    size_t numMotors = j.at("controllerProperties").at("numMotors").get<size_t>();
    c.motorData = new MotorData[numMotors];
    auto motors = j.at("motorData").get<std::vector<MotorData>>();
    for (size_t i = 0; i < numMotors; i++) {
        c.motorData[i] = motors[i];
    }
    j.at("controllerProperties").get_to(c.controllerProperties);
}



bool loadControllerDataFromJson(const std::string& filename, ControllerData& data, rclcpp::Logger logger) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        RCLCPP_ERROR(logger, "Failed to open JSON file: %s", filename.c_str());
        return false;
    }
    
    json j;
    try {
        file >> j; // Parse JSON file
    } catch (const json::parse_error& e) {
        RCLCPP_ERROR(logger, "JSON parsing error in file %s: %s", filename.c_str(), e.what());
        return false;
    }
    
    file.close();
    
    try {
        j.get_to(data); // Convert JSON to ControllerData
    } catch (const json::exception& e) {
        RCLCPP_ERROR(logger, "Error converting JSON to ControllerData: %s", e.what());
        return false;
    }
    
    RCLCPP_INFO(logger, "Successfully loaded controller data from: %s", filename.c_str());
    return true;
}
