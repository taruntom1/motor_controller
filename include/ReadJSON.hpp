#pragma once

#include <iostream>
#include <fstream>
#include <nlohmann/json.hpp>
#include "Structs.hpp"
#include <rclcpp/rclcpp.hpp>

using json = nlohmann::json;

void from_json(const json& j, OdoBroadcastStatus& o);
void from_json(const json& j, PIDConstants& p);
void from_json(const json& j, PWMLimits& p);
void from_json(const json& j, MotorConnections& m);
void from_json(const json& j, OdometryData& o);
void from_json(const json& j, Setpoint& s);
void from_json(const json& j, UpdateFrequenciesWheel& u);
void from_json(const json& j, MotorData& m);
void from_json(const json& j, UpdateFrequencies& u);
void from_json(const json& j, ControllerProperties& c);
void from_json(const json& j, ControllerData& c);

bool loadControllerDataFromJson(const std::string& filename, ControllerData& data,  rclcpp::Logger logger);