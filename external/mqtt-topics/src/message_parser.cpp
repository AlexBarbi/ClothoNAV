#include "message_parser.h"

#include <algorithm>

namespace MQTTTopics
{
MessageParser::Variables::Variables(const std::string& vehicleId, const std::string& deviceId, const std::string& transactionId) :
    vehicleId(vehicleId), deviceId(deviceId), transactionId(transactionId) {}

void MessageParser::TopicNode::addNode(const std::string& topic) {
    if(topic.empty()) {
        return;
    }

    auto slash = std::find(topic.begin(), topic.end(), '/');

    std::string subTopic = std::string(topic.begin(), slash);
    auto iter = this->adjacent.find(subTopic);

    if(iter == this->adjacent.end()) {
        iter = this->adjacent.emplace(subTopic, std::make_unique<TopicNode>(TopicNode())).first;
    }

    if(slash != topic.end()) {
        iter->second->addNode(std::string(slash + 1, topic.end()));
    }
}

std::unique_ptr<MessageParser::TopicNode>* MessageParser::TopicNode::findNode(const std::string& topic) {
    auto slash = std::find(topic.begin(), topic.end(), '/');

    std::string subTopic = std::string(topic.begin(), slash);
    auto iter = this->adjacent.find(subTopic);

    if(iter == this->adjacent.end()) {
        return nullptr;
    } 
    
    if(slash == topic.end()) {
        return &(iter->second);
    } else {
        return iter->second->findNode(std::string(slash + 1, topic.end()));
    }
}

std::vector<std::unique_ptr<MessageParser::TopicNode>*> MessageParser::TopicNode::findNodesVariables(const std::string& topic, const Variables& variables) {
    std::vector<std::unique_ptr<TopicNode>*> ret;
    this->findNodesVariablesRec(topic, variables, false, ret);

    return ret;
}

void MessageParser::TopicNode::findNodesVariablesRec(const std::string& topic, const Variables& variables, bool hashtag, std::vector<std::unique_ptr<TopicNode>*>& ret) {    
    auto slash = std::find(topic.begin(), topic.end(), '/');
    std::string subTopic = std::string(topic.begin(), slash);

    if(subTopic == "#") {
        hashtag = true;
    }

    if(hashtag) {
        for(auto& next : this->adjacent) {
            ret.push_back(&(next.second));
            next.second->findNodesVariablesRec("", variables, hashtag, ret);
        }
    } else {
        for(auto& next : this->adjacent) {
            bool match = false;

            if(subTopic == "+" || subTopic == next.first) {
                match = true;
            } else if(next.first == "<vehicleId>" && subTopic == variables.vehicleId) {
                match = true;
            } else if(next.first == "<deviceId>" && subTopic == variables.deviceId) {
                match = true;
            } else if(next.first == "<transactionId>" && subTopic == variables.transactionId) {
                match = true;
            }

            if(match) {
                if(slash == topic.end()) {
                    ret.push_back(&(next.second));
                } else {
                    next.second->findNodesVariablesRec(std::string(slash + 1, topic.end()), variables, hashtag, ret);
                }
            }
        }
    }
}

MessageParser::MessageParser() {
    this->tree.addNode("<vehicleId>");
    this->tree.addNode("<vehicleId>/<deviceId>");
    this->tree.addNode("<vehicleId>/<deviceId>/version");
    this->tree.addNode("<vehicleId>/<deviceId>/data");
    this->tree.addNode("<vehicleId>/<deviceId>/data/primary");
    this->tree.addNode("<vehicleId>/<deviceId>/data/secondary");
    this->tree.addNode("<vehicleId>/<deviceId>/data/bms");
    this->tree.addNode("<vehicleId>/<deviceId>/data/inverters");
    this->tree.addNode("<vehicleId>/<deviceId>/data/simulator");
    this->tree.addNode("<vehicleId>/<deviceId>/data/gps");
    this->tree.addNode("<vehicleId>/<deviceId>/data/brusa");
    this->tree.addNode("<vehicleId>/<deviceId>/data/temporary");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/imuAngularRate");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/imuAcceleration");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/vehiclePosition");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/vehicleSpeed");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/frontAngularVelocity");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/rearLeftAngularVelocity");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/rearRightAngularVelocity");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/can/steerAngle");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/GPS");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/GPS/hpposllh");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/GPS/pvt");
    this->tree.addNode("<vehicleId>/<deviceId>/data/cameraLogger/GPS/relposned");
    this->tree.addNode("<vehicleId>/<deviceId>/extra_data_to_log");
    this->tree.addNode("<vehicleId>/<deviceId>/extra_tlm_data");
    this->tree.addNode("<vehicleId>/<deviceId>/extra_tlm_data/vehicleState");
    this->tree.addNode("<vehicleId>/<deviceId>/extra_tlm_data/baseline");
    this->tree.addNode("<vehicleId>/<deviceId>/extra_tlm_data/gpsMapOrigins");
    this->tree.addNode("<vehicleId>/<deviceId>/last_update");
    this->tree.addNode("<vehicleId>/<deviceId>/status");
    this->tree.addNode("<vehicleId>/<deviceId>/status/info");
    this->tree.addNode("<vehicleId>/<deviceId>/status/error");
    this->tree.addNode("<vehicleId>/<deviceId>/status/alert");
    this->tree.addNode("<vehicleId>/<deviceId>/status/canFrequencies");
    this->tree.addNode("<vehicleId>/<deviceId>/status/lapCounterStatus");
    this->tree.addNode("<vehicleId>/<deviceId>/status/lapCounterLaps");
    this->tree.addNode("<vehicleId>/<deviceId>/as");
    this->tree.addNode("<vehicleId>/<deviceId>/as/commands");
    this->tree.addNode("<vehicleId>/<deviceId>/as/commands/setValues");
    this->tree.addNode("<vehicleId>/<deviceId>/as/commands/setStatus");
    this->tree.addNode("<vehicleId>/<deviceId>/fileTransaction/request");
    this->tree.addNode("<vehicleId>/<deviceId>/fileTransaction/response");
    this->tree.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>");
    this->tree.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>/begin");
    this->tree.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>/end");
    this->tree.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>/chunk");
    this->tree.addNode("<vehicleId>/<deviceId>/fileTransaction/<transactionId>/chunk_ack");
    this->tree.addNode("<vehicleId>/<deviceId>/info");
    this->tree.addNode("<vehicleId>/<deviceId>/info/version");
    this->tree.addNode("<vehicleId>/<deviceId>/info/user");
    this->tree.addNode("<vehicleId>/<deviceId>/info/telemetryReplay");
    this->tree.addNode("<vehicleId>/<deviceId>/info/telemetryReplay/ready");
    this->tree.addNode("<vehicleId>/<deviceId>/info/telemetryReplay/start");
    this->tree.addNode("<vehicleId>/<deviceId>/info/session");
    this->tree.addNode("<vehicleId>/<deviceId>/info/session/started");
    this->tree.addNode("<vehicleId>/<deviceId>/info/session/stopped");
    this->tree.addNode("<vehicleId>/<deviceId>/info/session/logging");
    this->tree.addNode("<vehicleId>/<deviceId>/action");
    this->tree.addNode("<vehicleId>/<deviceId>/action/telemetryConfig");
    this->tree.addNode("<vehicleId>/<deviceId>/action/telemetryConfig/set");
    this->tree.addNode("<vehicleId>/<deviceId>/action/telemetryConfig/get");
    this->tree.addNode("<vehicleId>/<deviceId>/action/telemetryConfig/content");
    this->tree.addNode("<vehicleId>/<deviceId>/action/sessionConfig");
    this->tree.addNode("<vehicleId>/<deviceId>/action/sessionConfig/set");
    this->tree.addNode("<vehicleId>/<deviceId>/action/sessionConfig/get");
    this->tree.addNode("<vehicleId>/<deviceId>/action/sessionConfig/content");
    this->tree.addNode("<vehicleId>/<deviceId>/action/carConfig");
    this->tree.addNode("<vehicleId>/<deviceId>/action/carConfig/set");
    this->tree.addNode("<vehicleId>/<deviceId>/action/carConfig/get");
    this->tree.addNode("<vehicleId>/<deviceId>/action/carConfig/content");
    this->tree.addNode("<vehicleId>/<deviceId>/action/baselineConfig");
    this->tree.addNode("<vehicleId>/<deviceId>/action/baselineConfig/set");
    this->tree.addNode("<vehicleId>/<deviceId>/action/baselineConfig/get");
    this->tree.addNode("<vehicleId>/<deviceId>/action/baselineConfig/content");
    this->tree.addNode("<vehicleId>/<deviceId>/action/handcartSettings");
    this->tree.addNode("<vehicleId>/<deviceId>/action/handcartSettings/set");
    this->tree.addNode("<vehicleId>/<deviceId>/action/handcartSettings/get");
    this->tree.addNode("<vehicleId>/<deviceId>/action/handcartSettings/content");
    this->tree.addNode("<vehicleId>/<deviceId>/action/rawJsonConfig");
    this->tree.addNode("<vehicleId>/<deviceId>/action/rawJsonConfig/set");
    this->tree.addNode("<vehicleId>/<deviceId>/action/rawJsonConfig/get");
    this->tree.addNode("<vehicleId>/<deviceId>/action/rawJsonConfig/content");
    this->tree.addNode("<vehicleId>/<deviceId>/action/lapCounterConfig");
    this->tree.addNode("<vehicleId>/<deviceId>/action/lapCounterConfig/set");
    this->tree.addNode("<vehicleId>/<deviceId>/action/lapCounterConfig/get");
    this->tree.addNode("<vehicleId>/<deviceId>/action/lapCounterConfig/content");
    this->tree.addNode("<vehicleId>/<deviceId>/action/lapCounterTrack");
    this->tree.addNode("<vehicleId>/<deviceId>/action/lapCounterTrack/set");
    this->tree.addNode("<vehicleId>/<deviceId>/action/lapCounterTrack/get");
    this->tree.addNode("<vehicleId>/<deviceId>/action/lapCounterTrack/content");
    this->tree.addNode("<vehicleId>/<deviceId>/action/kill");
    this->tree.addNode("<vehicleId>/<deviceId>/action/start");
    this->tree.addNode("<vehicleId>/<deviceId>/action/reset");
    this->tree.addNode("<vehicleId>/<deviceId>/action/stop");
    this->tree.addNode("<vehicleId>/<deviceId>/action/startBaseline");
    this->tree.addNode("<vehicleId>/<deviceId>/action/stopBaseline");
    this->tree.addNode("<vehicleId>/<deviceId>/action/precharge");
    this->tree.addNode("<vehicleId>/<deviceId>/action/balance");
    this->tree.addNode("<vehicleId>/<deviceId>/action/stopBalance");
    this->tree.addNode("<vehicleId>/<deviceId>/action/charge");
    this->tree.addNode("<vehicleId>/<deviceId>/action/stopCharge");
    this->tree.addNode("<vehicleId>/<deviceId>/action/raw");
    this->tree.addNode("<vehicleId>/<deviceId>/action/resetLapcounter");
    this->tree.addNode("<vehicleId>/<deviceId>/action/setLapcounterStatus");
    this->tree.addNode("<vehicleId>/simulator");
    this->tree.addNode("<vehicleId>/simulator/inputs");
    this->tree.addNode("<vehicleId>/simulator/outputs");
    this->tree.addNode("<vehicleId>/simulator/initialState");
}

void MessageParser::setMessageParse(Topic topic, parse_t parse, void* argument) {
    auto node = this->tree.findNode(GetTopic(topic).topic);

    if(node != nullptr) {
        (*node)->parse = std::make_unique<parse_t>(parse);
        
        if(argument != nullptr) {
            (*node)->argument = argument; 
        }
    }
}

void MessageParser::parseMessage(const Variables& variables, const std::string& topic, const std::string& payload) {
    auto nodes = this->tree.findNodesVariables(topic, variables);

    for(auto& node : nodes) {
        if((*node)->parse != nullptr) {
            (*(*node)->parse)(payload, (*node)->argument);
        }
    }
}
}