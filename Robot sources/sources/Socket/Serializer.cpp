#include "Serializer.hpp"


/// Public functions
std::string Serializer::serialize(std::map<uint8_t, Network::Node*>* nodelist, const bool& zipped)
{
    std::string endl = (zipped ? "" : "\n");
    std::string space = (zipped ? "" : " ");
    std::string tab = (zipped ? "" : "\t");
    std::string result;

    result += nodeType(nodelist->begin()->second)+ "List" + endl;
    result += '{' + endl;

    for(std::map<uint8_t, Network::Node*>::iterator it = nodelist->begin(); it != nodelist->end(); ++it)
    {
        if(it->second->type == Network::NODE_SCS15)
            result += serialize(static_cast<Network::Scs15*>(it->second), it->first, (zipped ? -1 : 1));
        else if(it->second->type == Network::NODE_LEGBOARD)
            result += serialize(static_cast<Network::LegBoard*>(it->second), it->first, (zipped ? -1 : 1));
    }

    result += '}' + endl;
    return result;
}
std::string Serializer::serialize(const MyVector3f& v, std::string input)
{
    return input + "{\"x\":" + std::to_string(v.x) + ",\"y\":" + std::to_string(v.y) + ",\"z\":" + std::to_string(v.z) + "}";
}
std::string Serializer::serialize(const MyVector4f& v, std::string input)
{
    return input + "{\"x\":" + std::to_string(v.x) + ",\"y\":" + std::to_string(v.y) + ",\"z\":" + std::to_string(v.z) + ",\"w\":" + std::to_string(v.w) + "}";
}
//

/// Protected functions
std::string Serializer::nodeType(Network::Node* node)
{
    switch(node->type)
    {
        case Network::NODE_LEGBOARD: return "LegBoard";
        case Network::NODE_SCS15: return "SCS15";
        default: return "unknown";
    }
}


std::string Serializer::serialize(Network::LegBoard* legBoard, const uint8_t& id, const int& indent)
{
    std::string begin = "";
    for(int i=0; i<indent; i++)
        begin += '\t';
    std::string endl = ((indent>0) ? "\n" : "");
    std::string space = ((indent>0) ? " " : "");
    std::string tab = ((indent>0) ? "\t" : "");

    std::string json;
    json += begin + "lb" + std::to_string((int)id) + endl;
    json += begin + '{' + endl;

    json += begin + tab + "\"i\"" + space + ":"  + space  + std::to_string((int)id) + "," + endl;
    json += begin + tab + "\"s\"" + space + ":"  + space  + std::to_string(legBoard->state) + "," + endl;
    json += begin + tab + "\"S\"" + space + ":"  + space  + std::to_string(legBoard->shield) + "," + endl;
    json += begin + tab + "\"F\"" + space + ":"  + space  + std::to_string(legBoard->force) + "," + endl;
    json += begin + tab + "\"d\"" + space + ":"  + space  + std::to_string(legBoard->distance) + "," + endl;
    json += begin + tab + "\"r\"" + space + ":"  + space  + std::to_string(legBoard->red) + "," + endl;
    json += begin + tab + "\"g\"" + space + ":"  + space  + std::to_string(legBoard->green) + "," + endl;
    json += begin + tab + "\"b\"" + space + ":"  + space  + std::to_string(legBoard->blue) + endl;

    json += begin + '}' + endl;
    return json;
}
std::string Serializer::serialize(Network::Scs15* motor, const uint8_t& id, const int& indent)
{
    std::string begin = "";
    for(int i=0; i<indent; i++)
        begin += '\t';
    std::string endl = ((indent>0) ? "\n" : "");
    std::string space = ((indent>0) ? " " : "");
    std::string tab = ((indent>0) ? "\t" : "");

    std::string json;
    json += begin + "m" + std::to_string((int)id) + endl;
    json += begin + '{' + endl;

    json += begin + tab + "\"i\"" + space + ":"  + space  + std::to_string((int)id) + "," + endl;
    json += begin + tab + "\"p\"" + space + ":"  + space  + std::to_string(motor->presentPos) + "," + endl;
    json += begin + tab + "\"P\"" + space + ":"  + space  + std::to_string(motor->targetPos) + "," + endl;
    json += begin + tab + "\"s\"" + space + ":"  + space  + std::to_string(motor->speed) + "," + endl;
    json += begin + tab + "\"t\"" + space + ":"  + space  + std::to_string(motor->torque) + "," + endl;
    json += begin + tab + "\"tM\"" + space + ":"  + space  + std::to_string(motor->torqueLimit) + "," + endl;
    json += begin + tab + "\"T\"" + space + ":"  + space  + std::to_string(motor->temperature) + endl;

    json += begin + '}' + endl;
    return json;
}
//
