#include "Network.hpp"


/// Default
Network::Network()
{
    mutex = PTHREAD_MUTEX_INITIALIZER;
    nodeDiscoveryType = (int)NODE_LEGBOARD;
    nodeDiscoveryID = 0;
}
//

/// Set/get functions
void Network::nodeBroadcast(const NodeType& type, const NodeAttributes& attribute, const uint16_t& value)
{
    pthread_mutex_lock(&mutex);
    nodesBroadcastMessage[type].push_back(std::pair<NodeAttributes, uint16_t>(attribute, value));
    pthread_mutex_unlock(&mutex);
}
void Network::getNodeMap(std::map<NodeType, std::map<uint8_t, Node*> >* destination)
{
    pthread_mutex_lock(&mutex);
    if(!nodeMap.empty() && destination)
        *destination = nodeMap;
    else if(destination) destination->clear();
    pthread_mutex_unlock(&mutex);
}
void Network::setNodeAttributes(const NodeType& type, const NodeAttributes& attribute, const uint8_t& id, const uint16_t& value)
{
    pthread_mutex_lock(&mutex);
    auto it = nodeMap[type].find(id);
    if(it != nodeMap[type].end())
    {
        if(type == NODE_LEGBOARD)
        {
            LegBoard* const leg = static_cast<LegBoard*>(it->second);
            switch(attribute)
            {
                case LEGBOARD_RED:   leg->red = value;   leg->needSync = 1; break;
                case LEGBOARD_GREEN: leg->green = value; leg->needSync = 1; break;
                case LEGBOARD_BLUE:  leg->blue = value;  leg->needSync = 1; break;
                default: std::cout<<"LegBoard attribute not supported yet "<<std::endl; break;
            }
        }
        else if(type == NODE_SCS15)
        {
            Scs15* const scs15 = static_cast<Scs15*>(it->second);
            switch(attribute)
            {
                case SCS15_TARGET_POSITION: scs15->targetPos = value;   break;
                case SCS15_TORQUE:          scs15->torque = value;      scs15->needSync = 1; break;
                case SCS15_SPEED:           scs15->speed = value;       scs15->needSync = 1; break;
                case SCS15_TORQUE_LIMIT:    scs15->torqueLimit = value; scs15->needSync = 1; break;
                default: std::cout<<"Scs15 attribute not supported"<<std::endl; break;
            }
        }
        else std::cout<<"Node type not supported yet"<<std::endl;
    }
    pthread_mutex_unlock(&mutex);
}
void Network::setSyncNodeAttributes(const NodeType& type, const NodeAttributes& attribute, const uint8_t& idLength, const uint8_t* const id, const uint16_t* const value)
{
    for(uint8_t i=0; i<idLength; i++)
        setNodeAttributes(type, attribute, id[i], value[i]);
}
//

/// Special
void Network::initialize(const bool& verbose)
{
    pthread_mutex_lock(&mutex);
    //  try mapping list SCS15 -> starting byte is 0xFF
    for(uint8_t i=0; i<NET_NODE_COUNT; i++)
        tryMappingNode(NODE_SCS15, i, verbose);

    //  try mapping list Slaves -> starting byte is 0xFE -> Legboard controller is standard for this
    for(uint8_t i=0; i<NET_NODE_COUNT; i++)
    {
        if(LEGBOARD.ping(i))
        {
            uint8_t type = LEGBOARD.getType(i);
            if(type == 0x01) // Firmware constant for LEGBOARD slaves internal type
                tryMappingNode(NODE_LEGBOARD, i, verbose);
            else if(verbose)
                std::cout << "Automatic slave mapping fail : unknown internal type" << std::endl;
        }
    }


    pthread_mutex_unlock(&mutex);
}
int Network::synchonize(const bool& verbose)
{
    int updates = 0;
    pthread_mutex_lock(&mutex);

    /// Broadcast pass
    broadcastAll(verbose);

    /// Synchronize pass
    updates += synchronizeScs15(verbose);
    updates += synchronizeLegBoard(verbose);

    /// clean dead node pass
    for(auto it = nodeMap.begin(); it!= nodeMap.end(); it++)
    {
        for(auto it2 = it->second.begin(); it2 != it->second.end();)
        {
            if(it2->second->connectionFails >= NET_STOP_CONTACT)
            {
                if(verbose)
                    std::cout << "loose connection with " << verboseNodeType(it->first) << " " << (int)it2->first << std::endl;
                delete it2->second;
                it2 = it->second.erase(it2);
            }
            else it2++;
        }
    }

    pthread_mutex_unlock(&mutex);
    return updates;
}
void Network::nodeScan(const bool& verbose)
{
    for(uint8_t i=0; i<NET_NODE_COUNT; i++,nodeDiscoveryID++)
    {
        //  increment safely discoveryID
        if(nodeDiscoveryID >= NET_NODE_COUNT)
        {
            nodeDiscoveryID = 0;
            nodeDiscoveryType++;
            nodeDiscoveryType %= (int)TYPE_SIZE;
        }

        //  check if current discovery node is mapped
        pthread_mutex_lock(&mutex);
        if(nodeMap[(NodeType)nodeDiscoveryType].find(nodeDiscoveryID) == nodeMap[(NodeType)nodeDiscoveryType].end())
        {
            Node* n = tryMappingNode((NodeType)nodeDiscoveryType, nodeDiscoveryID, verbose);
            if(n)
            {
                nodeMap[(NodeType)nodeDiscoveryType][nodeDiscoveryID] = n;
                nodeDiscoveryID++;
                pthread_mutex_unlock(&mutex);
                break;
            }
        }
        pthread_mutex_unlock(&mutex);
    }
}
void Network::scheduler()
{

}
//


/// Standard frame data timing functions
void Network::broadcastAll(const bool& verbose)
{
    for(auto it = nodesBroadcastMessage.begin(); it!= nodesBroadcastMessage.end(); it++)
    {
        for(unsigned int i=0; i<it->second.size(); i++)
        {
            const NodeType& type = it->first;
            const NodeAttributes& attribute = it->second[i].first;
            const uint16_t value = it->second[i].second;
            const uint8_t lowValue = value&0xFF;

            switch(type)
            {
                case NODE_LEGBOARD:
                    switch(attribute)
                    {
                        case LEGBOARD_ACTION: LEGBOARD.action(LegBoardController::BROADCAST_ID); break;
                        case LEGBOARD_RESET:  LEGBOARD.reset(LegBoardController::BROADCAST_ID);  break;

                        case LEGBOARD_RED:    LEGBOARD.setRegister(LegBoardController::BROADCAST_ID, LegBoardController::REG_RED, 1, &lowValue);   break;
                        case LEGBOARD_GREEN:  LEGBOARD.setRegister(LegBoardController::BROADCAST_ID, LegBoardController::REG_GREEN, 1, &lowValue); break;
                        case LEGBOARD_BLUE:   LEGBOARD.setRegister(LegBoardController::BROADCAST_ID, LegBoardController::REG_BLUE, 1, &lowValue);  break;

                        default: std::cout << "LEGBOARD broadcast attribute not supported yet" << std::endl; break;
                    }
                    break;

                case NODE_SCS15:
                    switch(attribute)
                    {
                        case SCS15_RESET: SCS15.reset(SCS15Controller::BROADCAST_ID);  break;

                        case SCS15_TORQUE:           SCS15.setRegister(SCS15Controller::BROADCAST_ID, SCS15Controller::P_TORQUE_ENABLE, 1, &lowValue);            break;
                        case SCS15_TARGET_POSITION:  SCS15.setRegister(SCS15Controller::BROADCAST_ID, SCS15Controller::P_GOAL_POSITION_L, 2, (uint8_t*)&value);   break;
                        case SCS15_SPEED:            SCS15.setRegister(SCS15Controller::BROADCAST_ID, SCS15Controller::P_GOAL_TIME_L, 2, (uint8_t*)&value);       break;
                        case SCS15_TORQUE_LIMIT:     SCS15.setRegister(SCS15Controller::BROADCAST_ID, SCS15Controller::P_MAX_TORQUE_L, 2, (uint8_t*)&value);      break;

                        default: std::cout << "scs15 broadcast attribute not supported yet" << std::endl; break;
                    }
                    break;

                default:
                    std::cout << "Node type broadcasting not supported yet" << std::endl;
                    break;
            }
        }
        it->second.clear();
    }
}

int Network::synchronizeScs15(const bool& verbose)
{
    int updates = 0;
    uint8_t respondingIDN = 0;
    uint8_t idsSend[20];
    uint16_t posSend[20];
    for(auto it = nodeMap[NODE_SCS15].begin(); it != nodeMap[NODE_SCS15].end(); it++)
    {
        Scs15* const scs15 = static_cast<Scs15*>(it->second);

        /// Try contacting node
        int pos = SCS15.getPosition(it->first);
        if(pos >= 1024)
        {
            TTLbusError++;
        }
        else if(pos >= 0)
        {
            scs15->presentPos = pos;
            scs15->connectionFails = 0;

            idsSend[respondingIDN] = it->first;
            posSend[respondingIDN] = scs15->targetPos;
            respondingIDN++;

            updates++;
        }
        else scs15->connectionFails++;

        /// Send a sync packet
        if(respondingIDN == 20)
        {
          SCS15.syncSetPosition(idsSend, respondingIDN, posSend);
          respondingIDN = 0;
        }
    }
    if(respondingIDN) SCS15.syncSetPosition(idsSend, respondingIDN, posSend);
    return updates;
}
int Network::synchronizeLegBoard(const bool& verbose)
{
    int updates = 0;
    for(auto it = nodeMap[NODE_LEGBOARD].begin(); it != nodeMap[NODE_LEGBOARD].end(); it++)
    {
        LegBoard* const leg = static_cast<LegBoard*>(it->second);

        int state = LEGBOARD.getState(it->first);
        if(state >= 0)
        {
            uint8_t mask = (1<<LegBoardController::SHIELD_CONTACT)|(1<<LegBoardController::DISTANCE_THSD)|(1<<LegBoardController::FORCE_THSD);
            if((state&mask) != ((leg->state)&mask))
            {
                uint8_t response[11];
                if(LEGBOARD.readMemory(it->first, LegBoardController::REG_DISTANCE_H, 5, response))
                {
                    //  update LegBoard node infos
                    leg->shield = response[9];
                    leg->distance = LEGBOARD.bytes2Int(response[6], response[5]);
                    leg->force = LEGBOARD.bytes2Int(response[8], response[7]);

                    if(verbose)
                    {
                        std::cout << "  LegBoard " << (int)it->first << " update" << std::endl;
                        std::cout << "     type : " << (int)leg->type << std::endl;
                        std::cout << "     state : " << std::hex << (int)leg->state << std::dec << std::endl;
                        std::cout << "     shield : " << std::hex << (int)leg->shield << std::dec << std::endl;
                        std::cout << "     distance : " << (int)leg->distance << std::endl;
                        std::cout << "     force : " << (int)leg->force << std::endl;
                        std::cout << "     red : " << (int)leg->red << std::endl;
                        std::cout << "     green : " << (int)leg->green << std::endl;
                        std::cout << "     blue : " << (int)leg->blue << std::endl;
                    }
                }
                else
                {
                    if(verbose)
                        std::cout << "  LegBoard " << (int)it->first << " update fail" << std::endl;
                    TTLbusError++;
                }
            }
            else if((state&mask) == (1<<LegBoardController::DISTANCE_THSD))
            {
                int d = LEGBOARD.getDistance(it->first);
                if(d >= 0)
                {
                    leg->distance = d;
                    if(verbose)
                    {
                        std::cout << "  slave " << (int)it->first << " distance update" << std::endl;
                        std::cout << "     type : " << (int)leg->type << std::endl;
                        std::cout << "     state : " << std::hex << (int)leg->state << std::dec << std::endl;
                        std::cout << "     shield : " << std::hex << (int)leg->shield << std::dec << std::endl;
                        std::cout << "     distance : " << (int)leg->distance << std::endl;
                        std::cout << "     force : " << (int)leg->force << std::endl;
                        std::cout << "     red : " << (int)leg->red << std::endl;
                        std::cout << "     green : " << (int)leg->green << std::endl;
                        std::cout << "     blue : " << (int)leg->blue << std::endl;
                    }
                }
                else
                {
                    if(verbose)
                        std::cout << "  slave " << (int)it->first << " distance update fail" << std::endl;
                    TTLbusError++;
                }
            }

            leg->state = state;
            it->second->connectionFails = 0;
            updates++;
        }
        else it->second->connectionFails++;
    }
    return updates;
}
//

/// Protected functions
std::string Network::verboseNodeType(const NodeType& type) const
{
    switch(type)
    {
        case NODE_LEGBOARD: return "LegBoard";
        case NODE_SCS15:    return "Scs15";
        default:            return "Unknown";
    }
}
Network::Node* Network::tryMappingNode(const NodeType& type, const uint8_t& ID, const bool& verbose)
{
    uint8_t response[100];
    if(type == NODE_SCS15 && SCS15.ping(ID))
    {
        if(SCS15.debug(ID, response))
        {
            Scs15* s = new Scs15();
                s->type = NODE_SCS15;
                s->connectionFails = 0;
                s->temperature = response[68];
                s->presentPos = SCS15.bytes2Int(response[62], response[61]);
                s->targetPos = s->presentPos;
                s->torque = response[45];
                s->torqueLimit = SCS15.bytes2Int(response[50], response[49]);
                s->speed = SCS15.bytes2Int(response[22], response[21]);
                s->needSync = 0;

            if(verbose)
            {
                std::cout << "  scs15 " << (int)ID << " map created" << std::endl;
                std::cout << "     temperature : " << (int)s->temperature << std::endl;
                std::cout << "     position : " << (int)s->presentPos << std::endl;
                std::cout << "     goal position : " << (int)s->targetPos << std::endl;
                std::cout << "     torque : " << (int)s->torque << std::endl;
            }
            return s;
        }
        else if(verbose)
        {
            std::cout << "  scs15 " << (int)ID << " map fail" << std::endl;
            TTLbusError++;
        }
    }
    else if(type == NODE_LEGBOARD && LEGBOARD.ping(ID))
    {
        if(LEGBOARD.debug(ID, response))
        {
            //  create RAM mapping
            LegBoard* s = new LegBoard();
                s->type = NODE_LEGBOARD;
                s->connectionFails = 0;
                s->state = response[5];
                s->shield = response[10];
                s->distance = LEGBOARD.bytes2Int(response[7], response[6]);
                s->force = LEGBOARD.bytes2Int(response[9], response[8]);
                s->red = response[11];
                s->green = response[12];
                s->blue = response[13];
                s->needSync = 0;

            if(verbose)
            {
                std::cout << "  slave " << (int)ID << " map created" << std::endl;
                std::cout << "     type : " << (int)s->type << std::endl;
                std::cout << "     state : " << std::hex << (int)s->state << std::dec << std::endl;
                std::cout << "     shield : " << std::hex << (int)s->shield << std::dec << std::endl;
                std::cout << "     distance : " << (int)s->distance << std::endl;
                std::cout << "     force : " << (int)s->force << std::endl;
                std::cout << "     red : " << (int)s->red << std::endl;
                std::cout << "     green : " << (int)s->green << std::endl;
                std::cout << "     blue : " << (int)s->blue << std::endl;
            }
            return s;
        }
    }
    else if(verbose && type != NODE_SCS15 && type != NODE_LEGBOARD)
        std::cout << "node type mapping unsupported yet" << std::endl;
    return nullptr;
}
//

