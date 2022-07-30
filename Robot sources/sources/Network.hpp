#ifndef NETWORK_HPP_INCLUDED
#define NETWORK_HPP_INCLUDED

#include <map>
#include <vector>
#include <atomic>
#include <pthread.h>

#include "Controllers/SCS15Controller.hpp"
#include "Controllers/LegBoardController.hpp"

#define NET_STOP_CONTACT    10
#define NET_NODE_COUNT      254

class Network
{
    public:
        bool TORQUE;


        /// Miscellaneous
        enum NodeType
        {
            NODE_LEGBOARD = 0,
            NODE_SCS15,

            TYPE_SIZE = 2
        };
        struct Node
        {
            NodeType type;
            uint8_t connectionFails;
            uint8_t needSync;
        };
        enum NodeAttributes // only attributes to overwrite
        {
            ///  Leg board
            LEGBOARD_ACTION,
            LEGBOARD_RESET,

            LEGBOARD_RED,
            LEGBOARD_GREEN,
            LEGBOARD_BLUE,

            ///  SCS15
            SCS15_RESET,

            SCS15_TARGET_POSITION,
            SCS15_TORQUE,
            SCS15_SPEED,
            SCS15_TORQUE_LIMIT
        };
        struct LegBoard : public Node
        {
            uint16_t distance;
            uint16_t force;
            uint8_t shield;
            uint8_t red;
            uint8_t green;
            uint8_t blue;
            uint8_t state;
        };
        struct Scs15 : public Node
        {
            uint16_t targetPos;
            uint16_t presentPos;
            uint16_t speed;
            uint16_t torqueLimit;
            uint8_t temperature;
            uint8_t torque;
        };

        enum Configuration
        {
            CONFIG_ACCURATE_DISTANCE = (1 << 0),
        };
        //

        /// Default
        Network();
        //

        /// Set/get functions
        void nodeBroadcast(const NodeType& type, const NodeAttributes& attribute, const uint16_t& value = 0);
        void getNodeMap(std::map<NodeType, std::map<uint8_t, Node*> >* destination);
        void setNodeAttributes(const NodeType& type, const NodeAttributes& attribute, const uint8_t& id, const uint16_t& value);
        void setSyncNodeAttributes(const NodeType& type, const NodeAttributes& attribute, const uint8_t& idLength, const uint8_t* const id, const uint16_t* const value);
        //

        /// Special
        void initialize(const bool& verbose = false);
        int synchonize(const bool& verbose = false);
        void nodeScan(const bool& verbose = false);
        void scheduler();
        //

        /// Attributes
        std::atomic<uint32_t> TTLbusError;
        std::atomic<uint8_t> configuration;
        //

    protected:
        /// Standard frame data timing functions
        int synchronizeScs15(const bool& verbose);
        int synchronizeLegBoard(const bool& verbose);

        void setNodeAttributesNoLock(const NodeType& type, const NodeAttributes& attribute, const uint8_t& id, const uint16_t& value, const uint8_t& needUpdate = 1);
        //

        /// Protected functions
        std::string verboseNodeType(const NodeType& type) const;
        void broadcastAll(const bool& verbose);
        Node* tryMappingNode(const NodeType& type, const uint8_t& ID, const bool& verbose);
        Node* tryMappingSlave(const uint8_t& ID, const bool& verbose);
        //

        /// Attributes
        LegBoardController LEGBOARD;
        SCS15Controller SCS15;

        pthread_mutex_t mutex;
        std::map<NodeType, std::map<uint8_t, Node*> > nodeMap;
        std::map<NodeType, std::vector<std::pair<NodeAttributes, uint16_t> > > nodesBroadcastMessage;

        int nodeDiscoveryType;
        uint8_t nodeDiscoveryID;
        //
};

#endif // NETWORK_HPP_INCLUDED
