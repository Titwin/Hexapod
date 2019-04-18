#ifndef SERIALIZER_HPP_INCLUDED
#define SERIALIZER_HPP_INCLUDED


#include <string>

#include "Network.hpp"
#include "Maths/MyVector.hpp"

class Serializer
{
    public:
        /// Special
        enum MessageType
        {
            SYSTEM = 0,
            MOTORS,
            SLAVES
        };

        /// Default
        Serializer();
        //

        /// Public functions
        static std::string serialize(std::map<uint8_t, Network::Node*>* nodelist, const bool& zipped = true);
        static std::string serialize(const MyVector3f& v, std::string input = "");
        static std::string serialize(const MyVector4f& v, std::string input = "");
        //

    protected:
        /// Protected functions
        static std::string nodeType(Network::Node* node);
        static std::string serialize(Network::LegBoard* legBoard, const uint8_t& id, const int& indent);
        static std::string serialize(Network::Scs15* motor, const uint8_t& id, const int& indent);
        //
};

#endif // SERIALIZER_HPP_INCLUDED
