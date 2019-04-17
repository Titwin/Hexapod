#ifndef LOCALIZATION_HPP_INCLUDED
#define LOCALIZATION_HPP_INCLUDED

#include <utility>
#include <string>
#include <vector>
#include <map>

#include "Maths/MyVector.hpp"
#include "Maths/MyMatrix.hpp"
#include "Maths/MyQuaternion.hpp"


class Localization
{
    public:
        //Default
        Localization(const std::string& totemFileName);
        ~Localization();
        //

        //  Public functions
        void update(std::string visionResult);

        MyVector3f getRobotPosition() const;
        bool setTotemTransform(const uint8_t& id, const MyVector3f& p, const MyQuaternionf& q);
        //

        struct Totem
        {
            uint8_t id;
            MyMatrix4f transform;
            std::map<uint8_t, MyMatrix4f> markerList;
        };

    protected:
        //  Protected functions
        std::pair<uint8_t, MyMatrix4f> parse(std::string s);
        std::pair<uint8_t, MyMatrix4f> parseScale(std::string s);
        std::pair<int, MyMatrix4f> getTotemAndMarkerTransform(const uint8_t& id);
        //

        //  Attributes
        MyMatrix4f visionTransform;
        MyMatrix4f odometryTransform;

        float visionConfidence;
        float odometryConfidence;

        std::vector<Totem> totemList; //  list of anchors, anchors are dictionary of markers id : transform
        std::map<uint8_t, MyMatrix4f> seenMarkers;
        //
};

#endif // LOCALIZATION_HPP_INCLUDED
