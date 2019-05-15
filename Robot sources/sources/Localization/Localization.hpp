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
        void update(std::string visionResult, const float& elapsedTime, const MyVector3f& robotTranslationSpeed, const MyVector3f& robotRotationSpeed);


        MyMatrix4f getRobotTransform() const;
        MyMatrix4f getEmbedRobotTransform();
        bool setTotemTransform(const uint8_t& id, const MyVector3f& p, const MyQuaternionf& q);
        void setRobotTransform(const MyVector3f& position, const MyQuaternionf& rotation);
        void setCameraTransform(const MyVector3f& position, const MyQuaternionf& rotation, const MyVector3f& scale);
        //

        //  Useful structures
        class SmallTransform
        {
            public:
                SmallTransform() : position(0,0,0), rotation(0,0,0,1), scale(1,1,1){};
                MyMatrix4f getTransform() const;
                MyVector3f position;
                MyQuaternionf rotation;
                MyVector3f scale;
        };
        struct Totem
        {
            Totem() : id(0xFF){};
            uint8_t id;
            MyMatrix4f transform;
            std::map<uint8_t, MyMatrix4f> markerList;
        };
        //

    protected:
        //  Protected functions
        std::pair<uint8_t, SmallTransform> parse(std::string s);
        std::pair<uint8_t, MyMatrix4f> parseScale(std::string s);
        std::pair<int, MyMatrix4f> getTotemAndMarkerTransform(const uint8_t& id);
        const Totem& getTotem(const uint8_t& totemId);
        //

        //  Attributes
        MyMatrix4f cameraToRobotTranfsorm;
        MyMatrix4f robotTransform;
        MyMatrix4f visionTransform;
        MyMatrix4f odometryTransform;

        float visionConfidence;
        float odometryConfidence;

        std::vector<Totem> totemList;                                                   //  list of anchors, anchors are dictionary of markers id : transform
        std::map<uint8_t, SmallTransform> seenMarkers;                                  //  all markers seen in the frame
        std::map<uint8_t, std::vector<std::pair<float, MyMatrix4f> > > positionCloud;   //  reconstructed cloud for each totem (transform and error estimation for each)
        std::map<uint8_t, MyMatrix4f> orphanMarkers;                                    //  for frame orphan markers a transform
        std::map<uint8_t, std::pair<float, MyMatrix4f> > centroids;                     //  reconstructed centroid for each totems (with standard deviation);
        //
};

#endif // LOCALIZATION_HPP_INCLUDED
