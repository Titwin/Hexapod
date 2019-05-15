#include "Localization.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>

#include "Utils/Utils.hpp"
#include "Maths/MathConversion.hpp"

//Default
Localization::Localization(const std::string& totemFileName): visionConfidence(0.f), odometryConfidence(0.f)
{
    std::ifstream myfile;
    myfile.open(totemFileName, std::ifstream::in);
    if(!myfile.is_open())
        throw std::invalid_argument("Localization : fail to load " + totemFileName + " file");

    std::string s;
    while(std::getline(myfile, s, '\n'))
    {
        if(s.empty())
            continue;
        if(s.find("Totem") != std::string::npos)
        {
            std::pair<uint8_t, SmallTransform> p = parse(s.substr(s.find("Totem") + 6));
            /*int totemId;
            std::stringstream sstream(s);
            sstream.ignore(s.find("Totem") + 2, 'm');
            sstream >> totemId;*/

            Totem totem;
            totem.transform = p.second.getTransform();
            totem.id = p.first;
            totemList.push_back(totem);
        }
        else if(s.size() > 10)
        {
            std::pair<uint8_t, SmallTransform> p = parse(s);
            totemList.back().markerList[p.first] = p.second.getTransform();;
        }
    }
    myfile.close();


    /*for(unsigned int i=0; i<totemList.size(); i++)
    {
        std::cout<<"Totem : "<<(int)totemList[i].id<<std::endl;
        std::cout<<totemList[i].transform;
        for(auto it = totemList[i].markerList.begin(); it != totemList[i].markerList.end(); it++)
            std::cout<<" marker "<<(int)it->first<<std::endl;
        std::cout<<std::endl;
    }*/
}
Localization::~Localization()
{}
//

//  Public functions
void Localization::update(std::string visionResult, const float& elapsedTime, const MyVector3f& robotTranslationSpeed, const MyVector3f& robotRotationSpeed)
{
    //{ odomery integration
        float v = 0;
        if(robotTranslationSpeed.y > 0)
            v = 10;
        else if(robotTranslationSpeed.y < 0)
            v = -10;

        float a = 0;
        if(robotRotationSpeed.x > 0)
            a = -18;
        else if(robotRotationSpeed.x < 0)
            a = 18;

        MyMatrix4f M = MyMatrix4f::translation(elapsedTime * MyVector3f(v,0,0)) *
                       MyMatrix4f::rotation(elapsedTime * a, MyVector3f(0,1,0));

        odometryTransform = odometryTransform * M;
        odometryConfidence += elapsedTime * 4;      // 4cm per second
    //}

    //  clear per frame data
    seenMarkers.clear();
    positionCloud.clear();
    orphanMarkers.clear();

    //  parse vision module result
    if(!visionResult.empty())
    {
        ///  parse vision module result
        std::istringstream iss(visionResult);
        std::string s;
        while(std::getline(iss, s, '\n'))
        {
            std::pair<uint8_t, SmallTransform> p = parse(s);
            seenMarkers[p.first] = p.second;
        }

        ///  compute each camera position
        for(std::map<uint8_t, SmallTransform>::iterator it=seenMarkers.begin(); it!=seenMarkers.end(); it++)
        {
            std::pair<int, MyMatrix4f> p = getTotemAndMarkerTransform(it->first);
            if(p.first >= 0)
            {
                try
                {
                    std::pair<float, MyMatrix4f> recontructedPoint;

                    MyMatrix4f w2m = totemList[p.first].transform *         //  world to totem
                                     p.second;                              //  totem to marker
                    MyMatrix4f m2c = it->second.getTransform().inverse();   //  marker to camera
                    MyVector3f z1 = w2m.getZ().normalize();                 //  marker z vector direction (in world space)
                    MyVector3f z2 = robotTransform.getZ().normalize();      //  camera z vector direction (in marker space)

                    //std::cout<<(int)it->first<<" "<<w2m.getOrigin()<<std::endl;

                    float w = std::abs(z1*z2);
                    constexpr float thresholdAngle(30.f);
                    constexpr float threshold1(cos(3.14159265/2 - thresholdAngle * 3.14159265 / 180.));
                    constexpr float threshold2(cos(thresholdAngle * 3.14159265 / 180.));

                    if(w < threshold1)
                        recontructedPoint.first = MathConversion::remap(w, 0.f, threshold1, 0.f, 1.f);
                    else if(w > threshold2)
                        recontructedPoint.first = MathConversion::remap(w, threshold2, 1.f, 1.f, 0.3f);
                    else
                        recontructedPoint.first = 1.f;

                    w = std::max(0.1f, std::min(1.f, 1.3f - w));
                    recontructedPoint.first = w*w*w*w;
                    recontructedPoint.second = w2m * m2c;                   //  world to camera

                    positionCloud[p.first].push_back(recontructedPoint);
                }
                catch(const std::exception& e)
                {
                    std::cout<<Utils::WARNING<<" Localization : non invertible marker matrix : "<<(int)it->first<<'\n'<<e.what();
                }
            }
            else
            {
                orphanMarkers[it->first] = it->second.getTransform();
            }
        }

        /// compute cloud centroid and error estimation
        //std::cout<<"----"<<std::endl;
        for(auto it=positionCloud.begin(); it!=positionCloud.end(); it++)
        {
            MyMatrix4f centroidTransform = MyMatrix4f::zero();
            float totalWeight = 0.0f;

            for(unsigned int i=0; i<it->second.size(); i++)
            {
                float w = it->second[i].first;
                MyMatrix4f M = it->second[i].second;

                centroidTransform += w*M;
                totalWeight += w;
            }

            centroidTransform /= totalWeight;
            MyVector3f centroidPosition = centroidTransform.getOrigin();
            float variance = 0.0f;

            for(unsigned int i=0; i<it->second.size(); i++)
                variance += (centroidPosition  - it->second[i].second.getOrigin()).square();
            float standardDeviation = std::sqrt(variance) / it->second.size();
            centroids[it->first] = std::pair<float, MyMatrix4f>(standardDeviation, centroidTransform);

            //std::cout<<standardDeviation<<std::endl<<centroidTransform<<std::endl;
        }

        /// compute centroids weighted average
        visionConfidence = 0.f;
        MyMatrix4f M = MyMatrix4f::zero();
        for(auto it=centroids.begin(); it!=centroids.end(); it++)
        {
            M += it->second.second;
            visionConfidence += it->second.first;
        }
        M /= centroids.size();
        visionConfidence /= centroids.size();

        for(auto it = orphanMarkers.begin(); it!=orphanMarkers.end(); it++)
            it->second = M * it->second;
        visionTransform = M;// * cameraToRobotTranfsorm;

        //std::cout<<"final "<<visionConfidence<<std::endl<<visionTransform<<std::endl;

        odometryTransform.a[0][3] = visionTransform.getOrigin().x;
        odometryTransform.a[2][3] = visionTransform.getOrigin().z;
        odometryConfidence = visionConfidence;
    }
    robotTransform = odometryTransform;
}


MyMatrix4f Localization::getRobotTransform() const
{
    return robotTransform;
}
MyMatrix4f Localization::getEmbedRobotTransform()
{
    return robotTransform*cameraToRobotTranfsorm;
}
bool Localization::setTotemTransform(const uint8_t& id, const MyVector3f& p, const MyQuaternionf& q)
{
    for(unsigned int i=0; i<totemList.size(); i++)
    {
        if(totemList[i].id == id)
        {
            totemList[i].transform = MyMatrix4f::translation(p) * MathConversion::toMat4(q);
            return true;
        }
    }
    return false;
}
void Localization::setRobotTransform(const MyVector3f& position, const MyQuaternionf& rotation)
{
    robotTransform = MyMatrix4f::translation(position) * MathConversion::toMat4(rotation);
}
void Localization::setCameraTransform(const MyVector3f& position, const MyQuaternionf& rotation, const MyVector3f& scale)
{
    cameraToRobotTranfsorm = MyMatrix4f::translation(position) * MathConversion::toMat4(rotation) * MyMatrix4f::scale(scale);
}
//


//  Protected functions
std::pair<uint8_t, Localization::SmallTransform> Localization::parse(std::string s)
{
    SmallTransform t;

    int id;
    std::stringstream sstream(s);

    sstream >> id;
    sstream.ignore(3, ':');
    sstream >> t.position.x; sstream >> t.position.y; sstream >> t.position.z;
    sstream >> t.rotation.x; sstream >> t.rotation.y; sstream >> t.rotation.z; sstream >> t.rotation.w;
    sstream >> t.scale.x; sstream >> t.scale.y; sstream >> t.scale.z;
    t.position *= 100;// from m to cm
    t.rotation.normalize();

    return std::pair<uint8_t, SmallTransform>(id, t);
}
std::pair<int, MyMatrix4f> Localization::getTotemAndMarkerTransform(const uint8_t& id)
{
    for(unsigned int i=0; i<totemList.size(); i++)
    {
        std::map<uint8_t, MyMatrix4f>::iterator it = totemList[i].markerList.find(id);
        if(it != totemList[i].markerList.end())
            return std::pair<int, MyMatrix4f>(i, it->second);
    }
    return std::pair<int, MyMatrix4f>(-1, MyMatrix4f());
}
const Localization::Totem& Localization::getTotem(const uint8_t& totemId)
{
    for(unsigned int i=0; i<totemList.size(); i++)
    {
        if(totemList[i].id == totemId)
            return totemList[i];
    }
    return Totem();
}


MyMatrix4f Localization::SmallTransform::getTransform() const
{
    return MyMatrix4f::translation(position) * MathConversion::toMat4(rotation) * MyMatrix4f::scale(scale);
};
//
