#include "Localization.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iterator>

#include "Utils/Utils.hpp"

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
            int totemId;
            std::stringstream sstream(s);
            sstream.ignore(s.find("Totem") + 2, 'm');
            sstream >> totemId;

            Totem totem;
            totem.transform = MyMatrix4f();
            totem.id = totemId;
            totemList.push_back(totem);
        }
        else if(s.find("s ") != std::string::npos)
        {
            std::pair<uint8_t, MyMatrix4f> p = parseScale(s);
            totemList.back().markerList[p.first] = totemList.back().markerList[p.first] * p.second;
        }
        else if(s.size() > 10)
        {
            std::pair<uint8_t, MyMatrix4f> p = parse(s);
            totemList.back().markerList[p.first] = p.second;
        }
    }
    myfile.close();
}
Localization::~Localization()
{}
//

//  Public functions
void Localization::update(std::string visionResult)
{
    seenMarkers.clear();

    //  parse vision module result
    if(!visionResult.empty())
    {
        //  parse vision module result
        std::istringstream iss(visionResult);
        std::string s;
        while(std::getline(iss, s, '\n'))
        {
            std::pair<uint8_t, MyMatrix4f> p = parse(s);
            seenMarkers[p.first] = p.second;
        }

        //  compute each camera position
        for(std::map<uint8_t, MyMatrix4f>::iterator it=seenMarkers.begin(); it!=seenMarkers.end(); it++)
        {
            std::pair<int, MyMatrix4f> p = getTotemAndMarkerTransform(it->first);
            if(p.first >= 0)
            {
                try
                {
                    MyMatrix4f worldToTotem = totemList[p.first].transform;
                    MyMatrix4f totemToMarker = p.second;
                    MyMatrix4f markerToCamera = it->second.inverse();

                    MyMatrix4f M = it->second;
                    //MyMatrix4f M = worldToTotem * totemToMarker * markerToCamera;

                    std::cout<<(int)it->first<<std::endl<<M<<std::endl;
                }
                catch(const std::exception& e)
                {
                    std::cout<<Utils::WARNING<<" Localization : non invertible marker matrix : "<<(int)it->first;
                }
            }
            else
            {
                std::cout<<"lone marker found : "<<(int)it->first<<std::endl;
            }
        }
    }
}


MyVector3f Localization::getRobotPosition() const
{
    return MyVector3f(visionTransform.a[0][3], visionTransform.a[1][3], visionTransform.a[2][3]);
}
bool Localization::setTotemTransform(const uint8_t& id, const MyVector3f& p, const MyQuaternionf& q)
{
    for(unsigned int i=0; i<totemList.size(); i++)
    {
        if(totemList[i].id == id)
        {
            totemList[i].transform = MyMatrix4f::translation(p) * q.toMatrix4();
            return true;
        }
    }
    return false;
}
//


//  Protected functions
std::pair<uint8_t, MyMatrix4f> Localization::parse(std::string s)
{
    int id;
    MyVector3f p;
    MyQuaternionf q;
    std::stringstream sstream(s);

    sstream >> id;
    sstream.ignore(3, ':');
    sstream >> p.x; sstream >> p.y; sstream >> p.z;
    sstream >> q.x; sstream >> q.y; sstream >> q.z; sstream >> q.w;

    return std::pair<uint8_t, MyMatrix4f>(id, MyMatrix4f::translation(100 * p) * q.toMatrix4()); // from m to cm
}
std::pair<uint8_t, MyMatrix4f> Localization::parseScale(std::string s)
{
    int id;
    MyVector3f v;
    std::stringstream sstream(s);

    sstream.ignore(3, 's');
    sstream >> id;
    sstream.ignore(3, ':');
    sstream >> v.x; sstream >> v.y; sstream >> v.z;

    return std::pair<uint8_t, MyMatrix4f>(id, MyMatrix4f::scale(v));
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
//
