#include "Hexapod.hpp"

#define ZERO_HEIGHT (-6)
#define RADIUS_DOMAIN 16
#define DOMAIN_ANGLE 20


//  Default
Hexapod::Hexapod(std::string s) : name(s),wellInitialized(false)
{
    forwardKModule = new ForwardKinematics();
    inverseKModule = new InverseKinematics();
        inverseKModule->setForwardKinematicsModule(forwardKModule);
    arduino = new Serial();

    legPosition = new MyVector3f[6];
    legTarget = new MyVector3f[6];
    legDisplacement = new MyVector3f[6];
    legDomainCenter = new MyVector3f[6];
    takeoffdownPosition = new MyVector3f[6];
    legSendPosition = new MyVector3f[6];
    legErrorIntegral = new MyVector3f[6];

    //{ Initialize angles offsets
        motorAnglesOffset[0] = 39;      sens[0] = -1;
        motorAnglesOffset[1] = 74;      sens[1] =  1;
        motorAnglesOffset[2] =-42;      sens[2] = -1;
        motorAnglesOffset[3] =-49;      sens[3] =  1;

        motorAnglesOffset[4] = 17;      sens[4] = -1;
        motorAnglesOffset[5] = 36;      sens[5] =  1;
        motorAnglesOffset[6] = 89;      sens[6] = -1;
        motorAnglesOffset[7] = -20;      sens[7] =  1;

        motorAnglesOffset[8]  = 0;      sens[8]  = -1;
        motorAnglesOffset[9]  = 2;      sens[9]  =  1;
        motorAnglesOffset[10] = 22;     sens[10] = -1;
        motorAnglesOffset[11] = 13;     sens[11] =  1;

        motorAnglesOffset[12] = 51;     sens[12] = -1;
        motorAnglesOffset[13] =-67;     sens[13] =  1;
        motorAnglesOffset[14] = 76;     sens[14] = -1;
        motorAnglesOffset[15] = 38;     sens[15] =  1;

        motorAnglesOffset[16] =-54;     sens[16] = -1;
        motorAnglesOffset[17] =-9;      sens[17] =  1;
        motorAnglesOffset[18] =-19;     sens[18] = -1;
        motorAnglesOffset[19] =-20;     sens[19] =  1;

        motorAnglesOffset[20] = 2;      sens[20] = -1;
        motorAnglesOffset[21] =-6;      sens[21] =  1;
        motorAnglesOffset[22] =-42;     sens[22] = -1;
        motorAnglesOffset[23] = 25;     sens[23] =  1;
    //}

    //{ Initialize legs machine state attributes
        legsGroup[0] = 0;           legsGroup[3] = 1;
        legsGroup[1] = 1;           legsGroup[4] = 0;
        legsGroup[2] = 0;           legsGroup[5] = 1;

        legsState[0] = ON_GROUND;   legsState[1] = IN_AIR;
        legsState[5] = IN_AIR;      legsState[2] = ON_GROUND;
        legsState[4] = ON_GROUND;   legsState[3] = IN_AIR;
    //}

    //{ Initialize some walk parameters
        stepLength = 7; stepHeight = 6;
        nbGroup = 2;    swingGroup = 1;
        robotHeight = ZERO_HEIGHT;
        torque = false;

        robotGait = TRIPOD;
        robotTargetGait = TRIPOD;
        speedGain = 1.f;
        transitState = 0;

        limitForSwap = 4*stepLength;
        initialLimitForSwap = limitForSwap;

        maxTranslationSpeedMagnitude = 0.5;
        maxRotationSpeedMagnitude = maxTranslationSpeedMagnitude/20.f;

        gainP = 0.7;
    //}

    //{ Initialize some geometric parameter
        initiateLegDomainCenter(RADIUS_DOMAIN,DOMAIN_ANGLE,ZERO_HEIGHT);

        for(int i=0;i<6;i++)
        {
            blockAnimFlag[i] = false;
            legsIncidence[i] = 0;
            takeoffdownPosition[i] = legDomainCenter[i];
        }
    //}

    //{ Finish initialization
        wellInitialized = tryConnectingToArduino();
        setState(INIT);
    //}

    std::cout<<name<<" created !"<<std::endl;
}
Hexapod::~Hexapod()
{
    delete forwardKModule;
    delete inverseKModule;
    delete arduino;

    delete[] legPosition;
    delete[] legTarget;
    delete[] legDomainCenter;
    delete[] legDisplacement;
    delete[] takeoffdownPosition;
    delete[] legSendPosition;
    delete[] legErrorIntegral;
}
//



//  Public functions
void Hexapod::animate(float elapseTime, MyVector3f translationSpeed, MyVector3f rotationSpeed)
{
    updateLegPositionFromMotorsAngles();

    switch(robotState)
    {
        case INIT:
            animateGotoStart(elapseTime);
            break;
        case WALK:
            animateWalk(elapseTime,translationSpeed,rotationSpeed);
            animateAutonomousLegs(elapseTime);
            updateMotorsAnglesFromLegPosition();
            break;
        case TRANSITION:
            animateTransition(elapseTime);
            break;
        default: robotState = STOP; break;
    }
}

//  Private functions
void Hexapod::animateWalk(float elapseTime, MyVector3f translationSpeed, MyVector3f rotationSpeed)
{
    if(!torque) return;
    if(translationSpeed.length() == 0 && rotationSpeed.length() == 0) return;

    if(robotGait == METACHRONAL_4 || robotGait == TETRAPOD)
    {
        dummy += elapseTime;
        if(dummy >= 2000*M_PI) dummy -= 2000*M_PI;
        translationSpeed = MyVector3f(cos(dummy/1000),sin(dummy/1000),0);
    }
    else dummy = 0;

    //  Prepare parameters for use
    if(elapseTime>16) elapseTime = 16;

    if(robotHeight > -1 && translationSpeed.z < 0) translationSpeed.z = 0;
    else if(robotHeight < -15 && translationSpeed.z > 0) translationSpeed.z = 0;

    if(translationSpeed.length() > speedGain*maxTranslationSpeedMagnitude)
    {
        translationSpeed.normalize();
        translationSpeed *= speedGain*maxTranslationSpeedMagnitude;
    }
    if(rotationSpeed.length() > speedGain*maxRotationSpeedMagnitude)
    {
        rotationSpeed.normalize();
        rotationSpeed *= speedGain*maxRotationSpeedMagnitude;
    }
    if(translationSpeed.length() != 0 && rotationSpeed.length() != 0)
    {
        translationSpeed *= 0.5f;
        rotationSpeed *= 0.5f;
    }

    bool swap = false;
    if(limitForSwap<0.1)
    {
        swap = true;
        swingGroup = (swingGroup+1)%nbGroup;
    }
    limitForSwap = 4*stepLength;

    //  Legs machine state
    for(int i=0;i<6;i++)
    {
        legDisplacement[i] = getLegDisplacement(i,-translationSpeed,-rotationSpeed);
        float limit;

        switch(legsState[i])
        {
            case ON_GROUND:
                if(swap)
                {
                    takeoffdownPosition[i] = legTarget[i];
                    if(legsGroup[i] == swingGroup)
                        legsState[i] = IN_AIR;
                    else
                    {
                        limit = getLegLimitFromDirection(i,legDisplacement[i]);
                        limitForSwap = std::min(limitForSwap,limit);
                    }
                }
                else
                {
                    limit = getLegLimitFromDirection(i,legDisplacement[i]);
                    limitForSwap = std::min(limitForSwap,limit);
                }
                break;
            case IN_AIR:
                if(swap)
                {
                    legsState[i] = ON_GROUND;
                    takeoffdownPosition[i] = legTarget[i];
                    limit = getLegLimitFromDirection(i,legDisplacement[i]);
                    limitForSwap = std::min(limitForSwap,limit);
                }
                break;
            default: break;
        }
    }

    //  finish swap
    if(swap)
    {
        initialLimitForSwap = limitForSwap;
        if(initialLimitForSwap<=0)
            initialLimitForSwap = stepLength/(nbGroup-1);
    }

    //  compute new legs positions
    for(int i=0;i<6;i++)
    {
        legDomainCenter[i].z += legDisplacement[i].z;
        takeoffdownPosition[i].z += legDisplacement[i].z;
        robotHeight += legDisplacement[i].z/6.f;

        if(legsState[i] == AUTONOMOUS) continue;
        switch(legsState[i])
        {
            case ON_GROUND:
                legTarget[i] += legDisplacement[i];
                break;
            case IN_AIR:
                legTarget[i] = getAirTarget(i,takeoffdownPosition[i],1.f - limitForSwap/initialLimitForSwap);
                break;
            default: break;
        }

        correctPositionError(i);
        targetAnglesArray[i] = inverseKModule->getAnglesFromPosition(i,legSendPosition[i],legsIncidence[i],motorAnglesArray[i]);
    }
}
void Hexapod::animateGotoStart(float elapseTime)
{
    if(!torque) return;

    uint8_t legAtPosition = 0;
    MyVector3f direction;
    for(int i=0;i<6;i++)
    {
        direction = legTrajectory[i].front().first - legTarget[i];
        if(direction.length() < 0.5f && std::abs(legsIncidence[i]-legTrajectory[i].front().second) < 1)
            legAtPosition++;
        else
        {
            legTarget[i] += 0.01f*elapseTime*direction.normalize();
            if(legsIncidence[i] < legTrajectory[i].front().second)
                legsIncidence[i] += 1;
            else
                legsIncidence[i] -= 1;
        }
    }

    if(legAtPosition == 6)
    {
        for(int i=0;i<6;i++)
        {
            legTrajectory[i].pop_front();
            if(legTrajectory[i].empty()) { setState(WALK); return; }
        }
    }
    else
    {
        for(int i=0;i<6;i++)
        {
            correctPositionError(i);
            targetAnglesArray[i] = inverseKModule->getAnglesFromPosition(i,legSendPosition[i],legsIncidence[i],motorAnglesArray[i]);
        }
    }

    updateMotorsAnglesFromLegPosition();
}
void Hexapod::animateTransition(float elapseTime)
{
    if(!torque) return;

    uint8_t legAtPosition = 0;
    MyVector3f direction;
    for(int i=0;i<6;i++)
    {
        direction = legTrajectory[i].front().first - legTarget[i];
        if(direction.length() < 0.5f && std::abs(legsIncidence[i]-legTrajectory[i].front().second) < 1)
            legAtPosition++;
        else
        {
            legTarget[i] += 0.01f*elapseTime*direction.normalize();
            if(legsIncidence[i] < legTrajectory[i].front().second)
                legsIncidence[i] += 1;
            else
                legsIncidence[i] -= 1;
        }
    }

    if(legAtPosition == 6)
    {
        for(int i=0;i<6;i++)
        {
            legTrajectory[i].pop_front();
            if(legTrajectory[i].empty())
            {
                if(robotGait==TETRAPOD || robotGait==METACHRONAL_4)
                {
                    legDomainCenter[0] = MyVector3f( 17*cos(2*M_PI/180.f),
                                                     17*sin(2*M_PI/180.f),
                                                     3);
                    legDomainCenter[1] = MyVector3f( 17*cos(2*M_PI/180.f),
                                                    -17*sin(2*M_PI/180.f),
                                                     3);

                    legTrajectory[0].push_back(std::pair<MyVector3f,float>(legDomainCenter[0],0));
                    legTrajectory[1].push_back(std::pair<MyVector3f,float>(legDomainCenter[1],0));
                }
                setState(WALK);
                return;
            }
        }
        if(legTrajectory[0].size() == 1)
        {
            if(robotGait==TETRAPOD || robotGait==METACHRONAL_4) {
                   legsIncidence[2] = 40; legsIncidence[5] = -40; }
            else { legsIncidence[2] = 0;  legsIncidence[5] = 0; }
        }
    }
    else
    {
        for(int i=0;i<6;i++)
        {
            correctPositionError(i);
            targetAnglesArray[i] = inverseKModule->getAnglesFromPosition(i,legSendPosition[i],legsIncidence[i],motorAnglesArray[i]);
        }
    }

    updateMotorsAnglesFromLegPosition();
}
void Hexapod::animateAutonomousLegs(float elapseTime)
{
    MyVector3f direction;
    for(int i=0;i<6;i++)
    {
        if(legsState[i] != AUTONOMOUS) continue;
        if(legTrajectory[i].size() != 0)
        {
            direction = legTrajectory[i].front().first - legTarget[i];
            if(direction.length() < 0.5f && std::abs(legsIncidence[i]-legTrajectory[i].front().second) < 1)
                legTrajectory[i].pop_front();
            else
            {
                legTarget[i] += 0.01f*elapseTime*direction.normalize();
                if(legsIncidence[i] < legTrajectory[i].front().second)
                    legsIncidence[i] += 0.02f*elapseTime;
                else
                    legsIncidence[i] -= 0.02f*elapseTime;
            }

            correctPositionError(i);
            targetAnglesArray[i] = inverseKModule->getAnglesFromPosition(i,legSendPosition[i],legsIncidence[i],motorAnglesArray[i]);
        }
        else if(!blockAnimFlag[i])
        {
            legTarget[i] = legDomainCenter[i];
            if(legsGroup[i] == swingGroup) legsState[i] = IN_AIR;
            else legsState[i] = ON_GROUND;
        }
    }
}


void Hexapod::setState(RobotState s)
{
    for(int i=0;i<6;i++)
        legErrorIntegral[i] = MyVector3f(0,0,0);

    switch(s)
    {
        case INIT:
            for(int i=0;i<6;i++)
            {
                legTarget[i] = legPosition[i];

                legTrajectory[i].clear();
                legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i] + MyVector3f(0,0,3-robotHeight),0));
                legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
            }
            break;
        case WALK:
            limitForSwap = -1;
            initialLimitForSwap = limitForSwap;
            for(int i=0;i<6;i++)
            {
                takeoffdownPosition[i] = legDomainCenter[i];
            }
            break;
        case TRANSITION:
            for(int i=0;i<6;i++)
            {
                legTarget[i] = legPosition[i];

                legTrajectory[i].clear();
                legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i] + MyVector3f(0,0,3-robotHeight),0));
                if(robotGait == METACHRONAL_4 || robotGait == TETRAPOD)
                {
                    if(i == 2) legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],20));
                    else if(i == 3) if(i==0) legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],-20));
                    else legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
                }
                else legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
            }
            break;
        default: break;
    }
    robotState = s;
}
void Hexapod::setGait(RobotGait g)
{
    std::cout<<"gait changed for ";
    switch(g)
    {
        case TRIPOD:            std::cout<<"tripod"<<std::endl;             break;
        case RIPPLE:            std::cout<<"ripple"<<std::endl;             break;
        case METACHRONAL_6:     std::cout<<"metachronal 6 leg"<<std::endl;  break;
        case TETRAPOD:          std::cout<<"tetrapod"<<std::endl;           break;
        case METACHRONAL_4:     std::cout<<"metachronal 4 leg"<<std::endl;  break;
        default:                std::cout<<"undefined"<<std::endl;          break;
    }

    if(g==TETRAPOD || g==METACHRONAL_4)
    {
        initiateLegDomainCenter(15,DOMAIN_ANGLE,robotHeight);
        legDomainCenter[2] = legDomainCenter[1];
        legDomainCenter[5] = legDomainCenter[0];

        legDomainCenter[1] = MyVector3f( 35*cos(2*M_PI/180.f),
                                        -35*sin(2*M_PI/180.f),
                                         2);
        legDomainCenter[0] = MyVector3f( 35*cos(2*M_PI/180.f),
                                         35*sin(2*M_PI/180.f),
                                         2);

        legDomainCenter[3] = MyVector3f(-RADIUS_DOMAIN*cos(45*M_PI/180.f),
                                        -RADIUS_DOMAIN*sin(45*M_PI/180.f),
                                         robotHeight);
        legDomainCenter[4] = MyVector3f(-RADIUS_DOMAIN*cos(45*M_PI/180.f),
                                         RADIUS_DOMAIN*sin(45*M_PI/180.f),
                                         robotHeight);
    }
    else initiateLegDomainCenter(RADIUS_DOMAIN,DOMAIN_ANGLE,robotHeight);


    switch(g)
    {
        case TRIPOD:
            nbGroup = 2;
            swingGroup = 0;

            legsGroup[0] = 0;           legsGroup[1] = 1;
            legsGroup[5] = 1;           legsGroup[2] = 0;
            legsGroup[4] = 0;           legsGroup[3] = 1;

            legsState[0] = ON_GROUND;   legsState[1] = IN_AIR;
            legsState[5] = IN_AIR;      legsState[2] = ON_GROUND;
            legsState[4] = ON_GROUND;   legsState[3] = IN_AIR;
            break;
        case RIPPLE:
            nbGroup = 3;
            swingGroup = 0;

            legsGroup[0] = 0;           legsGroup[1] = 2;
            legsGroup[5] = 1;           legsGroup[2] = 0;
            legsGroup[4] = 2;           legsGroup[3] = 1;

            legsState[0] = IN_AIR;      legsState[1] = ON_GROUND;
            legsState[5] = ON_GROUND;   legsState[2] = IN_AIR;
            legsState[4] = ON_GROUND;   legsState[3] = ON_GROUND;
            break;
        case METACHRONAL_6:
            nbGroup = 6;
            swingGroup = 0;

            legsGroup[0] = 0;           legsGroup[1] = 1;
            legsGroup[5] = 2;           legsGroup[2] = 3;
            legsGroup[4] = 4;           legsGroup[3] = 5;

            legsState[0] = IN_AIR;      legsState[1] = ON_GROUND;
            legsState[5] = ON_GROUND;   legsState[2] = ON_GROUND;
            legsState[4] = ON_GROUND;   legsState[3] = ON_GROUND;
            break;
        case TETRAPOD:
            nbGroup = 2;
            swingGroup = 0;

            legsGroup[0] = nbGroup+1;   legsGroup[1] = nbGroup+1;
            legsGroup[5] = 1;           legsGroup[2] = 1;
            legsGroup[4] = 1;           legsGroup[3] = 1;

            legsState[0] = AUTONOMOUS;  legsState[1] = AUTONOMOUS;
            legsState[5] = ON_GROUND;   legsState[2] = ON_GROUND;
            legsState[4] = ON_GROUND;   legsState[3] = ON_GROUND;
            break;
        case METACHRONAL_4:
            nbGroup = 4;
            swingGroup = 0;

            legsGroup[0] = nbGroup+1;   legsGroup[1] = nbGroup+1;
            legsGroup[5] = 0;           legsGroup[2] = 1;
            legsGroup[4] = 2;           legsGroup[3] = 3;

            legsState[0] = AUTONOMOUS;  legsState[1] = AUTONOMOUS;
            legsState[5] = IN_AIR;      legsState[2] = ON_GROUND;
            legsState[4] = ON_GROUND;   legsState[3] = ON_GROUND;
            break;
        default: break;
    }
    speedGain = 1.f/(nbGroup-1);

    robotGait = g;
    setState(TRANSITION);
}
void Hexapod::toogleTorque()
{
    torque = !torque;
    //std::cout<<"torque : "<<(torque?"enable":"disable")<<std::endl;

    uint8_t bytes;
    if(torque) bytes = 0x01;
    else bytes = 0xFD;
    uint8_t tmpBuff[4];

    for(int i=0;i<100;i++)
    {
        arduino->send(Serial::EnableDisableTorque,&bytes,1);
        if(arduino->readMessage(tmpBuff,4,16))
            return;
        usleep(1000);
    }
}
void Hexapod::attack(int legIndex)
{
    if(robotGait==METACHRONAL_6 || robotGait==RIPPLE)
    {
        if(legIndex == 1 && legsState[0] == AUTONOMOUS) return;
        else if(legIndex == 0 && legsState[1] == AUTONOMOUS) return;

        legsState[legIndex] = AUTONOMOUS;
        legTrajectory[legIndex].clear();
        if(legIndex == 0)
            legTrajectory[legIndex].push_back(std::pair<MyVector3f,float>(MyVector3f(30,-6,20),0));
        if(legIndex == 1)
            legTrajectory[legIndex].push_back(std::pair<MyVector3f,float>(MyVector3f(30, 6,20),0));
        legTrajectory[legIndex].push_back(std::pair<MyVector3f,float>(legDomainCenter[legIndex],0));
    }
}
void Hexapod::setPushPosition(bool enable)
{
    if(robotGait==METACHRONAL_6 || robotGait==RIPPLE)
    {
        legsState[0] = AUTONOMOUS;
        legTrajectory[0].clear();
        if(enable)
        {
            blockAnimFlag[0] = true;
            legTrajectory[0].push_back(std::pair<MyVector3f,float>(legDomainCenter[0]+MyVector3f(0,0,-robotHeight),0));
            legTrajectory[0].push_back(std::pair<MyVector3f,float>(MyVector3f(17,-7,robotHeight +0.5),-55));
            legDomainCenter[5] = MyVector3f( RADIUS_DOMAIN*sin(M_PI/6), RADIUS_DOMAIN*cos(M_PI/6), robotHeight);
            legDomainCenter[4] = MyVector3f(-RADIUS_DOMAIN*cos(M_PI/4), RADIUS_DOMAIN*sin(M_PI/4), robotHeight);
        }
        else
        {
            blockAnimFlag[0] = false;
            legTrajectory[0].push_back(std::pair<MyVector3f,float>(legDomainCenter[0]+MyVector3f(0,0,-robotHeight),0));
            legTrajectory[0].push_back(std::pair<MyVector3f,float>(legDomainCenter[0],0));
            initiateLegDomainCenter(RADIUS_DOMAIN,DOMAIN_ANGLE,ZERO_HEIGHT);
        }
    }

    /*if(enable)
    {
        float angle = 30*M_PI/180.f;
        legDomainCenter[0] = MyVector3f( RADIUS_DOMAIN*cos(angle), RADIUS_DOMAIN*sin(angle), robotHeight);
        legDomainCenter[1] = MyVector3f( RADIUS_DOMAIN*cos(angle),-RADIUS_DOMAIN*sin(angle), robotHeight);
    }
    else
    {
        float angle = DOMAIN_ANGLE*M_PI/180.f;
        legDomainCenter[0] = MyVector3f( RADIUS_DOMAIN*cos(angle), RADIUS_DOMAIN*sin(angle), robotHeight);
        legDomainCenter[1] = MyVector3f( RADIUS_DOMAIN*cos(angle),-RADIUS_DOMAIN*sin(angle), robotHeight);
    }*/
}
void Hexapod::openUmbrella()
{
    std::cout<<"Opening umbrella"<<std::endl;
    for(int i=0;i<100;i++)
    {
        arduino->send(Serial::OpenUmbrella,NULL,0);
        if(!arduino->readMessage(NULL,0,12)) return;
    }
}
bool Hexapod::legsInPosition()
{
    for(int i=0;i<6;i++)
        if(legsState[i] == AUTONOMOUS && legTrajectory[i].size() != 0)
            return false;
    return true;
}


MyVector3f Hexapod::getLegDisplacement(int legIndex,const MyVector3f& translationSpeed,const MyVector3f& rotationSpeed)
{
    //  Translation part
    MyVector3f result = translationSpeed;

    //rotation 1 part
    double tmpl = sqrt(legTarget[legIndex].x*legTarget[legIndex].x + legTarget[legIndex].y*legTarget[legIndex].y);
    MyVector3f tmpv(-legTarget[legIndex].y, legTarget[legIndex].x, 0);
    tmpv.normalize();
    result += tmpl*rotationSpeed.x*tmpv;

    //rotation 2 part
    tmpl = sqrt(legTarget[legIndex].x*legTarget[legIndex].x + legTarget[legIndex].z*legTarget[legIndex].z);
    tmpv = MyVector3f(legTarget[legIndex].z, 0, -legTarget[legIndex].x);
    tmpv.normalize();
    result += tmpl*rotationSpeed.y*tmpv;

    //rotation 3 part
    tmpl = sqrt(legTarget[legIndex].z*legTarget[legIndex].z + legTarget[legIndex].y*legTarget[legIndex].y);
    tmpv = MyVector3f(0, -legTarget[legIndex].z, legTarget[legIndex].y);
    tmpv.normalize();
    result += tmpl*rotationSpeed.z*tmpv;

    //  end
    return result;
}
void Hexapod::updateLegPositionFromMotorsAngles()
{
    uint8_t tmpBuff[54];
    arduino->send(Serial::GetMotorPosition,NULL,0);
    if(arduino->readMessage(tmpBuff,54,34))
    {
        for(int i=0;i<6;i++)
        {
            motorAnglesArray[i].angle1 = motorAngleFromBytes(tmpBuff[8*i+1],tmpBuff[8*i  ],sens[4*i  ],motorAnglesOffset[4*i  ]);
            motorAnglesArray[i].angle2 = motorAngleFromBytes(tmpBuff[8*i+3],tmpBuff[8*i+2],sens[4*i+1],motorAnglesOffset[4*i+1]);
            motorAnglesArray[i].angle3 = motorAngleFromBytes(tmpBuff[8*i+5],tmpBuff[8*i+4],sens[4*i+2],motorAnglesOffset[4*i+2]);
            motorAnglesArray[i].angle4 = motorAngleFromBytes(tmpBuff[8*i+7],tmpBuff[8*i+6],sens[4*i+3],motorAnglesOffset[4*i+3]);

            legPosition[i] = forwardKModule->getLegExtremityPosition(i,motorAnglesArray[i]);
        }
        frontDistance[0] = (int)(tmpBuff[49]<<8)+tmpBuff[48];   //if(frontDistance[0] < 20) std::cout<<"down ";
        frontDistance[1] = (int)(tmpBuff[51]<<8)+tmpBuff[50];   //if(frontDistance[1] < 20) std::cout<<"right ";
        frontDistance[2] = (int)(tmpBuff[53]<<8)+tmpBuff[52];   //if(frontDistance[2] < 20) std::cout<<"left ";
        //std::cout<<std::endl;
    }
    else std::cout<<"read timeout"<<std::endl;
}
void Hexapod::updateMotorsAnglesFromLegPosition()
{
    int16_t tmpBuff[24];
    for(int i=0;i<6;i++)
    {
        tmpBuff[4*i  ] = sens[4*i  ]*targetAnglesArray[i].angle1*1024/220.+512+motorAnglesOffset[4*i  ];
        tmpBuff[4*i+1] = sens[4*i+1]*targetAnglesArray[i].angle2*1024/220.+512+motorAnglesOffset[4*i+1];
        tmpBuff[4*i+2] = sens[4*i+2]*targetAnglesArray[i].angle3*1024/220.+512+motorAnglesOffset[4*i+2];
        tmpBuff[4*i+3] = sens[4*i+3]*targetAnglesArray[i].angle4*1024/220.+512+motorAnglesOffset[4*i+3];
    }
    arduino->send(Serial::SetMotorPosition,(uint8_t*)tmpBuff,48);
}
bool Hexapod::tryConnectingToArduino()
{
    uint8_t tmpBuff[54];
    for(int i=0;i<1000;i++)
    {
        //  Ping
        arduino->send(Serial::Ping,NULL,0);
        if(!arduino->readMessage(NULL,0,12)) continue;

        //  Initialize legs position
        arduino->send(Serial::GetMotorPosition,NULL,0);
        if(arduino->readMessage(tmpBuff,54,34))
        {
            for(int i=0;i<6;i++)
            {
                motorAnglesArray[i].angle1 = motorAngleFromBytes(tmpBuff[8*i+1],tmpBuff[8*i  ],sens[4*i  ],motorAnglesOffset[4*i  ]);
                motorAnglesArray[i].angle2 = motorAngleFromBytes(tmpBuff[8*i+3],tmpBuff[8*i+2],sens[4*i+1],motorAnglesOffset[4*i+1]);
                motorAnglesArray[i].angle3 = motorAngleFromBytes(tmpBuff[8*i+5],tmpBuff[8*i+4],sens[4*i+2],motorAnglesOffset[4*i+2]);
                motorAnglesArray[i].angle4 = motorAngleFromBytes(tmpBuff[8*i+7],tmpBuff[8*i+6],sens[4*i+3],motorAnglesOffset[4*i+3]);

                legPosition[i] = forwardKModule->getLegExtremityPosition(i,motorAnglesArray[i]);
            }
            frontDistance[0] = (int)(tmpBuff[49]<<8)+tmpBuff[48];
            frontDistance[1] = (int)(tmpBuff[51]<<8)+tmpBuff[50];
            frontDistance[2] = (int)(tmpBuff[53]<<8)+tmpBuff[52];
        }
        else continue;

        std::cout<<"Connection with Arduino operational"<<std::endl;
        for(int i=0;i<6;i++)
            std::cout<<"leg "<<i<<" at : "<<legPosition[i];
        std::cout<<std::endl;

        return true;
    }
    std::cout<<"unable to connect to Arduino"<<std::endl;
    return false;
}
float Hexapod::getLegLimitFromDirection(int legIndex,MyVector3f direction)
{
    direction.z = 0; direction.normalize();
    float Ri = limit(legDomainCenter[legIndex],stepLength/2.f,takeoffdownPosition[legIndex],direction);
    if(Ri<0) return -1;

    Ri /= 2.f*((legsGroup[legIndex]-swingGroup + nbGroup)%nbGroup);
    MyVector3f Ci = takeoffdownPosition[legIndex] + Ri*direction;

    float l = limit(Ci,Ri,legTarget[legIndex],direction);

    return l;
}
MyVector3f Hexapod::getAirTarget(int legIndex,MyVector3f takeoffPos, float t)
{
    MyVector3f planarDirection = MyVector3f(-legDisplacement[legIndex].x,-legDisplacement[legIndex].y,0);
        planarDirection.normalize();
    if(t>1) t=1; else if(t<0) t=0;

    float mu = 0;
    MyVector3f target = legDomainCenter[legIndex] + stepLength/2.f*planarDirection;

    MyVector3f v0 = MyVector3f(0,0, 4.f/3.f*stepHeight) - mu*planarDirection;
    MyVector3f v1 = MyVector3f(0,0,-4.f/3.f*stepHeight) - mu*planarDirection;
    MyVector3f C =  3*(target-takeoffPos) +2*v0 - v1;
    MyVector3f D = -2*(target-takeoffPos) -3*v0 + v1;

    return takeoffPos +t*v0 +t*t*C +t*t*t*D;
}
void Hexapod::correctPositionError(int legIndex)
{
    MyVector3f error = legTarget[legIndex] - legPosition[legIndex];
    legErrorIntegral[legIndex] += error;
    if(legErrorIntegral[legIndex].length() > 2.f)
    {
        legErrorIntegral[legIndex].normalize();
        legErrorIntegral[legIndex] *= 2.f;
    }

    legSendPosition[legIndex] = legTarget[legIndex];// + gainP*error + 0.1*legErrorIntegral[legIndex];
}
void Hexapod::initiateLegDomainCenter(float radius,float angle,float zeroHeight)
{
    angle *= M_PI/180.f;

    legDomainCenter[1] = MyVector3f( radius*cos(angle),
                                    -radius*sin(angle),
                                     zeroHeight);
    legDomainCenter[2] = MyVector3f( 0,
                                    -radius,
                                     zeroHeight);
    legDomainCenter[3] = MyVector3f(-radius*cos(angle),
                                    -radius*sin(angle),
                                     zeroHeight);

    legDomainCenter[0] = MyVector3f( radius*cos(angle),
                                     radius*sin(angle),
                                     zeroHeight);
    legDomainCenter[5] = MyVector3f( 0,
                                     radius,
                                     zeroHeight);
    legDomainCenter[4] = MyVector3f(-radius*cos(angle),
                                     radius*sin(angle),
                                     zeroHeight);
}


inline float Hexapod::motorAngleFromBytes(const uint8_t& b1,const uint8_t& b2,const int8_t& s,const int16_t& offset)
{
    int16_t angle = (int)(b1<<8)+b2;
    return s*(angle - offset - 512.)/1024.*220.;
}
inline float Hexapod::limit(MyVector3f C,float R,MyVector3f P,MyVector3f u)
{
    P -= C;
    float a = std::abs(P.x*u.y-u.x*P.y);

    if(a>R) return -1;
    else if(P.length()>R && P*u>0) return -1;
    else return R*cos(asin(a/R)) - P*u;
}
//

