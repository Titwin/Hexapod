#include "Hexapod.hpp"

#define ZERO_HEIGHT (-7)
#define STEP_LENGTH 7
#define RADIUS_DOMAIN 20
#define DOMAIN_ANGLE 40


//  Default
const uint8_t Hexapod::motorIDs[24] = {2,3,4,5, 6,7,8,9, 10,11,12,13, 14,15,16,17, 18,19,20,21, 22,23,24,25};
Hexapod::Hexapod(std::string s) : name(s),wellInitialized(false)
{
    forwardKModule = new ForwardKinematics();
    inverseKModule = new InverseKinematics();
        inverseKModule->setForwardKinematicsModule(forwardKModule);

    legPosition = new MyVector3f[6];
    legTarget = new MyVector3f[6];
    legDisplacement = new MyVector3f[6];
    legDomainCenter = new MyVector3f[6];
    takeoffdownPosition = new MyVector3f[6];

    //{ Initialize angles offsets
        motorAnglesOffset[0][0] = -52;    sens[0][0] = -1;
        motorAnglesOffset[0][1] = -15;    sens[0][1] =  1;
        motorAnglesOffset[0][2] =  5;     sens[0][2] = -1;
        motorAnglesOffset[0][3] =  0;     sens[0][3] =  1;

        motorAnglesOffset[1][0] = -25;    sens[1][0] = -1;
        motorAnglesOffset[1][1] =  50;    sens[1][1] =  1;
        motorAnglesOffset[1][2] =  0;     sens[1][2] = -1;
        motorAnglesOffset[1][3] =  10;    sens[1][3] =  1;

        motorAnglesOffset[2][0] = -15;    sens[2][0] = -1;
        motorAnglesOffset[2][1] = -21;    sens[2][1] =  1;
        motorAnglesOffset[2][2] =  32;    sens[2][2] = -1;
        motorAnglesOffset[2][3] = -32;    sens[2][3] =  1;

        motorAnglesOffset[3][0] = -12;    sens[3][0] = -1;
        motorAnglesOffset[3][1] = -17;    sens[3][1] =  1;
        motorAnglesOffset[3][2] =  40;    sens[3][2] = -1;
        motorAnglesOffset[3][3] =  80;    sens[3][3] =  1;

        motorAnglesOffset[4][0] = -14;    sens[4][0] = -1;
        motorAnglesOffset[4][1] =  -10;    sens[4][1] =  1;
        motorAnglesOffset[4][2] = -50;    sens[4][2] = -1;
        motorAnglesOffset[4][3] = -33;    sens[4][3] =  1;

        motorAnglesOffset[5][0] = -4;     sens[5][0] = -1;
        motorAnglesOffset[5][1] =  51;    sens[5][1] =  1;
        motorAnglesOffset[5][2] =  15;    sens[5][2] = -1;
        motorAnglesOffset[5][3] =  0;     sens[5][3] =  1;
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
        lastMoveTime = 0.f;

        stepLength = STEP_LENGTH; stepHeight = 4;
        nbGroup = 2;    swingGroup = 1;
        robotHeight = ZERO_HEIGHT;
        torque = false;

        robotGait = TRIPOD;
        robotTargetGait = TRIPOD;
        speedGain = 1.f;
        transitState = 0;

        limitForSwap = 4*stepLength;
        initialLimitForSwap = limitForSwap;

        maxTranslationSpeedMagnitude = 1.f;
        maxRotationSpeedMagnitude = maxTranslationSpeedMagnitude/20.f;

        gainP = 0.7;
    //}

    //{ Initialize some geometric parameter and initial target
        initiateLegDomainCenter(RADIUS_DOMAIN, DOMAIN_ANGLE,ZERO_HEIGHT);

        for(int i=0;i<6;i++)
        {
            blockAnimFlag[i] = false;
            legsIncidence[i] = 0;
            takeoffdownPosition[i] = legDomainCenter[i];
            targetAnglesArray[i] = LegAngleVector(0.0, 0.0, 0.0, 0.0);
        }
    //}

    //{ Finish initialization
        wellInitialized = false;
        setState(INIT);
    //}

    std::cout<<name<<" created !"<<std::endl;
}
Hexapod::~Hexapod()
{
    delete forwardKModule;
    delete inverseKModule;

    delete[] legPosition;
    delete[] legTarget;
    delete[] legDomainCenter;
    delete[] legDisplacement;
    delete[] takeoffdownPosition;
}
//



//  Public functions
void Hexapod::setMotorAngles(uint8_t* data)
{
    for(unsigned int i=0; i<6; i++)
    {
        motorAnglesArray[i].angle1 = sens[i][0]*((int)(data[8*i + 1]*256 + data[8*i    ]) - motorAnglesOffset[i][0] - 512.)/1024.*200.;
        motorAnglesArray[i].angle2 = sens[i][1]*((int)(data[8*i + 3]*256 + data[8*i + 2]) - motorAnglesOffset[i][1] - 512.)/1024.*200.;
        motorAnglesArray[i].angle3 = sens[i][2]*((int)(data[8*i + 5]*256 + data[8*i + 4]) - motorAnglesOffset[i][2] - 512.)/1024.*200.;
        motorAnglesArray[i].angle4 = sens[i][3]*((int)(data[8*i + 7]*256 + data[8*i + 6]) - motorAnglesOffset[i][3] - 512.)/1024.*200.;
    }
    //int leg = 2;
    //std::cout << motorAnglesArray[leg].angle1 << ' ' << motorAnglesArray[leg].angle2 << ' ' << motorAnglesArray[leg].angle3 << ' ' << motorAnglesArray[leg].angle4 << std::endl;
}
const uint8_t* Hexapod::getMotorsIds() const
{
    return motorIDs;
}
uint16_t* Hexapod::getGoalMotorAngles()
{
    for(int i=0;i<6;i++)
    {
        goalMotorAngle[4*i    ] = sens[i][0]*targetAnglesArray[i].angle1 * 1024/200. + 512 + motorAnglesOffset[i][0];
        goalMotorAngle[4*i + 1] = sens[i][1]*targetAnglesArray[i].angle2 * 1024/200. + 512 + motorAnglesOffset[i][1];
        goalMotorAngle[4*i + 2] = sens[i][2]*targetAnglesArray[i].angle3 * 1024/200. + 512 + motorAnglesOffset[i][2];
        goalMotorAngle[4*i + 3] = sens[i][3]*targetAnglesArray[i].angle4 * 1024/200. + 512 + motorAnglesOffset[i][3];
    }
    return goalMotorAngle;
}


void Hexapod::setTorque(bool b)
{
    if(robotState == INIT)
        setState(INIT);
    torque = b;
}


void Hexapod::animate(float elapseTime, MyVector3f translationSpeed, MyVector3f rotationSpeed)
{
    for(int i=0; i<6; i++)
        legPosition[i] = forwardKModule->getLegExtremityPosition(i, motorAnglesArray[i]);

    //for(int i=3; i<6; i++)
    //    targetAnglesArray[i] = inverseKModule->getAnglesFromExtremityAndIncidence(i, MyVector3f(-legPosition[i-3].x, legPosition[i-3].y, legPosition[i-3].z), 0, motorAnglesArray[i]);
    //std::cout<<legPosition[5]<<std::endl;

    switch(robotState)
    {
        case INIT:
            animateGotoStart(elapseTime);
            lastMoveTime = -1;
            break;
        case WALK:
            animateWalk(elapseTime,translationSpeed,rotationSpeed);
            animateAutonomousLegs(elapseTime);
            break;
        case TRANSITION:
            animateTransition(elapseTime);
            lastMoveTime = -1;
            break;
        default: robotState = STOP; break;
    }
}

//  Private functions
void Hexapod::animateWalk(float elapseTime, MyVector3f translationSpeed, MyVector3f rotationSpeed)
{
    if(!torque) return;
    if(translationSpeed.length() == 0 && rotationSpeed.length() == 0)
    {
        if(lastMoveTime >= 0) lastMoveTime += elapseTime;
        if(lastMoveTime >= 1000)
        {
            lastMoveTime = -1;
            setState(TRANSITION);
        }
        return;
    }
    else lastMoveTime = 0.f;

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

    float magnitude = 0.f;
    for(int i=0;i<6;i++)
        magnitude = std::max(magnitude, getLegDisplacement(i,-translationSpeed,-rotationSpeed).length());
    if(magnitude > 0.3f)
    {
        translationSpeed *= 0.3f/magnitude;
        rotationSpeed *= 0.3f/magnitude;
    }


    /*if(translationSpeed.length() != 0 && rotationSpeed.length() != 0)
    {
        translationSpeed *= 0.5f;
        rotationSpeed *= 0.5f;
    }*/

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

        targetAnglesArray[i] = inverseKModule->getAnglesFromExtremityAndIncidence(i, legTarget[i], legsIncidence[i], motorAnglesArray[i]);
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
            targetAnglesArray[i] = inverseKModule->getAnglesFromExtremityAndIncidence(i, legTarget[i], legsIncidence[i], motorAnglesArray[i]);
        }
    }
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
            legTarget[i] += 0.02f*elapseTime*direction.normalize();
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
            targetAnglesArray[i] = inverseKModule->getAnglesFromExtremityAndIncidence(i,legTarget[i],legsIncidence[i],motorAnglesArray[i]);
        }
    }
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

            targetAnglesArray[i] = inverseKModule->getAnglesFromExtremityAndIncidence(i,legTarget[i],legsIncidence[i],motorAnglesArray[i]);
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
    switch(s)
    {
        case INIT:
            stepHeight = 6;
            robotHeight = ZERO_HEIGHT;
            initiateLegDomainCenter(RADIUS_DOMAIN, DOMAIN_ANGLE, robotHeight);

            for(int i=0;i<6;i++)
            {
                legTarget[i] = legPosition[i];

                legTrajectory[i].clear();
                legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i] - MyVector3f(0,0,robotHeight),0));
                legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
            }
            break;

        case WALK:
            limitForSwap = -1;
            initialLimitForSwap = limitForSwap;
            for(int i=0;i<6;i++)
                takeoffdownPosition[i] = legDomainCenter[i];
            break;

        case TRANSITION:
            /*for(int i=0;i<6;i++)
            {
                legTarget[i] = legPosition[i];

                legTrajectory[i].clear();
                legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i] + MyVector3f(0,0,-robotHeight),0));
                if(robotGait == METACHRONAL_4 || robotGait == TETRAPOD)
                {
                    if(i == 2) legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],20));
                    else if(i == 3) if(i==0) legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],-20));
                    else legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
                }
                else legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
            }*/
            for(int i=0; i<6; i++)
            {
                legTarget[i] = legPosition[i];
                legTrajectory[i].clear();
                bool goodPosition = false;
                for(int j=swingGroup; j<nbGroup+swingGroup; j++)
                {
                    int tmp = j%nbGroup;
                    if(legsGroup[i] == tmp)
                    {
                        legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i] + MyVector3f(0,0,-robotHeight),0));
                        legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
                        goodPosition = true;
                    }
                    else if(goodPosition)
                    {
                        legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
                        legTrajectory[i].push_back(std::pair<MyVector3f,float>(legDomainCenter[i],0));
                    }
                    else
                    {
                        legTrajectory[i].push_back(std::pair<MyVector3f,float>(legPosition[i],0));
                        legTrajectory[i].push_back(std::pair<MyVector3f,float>(legPosition[i],0));
                    }
                }
            }
            break;

        default: break;
    }
    robotState = s;
}
void Hexapod::setGait(RobotGait g, bool log)
{
    if(log)
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

void Hexapod::initiateLegDomainCenter(float radius,float angle,float zeroHeight)
{
    angle *= M_PI/180.f;

    legDomainCenter[0] = MyVector3f( radius*cos(angle), radius*sin(angle), zeroHeight);
    legDomainCenter[1] = MyVector3f( radius, 0, zeroHeight);
    legDomainCenter[2] = MyVector3f( radius*cos(angle),-radius*sin(angle), zeroHeight);

    legDomainCenter[3] = MyVector3f(-radius*cos(angle), radius*sin(angle), zeroHeight);
    legDomainCenter[4] = MyVector3f(-radius, 0, zeroHeight);
    legDomainCenter[5] = MyVector3f(-radius*cos(angle),-radius*sin(angle), zeroHeight);
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

