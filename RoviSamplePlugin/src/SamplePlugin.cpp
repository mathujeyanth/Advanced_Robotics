#include "SamplePlugin.hpp"

SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

    // now connect stuff from the ui component
//    connect(_btn_im         ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
//    connect(_btn_scan       ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
//    connect(_btn0           ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_btn_runPath    ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_maxIteration  ,SIGNAL(valueChanged(int)),  this, SLOT(btnPressed()) );
    connect(_jointStepSize  ,SIGNAL(valueChanged(double)),  this, SLOT(btnPressed()) );
//    connect(_btnPtPi        ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
//    connect(_btnPtP         ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_placeBottle    ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_home           ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_printTest      ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );

    _framegrabber = NULL;
}

SamplePlugin::~SamplePlugin()
{
    delete _textureRender;
    delete _bgRender;
}

void SamplePlugin::initialize() {
    log().info() << "INITALIZE" << "\n";

    getRobWorkStudio()->stateChangedEvent().add(std::bind(&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Auto load workcell
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/student/Desktop/Project/Advanced_Robotics/Project_WorkCell/Scene.wc.xml");
    getRobWorkStudio()->setWorkCell(wc);
    srand(time(NULL)); //Seed for random number generator - Function fRand


}

void SamplePlugin::open(WorkCell* workcell)
{
    log().info() << "OPEN" << "\n";
    _wc = workcell;
    _state = _wc->getDefaultState();

    log().info() << workcell->getFilename() << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("TextureImage",_textureRender,textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio()->getWorkCellScene()->addRender("BackgroundImage",_bgRender,bgFrame);
        }


        _device1 = _wc->findDevice("1_UR-6-85-5-A");
        _device2 = _wc->findDevice("2_UR-6-85-5-A");
        _step = -1;

    }
    setupRobotPtrs();
}

void SamplePlugin::close() {
    log().info() << "CLOSE" << "\n";

    // Stop the timer
    _timer->stop();
    // Remove the texture render
    Frame* textureFrame = _wc->findFrame("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("TextureImage",textureFrame);
    }
    // Remove the background render
    Frame* bgFrame = _wc->findFrame("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio()->getWorkCellScene()->removeDrawable("BackgroundImage",bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc = NULL;
}

void SamplePlugin::btnPressed() {
    QObject *obj = sender();

    for(auto &th : active_threads)
        th.join();

    active_threads.clear();

    if (obj == _home)
    {
        _timer->stop();
        //rw::math::Math::seed();
        Q to(6, 1.571, -1.158, -2.728, 0.771, 1.571, 0); //From pose estimation
        _device1->setQ(to,_state);
        _device2->setQ(to,_state);
        getRobWorkStudio()->setState(_state);
        //createPathRRTConnect(from, to, extend, maxTime);
    }
    else if (obj == _placeBottle)
    {
        float rn_x = -0.35+(rand() % 71)/100.0f;
        float rn_y = 0.35 +((rand() % 21)/100.0f);
        std::cout << "x: " << rn_x << " y: " << rn_y << std::endl;
        _wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->moveTo(
                    rw::math::Transform3D<>(rw::math::Vector3D<>(rn_x,rn_y,0.21f),
                                            rw::math::RPY<>(0,0,90*rw::math::Deg2Rad)
                                            ), _state);

        getRobWorkStudio()->setState(_state);
    }
    else if (obj == _printTest)
    {
        rw::math::Q conf(6,0.724, -0.928, 1.854, -4.101, 0.437, 1.668);

        cout << "Test button pressed" << endl;
//        rw::math::Vector3D<> p1 = rw::math::Vector3D<>(-0.3,-0.5,0.3);
//        rw::math::Vector3D<> p2 = rw::math::Vector3D<>(0.3,-0.5,0.3);
//        rw::math::Vector3D<> p3 = rw::math::Vector3D<>(0,0.60,0.45);
//        rw::geometry::Plane aPlane = rw::geometry::Plane(p1,p2,p3);
//        createTree(aPlane,_state,1,10000);
//        lazyConnectGraph(1);
//        lazyFindPath(p1,p3,_state,1);
        DualPRM(conf,conf,_state,_maxIteration->value(),_jointStepSize->value());
        DualPath();
        _timer->stop();
        //active_threads.push_back(std::thread(&SamplePlugin::createTree,this,aPlane,_state,1,10000));
        //active_threads.push_back(std::thread(&SamplePlugin::createTree,this,aPlane,_state,2,10000));

        _attachIdx = -1;;
    }
    else if(obj==_btn_runPath){
        if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;

    }

}

void SamplePlugin::timer() {
    //_wc->findDevice("WSG50")-> setQ(rw::math::Q(1, 0.055),_state);
    int length = 0;

    if (_path1.size() > _path2.size())
        length = _path1.size();
    else
        length = _path2.size();

    if(0 <= _step && _step < length){
        if (_step <_path1.size())
            _device1->setQ(_path1.at(_step),_state);
        if (_step <_path2.size())
            _device2->setQ(_path2.at(_step),_state);

        getRobWorkStudio()->setState(_state);
        _step++;
    }
}

void SamplePlugin::stateChangedListener(const State& state) {
    _state = state;
}

bool SamplePlugin::checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
        }
        return false;
    }
    return true;
}

void SamplePlugin::createPtPPath(rw::math::Vector3D<> to)
{
    // Source: "ReachabilityAnalysis_solution.zip" provided by Jeppe Langaa on BlackBoard
    // Parabolic blend and PtP : robwork.dk/apidoc/cpp/doxygen/classrw_1_1trajectory_1_1InterpolatorTrajectory.html
    log().info() << "PtP path" << "\n";
    // find relevant frames
    rw::kinematics::MovableFrame::Ptr Bottle = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    rw::kinematics::FixedFrame::Ptr placeArea = _wc->findFrame<rw::kinematics::FixedFrame>("placeArea");
    rw::models::SerialDevice::Ptr robotUR5 = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    rw::models::Device::Ptr gripperDevice = _wc->findDevice("WSG50");

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    // Gripper Q to be open
    const rw::math::Q gQ(1, 0.055); // 2.886

    // get the default state
    rw::kinematics::State state = _state;
    rw::math::Q first = robotUR5->getQ(state);
    rw::math::Q start;
    rw::math::Q end;


    rw::math::Q PtParray[6] = {};
    PtParray[1] = rw::math::Q(6,0.977611, -1.33427, -1.57165, -1.80647, 1.5708, -0.593185);
    PtParray[2] = rw::math::Q(6,0.512516, -1.38618, -1.53181, -1.79441, 1.5708, -1.05828);
    PtParray[3] = rw::math::Q(6,0.0225671, -1.38618, -1.53181, -1.79441, 1.5708, -1.54823);
    PtParray[4] = rw::math::Q(6,-0.411863, -1.33427, -1.57165, -1.80647, 1.5708, -1.98266);
    double bestSolution = 9999.9;

    for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
        Bottle->moveTo(
                    rw::math::Transform3D<>(rw::math::Vector3D<>(Bottle->getTransform(state).P()),
                                            rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,90*rw::math::Deg2Rad)
                                            ), state);
        std::vector<rw::math::Q> solutions = getConfigurations("Bottle", "GraspTCP", robotUR5, _wc, state);
        double curSolution = 0.0;
        for(unsigned int i=0; i<solutions.size(); i++){
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions[i], state);
            gripperDevice -> setQ(gQ,state);
            if( !detector->inCollision(state,NULL,true) )
            {
                rw::math::Q tempQ = solutions[i]-first;
                curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                if (curSolution<bestSolution)
                {
                    bestSolution = curSolution;
                    start = solutions[i];
                }
                break; // we only need one
            }
        }
    }
    bestSolution = 9999.9;
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
        Bottle->moveTo(
                    rw::math::Transform3D<>(rw::math::Vector3D<>(placeArea->getTransform(state).P()),
                                            rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,90*rw::math::Deg2Rad)
                                            ), state);
        std::vector<rw::math::Q> solutions = getConfigurations("Bottle", "GraspTCP", robotUR5, _wc, state);
        double curSolution = 0.0;
        for(unsigned int i=0; i<solutions.size(); i++){
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions[i], state);
            gripperDevice -> setQ(gQ,state);
            if( !detector->inCollision(state,NULL,true) ){
                rw::math::Q tempQ = solutions[i]-first;
                curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                if (curSolution<bestSolution)
                {
                    bestSolution = curSolution;
                    end = solutions[i];
                }
                break; // we only need one
            }
        }
    }

    // Point to Point interpolation
    PtParray[0] = start;
    PtParray[5] = end;
    _attachQ = start;
    _deattachQ = end;
    Q tempQ = first-start;
    double deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
    deltaD = sqrt(deltaD);
    _attachIdx = 20*deltaD;
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
    rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr currentLinearIntPol
            = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(first,start,deltaD));
    traj.add(currentLinearIntPol);
    for (int i = 0;i<5;i += 1)
    {
        rw::math::Q currentQ = PtParray[i+1];
        rw::math::Q formerQ = PtParray[i];
        tempQ = formerQ-currentQ;
        deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        currentLinearIntPol = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));
        traj.add(currentLinearIntPol);
    }

    QPath tempQPath;
    for (double s = 0.0;s<traj.duration();s += 0.05)
        tempQPath.push_back(traj.x(s));
    //_path = tempQPath;
}

void SamplePlugin::createPtPiPath(rw::math::Vector3D<> to)
{
    // Source: "ReachabilityAnalysis_solution.zip" provided by Jeppe Langaa on BlackBoard
    // Parabolic blend and PtP : robwork.dk/apidoc/cpp/doxygen/classrw_1_1trajectory_1_1InterpolatorTrajectory.html
    log().info() << "PtP with parabolic blend path" << "\n";
    // find relevant frames
    rw::kinematics::MovableFrame::Ptr Bottle = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    rw::kinematics::FixedFrame::Ptr placeArea = _wc->findFrame<rw::kinematics::FixedFrame>("placeArea");
    rw::models::SerialDevice::Ptr robotUR5 = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    rw::models::Device::Ptr gripperDevice = _wc->findDevice("WSG50");

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    // Gripper Q to be open
    const rw::math::Q gQ(1, 0.055); // 2.886

    // get the default state
    rw::kinematics::State state = _state;
    rw::math::Q first = robotUR5->getQ(state);
    rw::math::Q start;
    rw::math::Q end;


    rw::math::Q PtParray[6] = {};
    PtParray[1] = rw::math::Q(6,0.977611, -1.33427, -1.57165, -1.80647, 1.5708, -0.593185);
    PtParray[2] = rw::math::Q(6,0.512516, -1.38618, -1.53181, -1.79441, 1.5708, -1.05828);
    PtParray[3] = rw::math::Q(6,0.0225671, -1.38618, -1.53181, -1.79441, 1.5708, -1.54823);
    PtParray[4] = rw::math::Q(6,-0.411863, -1.33427, -1.57165, -1.80647, 1.5708, -1.98266);
    double bestSolution = 9999.9;

    for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
        Bottle->moveTo(
                    rw::math::Transform3D<>(rw::math::Vector3D<>(Bottle->getTransform(state).P()),
                                            rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,90*rw::math::Deg2Rad)
                                            ), state);
        std::vector<rw::math::Q> solutions = getConfigurations("Bottle", "GraspTCP", robotUR5, _wc, state);

        double curSolution = 0.0;
        for(unsigned int i=0; i<solutions.size(); i++){
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions[i], state);
            gripperDevice -> setQ(gQ,state);
            if( !detector->inCollision(state,NULL,true) )
            {
                rw::math::Q tempQ = solutions[i]-first;
                curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                if (curSolution<bestSolution)
                {
                    bestSolution = curSolution;
                    start = solutions[i];
                }
                break; // we only need one
            }
        }
    }
    bestSolution = 9999.9;
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
        Bottle->moveTo(
                    rw::math::Transform3D<>(rw::math::Vector3D<>(placeArea->getTransform(state).P()),
                                            rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,90*rw::math::Deg2Rad)
                                            ), state);
        std::vector<rw::math::Q> solutions = getConfigurations("Bottle", "GraspTCP", robotUR5, _wc, state);
        double curSolution = 0.0;
        for(unsigned int i=0; i<solutions.size(); i++){
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions[i], state);
            gripperDevice -> setQ(gQ,state);
            if( !detector->inCollision(state,NULL,true) ){
                rw::math::Q tempQ = solutions[i]-first;
                curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                if (curSolution<bestSolution)
                {
                    bestSolution = curSolution;
                    end = solutions[i];
                }
                break; // we only need one
            }
        }
    }

    // Point to Point interpolation
    PtParray[0] = start;
    PtParray[5] = end;
    _attachQ = start;
    _deattachQ = end;
    Q tempQ = first-start;
    double deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
    deltaD = sqrt(deltaD);
    _attachIdx = 20*deltaD;
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
    rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr currentLinearIntPol
            = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(first,start,deltaD));
    traj.add(currentLinearIntPol);
    for (int i = 0;i<4;i += 1)
    {
        rw::math::Q nextQ = PtParray[i+2];
        rw::math::Q currentQ = PtParray[i+1];
        rw::math::Q formerQ = PtParray[i];
        tempQ = formerQ-currentQ;
        deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        double totalDD = deltaD;
        currentLinearIntPol = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));

        tempQ = currentQ-nextQ;
        deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        totalDD = (totalDD+deltaD)*0.25;
        rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr nextLinearIntPol
                = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(currentQ,nextQ,deltaD));
        rw::trajectory::ParabolicBlend<rw::math::Q>::Ptr paraBlend1
                = ownedPtr(new rw::trajectory::ParabolicBlend<rw::math::Q>(currentLinearIntPol,nextLinearIntPol,totalDD));
        if (i == 0)
            traj.add(currentLinearIntPol);
        traj.add(paraBlend1,nextLinearIntPol);
    }

    QPath tempQPath;
    for (double s = 0.0;s<traj.duration();s += 0.05)
        tempQPath.push_back(traj.x(s));
    //_path = tempQPath;
}

// Source: "ReachabilityAnalysis_solution.zip" provided by Jeppe Langaa on BlackBoard
std::vector<rw::math::Q> SamplePlugin::getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{    
    // Source: "ReachabilityAnalysis_solution.zip" provided by Jeppe Langaa on BlackBoard
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }
    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

void SamplePlugin::createPathRRTConnect(rw::math::Vector3D<> to, double eps){
    // Source: "ReachabilityAnalysis_solution.zip" provided by Jeppe Langaa on BlackBoard
    // And "lab6_sol.zip" provided by Jeppe Langaa on BlackBoard
    log().info() << "RRT path" << "\n";
    // find relevant frames
    rw::kinematics::MovableFrame::Ptr Bottle = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    rw::kinematics::Frame* Bottle_Frame = _wc->findFrame("Bottle");
    rw::kinematics::FixedFrame::Ptr placeArea = _wc->findFrame<rw::kinematics::FixedFrame>("placeArea");
    rw::kinematics::Frame* gripperTCP = _wc->findFrame("GraspTCP");
    rw::models::SerialDevice::Ptr robotUR5 = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    rw::models::Device::Ptr gripperDevice = _wc->findDevice("WSG50");

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    // Gripper Q to be open
    const rw::math::Q gQ(1, 0.055); // 2.886

    // get the default state
    rw::kinematics::State state = _state;
    rw::math::Transform3D<> bottleStart = Bottle->getTransform(state);
    rw::math::Q first = robotUR5->getQ(state);
    rw::math::Q start;
    rw::math::Q end;
    double bestSolution = 9999.9;

    for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
        Bottle->moveTo(
                    rw::math::Transform3D<>(rw::math::Vector3D<>(to),
                                            rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,90*rw::math::Deg2Rad)
                                            ), state);
        std::vector<rw::math::Q> solutions = getConfigurations("Bottle", "GraspTCP", robotUR5, _wc, state);
        Bottle->moveTo(bottleStart,state);
        double curSolution = 0.0;
        for(unsigned int i=0; i<solutions.size(); i++){
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions[i], state);
            gripperDevice -> setQ(gQ,state);
            if( !detector->inCollision(state,NULL,true) )
            {
                rw::math::Q tempQ = solutions[i]-first;
                curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                if (curSolution<bestSolution)
                {
                    bestSolution = curSolution;
                    start = solutions[i];
                }
                break; // we only need one
            }
        }
    }
    bestSolution = 9999.9;
    for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
        Bottle->moveTo(
                    rw::math::Transform3D<>(rw::math::Vector3D<>(placeArea->getTransform(state).P()),
                                            rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,90*rw::math::Deg2Rad)
                                            ), state);
        std::vector<rw::math::Q> solutions = getConfigurations("Bottle", "GraspTCP", robotUR5, _wc, state);
        Bottle->moveTo(bottleStart,state);
        double curSolution = 0.0;
        for(unsigned int i=0; i<solutions.size(); i++){
            // set the robot in that configuration and check if it is in collision
            robotUR5->setQ(solutions[i], state);
            gripperDevice -> setQ(gQ,state);
            if( !detector->inCollision(state,NULL,true) ){
                rw::math::Q tempQ = solutions[i]-first;
                curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                if (curSolution<bestSolution)
                {
                    bestSolution = curSolution;
                    end = solutions[i];
                }
                break; // we only need one
            }
        }
    }
    Bottle->moveTo(bottleStart,state);

    const CollisionStrategy::Ptr cdstrategy =
            rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
    if (cdstrategy.isNull())
        RW_THROW("PQP Collision Strategy could not be found.");
    const CollisionDetector::Ptr collisionDetector =
            ownedPtr(new CollisionDetector(_wc, cdstrategy));
    const rw::pathplanning::PlannerConstraint con =
            rw::pathplanning::PlannerConstraint::make(collisionDetector, robotUR5, state);

    const rw::pathplanning::QSampler::Ptr Qsampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(robotUR5),con.getQConstraintPtr());
    const rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    const rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(con, Qsampler, metric, eps, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    rw::trajectory::QPath result;
    planner->query(first, start, result);

    rw::math::Q currentQ = result[1];
    rw::math::Q formerQ = result[0];
    rw::math::Q tempQ = formerQ-currentQ;
    double deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
    deltaD = sqrt(deltaD);
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
    rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr currentLinearIntPol
            = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));

    for (int i = 0;i<result.size()-1;i += 1)
    {
        currentQ = result[i+1];
        formerQ = result[i];
        tempQ = formerQ-currentQ;
        deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        currentLinearIntPol = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));
        traj.add(currentLinearIntPol);

    }

    rw::trajectory::QPath tempQPath;

    for (double s = 0.0;s<traj.duration();s += 0.05)
        tempQPath.push_back(traj.x(s));

    _attachIdx = tempQPath.size();
    _attachQ = start;
    _deattachQ = end;

    result.clear();
    robotUR5->setQ(start, state);
    rw::kinematics::Kinematics::gripFrame(Bottle_Frame,gripperTCP,state);
    // Create a new RRT object, to include the bottle in the device
    const CollisionStrategy::Ptr cdstrategy1 =
            rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
    if (cdstrategy1.isNull())
        RW_THROW("PQP Collision Strategy could not be found.");
    const CollisionDetector::Ptr collisionDetector1 =
            ownedPtr(new CollisionDetector(_wc, cdstrategy1));
    const rw::pathplanning::PlannerConstraint con1 =
            rw::pathplanning::PlannerConstraint::make(collisionDetector1, robotUR5, state);

    const rw::pathplanning::QSampler::Ptr Qsampler1 = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(robotUR5),con1.getQConstraintPtr());
    const rw::pathplanning::QToQPlanner::Ptr planner1 = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(con1, Qsampler1, metric, eps, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    planner1->query(start, end, result);

    currentQ = result[1];
    formerQ = result[0];
    tempQ = formerQ-currentQ;
    deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
    deltaD = sqrt(deltaD);
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj2;
    currentLinearIntPol
            = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));
    for (int i = 0;i<result.size()-1;i += 1)
    {
        currentQ = result[i+1];
        formerQ = result[i];
        tempQ = formerQ-currentQ;
        deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        currentLinearIntPol = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));
        traj2.add(currentLinearIntPol);
    }
    for (double s = 0.0;s<traj2.duration();s += 0.05)
        tempQPath.push_back(traj2.x(s));
    //_path = tempQPath;

}

double SamplePlugin::fRand(double fMin, double fMax)
{
    //Code: https://stackoverflow.com/questions/2704521/generate-random-double-numbers-in-c
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

double SamplePlugin::wrapMax(double x, double max)
{
    /* integer math: `(max + x % max) % max` */
    return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
double SamplePlugin::wrapMinMax(double x, double min, double max)
{
    return min + wrapMax(x - min, max - min);
}

struct element{int index;rw::math::Q Q1; rw::math::Vector3D<double> Pos; rw::math::Vector3D<double> RPY;}; //Used in TCMP

void SamplePlugin::TCMP(){
    rw::common::Timer testTimer;
    /// Find initial Q pose
    rw::kinematics::MovableFrame::Ptr Bottle = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    cout << "detector " << !detector->inCollision(_state,NULL,true) << endl;
    Q HOME(6, 1.571, -1.158, -2.728, 0.771, 1.571, 0);
    Bottle->moveTo(
                rw::math::Transform3D<>(rw::math::Vector3D<>(0.3, -0.5, 0.21),
                                        rw::math::RPY<>(-1.571, 0, 1.571)
                                        ), _state);
    getRobWorkStudio()->setState(_state);// Position of the bottle: Q[6]{0.04, 0.835, 0.11, -1.571, 0, 1.571}
    //Random q
    rw::math::Q qVal(6,0,0,0,0,0,0);
    do{
        for(int i = 0; i<6;i++){
            qVal[i] = fRand(-M_PI,M_PI);
        }
        _device2->setQ(qVal,_state);
    }
    while(detector->inCollision(_state,NULL,true));

    //Update state and get TCP pos
    cout << "Initial config found - Qval " << qVal << endl;
    qVal = HOME;
    _device2->setQ(qVal,_state);
    getRobWorkStudio()->setState(_state);

    rw::kinematics::Frame* robotTCP = _wc->findFrame("2_UR-6-85-5-A.TCP");
    rw::math::Transform3D<double> worldTTCP = rw::kinematics::Kinematics::worldTframe(robotTCP,_state);

    cout << "Postion of end " << worldTTCP.P() << " and rotation in RPY" << rw::math::RPY<double>(worldTTCP.R()) << std::endl;
    rw::math::Vector3D<double> initPos = {-0.3, 1.7, 0.3}; // FOR INV KIN: Q[6]{0.05, 0.975, 0.247, -3.142, -0.003, -1.546}
    rw::math::Vector3D<double> initRPY = {0, 0, -1.546};


    rw::math::RPY<double> TCPRPY = rw::math::RPY<double>(worldTTCP.R());
    rw::math::Vector3D<double> RPYasVec = {TCPRPY[0],TCPRPY[1],TCPRPY[2]};

    rw::math::Vector3D<double> RPYerr = initRPY - RPYasVec;
    rw::math::Vector3D<double> POSerr = initPos - worldTTCP.P();

    cout << "Pos " << POSerr << " \t norm(POSerr) " << POSerr.norm2() << " \n RPY " << RPYerr << " \t norm(RPY) " << RPYerr.norm2() << endl;
    rw::math::Q qNew(6,0,0,0,0,0,0);
    rw::math::Vector3D<double> newRPYerr;
    rw::math::Vector3D<double> newPOSerr;
    int counter = 0;
    int numOfReset = 0;
    int collideCounter = 0;
    //rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    double maxJointStep = 0.05;
    float constraints[6][2] = {{3.142,-3.142},{1.570,-4.712},{3.142,-3.142},{1.570,-4.712},{3.142,-3.142},{3.142,-3.142}};
    testTimer.resetAndResume();
    while(POSerr.norm2() > 0.05 || RPYerr.norm2() > 0.1){ //
        //Add random dq
        for(int i = 0; i<6;i++){
            qNew[i] = wrapMinMax(qVal[i]+fRand(-3.142,3.142)*maxJointStep,constraints[i][1],constraints[i][0]);
        }
        //Find new TCP error
        _device2->setQ(qNew,_state);
        worldTTCP = rw::kinematics::Kinematics::worldTframe(robotTCP,_state);
        TCPRPY = rw::math::RPY<double>(worldTTCP.R());
        RPYasVec = {TCPRPY[0],TCPRPY[1],TCPRPY[2]};
        newRPYerr = initRPY - RPYasVec;
        newPOSerr = initPos - worldTTCP.P();


        if( ((newPOSerr.norm2() < POSerr.norm2() && RPYerr.norm2() < 0.05)) || (newPOSerr.norm2() < POSerr.norm2() && newRPYerr.norm2() < RPYerr.norm2()) && !detector->inCollision(_state,NULL,true) ){
            qVal = qNew;
            RPYerr = newRPYerr;
            POSerr = newPOSerr;
            //cout << "update and Qval " << qVal << endl;
            getRobWorkStudio()->setState(_state);
            collideCounter = 0;
        }else{
            collideCounter++;
        }

        counter++;
        if(counter  == 1000000){
            cout << "Reset robot to home and try again" << endl;
            qVal = HOME;
            _device2->setQ(qVal,_state);
            RPYerr = initRPY - RPYasVec;
            POSerr = initPos - worldTTCP.P();
            counter = 0;
            numOfReset++;

        }
        if (numOfReset > 9){
            return;
        }
    }

    cout<<"Found initial configuration! CounterVal " << counter+10000*numOfReset << "\t"<< qVal << " Number of resets " << numOfReset << " Took " << testTimer.getTimeMs() << " milliseconds " << endl;

    std::vector<element> Tree;

    Tree.push_back({1,qVal,worldTTCP.P(),RPYasVec});
    counter = 0; //Reset counter
     //Thus, max joint step is pi*0.05. I.e. maxJointStep value is used in generating random dq
    maxJointStep = 0.05;
    double discretizeStep = 0.01;

    int treeSize = 5000;
    int idx;
    rw::math::Q tempQ; //Used to check a new candidate Q that might go into the tree

    bool stopFlag=false;
    //############## Add goal position - given in XYZ and RPY
    rw::math::Vector3D<double> goalPos = {0, 0.90, 0.50}; // FOR INV KIN: Q[6]{0.047, 0.725, 0.59, -3.139, 0.006, -1.513}
    rw::math::Vector3D<double> goalRPY = {3.142, 0, -1.570};

    std::vector<rw::math::Vector3D<>> path = linePath(worldTTCP.P(),goalPos,discretizeStep);

    double distToCurr = (path[0]-worldTTCP.P()).norm2();
    double oldDistToNext = (path[1]-worldTTCP.P()).norm2();
    double newDistToNext;
    cout << "oldDistToNext " << oldDistToNext << " distToCúrr " << distToCurr << endl;
    //cin.get();

    int currTreeSize = 1; //The zero'ed position in the Tree is filled with initial configuration
    int pathIdx = 0;


    testTimer.resetAndResume();
    while(pathIdx < path.size()-1){
        tempQ = Tree[currTreeSize-1].Q1;//Directed RRT - thus not random index from the tree - But could be random from the Tree
        //_device2->setQ(tempQ,_state);
        //worldTTCP = rw::kinematics::Kinematics::worldTframe(robotTCP,_state);
        //oldDistToNext = (path[pathIdx+1]-worldTTCP.P()).norm2();
        //Add random dq
        for(int i = 0; i<5;i++){
            tempQ[i] = wrapMinMax(tempQ[i]+fRand(-3.142,3.142)*maxJointStep,constraints[i][1],constraints[i][0]);
        }

        //Move robot and update distances
        _device2->setQ(tempQ,_state);
        worldTTCP = rw::kinematics::Kinematics::worldTframe(robotTCP,_state);
        distToCurr = (path[currTreeSize]-worldTTCP.P()).norm2();
        newDistToNext = (path[currTreeSize+1]-worldTTCP.P()).norm2();
        //cout << "newDistToNext " << newDistToNext << " \tdistToCurr " << distToCurr << endl;

        if( (distToCurr < 2*discretizeStep) && !detector->inCollision(_state,NULL,true) ){
            Tree.push_back({currTreeSize,tempQ,worldTTCP.P(),RPYasVec});
            currTreeSize++;
            //cout<<"update " << pathIdx <<"/"<<path.size()<< endl;
            pathIdx++;

        }
        counter++;
        if(counter % 10000000 == 0){
            cout << "Counter value " << counter/10000000 << " million - Current pathIdx: " << pathIdx <<"/"<<path.size()<< endl;
        }else if(counter == 100000000){ //Break at 100 million
            stopFlag = true;
            break;
        }
    }
    testTimer.pause();



    if(stopFlag == true){
        cout << "Stopped after " << counter << " iterations because of STOPFLAG! - pathIdx is " << pathIdx <<"/"<<path.size()<< endl;
    }else{
        cout << "Found goal! In " << counter << " iterations. - Tree length is " << Tree.size() << " and it took " << testTimer.getTimeMs() <<" milliseconds to find path."<< endl;
    }
    //_path.clear();
    for(int i=0; i <Tree.size(); i++){
        //_path.push_back(Tree[i].Q1);
    }
    _device2->setQ(Tree[0].Q1,_state);//Reset robot
    //blendPath();

}

QPath SamplePlugin::blendPath(QPath path){
    rw::math::Q nextQ = path[2];
    rw::math::Q currentQ = path[1];
    rw::math::Q formerQ = path[0];
    rw::math::Q tempQ = formerQ-currentQ;
    double deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
    deltaD = sqrt(deltaD);
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
    rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr currentLinearIntPol
            = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(path[0],path[1],deltaD));
    traj.add(currentLinearIntPol);
    for (int i = 1;i<path.size()-2;i += 1)
    {
        nextQ = path[i+2];
        currentQ = path[i+1];
        formerQ = path[i];
        tempQ = formerQ-currentQ;
        deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        double totalDD = deltaD;
        currentLinearIntPol = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));

        tempQ = currentQ-nextQ;
        deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        totalDD = (totalDD+deltaD)*0.25;
        rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr nextLinearIntPol
                = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(currentQ,nextQ,deltaD));
        rw::trajectory::ParabolicBlend<rw::math::Q>::Ptr paraBlend1
                = ownedPtr(new rw::trajectory::ParabolicBlend<rw::math::Q>(currentLinearIntPol,nextLinearIntPol,totalDD));
        if (i == 1)
            traj.add(currentLinearIntPol);
        traj.add(paraBlend1,nextLinearIntPol);
    }

    QPath tempQPath;
    for (double s = 0.0;s<traj.duration();s += 0.05)
        tempQPath.push_back(traj.x(s));
    return tempQPath;
}

std::vector<rw::math::Vector3D<double>> SamplePlugin::linePath(rw::math::Vector3D<> start,rw::math::Vector3D<> end, double stepSize){
    rw::math::Vector3D<> diff = end - start;
    int Nsteps = diff.norm2()/stepSize;
    std::vector<rw::math::Vector3D<>> path;
    rw::math::Vector3D<> curr = start;
    for(int i=0; i<Nsteps; i++){
        curr+= diff/Nsteps;
        path.push_back(curr);
    }
    if(curr != end){
        path.push_back(end);
    }
    return path;
}

void SamplePlugin::kuusTest(){
    rw::models::SerialDevice::Ptr robot1 = _wc->findDevice<rw::models::SerialDevice>("1_UR-6-85-5-A");
    rw::models::SerialDevice::Ptr robot2 = _wc->findDevice<rw::models::SerialDevice>("2_UR-6-85-5-A");
    rw::kinematics::MovableFrame::Ptr Bottle = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    rw::kinematics::MovableFrame::Ptr preMatingArea1 = _wc->findFrame<rw::kinematics::MovableFrame>("preMatingArea1");
    rw::kinematics::MovableFrame::Ptr BottleGrip1 = _wc->findFrame<rw::kinematics::MovableFrame>("BottleGrip1");
    rw::kinematics::MovableFrame::Ptr BottleGrip2 = _wc->findFrame<rw::kinematics::MovableFrame>("BottleGrip2");
    rw::models::Device::Ptr gripperDevice1 = _wc->findDevice("1_WSG50");
    rw::models::Device::Ptr gripperDevice2 = _wc->findDevice("2_WSG50");
    rw::common::Timer myTimer;
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    const rw::math::Q gQ(1, 0.055);
    gripperDevice1 -> setQ(gQ,_state);
    gripperDevice2 -> setQ(gQ,_state);
    rw::kinematics::State state = _state;
    rw::math::Q sol1;
    rw::math::Q sol2;
    rw::math::Q starting;
    rw::math::Q startingConfig = robot1 ->getQ(state);
    std::vector<rw::math::Q> solutions = getConfigurations("preMatingArea1", "GraspTCP1", robot1, _wc, state);
    double bestSolution = 9999.9;
    myTimer.resetAndResume();
    int counterSolutions = 0;
    for(double yawAngle=0; yawAngle<360.0; yawAngle+=10.0){
        for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
            preMatingArea1->moveTo(
                        rw::math::Transform3D<>(rw::math::Vector3D<>(Bottle->getTransform(state).P()),
                                                rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,yawAngle*rw::math::Deg2Rad)
                                                ), state);
            std::vector<rw::math::Q> solutions = getConfigurations("preMatingArea1", "GraspTCP1", robot1, _wc, state);
            counterSolutions += solutions.size();
            for(unsigned int i=0; i<solutions.size(); i++){
                // set the robot in that configuration and check if it is in collision
                robot1->setQ(solutions[i], state);
                if( !detector->inCollision(state,NULL,true) )
                {
                    rw::math::Q tempQ = solutions[i]-startingConfig;
                    double curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                    if (curSolution<bestSolution)
                    {
                        bestSolution = curSolution;
                        starting = solutions[i];
                    }
                }
            }
        }
    }
    _device1->setQ(starting,state);
    _device2->setQ(starting,state);
    bestSolution = 9999.9;
    for(double yawAngle=90; yawAngle<360.0; yawAngle+=180.0){
        for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
            Bottle->moveTo(
                        rw::math::Transform3D<>(rw::math::Vector3D<>(Bottle->getTransform(state).P()),
                                                rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,yawAngle*rw::math::Deg2Rad)
                                                ), state);
            std::vector<rw::math::Q> solutions = getConfigurations("BottleGrip1", "GraspTCP1", robot1, _wc, state);
            counterSolutions += solutions.size();
            for(unsigned int i=0; i<solutions.size(); i++){
                // set the robot in that configuration and check if it is in collision
                robot1->setQ(solutions[i], state);
                if( !detector->inCollision(state,NULL,true) )
                {
                    rw::math::Q tempQ = solutions[i]-starting;
                    double curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                    if (curSolution<bestSolution)
                    {
                        bestSolution = curSolution;
                        sol1 = solutions[i];
                    }
                }
            }
        }
    }
    bestSolution = 9999.9;
    for(double yawAngle=90; yawAngle<360.0; yawAngle+=180.0){
        for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
            Bottle->moveTo(
                        rw::math::Transform3D<>(rw::math::Vector3D<>(Bottle->getTransform(state).P()),
                                                rw::math::RPY<>(rollAngle*rw::math::Deg2Rad,0,yawAngle*rw::math::Deg2Rad)
                                                ), state);
            std::vector<rw::math::Q> solutions = getConfigurations("BottleGrip2", "GraspTCP2", robot2, _wc, state);
            counterSolutions += solutions.size();
            for(unsigned int i=0; i<solutions.size(); i++){
                // set the robot in that configuration and check if it is in collision
                robot2->setQ(solutions[i], state);
                if( !detector->inCollision(state,NULL,true) )
                {
                    rw::math::Q tempQ = solutions[i]-starting;
                    double curSolution = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
                    if (curSolution<bestSolution)
                    {
                        bestSolution = curSolution;
                        sol2 = solutions[i];
                    }
                }
            }
        }
    }
    cout << "Spent " << myTimer.getTimeMs()/1000.0 << "s with " << counterSolutions << " solutions tested." << endl;
    _path1 = move(starting,sol1,robot1,_state);
    _path2 = move(starting,sol2,robot2,_state);
}

void SamplePlugin::planeFunc(){
    rw::math::Vector3D<> p1 = rw::math::Vector3D<>(-0.3,-0.5,0.21);
    rw::math::Vector3D<> p2 = rw::math::Vector3D<>(0.3,-0.5,0.21);
    rw::math::Vector3D<> p3 = rw::math::Vector3D<>(0,0.60,0.50);
    rw::geometry::Plane aPlane = rw::geometry::Plane(p1,p2,p3);
    cout << aPlane.distance(rw::math::Vector3D<>(0,0.45,0.5)) << endl;
}

void SamplePlugin::createTree(rw::geometry::Plane aPlane,rw::kinematics::State state, int robotNum, int size)
{
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    rw::common::Timer myTimer;
    robotPtr rPtr;
    if (robotNum == 1)
        rPtr = robotPtr1;
    else
        rPtr = robotPtr2;

    rw::math::Q qNew = rPtr.robot->getQ(state);
    rPtr.ptrGraph -> nodeVec.clear();
    myTimer.resetAndResume();
    while(rPtr.ptrGraph -> nodeVec.size() < size)
    {
        //Add random dq
        for(int i = 0; i<6;i++){
            qNew[i] = fRand(jointConstraints[i][0],jointConstraints[i][1]);
        }
        //Find new TCP error
        if (RGD_New_Config(aPlane,&qNew,rPtr,&state,0.1))
        {
            //rPtr.robot->setQ(qNew,state);
            if (!detector->inCollision(state,NULL,true))
            {
                graphNode newNode;
                newNode.configuration = qNew;
                newNode.postion = rw::kinematics::Kinematics::frameTframe(rPtr.table,rPtr.robotTCP,state).P();
                //newNode.postion = rw::kinematics::Kinematics::worldTframe(rPtr.robotTCP,state).P();
                newNode.index = rPtr.ptrGraph -> nodeVec.size();
                rPtr.ptrGraph -> nodeVec.push_back(newNode);
            }

        }
    }
    cout << "Time spent creating graph: " << myTimer.getTimeMs()/1000.0 << "s" << endl;
}

bool SamplePlugin::RGD_New_Config(rw::geometry::Plane aPlane, rw::math::Q* q, robotPtr rPtr, rw::kinematics::State* state, float dMax)
{
    rw::math::Transform3D<double> worldTTCP;
    rPtr.robot->setQ(*q,*state);
    worldTTCP = rw::kinematics::Kinematics::frameTframe(rPtr.table,rPtr.robotTCP,*state);

    float distance = 99999;
    float maxErr = 0.01;
    float newDistance = 0;

    int maxI = 1000;
    int maxJ = 10;
    int i = 0;
    int j = 0;

    rw::math::Q qUpdate = *q;
    while(maxI > i && maxJ > j)
    {
        i++;
        j++;
        //Random gradient decent
        qUpdate = *q;
        for(int i = 0; i<6;i++){
            qUpdate[i] = wrapMinMax(qUpdate[i]+fRand(jointConstraints[i][0],jointConstraints[i][1])*dMax,jointConstraints[i][0],jointConstraints[i][1]);
        }
        rPtr.robot->setQ(qUpdate,*state);
        worldTTCP = rw::kinematics::Kinematics::frameTframe(rPtr.table,rPtr.robotTCP,*state);
        newDistance = abs(aPlane.distance(worldTTCP.P()));
        if (newDistance < distance)
        {
            j = 0;
            distance = newDistance;
            *q = qUpdate;

            if (distance < maxErr)
                return true;
        }
    }
    return false;
}

void SamplePlugin::connectGraph(rw::kinematics::State state, int robotNum)
{
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    rw::common::Timer myTimer;
    robotPtr rPtr;
    if (robotNum == 1)
        rPtr = robotPtr1;
    else
        rPtr = robotPtr2;

    float dMax = 0.20;
    float cMax = 2;
    int maxConnections = 5;
    myTimer.resetAndResume();
    for(int i = 0;i<rPtr.ptrGraph->nodeVec.size();i++)
    {
        if (i%100 == 0 && i != 0)
            cout << "Connection index = " << i << endl;
        std::vector<connections> possibleConnections;

        for(int j = 0;j<rPtr.ptrGraph->nodeVec.size();j++)
        {
            if(j == i)
                continue;
            rw::math::Vector3D<> direction = rPtr.ptrGraph->nodeVec[j].postion-rPtr.ptrGraph->nodeVec[i].postion;
            rw::math::Q tempQ = rPtr.ptrGraph->nodeVec[i].configuration - rPtr.ptrGraph->nodeVec[j].configuration;
            float cDist = sqrt(tempQ(0)*tempQ(0)*jointWeights[0]+tempQ(1)*tempQ(1)*jointWeights[1]+tempQ(2)*tempQ(2)*jointWeights[2]+tempQ(3)*tempQ(3)*jointWeights[3]+tempQ(4)*tempQ(4)*jointWeights[4]+tempQ(5)*tempQ(5)*jointWeights[5]);
            float distance = direction.norm2();
            if(cDist < cMax) // distance < dMax &&
            {
                connections newC;
                newC.distance = distance;
                newC.cost = cDist;
                newC.index = j;
                auto it = std::lower_bound(possibleConnections.begin(), possibleConnections.end(),newC,lessThan);
                possibleConnections.insert(it,newC);
            }
        }
        int counter = 0;
        int jIterator = 0;
        bool connectExists = false;
        while (counter < maxConnections && jIterator < possibleConnections.size())
        {
            connectExists = false;
            for (int E = 0;E<rPtr.ptrGraph->nodeVec[i].connectionVec.size();E++)
            {
                if (rPtr.ptrGraph->nodeVec[i].connectionVec[E].index == possibleConnections[jIterator].index)
                {
                    connectExists = true;
                    break;
                }
            }

            if (connectExists)
            {
                counter++;
                jIterator++;
                continue;
            }

            int split = (possibleConnections[jIterator].cost/2) + 2;

            if(canConnect(rPtr.ptrGraph->nodeVec[i].configuration,rPtr.ptrGraph->nodeVec[possibleConnections[jIterator].index].configuration,rPtr,detector,state, split))
            {
                counter++;
                rPtr.ptrGraph->nodeVec[i].connectionVec.push_back(possibleConnections[jIterator]);
                connections newC;
                newC.distance = possibleConnections[jIterator].distance;
                newC.cost = possibleConnections[jIterator].cost;
                newC.index = i;
                rPtr.ptrGraph->nodeVec[possibleConnections[jIterator].index].connectionVec.push_back(newC);
            }
            jIterator++;
        }
    }
    cout << "Time spent connecting graph: "<<myTimer.getTimeMs()/1000.0 << "s" << endl;

}

void SamplePlugin::lazyConnectGraph(int robotNum)
{
    rw::common::Timer myTimer;
    robotPtr rPtr;

    if (robotNum == 1)
        rPtr = robotPtr1;
    else
        rPtr = robotPtr2;

    float cMax = 2.5;

    myTimer.resetAndResume();
    for(int i = 0;i<rPtr.ptrGraph->nodeVec.size();i++)
    {
        if (i%1000 == 0 && i != 0)
            cout << "Connection index = " << i << endl;
        std::vector<connections> possibleConnections;

        for(int j = 0;j<rPtr.ptrGraph->nodeVec.size();j++)
        {
            if(j == i)
                continue;
            rw::math::Vector3D<> direction = rPtr.ptrGraph->nodeVec[j].postion-rPtr.ptrGraph->nodeVec[i].postion;
            rw::math::Q tempQ = rPtr.ptrGraph->nodeVec[i].configuration - rPtr.ptrGraph->nodeVec[j].configuration;
            float cDist = sqrt(tempQ(0)*tempQ(0)*jointWeights[0]+tempQ(1)*tempQ(1)*jointWeights[1]+tempQ(2)*tempQ(2)*jointWeights[2]+tempQ(3)*tempQ(3)*jointWeights[3]+tempQ(4)*tempQ(4)*jointWeights[4]+tempQ(5)*tempQ(5)*jointWeights[5]);
            float distance = direction.norm2();
            if(cDist < cMax) // distance < dMax &&
            {
                connections newC;
                newC.distance = distance;
                newC.cost = cDist;
                newC.index = j;
                auto it = std::lower_bound(possibleConnections.begin(), possibleConnections.end(),newC,lessThan);
                possibleConnections.insert(it,newC);
            }
        }
        int counter = 0;
        int jIterator = 0;
        bool connectExists = false;
        while (jIterator < possibleConnections.size())
        {
            connectExists = false;
            for (int E = 0;E<rPtr.ptrGraph->nodeVec[i].connectionVec.size();E++)
            {
                if (rPtr.ptrGraph->nodeVec[i].connectionVec[E].index == possibleConnections[jIterator].index)
                {
                    connectExists = true;
                    break;
                }
            }

            if (connectExists)
            {
                counter++;
                jIterator++;
                continue;
            }

            rPtr.ptrGraph->nodeVec[i].connectionVec.push_back(possibleConnections[jIterator]);
            connections newC;
            newC.distance = possibleConnections[jIterator].distance;
            newC.cost = possibleConnections[jIterator].cost;
            newC.index = i;
            rPtr.ptrGraph->nodeVec[possibleConnections[jIterator].index].connectionVec.push_back(newC);

            jIterator++;
        }
    }
    cout << "Time spent connecting graph: "<<myTimer.getTimeMs()/1000.0 << "s" << endl;
}

bool SamplePlugin::canConnect(rw::math::Q q1, rw::math::Q q2, robotPtr rPtr, rw::proximity::CollisionDetector::Ptr detector, State state, int splits)
{
    rw::math::Q q3 = q1 + ((q2-q1)/2);

    rPtr.robot->setQ(q3,state);

    if (detector->inCollision(state,NULL,true))
        return false;

    if (splits == 0)
        return true;
    else
    {
        bool left = canConnect(q3,q2,rPtr,detector,state,splits-1);
        bool right = canConnect(q1,q3,rPtr,detector,state,splits-1);
        if (left && right)
            return true;
        else
            return false;
    }
}

void SamplePlugin::findPath(rw::math::Vector3D<> start, rw::math::Vector3D<> goal, int robotNum)
{
    robotPtr rPtr;
    if (robotNum == 1)
        rPtr = robotPtr1;
    else
        rPtr = robotPtr2;

    const int n = rPtr.ptrGraph->nodeVec.size();

    int closest_start_node_index,closest_goal_node_index;

    double min_start_dist=10000,min_goal_dist=10000, cur_err;

    //find the closest start and goal Qs in graph
    for(int i =0;i<rPtr.ptrGraph->nodeVec.size();i++)
    {
        cur_err=(rPtr.ptrGraph->nodeVec[i].postion-goal).norm2();
        if (cur_err<min_start_dist && rPtr.ptrGraph->nodeVec[i].connectionVec.size() > 0)
        {
            min_start_dist =cur_err;
            closest_start_node_index = i;
        }
        cur_err=(rPtr.ptrGraph->nodeVec[i].postion-start).norm2();
        if (cur_err<min_goal_dist && rPtr.ptrGraph->nodeVec[i].connectionVec.size() > 0)
        {
            min_goal_dist =cur_err;
            closest_goal_node_index = i;
        }
    }
    cout << "Goal index " << closest_goal_node_index << " start index " <<closest_start_node_index << endl;

    cout<<"\nQUERYING\n";
    // Find path with modified A*
    float* distance;
    float* pred;
    distance = new float[n];
    pred = new float[n];

    bool* visited;
    visited = new bool[n];
    int nextnode,i,j;
    float mindistance;

    for(i=0;i<n;i++) {
       distance[i]=costFunc(closest_start_node_index, i,robotNum);
       pred[i]=closest_start_node_index;
       visited[i]=false;
    }

    distance[closest_start_node_index]=0;
    visited[closest_start_node_index]=true;
    int count=1;
    while(count<n-1)
    {
        mindistance=50000;
        for(i=0;i<n;i++)
          if(distance[i]<mindistance&&!visited[i])
            {
              mindistance=distance[i];
              nextnode=i;
            }
        visited[nextnode]=true;
        for(i=0;i<n;i++)
          if(!visited[i])
               if(mindistance+costFunc(nextnode,i,robotNum)<distance[i])
                {
                  distance[i]=mindistance+costFunc(nextnode,i,robotNum);
                  pred[i]=nextnode;
               }
        count++;
    }

    vector<rw::math::Q> Q_path;

   std::cout<<"Distance of node "<<closest_goal_node_index<<"="<<distance[i] << endl;
   std::cout<<"Path = "<<closest_goal_node_index << endl;

   Q_path.push_back(rPtr.ptrGraph->nodeVec[closest_goal_node_index].configuration);
   j=closest_goal_node_index;
    saveGraphIdx.push_back(j);
    do {
        j=pred[j];
        Q_path.push_back(rPtr.ptrGraph->nodeVec[j].configuration);
        cout<<" <- "<<j << " Q: " << rPtr.ptrGraph->nodeVec[j].configuration << " Pos: "<< rPtr.ptrGraph->nodeVec[j].postion <<endl;
        saveGraphIdx.push_back(j);
    }while(j!=closest_start_node_index);

    cout << "PtP" << endl;
    // Point to Point interpolation
    if (Q_path.size() > 2)
    {
        Q tempQ = Q_path[0] - Q_path[1];
        double deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
        rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr currentLinearIntPol
                = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(Q_path[0],Q_path[1],deltaD));
        traj.add(currentLinearIntPol);
        for (int i = 1;i<Q_path.size()-1;i += 1)
        {
            rw::math::Q currentQ = Q_path[i+1];
            rw::math::Q formerQ = Q_path[i];
            tempQ = formerQ-currentQ;
            deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
            deltaD = sqrt(deltaD);
            currentLinearIntPol = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));
            traj.add(currentLinearIntPol);
        }

        QPath tempQPath;
        for (double s = 0.0;s<traj.duration();s += 0.05)
            tempQPath.push_back(traj.x(s));

         _path1 = tempQPath;
    }

}

void SamplePlugin::lazyFindPath(rw::math::Vector3D<> start, rw::math::Vector3D<> goal, State state, int robotNum)
{
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    rw::common::Timer myTimer;
    robotPtr rPtr;
    if (robotNum == 1)
        rPtr = robotPtr1;
    else
        rPtr = robotPtr2;
    const int n = rPtr.ptrGraph->nodeVec.size();
    int closest_start_node_index,closest_goal_node_index;
    double min_start_dist=10000,min_goal_dist=10000, cur_err;

    float* distance;
    float* pred;
    distance = new float[n];
    pred = new float[n];

    bool* visited;
    visited = new bool[n];
    int nextnode,i,j;
    float mindistance;

    bool collisionFound = false;
    vector<rw::math::Q> Q_path;
    vector<int> Q_idx;
    myTimer.resetAndResume();
    while(true)
    {
        for(int k =0;k<rPtr.ptrGraph->nodeVec.size();k++)
        {
            cur_err=(rPtr.ptrGraph->nodeVec[k].postion-goal).norm2();
            if (cur_err<min_start_dist && rPtr.ptrGraph->nodeVec[k].connectionVec.size() > 0)
            {
                min_start_dist =cur_err;
                closest_start_node_index = k;
            }
            cur_err=(rPtr.ptrGraph->nodeVec[k].postion-start).norm2();
            if (cur_err<min_goal_dist && rPtr.ptrGraph->nodeVec[k].connectionVec.size() > 0)
            {
                min_goal_dist =cur_err;
                closest_goal_node_index = k;
            }
        }
        cout << "Goal index " << closest_goal_node_index << " start index " <<closest_start_node_index << endl;
        cout<<"\nQUERYING\n";
        // Find path with modified A*

        int timeBefore = myTimer.getTimeMs();
//        for(i=0;i<n;i++) {
//           distance[i]=costFunc(closest_start_node_index, i,robotNum);
//           pred[i]=closest_start_node_index;
//           visited[i]=false;
//        }
//        cout << "First loop: "<< (myTimer.getTimeMs()-timeBefore)/1000.0 <<"s" << endl;
//        if (firstRun)
//        {
//            for(i=0;i<n;i++) {
//               distance[i]=costFunc(closest_start_node_index, i,robotNum);
//               pred[i]=closest_start_node_index;
//               visited[i]=false;
//            }
//            firstRun = false;
//        }
//        else
//        {
//            // cost to 500 for the connection that is removed.
//            distance[Q_idx[i]] = 500;
//            distance[Q_idx[i+1]] = 500;
//            for(i=0;i<n;i++) {
//               pred[i]=closest_start_node_index;
//               visited[i]=false;
//            }
//        }

//        distance[closest_start_node_index]=0;
//        visited[closest_start_node_index]=true;
//        int count=1;
//        timeBefore = myTimer.getTimeMs();
//        while(count<n-1)
//        {
//            mindistance=50000;
//            for(i=0;i<n;i++)
//              if(distance[i]<mindistance&&!visited[i])
//                {
//                  mindistance=distance[i];
//                  nextnode=i;
//                }
//            visited[nextnode]=true;
//            for(i=0;i<n;i++)
//              if(!visited[i])
//              {
//                  float someValue = costFunc(nextnode,i,robotNum);
//                   if(mindistance+someValue<distance[i])
//                    {
//                      distance[i]=mindistance+someValue;
//                      pred[i]=nextnode;
//                   }
//              }
//            count++;
//        }
//        cout << "Second loop: "<< (myTimer.getTimeMs()-timeBefore)/1000.0 <<"s" << endl;
        timeBefore = myTimer.getTimeMs();
        vector<int> aPath = Astar(closest_start_node_index,closest_goal_node_index,robotNum);
        cout << "Astar: "<< (myTimer.getTimeMs()-timeBefore)/1000.0 <<"s" << endl;
        std::cout<<"Distance of node "<<closest_goal_node_index<<"="<<distance[i] << endl;
        std::cout<<"Path = "<<closest_goal_node_index << endl;
        Q_path.clear();
        Q_idx.clear();
        //Q_path.push_back(rPtr.ptrGraph->nodeVec[closest_goal_node_index].configuration);
        //Q_idx.push_back(closest_goal_node_index);
        cout << "A path size: " << aPath.size() << endl;
        for(i = 0;i<aPath.size();i++)
        {
            Q_path.push_back(rPtr.ptrGraph->nodeVec[aPath[i]].configuration);
            Q_idx.push_back(aPath[i]);
        }
//        j=closest_goal_node_index;
//         saveGraphIdx.push_back(j);
//         do {
//             j=pred[j];
//             Q_path.push_back(rPtr.ptrGraph->nodeVec[j].configuration);
//             Q_idx.push_back(j);
//             cout<<" <- "<<j << " Q: " << rPtr.ptrGraph->nodeVec[j].configuration << " Pos: "<< rPtr.ptrGraph->nodeVec[j].postion <<endl;
//             saveGraphIdx.push_back(j);
//         }while(j!=closest_start_node_index);
        timeBefore = myTimer.getTimeMs();

        for (i = 0;i<Q_path.size()-1;i++)
         {
             int split = 2;
             int indexJ1,indexJ2;
             for(j = 0;j<rPtr.ptrGraph->nodeVec[Q_idx[i]].connectionVec.size();j++)
             {
                 if (rPtr.ptrGraph->nodeVec[Q_idx[i]].connectionVec[j].index == Q_idx[i+1])
                     indexJ1 = j;
             }
             for(j = 0;j<rPtr.ptrGraph->nodeVec[Q_idx[i+1]].connectionVec.size();j++)
             {
                 if (rPtr.ptrGraph->nodeVec[Q_idx[i+1]].connectionVec[j].index == Q_idx[i])
                     indexJ2 = j;
             }
             if (!rPtr.ptrGraph->nodeVec[Q_idx[i]].connectionVec[indexJ1].collisionChecked && !canConnect(Q_path[i],Q_path[i+1],rPtr,detector,state,split))
             {
                 cout << "Collision in lazy path finding between "<< Q_idx[i] << " and " << Q_idx[i+1] << endl;

                 rPtr.ptrGraph->nodeVec[Q_idx[i]].connectionVec.erase(rPtr.ptrGraph->nodeVec[Q_idx[i]].connectionVec.begin()+indexJ1);
                 rPtr.ptrGraph->nodeVec[Q_idx[i+1]].connectionVec.erase(rPtr.ptrGraph->nodeVec[Q_idx[i+1]].connectionVec.begin()+indexJ2);

                 collisionFound = true;
             }
             else
             {
                 rPtr.ptrGraph->nodeVec[Q_idx[i]].connectionVec[indexJ1].collisionChecked = true;
                 rPtr.ptrGraph->nodeVec[Q_idx[i+1]].connectionVec[indexJ2].collisionChecked = true;
             }

         }
        cout << "Third loop: "<< (myTimer.getTimeMs()-timeBefore)/1000.0 <<"s" << endl;

         if (!collisionFound)
             break;
         else
             collisionFound = false;

    }


    cout << "Time spent find a path: " << myTimer.getTimeMs()/1000.0 << "s" << endl;
    //find the closest start and goal Qs in graph


    cout << "PtP" << endl;
    // Point to Point interpolation
    if (Q_path.size() > 2)
    {
        Q tempQ = Q_path[0] - Q_path[1];
        double deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
        rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr currentLinearIntPol
                = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(Q_path[0],Q_path[1],deltaD));
        traj.add(currentLinearIntPol);
        for (int i = 1;i<Q_path.size()-1;i += 1)
        {
            rw::math::Q currentQ = Q_path[i+1];
            rw::math::Q formerQ = Q_path[i];
            tempQ = formerQ-currentQ;
            deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
            deltaD = sqrt(deltaD);
            currentLinearIntPol = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));
            traj.add(currentLinearIntPol);
        }

        QPath tempQPath;
        for (double s = 0.0;s<traj.duration();s += 0.05)
            tempQPath.push_back(traj.x(s));

        if (robotNum == 1)
            _path1 = tempQPath;
        else
            _path2 = tempQPath;
    }

}

vector<int> SamplePlugin::Astar(int startIdx, int goalIdx, int robotNum)
{
    robotPtr rPtr;
    if (robotNum == 1)
        rPtr = robotPtr1;
    else
        rPtr = robotPtr2;

    priority_queue<graphNode *,vector<graphNode *>,compareNodes> openList;

    set<int> closedList;
    rPtr.ptrGraph->nodeVec[startIdx].heuristic = (rPtr.ptrGraph->nodeVec[startIdx].postion-rPtr.ptrGraph->nodeVec[goalIdx].postion).norm2();
    rPtr.ptrGraph->nodeVec[startIdx].cost = rPtr.ptrGraph->nodeVec[startIdx].heuristic;
    rPtr.ptrGraph->nodeVec[startIdx].distance = 0;
    openList.push(&rPtr.ptrGraph->nodeVec[startIdx]);

    while(!openList.empty())
    {
        graphNode * curPtr = openList.top();

        openList.pop();

        closedList.insert(curPtr->index);

        if (curPtr->index == goalIdx)
            break;

        for (int i = 0;i<curPtr->connectionVec.size();i++)
        {
            if (closedList.find(curPtr->connectionVec[i].index) == closedList.end())
            {
                rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index].heuristic =  (rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index].postion-rPtr.ptrGraph->nodeVec[goalIdx].postion).norm2();
                rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index].distance = curPtr->distance+(rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index].postion-rPtr.ptrGraph->nodeVec[curPtr->index].postion).norm2();
                rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index].parentIdx = curPtr->index;
                rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index].cost = rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index].heuristic + rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index].distance;
                openList.push(&rPtr.ptrGraph->nodeVec[curPtr->connectionVec[i].index]);
            }

        }
    }
    vector<int> path;
    graphNode * curPtr = &rPtr.ptrGraph->nodeVec[goalIdx];
    while(curPtr->index != startIdx)
    {
        path.push_back(curPtr->index);
        curPtr = &rPtr.ptrGraph->nodeVec[curPtr->parentIdx];
    }
    return path;
}

float SamplePlugin::costFunc(int iIdx, int jIdx, int robotNum)
{
    robotPtr rPtr;
    if (robotNum == 1)
        rPtr = robotPtr1;
    else
        rPtr = robotPtr2;

    for (int i = 0;i<rPtr.ptrGraph->nodeVec[iIdx].connectionVec.size();i++)
    {
        if (rPtr.ptrGraph->nodeVec[iIdx].connectionVec[i].index == jIdx)
            return rPtr.ptrGraph->nodeVec[iIdx].connectionVec[i].distance; // .cost
    }
    return 500;
}

void SamplePlugin::DualPRM(rw::math::Q goalConfRobot1, rw::math::Q goalConfRobot2, rw::kinematics::State state, int maximumIterations, float jointStep)
{
    curDualGraph.r1Goal.clear();
    curDualGraph.r2Goal.clear();
    curDualGraph.r1Start.clear();
    curDualGraph.r2Start.clear();
    Our4Darray.Goal.clear();
    Our4Darray.Start.clear();

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    rw::common::Timer myTimer;
    dualGraphNode n1, n2, *ptrn1,*ptrn2;
    int x1,x2,y1,y2;
    rw::math::Vector3D<> posr1, posr2;
    std::vector<int> tempVec;
    tempVec.push_back(0);

    n1.parent = -1;
    n2.parent = -1;

    n1.configuration = robotPtr1.robot->getQ(state);
    n2.configuration = robotPtr2.robot->getQ(state);

    curDualGraph.r1Start.push_back(n1);
    curDualGraph.r2Start.push_back(n2);

    posr1 = rw::kinematics::Kinematics::worldTframe(robotPtr1.robotTCP,state).P();
    posr2 = rw::kinematics::Kinematics::worldTframe(robotPtr2.robotTCP,state).P();
    x1 = posr1[0]*10, x2 = posr2[0]*10, y1 =  posr1[1]*10, y2 =  posr2[1]*10;

    Our4Darray.Start.push_back(D4Node{x1,x2,y1,y2,tempVec,1,0});

    n1.configuration = goalConfRobot1;
    n2.configuration = goalConfRobot2;

    curDualGraph.r1Goal.push_back(n1);
    curDualGraph.r2Goal.push_back(n2);

    robotPtr1.robot->setQ(n1.configuration,state);
    robotPtr2.robot->setQ(n2.configuration,state);

    posr1 = rw::kinematics::Kinematics::worldTframe(robotPtr1.robotTCP,state).P();
    posr2 = rw::kinematics::Kinematics::worldTframe(robotPtr2.robotTCP,state).P();
    x1 = posr1[0]*10, x2 = posr2[0]*10, y1 =  posr1[1]*10, y2 =  posr2[1]*10;


    Our4Darray.Goal.push_back(D4Node{x1,x2,y1,y2,tempVec,1,0});

    bool startGraph = false;
    int iterationCounter = 0;
    while(iterationCounter<maximumIterations)
    {
        if (iterationCounter%100 == 0 && iterationCounter != 0)
            cout << "Iteration: " << iterationCounter <<"/" << maximumIterations << " with a joint step size of " << jointStep <<endl;
        iterationCounter++;
        float dMax = jointStep; // Maximum joint step

        // Find which graph to expand
        if (rand()%2)
        {

            int p = randExpand(true);

            ptrn1 = &curDualGraph.r1Start[p];
            ptrn2 = &curDualGraph.r2Start[p];

            n1.parent = p;
            n2.parent = p;
            //cout << "Start Parent: " << p << endl;
            startGraph = true;
        }
        else
        {
            int p = randExpand(false);

            ptrn1 = &curDualGraph.r1Goal[p];
            ptrn2 = &curDualGraph.r2Goal[p];

            n1.parent = p;
            n2.parent = p;
            //cout << "Goal Parent: " << p << endl;
            startGraph = false;
        }

        while(true)
        {
            for(int i = 0; i<6;i++){
                n1.configuration[i] = wrapMinMax(ptrn1->configuration[i]+fRand(jointConstraints[i][0],jointConstraints[i][1])*dMax,jointConstraints[i][0],jointConstraints[i][1]);
                n2.configuration[i] = wrapMinMax(ptrn2->configuration[i]+fRand(jointConstraints[i][0],jointConstraints[i][1])*dMax,jointConstraints[i][0],jointConstraints[i][1]);
            }
            //cout << "Set Q: " << n1.configuration << " "<< n2.configuration<<  endl;
            robotPtr1.robot->setQ(n1.configuration,state);
            robotPtr2.robot->setQ(n2.configuration,state);

            if (!detector->inCollision(state,NULL,true))
            {
                //cout << "Expanion" << endl;
                if (startGraph)
                {
                    posr1 = rw::kinematics::Kinematics::worldTframe(robotPtr1.robotTCP,state).P();
                    posr2 = rw::kinematics::Kinematics::worldTframe(robotPtr2.robotTCP,state).P();
                    //cout << "Pos1: " << posr1 << " pos2: " << posr2 << endl;
                    x1 = posr1[0]*10, x2 = posr2[0]*10, y1 =  posr1[1]*10, y2 =  posr2[1]*10;

                    for (std::vector<D4Node>::iterator it = Our4Darray.Start.begin(); it != Our4Darray.Start.end();++it)
                    {
                        //cout << "x1: " << (*it).x1 << " " << x1 << " x2: " << (*it).x2 << " " << x2 << " y1: " << (*it).y1 << " " << y1 << " y2: " << (*it).y2 << " " << y2 << endl;
                        if ((*it).x1 < x1 || ((*it).y1 < y1 && (*it).x1 == x1) || ((*it).x2 < x2 && (*it).x1 == x1 && (*it).y1 == y1) || ((*it).y2 < y2 && (*it).x1 == x1 && (*it).y1 == y1 && (*it).x2 == x2))
                        {
                            tempVec.clear();
                            tempVec.push_back(curDualGraph.r1Start.size());
                            Our4Darray.Start.insert(it,D4Node{x1,x2,y1,y2,tempVec,1,0});
                            cout << "x1: " << (*it).x1 << " " << x1 << " x2: " << (*it).x2 << " " << x2 << " y1: " << (*it).y1 << " " << y1 << " y2: " << (*it).y2 << " " << y2 << endl;
                            cout << "Insert node with parent "<<  tempVec[0]<< endl;
                            for (std::vector<D4Node>::iterator it2 = Our4Darray.Goal.begin(); it2 != Our4Darray.Goal.end();++it2)
                            {
                                if ((*it).x1 == (*it2).x1 && (*it).y1 == (*it2).y1 && (*it).x2 == (*it2).x2 && (*it).y2 == (*it2).y2)
                                {
                                    //cout << "x1: " << (*it).x1 << " " << (*it2).x1 << " x2: " << (*it).x2 << " " << (*it2).x2 << " y1: " << (*it).y1 << " " << (*it2).y1 << " y2: " << (*it).y2 << " " << (*it2).y2 << endl;
                                    //cout << "Success" << endl;
                                    connectionIdx[0] = (*it).idx[rand()%int((*it).idx.size())];
                                    //cout << "Connection " << connectionIdx[0] << " size: "<< curDualGraph.r1Start.size() << endl;
                                    connectionIdx[1] = (*it2).idx[rand()%int((*it2).idx.size())];
                                    //cout << "Connection " << connectionIdx[1] << " size: "<< curDualGraph.r1Goal.size() << endl;
                                    curDualGraph.r1Start.push_back(n1);
                                    curDualGraph.r2Start.push_back(n2);
                                    return;
                                }
                            }
                            break;
                        }
                        else if ((*it).x1 == x1 && (*it).y1 == y1 && (*it).x2 == x2 && (*it).y2 == y2)
                        {
                            (*it).idx.push_back(curDualGraph.r1Start.size());
                            (*it).probability = 1/(*it).idx.size();
                            //cout << "Insert element" << endl;
                            break;
                        }
                        else if ((*it).x1 > x1 || ((*it).y1 > y1 && (*it).x1 == x1) || ((*it).x2 > x2 && (*it).x1 == x1 && (*it).y1 == y1) || ((*it).y2 > y2 && (*it).x1 == x1 && (*it).y1 == y1 && (*it).x2 == x2))
                        {
                            tempVec.clear();
                            tempVec.push_back(curDualGraph.r1Start.size());
                            Our4Darray.Start.insert(it,D4Node{x1,x2,y1,y2,tempVec,1,0});
                            cout << "x1: " << (*it).x1 << " " << x1 << " x2: " << (*it).x2 << " " << x2 << " y1: " << (*it).y1 << " " << y1 << " y2: " << (*it).y2 << " " << y2 << endl;
                            cout << "Insert node with parent "<<  tempVec[0]<< endl;
                            for (std::vector<D4Node>::iterator it2 = Our4Darray.Goal.begin(); it2 != Our4Darray.Goal.end();++it2)
                            {
                                if ((*it).x1 == (*it2).x1 && (*it).y1 == (*it2).y1 && (*it).x2 == (*it2).x2 && (*it).y2 == (*it2).y2)
                                {
                                    //cout << "x1: " << (*it).x1 << " " << (*it2).x1 << " x2: " << (*it).x2 << " " << (*it2).x2 << " y1: " << (*it).y1 << " " << (*it2).y1 << " y2: " << (*it).y2 << " " << (*it2).y2 << endl;
                                    //cout << "Success" << endl;
                                    connectionIdx[0] = (*it).idx[rand()%int((*it).idx.size())];
                                    //cout << "Connection " << connectionIdx[0] << " size: "<< curDualGraph.r1Start.size() << endl;
                                    connectionIdx[1] = (*it2).idx[rand()%(*it2).idx.size()];
                                    //cout << "Connection " << connectionIdx[1] << " size: "<< curDualGraph.r1Goal.size() << endl;
                                    curDualGraph.r1Start.push_back(n1);
                                    curDualGraph.r2Start.push_back(n2);
                                    return;
                                }
                            }
                            break;
                        }
                    }

                    curDualGraph.r1Start.push_back(n1);
                    curDualGraph.r2Start.push_back(n2);

                }
                else
                {
                    posr1 = rw::kinematics::Kinematics::worldTframe(robotPtr1.robotTCP,state).P();
                    posr2 = rw::kinematics::Kinematics::worldTframe(robotPtr2.robotTCP,state).P();
                    x1 = posr1[0]*10, x2 = posr2[0]*10, y1 =  posr1[1]*10, y2 =  posr2[1]*10;
                    //cout << "Pos1: " << posr1 << " pos2: " << posr2 << endl;
                    for (std::vector<D4Node>::iterator it = Our4Darray.Goal.begin(); it != Our4Darray.Goal.end();++it)
                    {
                        //cout << "x1: " << (*it).x1 << " " << x1 << " x2: " << (*it).x2 << " " << x2 << " y1: " << (*it).y1 << " " << y1 << " y2: " << (*it).y2 << " " << y2 << endl;
                        if (((*it).x1 < x1) || ((*it).y1 < y1 && (*it).x1 == x1) || ((*it).x2 < x2 && (*it).x1 == x1 && (*it).y1 == y1) || ((*it).y2 < y2 && (*it).x1 == x1 && (*it).y1 == y1 && (*it).x2 == x2))
                        {
                            tempVec.clear();
                            tempVec.push_back(curDualGraph.r1Goal.size());
                            Our4Darray.Goal.insert(it,D4Node{x1,x2,y1,y2,tempVec,1,0});
                            cout << "x1: " << (*it).x1 << " " << x1 << " x2: " << (*it).x2 << " " << x2 << " y1: " << (*it).y1 << " " << y1 << " y2: " << (*it).y2 << " " << y2 << endl;
                            cout << "Insert node with parent "<<  tempVec[0]<< endl;
                            for (std::vector<D4Node>::iterator it2 = Our4Darray.Start.begin(); it2 != Our4Darray.Start.end();++it2)
                            {
                                if ((*it).x1 == (*it2).x1 && (*it).y1 == (*it2).y1 && (*it).x2 == (*it2).x2 && (*it).y2 == (*it2).y2)
                                {
                                    //cout << "x1: " << (*it).x1 << " " << (*it2).x1 << " x2: " << (*it).x2 << " " << (*it2).x2 << " y1: " << (*it).y1 << " " << (*it2).y1 << " y2: " << (*it).y2 << " " << (*it2).y2 << endl;
                                    //cout << "Success" << endl;
                                    connectionIdx[1] = (*it).idx[rand()%(*it).idx.size()];
                                    //cout << "Connection " << connectionIdx[1] << " size: "<< curDualGraph.r1Goal.size() << endl;
                                    connectionIdx[0] = (*it2).idx[rand()%(*it2).idx.size()];
                                    //cout << "Connection " << connectionIdx[0] << " size: "<< curDualGraph.r1Start.size() << endl;
                                    curDualGraph.r1Goal.push_back(n1);
                                    curDualGraph.r2Goal.push_back(n2);
                                    return;
                                }
                            }
                            break;
                        }
                        else if ((*it).x1 == x1 && (*it).y1 == y1 && (*it).x2 == x2 && (*it).y2 == y2)
                        {
                            (*it).idx.push_back(curDualGraph.r1Goal.size());
                            (*it).probability = 1/(*it).idx.size();
                            //cout << "Insert element" << endl;
                            break;
                        }
                        else if (((*it).x1 > x1) || ((*it).y1 > y1 && (*it).x1 == x1) || ((*it).x2 > x2 && (*it).x1 == x1 && (*it).y1 == y1) || ((*it).y2 > y2 && (*it).x1 == x1 && (*it).y1 == y1 && (*it).x2 == x2))
                        {
                            tempVec.clear();
                            tempVec.push_back(curDualGraph.r1Goal.size());
                            Our4Darray.Goal.insert(it,D4Node{x1,x2,y1,y2,tempVec,1,0});
                            cout << "x1: " << (*it).x1 << " " << x1 << " x2: " << (*it).x2 << " " << x2 << " y1: " << (*it).y1 << " " << y1 << " y2: " << (*it).y2 << " " << y2 << endl;
                            cout << "Insert node with parent "<<  tempVec[0]<< endl;
                            for (std::vector<D4Node>::iterator it2 = Our4Darray.Start.begin(); it2 != Our4Darray.Start.end();++it2)
                            {
                                if ((*it).x1 == (*it2).x1 && (*it).y1 == (*it2).y1 && (*it).x2 == (*it2).x2 && (*it).y2 == (*it2).y2)
                                {
                                    //cout << "x1: " << (*it).x1 << " " << (*it2).x1 << " x2: " << (*it).x2 << " " << (*it2).x2 << " y1: " << (*it).y1 << " " << (*it2).y1 << " y2: " << (*it).y2 << " " << (*it2).y2 << endl;
                                    //cout << "Success" << endl;
                                    connectionIdx[1] = (*it).idx[rand()%(*it).idx.size()];
                                    //cout << "Connection " << connectionIdx[1] << " size: "<< curDualGraph.r1Goal.size() << endl;
                                    connectionIdx[0] = (*it2).idx[rand()%(*it2).idx.size()];
                                    //cout << "Connection " << connectionIdx[0] << " size: "<< curDualGraph.r1Start.size() << endl;
                                    curDualGraph.r1Goal.push_back(n1);
                                    curDualGraph.r2Goal.push_back(n2);
                                    return;
                                }
                            }
                            break;
                        }

                    }

                    curDualGraph.r1Goal.push_back(n1);
                    curDualGraph.r2Goal.push_back(n2);
                }
//                cout << "Size start: " << curDualGraph.r1Start.size() << endl;
//                cout << "Size goal: " << curDualGraph.r1Goal.size() << endl;
                break;
            }
            dMax /= 2;
        }


    }
    cout << "I failed you master :("<< endl;
}

void SamplePlugin::DualPath()
{
//    cout << "Print dual graph" << endl;
//    for (int i = 0;i<curDualGraph.r1Goal.size();i++)
//        cout << "Idx: "<< i << " Parent: "<< curDualGraph.r1Goal[i].parent <<" Config:" <<curDualGraph.r1Goal[i].configuration << endl;
//    for (int i = 0;i<curDualGraph.r2Goal.size();i++)
//        cout << "Idx: "<< i << " Parent: "<< curDualGraph.r2Goal[i].parent <<" Config:" <<curDualGraph.r2Goal[i].configuration << endl;
//    for (int i = 0;i<curDualGraph.r1Start.size();i++)
//        cout << "Idx: "<< i << " Parent: "<< curDualGraph.r1Start[i].parent <<" Config:" <<curDualGraph.r1Start[i].configuration << endl;
//    for (int i = 0;i<curDualGraph.r2Start.size();i++)
//        cout << "Idx: "<< i << " Parent: "<< curDualGraph.r2Start[i].parent <<" Config:" <<curDualGraph.r2Start[i].configuration << endl;

    std::vector<rw::math::Q> r1Path,r2Path;
    int index = connectionIdx[1];

    while(index != -1)
    {
        cout << "Add index to path: " << index << endl;
        r1Path.push_back(curDualGraph.r1Goal[index].configuration);
        r2Path.push_back(curDualGraph.r2Goal[index].configuration);
        cout << "Q: " << curDualGraph.r1Goal[index].configuration << " " << curDualGraph.r2Goal[index].configuration << endl;
        index = curDualGraph.r2Goal[index].parent;

//        cout << "Parent: " << index << endl;
    }

    index = connectionIdx[0];

    while(index != -1)
    {
        cout << "Add index to path: " << index << endl;
        r1Path.insert(r1Path.begin(),curDualGraph.r1Start[index].configuration);
        r2Path.insert(r2Path.begin(),curDualGraph.r2Start[index].configuration);
        cout << "Q: " << curDualGraph.r1Start[index].configuration << " " << curDualGraph.r2Start[index].configuration << endl;
        index = curDualGraph.r2Start[index].parent;

    }

    _path1 = r1Path;
    _path2 = r2Path;
}

int SamplePlugin::randExpand(bool start)
{

    if (start)
    {
        float accu = 0;
        for(int i = 0;i<Our4Darray.Start.size();i++)
        {
            Our4Darray.Start[i].accumulatedProbability = accu;
            accu += Our4Darray.Start[i].probability;
        }
        float r_float = static_cast <float> (rand())/ (static_cast <float> (RAND_MAX/accu));
        int idx = 0;
        //cout << "accu: " << accu << endl;
        //cout << "r_float: " << r_float << endl;
        for(int i = 0;i<Our4Darray.Start.size();i++)
        {
            //cout << "Accu: " << Our4Darray.Start[i].accumulatedProbability << endl;
            if(Our4Darray.Start[i].accumulatedProbability>r_float)
            {
                idx = i;
                break;
            }
        }

        //cout << "size " << Our4Darray.Start[idx].idx.size() << endl;

        return Our4Darray.Start[idx].idx[rand()%Our4Darray.Start[idx].idx.size()];
    }
    else
    {
        float accu = 0;
        for(int i = 0;i<Our4Darray.Goal.size();i++)
        {
            Our4Darray.Goal[i].accumulatedProbability = accu;
            accu += Our4Darray.Goal[i].probability;
        }
        float r_float = static_cast <float> (rand())/ (static_cast <float> (RAND_MAX/accu));
        //cout << "accu: " << accu << endl;
        //cout << "r_float: " << r_float << endl;
        int idx = 0;

        for(int i = 0;i<Our4Darray.Goal.size();i++)
        {
            //cout << "Accu: " << Our4Darray.Goal[i].accumulatedProbability << endl;
            if(Our4Darray.Goal[i].accumulatedProbability>r_float)
            {
                idx = i;
                break;
            }
        }

        //cout << "size " << Our4Darray.Goal[idx].idx.size() << endl;

        return Our4Darray.Goal[idx].idx[rand()%Our4Darray.Goal[idx].idx.size()];
    }
}

void SamplePlugin::saveTree(int robotNum)
{
    graph* ptrGraph;
    if(robotNum == 1)
        ptrGraph = &robot1Graph;
    else
        ptrGraph = &robot2Graph;

    std::ofstream xyzTree_file(std::to_string((robotNum))+"xyzTree.txt"); //Save the positions of all the nodes in the tree
    std::ofstream pathTree_file(std::to_string((robotNum))+"pathTree.txt"); //Save the positions of all the nodes in the tree
    for(int i = 0; i<ptrGraph->nodeVec.size();i++) {
        for(int j=0; j<3;j++){
            xyzTree_file << ptrGraph->nodeVec[i].postion[j] << " ";
        }
        xyzTree_file << '\n';
    }
    xyzTree_file.close();
    for(int i = 0;i<saveGraphIdx.size();i++)
    {
        for(int j=0; j<3;j++)
            pathTree_file << ptrGraph->nodeVec[saveGraphIdx[i]].postion[j] << " ";
        pathTree_file << "\n";
    }
    pathTree_file.close();
}

QPath SamplePlugin::move(rw::math::Q From, rw::math::Q To, SerialDevice::Ptr robot, State state){
    const CollisionStrategy::Ptr cdstrategy =
            rwlibs::proximitystrategies::ProximityStrategyFactory::makeCollisionStrategy("PQP");
    if (cdstrategy.isNull())
        RW_THROW("PQP Collision Strategy could not be found.");
    const CollisionDetector::Ptr collisionDetector =
            ownedPtr(new CollisionDetector(_wc, cdstrategy));
    const rw::pathplanning::PlannerConstraint con =
            rw::pathplanning::PlannerConstraint::make(collisionDetector, robot, state);
    double eps = 0.01;
    const rw::pathplanning::QSampler::Ptr Qsampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(robot),con.getQConstraintPtr());
    const rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    const rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(con, Qsampler, metric, eps, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    rw::trajectory::QPath result;
    planner->query(From, To, result);

    rw::math::Q currentQ = result[1];
    rw::math::Q formerQ = result[0];
    rw::math::Q tempQ = formerQ-currentQ;
    double deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
    deltaD = sqrt(deltaD);
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
    rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr currentLinearIntPol
            = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));

    for (int i = 0;i<result.size()-1;i += 1)
    {
        currentQ = result[i+1];
        formerQ = result[i];
        tempQ = formerQ-currentQ;
        deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
        deltaD = sqrt(deltaD);
        currentLinearIntPol = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(formerQ,currentQ,deltaD));
        traj.add(currentLinearIntPol);

    }

    rw::trajectory::QPath tempQPath;
    for (double s = 0.0;s<traj.duration();s += 0.05)
        tempQPath.push_back(traj.x(s));

    return tempQPath;
}

void SamplePlugin::setupRobotPtrs()
{
    robotPtr1.robotTCP = _wc->findFrame("1_UR-6-85-5-A.TCP");
    robotPtr1.table = _wc->findFrame("Table1");
    robotPtr1.robot = _wc->findDevice<rw::models::SerialDevice>("1_UR-6-85-5-A");
    robotPtr1.BottleGrip = _wc->findFrame<rw::kinematics::MovableFrame>("BottleGrip1");
    robotPtr1.gripperDevice = _wc->findDevice("1_WSG50");
    robotPtr1.ptrGraph = &robot1Graph;

    robotPtr2.table = _wc->findFrame("Table2");
    robotPtr2.robotTCP = _wc->findFrame("2_UR-6-85-5-A.TCP");
    robotPtr2.robot = _wc->findDevice<rw::models::SerialDevice>("2_UR-6-85-5-A");
    robotPtr2.BottleGrip = _wc->findFrame<rw::kinematics::MovableFrame>("BottleGrip2");
    robotPtr2.gripperDevice = _wc->findDevice("2_WSG50");
    robotPtr2.ptrGraph = &robot2Graph;
}
