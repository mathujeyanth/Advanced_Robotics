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
//    connect(_doubleSpinBox  ,SIGNAL(valueChanged(double)),  this, SLOT(btnPressed()) );
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
        rw::math::Vector3D<> p1 = rw::math::Vector3D<>(-0.3,-0.5,0.3);
        rw::math::Vector3D<> p2 = rw::math::Vector3D<>(0.3,-0.5,0.3);
        rw::math::Vector3D<> p3 = rw::math::Vector3D<>(0,0.60,0.45);
        rw::geometry::Plane aPlane = rw::geometry::Plane(p1,p2,p3);
        createTree(aPlane,_state,1,1000);
        cout << "Size of tree: "<< robot1Graph.nodeVec.size() << endl;
        connectGraph(_state,1);
        //saveTree(1);
        _timer->stop();
        //active_threads.push_back(std::thread(&SamplePlugin::createTree,this,aPlane,_state,1,10000));
        //active_threads.push_back(std::thread(&SamplePlugin::createTree,this,aPlane,_state,2,10000));

        _attachIdx = -1;;
    }
    else if(obj==_btn_runPath){
        rw::math::Vector3D<> p1 = rw::math::Vector3D<>(-0.3,-0.5,0.3);
        rw::math::Vector3D<> p2 = rw::math::Vector3D<>(0.3,-0.5,0.3);
        rw::math::Vector3D<> p3 = rw::math::Vector3D<>(0,0.60,0.60);
        rw::geometry::Plane aPlane = rw::geometry::Plane(p1,p2,p3);

        int randomValue = rand() % 10000;
        _device1->setQ(robotPtr1.ptrGraph->nodeVec[randomValue].configuration,_state);
        _device2->setQ(robotPtr2.ptrGraph->nodeVec[randomValue].configuration,_state);
        getRobWorkStudio()->setState(_state);

        cout << aPlane.distance(rw::kinematics::Kinematics::frameTframe(robotPtr1.table,robotPtr1.robotTCP,_state).P()) << endl;
        cout << aPlane.distance(rw::kinematics::Kinematics::frameTframe(robotPtr2.table,robotPtr2.robotTCP,_state).P()) << endl;
        //if (!_timer->isActive()){
        //    _timer->start(100); // run 10 Hz
        //    _step = 0;
        //}
        //else
        //    _step = 0;

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
                newNode.postion = rw::kinematics::Kinematics::worldTframe(rPtr.robotTCP,state).P();
                newNode.index = rPtr.ptrGraph -> nodeVec.size();
                rPtr.ptrGraph -> nodeVec.push_back(newNode);
            }

        }
    }
    cout << "Spent " << myTimer.getTimeMs()/1000.0 << "s" << endl;
    saveTree(robotNum);
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
    robotPtr rPtr;
    if (robotNum == 1)
        rPtr = robotPtr1;
    else
        rPtr = robotPtr2;

    float dMax = 0.10;
    int maxConnections = 4;

    for(int i = 0;i<rPtr.ptrGraph->nodeVec.size();i++)
    {
        if (i%10)
            cout << i << endl;
        std::vector<connections> possibleConnections;

        for(int j = 0;i<rPtr.ptrGraph->nodeVec.size();j++)
        {
            if(j == i)
                continue;

            float distance = (rPtr.ptrGraph->nodeVec[i].postion-rPtr.ptrGraph->nodeVec[j].postion).norm2();

            if(distance < dMax)
            {
                connections newC;
                newC.distance = distance;
                rw::math::Q tempQ = rPtr.ptrGraph->nodeVec[i].configuration - rPtr.ptrGraph->nodeVec[j].configuration;
                newC.cost = sqrt(tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5));
                newC.index = j;
                auto it = std::lower_bound(possibleConnections.begin(), possibleConnections.end(),newC,lessThan);
                possibleConnections.insert(it,newC);
            }
        }
        int counter = 0;
        int j = 0;
        while (counter < maxConnections && j < possibleConnections.size())
        {
            int split = possibleConnections[j].cost+1;
            if(canConnect(rPtr.ptrGraph->nodeVec[i].configuration,rPtr.ptrGraph->nodeVec[possibleConnections[j].index].configuration,rPtr,detector,state, split))
            {
                counter++;
                rPtr.ptrGraph->nodeVec[i].connectionVec.push_back(possibleConnections[j]);
            }
            j++;
        }
    }
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

void SamplePlugin::saveTree(int robotNum)
{
    graph* ptrGraph;
    if(robotNum == 1)
        ptrGraph = &robot1Graph;
    else
        ptrGraph = &robot2Graph;
    std::ofstream xyzTree_file(std::to_string((robotNum))+"xyzTree.txt"); //Save the positions of all the nodes in the tree
        for(int i = 0; i<ptrGraph->nodeVec.size();i++) {
            for(int j=0; j<3;j++){
                xyzTree_file << ptrGraph->nodeVec[i].postion[j] << " ";
            }
            xyzTree_file << '\n';
        }
        xyzTree_file.close();
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
