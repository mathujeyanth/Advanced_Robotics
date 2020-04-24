#include "SamplePlugin.hpp"






SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

    // now connect stuff from the ui component
    connect(_btn_im         ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_btn_scan       ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_btn0           ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_btn_runPath    ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_doubleSpinBox  ,SIGNAL(valueChanged(double)),  this, SLOT(btnPressed()) );
    connect(_btnPtPi        ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_btnPtP         ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_placeBottle    ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_home           ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_printTest      ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_initConf      ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );
    connect(_goalConf      ,SIGNAL(pressed()),             this, SLOT(btnPressed()) );

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
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/peter/Git/Advanced_Robotics/Project_WorkCell/Scene.wc.xml");
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
//        float rn_x = -0.35+(rand() % 71)/100.0f;
//        float rn_y = 0.35 +((rand() % 21)/100.0f);
//        std::cout << "x: " << rn_x << " y: " << rn_y << std::endl;
//        _wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->moveTo(
//                    rw::math::Transform3D<>(rw::math::Vector3D<>(rn_x,rn_y,0.21f),
//                                            rw::math::RPY<>(0,0,90*rw::math::Deg2Rad)
//                                            ), _state);
        _wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->moveTo(
                    rw::math::Transform3D<>(rw::math::Vector3D<>(0.04, 0.835, 0.21),
                                            rw::math::RPY<>(-1.571, 0, 1.571)
                                            ), _state);

        getRobWorkStudio()->setState(_state);
    }
    else if (obj == _btnPtP)
    {
        // timing for processing time, geeksforgeeks.org/measure-execution-time-function-cpp
        _timer->stop();
        auto start = std::chrono::high_resolution_clock::now();
        createPtPPath(_wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->getTransform(_state).P());
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << _path.size() << std::endl;
        std::cout << duration.count() / 1000 << " ms" << std::endl;
    }
    else if (obj==_btnPtPi)
    {
        _timer->stop();
        auto start = std::chrono::high_resolution_clock::now();
        createPtPiPath(_wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->getTransform(_state).P());
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        std::cout << _path.size() << std::endl;
        std::cout << duration.count() / 1000 << " ms" << std::endl;
    }
    else if(obj==_btn0){
        _timer->stop();
        rw::math::Math::seed();
        auto start = std::chrono::high_resolution_clock::now();
        createPathRRTConnect(_wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->getTransform(_state).P(),_doubleSpinBox->value());
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        printAblePathSize.push_back(_path.size());
        printAbleDurations.push_back(duration.count());
    }
    else if (obj == _printTest)
    {
//        std::cout << _doubleSpinBox->value() << std::endl;
//        for (int i = 0;i<10;i++)
//        {
//            _timer->stop();
//            rw::math::Math::seed();
//            auto start = std::chrono::high_resolution_clock::now();
//            createPathRRTConnect(_wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->getTransform(_state).P(),_doubleSpinBox->value());
//            auto stop = std::chrono::high_resolution_clock::now();
//            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
//            printAblePathSize.push_back(_path.size());
//            printAbleDurations.push_back(duration.count());
//        }
//        for (int i = 0;i<printAblePathSize.size();i++)
//        {
//            std::cout << printAblePathSize[i] << " ";
//        }
//        std::cout << "\n";
//        for (int i = 0;i<printAbleDurations.size();i++)
//        {
//            std::cout << printAbleDurations[i] << " ";
//        }
//        std::cout << "\n";
//        printAbleDurations.clear();
//        printAblePathSize.clear();
        _timer->stop();
        std::cout << "printTest button" << std::endl;
        double constraints[3][2] = {{0,0},{1,0.928},{0,0}};
        TCMP(constraints);
        _attachIdx = -1;
        std::cout << "printTest button - OVER" << std::endl;
    }
    else if(obj==_btn_runPath){
        if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;

    } else if(obj==_doubleSpinBox){
        log().info() << "spin value:" << _doubleSpinBox->value() << "\n";
    } else if(obj == _initConf){
        rw::math::Q qVal(6,1.992, -2.163, -1.849, 0.896, 1.148, -0.014); // Q[6]{1.992, -2.163, -1.849, 0.896, 1.148, -0.014}
        _device2->setQ(qVal,_state);
        getRobWorkStudio()->setState(_state);
    } else if(obj == _goalConf){
        rw::math::Q qVal(6,1.115, -2.115, -1.309, -2.837, -2.044, 0); // Q[6]{1.115, -2.115, -1.309, -2.837, -2.044, 0}
        _device2->setQ(qVal,_state);
        getRobWorkStudio()->setState(_state);
    }


}


void SamplePlugin::timer() {
    //_wc->findDevice("WSG50")-> setQ(rw::math::Q(1, 0.055),_state);
    if(0 <= _step && _step < _path.size()){
        //std::cout << _path.at(_step)(0) << " " << _path.at(_step)(1) << " " << _path.at(_step)(2) << " " << _path.at(_step)(3) << " " << _path.at(_step)(4) << " " << _path.at(_step)(5) << ";\n";
        _device2->setQ(_path.at(_step),_state);
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
    _path = tempQPath;
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
    _path = tempQPath;
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
    _path = tempQPath;

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


struct element{//Used in TCMP
    int index;
    rw::math::Q Q;
    double error;
    rw::math::Vector3D<double> Pos;
};

bool lessThan(const element a,const element b){
        return (b.error < a.error);
}

void SamplePlugin::TCMP(double constraints[][2]){
    //Find number of constraints
    int numOfConstraints=0;
    for(int i=0; i<3;i++){
        if(constraints[i][0] == 1){
         numOfConstraints++;
        }
    }

    rw::common::Timer testTimer;
    /// Find initial Q pose
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    cout << "detector " << !detector->inCollision(_state,NULL,true) << endl;
    Q HOME(6, 1.571, -1.158, -2.728, 0.771, 1.571, 0);
    _wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->moveTo(
                rw::math::Transform3D<>(rw::math::Vector3D<>(0.04, 0.835, 0.21),
                                        rw::math::RPY<>(-1.571, 0, 1.571)
                                        ), _state);
    double minusOneHalfPI = -M_PI*1.5;
    double plusHalfPI = M_PI*0.5;
    double maxJointStep = 0.1; //Thus, max joint step is pi*maxJointStep. I.e. maxJointStep value is used in generating random dq
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
    rw::math::Vector3D<double> initPos = {0.1, 0.928, 0.247}; // FOR INV KIN: Q[6]{0.1, 0.928, 0.247, -3.142, -0.003, -1.546}
    rw::math::Vector3D<double> initRPY = {-3.142, -0.003, -1.546};

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

    cout << "XYZ " <<  worldTTCP.P() << " and Z " << worldTTCP.P()[2] << endl;

    //rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    testTimer.resetAndResume();
    while(POSerr.norm2() > 0.05 || RPYerr.norm2() > 0.05){ //
        //Add random dq
        for(int i = 0; i<6;i++){
            if(i==1 || i==3){
                qNew[i] = wrapMinMax(qVal[i]+fRand(minusOneHalfPI,plusHalfPI)*maxJointStep,minusOneHalfPI,plusHalfPI);
            }else{
                qNew[i] = wrapMinMax(qVal[i]+fRand(-M_PI,M_PI)*maxJointStep,-M_PI,M_PI);
            }
        }
        //Find new TCP error
        _device2->setQ(qNew,_state);
        worldTTCP = rw::kinematics::Kinematics::worldTframe(robotTCP,_state);
        TCPRPY = rw::math::RPY<double>(worldTTCP.R());
        RPYasVec = {TCPRPY[0],TCPRPY[1],TCPRPY[2]};
        newRPYerr = initRPY - RPYasVec;
        newPOSerr = initPos - worldTTCP.P();


        if(( ((newPOSerr.norm2() < POSerr.norm2() && RPYerr.norm2() < 0.05)) || (newPOSerr.norm2() < POSerr.norm2() && newRPYerr.norm2() < RPYerr.norm2()) ) && !detector->inCollision(_state,NULL,true) && (worldTTCP.P()[2] > 0)  ){
            qVal = qNew;
            RPYerr = newRPYerr;
            POSerr = newPOSerr;
            //cout << "update and Qval " << qVal << endl;
        }

        counter++;
        if(counter  == 20000){
            cout << "Reset robot to home and try again" << endl;
            qVal = HOME;
            _device2->setQ(qVal,_state);
            RPYerr = initRPY - RPYasVec;
            POSerr = initPos - worldTTCP.P();
            counter = 0;
            numOfReset++;

        }
    }

    cout<<"Found initial configuration! CounterVal " << counter+20000*numOfReset << "\t"<< qVal << " Number of resets " << numOfReset << " Took " << testTimer.getTimeMs() << " milliseconds " << endl;



    //############## Add goal position - given in XYZ and RPY
    rw::math::Vector3D<double> goalPos = {-0.36, 0.928, 0.262}; // FOR INV KIN: Q[6]{0.047, 0.725, 0.59, -3.139, 0.006, -1.513}
    rw::math::Vector3D<double> goalRPY = {-3.139, 0.006, 1.551}; //Other side of boxes: Q[6]{-0.36, 0.928, 0.262, 0.017, 0.01, 1.551}
    //Initilize tree
    //std::vector<element> Tree;
    std::vector<element> sortTree;
    std::vector<element> randomTree;
    sortTree.push_back({0,qVal,(worldTTCP.P()-goalPos).norm2(),worldTTCP.P()});
    randomTree.push_back({0,qVal,(worldTTCP.P()-goalPos).norm2(),worldTTCP.P()});
    std::vector<std::array<double,3>> xyzTree;
    xyzTree.push_back({worldTTCP.P()[0],worldTTCP.P()[1],worldTTCP.P()[2]});
    // Parameters
    maxJointStep = 0.1; //Thus, max joint step is pi*maxJointStep. I.e. maxJointStep value is used in generating random dq
    counter = 0; //Reset counter
    int idx;
    rw::math::Q tempQ; //Used to check a new candidate Q that might go into the tree
    bool stopFlag=false;
    int currTreeSize = 1; //The zero'ed position in the Tree is filled with initial configuration
    int constraintsSatisfied = 0;
    testTimer.resetAndResume();
    double smallestErrorAndIdx[2] = {numeric_limits<double>::max(),0};
    rw::common::Timer timer1;
    double sumTime1=0;
    double sumTime2=0;
    while(true){
        timer1.resetAndResume();
        counter++;
        if(fRand(0,1) < 0.2){
            idx = (int)fRand(0,currTreeSize-1);
        }else{
            //idx = Tree.at(currTreeSize-1).index;
            idx = (int)fRand(0,currTreeSize-1);
        }

        //cout <<"idx " <<  idx << endl;

        //tempQ = sortTree[idx].Q;
        tempQ = randomTree[idx].Q;
        //cout << "tempQ " << tempQ << endl;
        //Add random dq
        for(int i = 0; i<6;i++){
            if(i==1 || i==3){
                tempQ[i] = wrapMinMax(tempQ[i]+fRand(minusOneHalfPI,plusHalfPI)*maxJointStep,minusOneHalfPI,plusHalfPI);
            }else{
                tempQ[i] = wrapMinMax(tempQ[i]+fRand(-M_PI,M_PI)*maxJointStep,-M_PI,M_PI);
            }
        }

        //Move robot and update distances
        _device2->setQ(tempQ,_state);
        worldTTCP = rw::kinematics::Kinematics::worldTframe(robotTCP,_state);
        sumTime1 += timer1.getTimeMs();
        timer1.resetAndResume(); //Need to be reset to account correctly for the next if-statement and the 'check of constraints'-part

        if(worldTTCP.P()[2] > 0 ){//Only check tempQ if it is above the table-plane (I.e. z>0)
            //Check constraints are satisfied
            constraintsSatisfied = 0;
            //cout << "check constraints" << endl;
            for(int i=0; i<3;i++){
                if(constraints[i][0] == 1){
                    if( abs(worldTTCP.P()[i]- constraints[i][1]) < 0.1){
                        constraintsSatisfied++;
                    }
                }
            }
            sumTime1 += timer1.getTimeMs();
            timer1.resetAndResume();
            if( (constraintsSatisfied==numOfConstraints) && !detector->inCollision(_state,NULL,true) ){
                //cout << "insert - Press enter" << endl;
                //cin.get();
                element node = {idx,tempQ,(worldTTCP.P()-goalPos).norm2(),worldTTCP.P()};
                //auto it = std::lower_bound(sortTree.begin(), sortTree.end(),node,lessThan); //lessThan is a operator defined as a "function"
                //sortTree.insert(it,{idx,tempQ,(worldTTCP.P()-goalPos).norm2()});
                randomTree.push_back(node);

                //cout << "inserted" << endl;
                currTreeSize++;

                xyzTree.push_back({worldTTCP.P()[0],worldTTCP.P()[1],worldTTCP.P()[2]});
                if((worldTTCP.P()-goalPos).norm2() < smallestErrorAndIdx[0]){
                    smallestErrorAndIdx[0] = (worldTTCP.P()-goalPos).norm2();
                    smallestErrorAndIdx[1] = currTreeSize;
                }
            }
            sumTime2 += timer1.getTimeMs();
        }


        if( (worldTTCP.P()-goalPos).norm2() < 0.05 ){
            break;
        }
        if(counter % 10000 == 0){
            cout << "Counter value " << counter/1000 << " thousand - Current tree size: " << randomTree.size() << endl;
            cout << "Time1 for the last 10.000 iterations: " << sumTime1 << "\t and Time2: " << sumTime2 << endl << endl;
            sumTime1 = 0; sumTime2 = 0;
            //cout << "Last error: " << randomTree[randomTree.size()-1].error << " and Qval: " << randomTree[randomTree.size()-1].Q << endl;
            //cout << "Smallest error seen: " << smallestErrorAndIdx[0] << ", seen at index " << smallestErrorAndIdx[1] << endl;
        }
        if(counter == 100000){ //Break for debug
            cout << "stop at " << counter << endl;
            stopFlag = true;
            break;
        }
    }
    testTimer.pause();



    if(stopFlag == true){
        cout << "Stopped after " << counter << " iterations because of STOPFLAG! - Current tree size: " << randomTree.size() << endl;
        cout << "Smallest error seen: " << smallestErrorAndIdx[0] << ", seen at index " << smallestErrorAndIdx[1] << endl;
    }else{
        cout << "Found goal! In " << counter << " iterations. - Tree length is " << randomTree.size() << " and it took " << testTimer.getTimeMs() <<" milliseconds to find path."<< endl;
    }

    //Create path by backtracking through the tree
    int iterator = randomTree[currTreeSize-1].index;
    std::vector<rw::math::Q> tempPath;
    std::ofstream xyzTreePath_file("xyzTreePath.txt"); //File to write position of end-effector to (Positions which is used for final path)
    tempPath.push_back(randomTree[currTreeSize-1].Q);
    xyzTreePath_file << randomTree[currTreeSize-1].Pos[0] << " "<< randomTree[currTreeSize-1].Pos[1] << " "<< randomTree[currTreeSize-1].Pos[2] << '\n';

    while(true){
        tempPath.push_back(randomTree[iterator].Q);
        for(int j=0; j<3;j++){ //Save the position of the end-effector on the path to a file
            xyzTreePath_file << randomTree[iterator].Pos[j] << " ";
        }
        xyzTreePath_file << '\n';
        if(iterator == 0 ){ //Goal condition - break
            break;
        }
        iterator = randomTree[iterator].index; //Find parent index of current node
    }

    xyzTreePath_file.close();
    _path.clear(); //Clear path - and copy path over in reverse order of that of the tree (Because of backtracking)
    for(int i=tempPath.size()-1; 0 <= i;i--){
        _path.push_back(tempPath[i]);
    }
    cout << "Length of found path " << _path.size() << endl;

    std::ofstream xyzTree_file("xyzTree.txt"); //Save the positions of all the nodes in the tree
    for(int i = 0; i<xyzTree.size();i++) {
        for(int j=0; j<3;j++){
            xyzTree_file << xyzTree[i][j] << " ";
        }
        xyzTree_file << '\n';
    }
    xyzTree_file.close();

    _device2->setQ(randomTree[0].Q,_state);//Reset robot to initial config

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

void SamplePlugin::blendPath(){
    rw::math::Q nextQ = _path[2];
    rw::math::Q currentQ = _path[1];
    rw::math::Q formerQ = _path[0];
    rw::math::Q tempQ = formerQ-currentQ;
    double deltaD = tempQ(0)*tempQ(0)+tempQ(1)*tempQ(1)+tempQ(2)*tempQ(2)+tempQ(3)*tempQ(3)+tempQ(4)*tempQ(4)+tempQ(5)*tempQ(5);
    deltaD = sqrt(deltaD);
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> traj;
    rw::trajectory::LinearInterpolator<rw::math::Q>::Ptr currentLinearIntPol
            = ownedPtr(new rw::trajectory::LinearInterpolator<rw::math::Q>(_path[0],_path[1],deltaD));
    traj.add(currentLinearIntPol);
    for (int i = 1;i<_path.size()-2;i += 1)
    {
        nextQ = _path[i+2];
        currentQ = _path[i+1];
        formerQ = _path[i];
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
    _path = tempQPath;
}
