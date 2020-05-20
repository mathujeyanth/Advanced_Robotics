#include "SamplePlugin.hpp"
#include <iomanip>
#include "KDTree.cpp"
#include <unsupported/Eigen/NonLinearOptimization>
#include <unsupported/Eigen/NumericalDiff>

#include <boost/config.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

template<typename _Scalar, int NX = Eigen::Dynamic, int NY = Eigen::Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Eigen::Matrix<Scalar,InputsAtCompileTime,1> InputType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,1> ValueType;
    typedef Eigen::Matrix<Scalar,ValuesAtCompileTime,InputsAtCompileTime> JacobianType;

    double m_inputs, m_values;
    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(double inputs, double values) : m_inputs(inputs), m_values(values) {}

    double inputs() const { return m_inputs; }
    double values() const { return m_values; }

};


Eigen::Matrix4f SamplePlugin::xyzrpyToTransformMatrix(double tx, double ty, double tz, double rz, double ry, double rx) {
    double cx(std::cos(rx)), cy(std::cos(ry)), cz(std::cos(rz));
    double sx(std::sin(rx)), sy(std::sin(ry)), sz(std::sin(rz));

    Eigen::Matrix4f T;
    T<<cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz,   tx,
       cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz,   ty,
       -sy,            sx*cy,            cx*cy,     tz,
        0,               0,                 0,      1;
    return T;
}
void SamplePlugin::staticxyzrpyToTransformMatrix(Eigen::Matrix4d &pose , double tx, double ty, double tz, double rz, double ry, double rx) {
    double cx(std::cos(rx)), cy(std::cos(ry)), cz(std::cos(rz));
    double sx(std::sin(rx)), sy(std::sin(ry)), sz(std::sin(rz));

    //Eigen::Matrix4d T;
    pose<<cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz,   tx,
       cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz,   ty,
       -sy,            sx*cy,            cx*cy,     tz,
        0,               0,                 0,      1;
    //return pose;
}
ostream& operator<<(ostream& out, const vector<double>& v) { // auxiliary
   for (int i=0; i<v.size(); ++i)
      out << v[i] << " ";
   out << endl;
}

struct my_functor : Functor<double>
{
    my_functor(void): Functor<double>(3,3) {}

    Eigen::Vector3d robot_1_init_location,robot_2_init_location;

    Eigen::Matrix4d grip1_transform; //bottle to gripper 1
    Eigen::Matrix4d grip2_transform; //bottle to gripper 2
    Eigen::Matrix4d grip12_transform;
    Eigen::Vector3d mating_object_pos; //Handover location

    Eigen::Vector3d zi1,zi2; // Initial z vectors of grippers

    //Eigen::Matrix4d grip1_mating_pose,grip2_mating_pose;
    const float pi = 3.141592653589793;
    int operator()(const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const//lets obj act as a function

    {
        fvec(2) = 0;

        Eigen::Matrix4d bottle_pose = get_bottle_pose(x); //with current rpy estimate

        Eigen::Matrix4d grip1_mating_pose = get_gripper1_pose(bottle_pose);
        Eigen::Matrix4d grip2_mating_pose = get_gripper2_pose(bottle_pose);

        //get Z vector of gripper rotations
        Eigen::Vector3d zf1; //final z vec of gripper 1 (when mating)
        zf1<< grip1_mating_pose.coeff(0,2) , grip1_mating_pose.coeff(1,2) , grip1_mating_pose.coeff(2,2);
        Eigen::Vector3d zf2; //final z vec of gripper 2 (when mating)
        zf2<< grip2_mating_pose.coeff(0,2), grip2_mating_pose.coeff(1,2) , grip2_mating_pose.coeff(2,2);

        std::cout<<"zf1 "<<zf1(0)<<" "<<zf1(1)<<" "<<zf1(2)<<" ; zf2 "<<zf2(0)<<" "<<zf2(1)<<" "<<zf2(2)<<"\n";

        // angles between the two initial and final gripper Z's that must be minimized
        double d1 = zi1.dot(zf1)/(zi1.norm()*zf1.norm());
        double d2 = zi2.dot(zf2)/(zi2.norm()*zf2.norm());

        Eigen::Affine3d grip_1_tx_affine;grip_1_tx_affine.matrix()=grip1_mating_pose;
        Eigen::Affine3d grip_2_tx_affine;grip_2_tx_affine.matrix()=grip2_mating_pose;

        fvec(2) = ((grip_1_tx_affine.translation()-robot_1_init_location).norm()+(grip_1_tx_affine.translation()-robot_1_init_location).norm());
        //fvec(3) = (grip_1_tx_affine.translation()-robot_1_init_location).norm();

        fvec(0)=acos(d1);//(d1<0.)? -d1:d1; //sqrt(d1*d1+d2*d2)/10.;//(d1+d2)*100/(d1*d2);
        fvec(1) =acos(d2);//(d2<0.)? -d2:d2;
        std::cout<<"x "<<x(0)<<" "<<x(1)<<" "<<x(2)<<" ; d1 "<<d1<<" d2 "<<d2<<" ; acosd1 "<<std::acos(d1)*180./pi<<" acosd2 "<<std::acos(d2)*180./pi<<"; fvec "<<fvec(0)<<" "<<fvec(1)<<" "<<fvec(2)<<"\n";
        return 0;
    }
    Eigen::Matrix4d get_bottle_pose(const Eigen::VectorXd &x)const {
        Eigen::Matrix4d bottle_pose;
        double cx(std::cos(x(2)*pi/180.)), cy(std::cos(x(1)*pi/180.)), cz(std::cos(x(0)*pi/180.));//std::cos(x(2))
        double sx(std::sin(x(2)*pi/180.)), sy(std::sin(x(1)*pi/180.)), sz(std::sin(x(0)*pi/180.));
        bottle_pose<<cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz,   mating_object_pos(0),
           cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz,   mating_object_pos(1),
           -sy,            sx*cy,            cx*cy,     mating_object_pos(2),
            0,               0,                 0,      1;
        return bottle_pose;
    }
    Eigen::Matrix4d get_gripper1_pose(const Eigen::Matrix4d bottle_pose)const {

        Eigen::Vector3d proposed_bottle_z;
        proposed_bottle_z<< bottle_pose.coeff(0, 2),bottle_pose.coeff(1, 2),bottle_pose.coeff(2, 2);


        return bottle_pose*(grip1_transform.inverse());
    }
    Eigen::Matrix4d get_gripper2_pose(const Eigen::Matrix4d bottle_pose)const {
        return bottle_pose*(grip2_transform.inverse());
    }

};
SamplePlugin::SamplePlugin():
    RobWorkStudioPlugin("SamplePluginUI", QIcon(":/pa_icon.png"))
{
    setupUi(this);

    _timer = new QTimer(this);
    connect(_timer, SIGNAL(timeout()), this, SLOT(timer()));

    // now connect stuff from the ui component
    connect(_btn_im    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_scan    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn0    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn1    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_doubleSpinBox  ,SIGNAL(valueChanged(double)), this, SLOT(btnPressed()) );
    connect(_btnPtPi  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btnPtP  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_placeBottle  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_home  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_printTest  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

    connect(grip_1_pose_init_button  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(grip_2_pose_init_button  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(grip_1_pose_final_button  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(grip_2_pose_final_button  ,SIGNAL(pressed()), this, SLOT(btnPressed()) );

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
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/downloads/SEM 2/Advanced_Robotics/Project_WorkCell/Scene.wc.xml");
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


        _device1 = _wc->findDevice<SerialDevice>("1_UR-6-85-5-A");
        _device2 = _wc->findDevice<SerialDevice>("2_UR-6-85-5-A");
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
        Q to(6, 1.571, -1.572, -1.572, -1.572, 1.571, 0); //From pose estimation
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
        std::cout << "printTest button" << std::endl;
        //findBestHandoverOrientation();

        rw::math::Vector3D<> p1 = rw::math::Vector3D<>(-0.3,-0.5,0.21);
       rw::math::Vector3D<> p2 = rw::math::Vector3D<>(0.3,-0.5,0.21);
       rw::math::Vector3D<> p3 = rw::math::Vector3D<>(0,0.60,0.50);
       rw::geometry::Plane aPlane = rw::geometry::Plane(p1,p2,p3);
       createTree(aPlane,_state,1,500);
       cout << "Size of tree: "<< robot1Graph.nodeVec.size() << endl;
        saveTree(1);
        std::cout << "printTest button - OVER" << std::endl;
    }
    else if(obj==_btn1){
        if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;

    }else if(obj==grip_1_pose_init_button){
        std::cout<<"gripper 1 init pos\n";
        _device1->setQ(robot_1_initial_config, _state);
        getRobWorkStudio()->setState(_state);

    }else if(obj==grip_2_pose_init_button){
        std::cout<<"gripper 2 init pos\n";
        _device1->setQ(robot_2_initial_config, _state);
        getRobWorkStudio()->setState(_state);

    }else if(obj==grip_1_pose_final_button){
        std::cout<<"gripper 1 final pos\n";
        _device1->setQ(robot_1_final_config, _state);
        getRobWorkStudio()->setState(_state);

    }else if(obj==grip_2_pose_final_button){
        std::cout<<"gripper 2 final pos\n";
        _device2->setQ(robot_2_final_config, _state);
        getRobWorkStudio()->setState(_state);

    }
    else if(obj==_doubleSpinBox){
        log().info() << "spin value:" << _doubleSpinBox->value() << "\n";
    }


}


void SamplePlugin::timer() {
    _wc->findDevice("WSG50")-> setQ(rw::math::Q(1, 0.055),_state);
    if(0 <= _step && _step < _path.size()){
        if (_attachIdx == _step)
        {
            _device1 -> setQ(_attachQ,_state);
            rw::kinematics::Kinematics::gripFrame(_wc->findFrame("Bottle"),_wc->findFrame("GraspTCP"),_state);
            getRobWorkStudio()->setState(_state);
            _step++;
        }else
        {
            std::cout << _path.at(_step)(0) << " " << _path.at(_step)(1) << " " << _path.at(_step)(2) << " " << _path.at(_step)(3) << " " << _path.at(_step)(4) << " " << _path.at(_step)(5) << ";\n";
            _device1->setQ(_path.at(_step),_state);
            getRobWorkStudio()->setState(_state);
            _step++;
        }

        if (_step == _path.size())
        {
            _device1 -> setQ(_deattachQ,_state);
            rw::kinematics::Kinematics::gripFrame(_wc->findFrame("Bottle"),_wc->findFrame("Table"),_state);
            getRobWorkStudio()->setState(_state);
        }
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

struct element{int index;rw::math::Q Q1; rw::math::Vector3D<double> Pos; rw::math::Vector3D<double> RPY;}; //Used in TCMP
struct PRMNode {
  int id;
  int parentId;
  float distanceFromTemp;
  float queryPathDistance;
  rw::math::Vector3D<double> Pos;
  rw::math::Vector3D<double> Ori;
  std::vector<PRMNode*> children_;
  std::vector<float> distances_;
};
struct PRMEdge {
  float distance;
  int ids[2];
};
//void minimum(struct Node *ptr, int n, int &min, int &min_diff)
//{
//    if (ptr == NULL)
//        return ;

//    // If n itself is present
//    if (ptr->data == n)
//    {
//        min_diff= n;
//        return;
//    }

//    // update min and min_diff by checking current node value
//    if (min > abs(ptr->data - n))
//    {
//        min = abs(ptr->data - n);
//        min_diff = ptr->data;
//    }

//    // if n is less than ptr->data then move in left subtree else in right subtree
//    if (n < ptr->data)
//        minimum(ptr->left, n, min, min_diff);
//    else
//        minimum(ptr->right, n, min, min_diff);
//}

Eigen::Matrix3f SamplePlugin::findBestHandoverOrientation()
{

    robot_1_initial_config = _device1->getQ(_state);
    robot_2_initial_config = _device2->getQ(_state);

    rw::kinematics::Frame* robot1TCP = _wc->findFrame("1_UR-6-85-5-A.TCP");
    rw::kinematics::Frame* robot1base = _wc->findFrame("1_UR-6-85-5-A.Base");
    rw::kinematics::Frame* robot2TCP = _wc->findFrame("2_UR-6-85-5-A.TCP");
    rw::kinematics::Frame* robot2base = _wc->findFrame("2_UR-6-85-5-A.Base");
    Eigen::Matrix3f final_R;

    float pi = 3.141592653589793;

    Eigen::VectorXd x(3);

   // x(3) = 0.0;//z
    rw::math::RPY<double> bottle_rpy(_wc->findFrame("Bottle")->getTransform(_state).R());
    x(0) = bottle_rpy(0);//z
    x(1) = bottle_rpy(1);//y
    x(2) = bottle_rpy(2);//x
    std::cout << "x: " << x << std::endl;

    my_functor functor;



    rw::kinematics::Frame* robotTCP1 = _wc->findFrame("GraspTCP1");
    rw::math::Transform3D<double> worldTTCP1 = rw::kinematics::Kinematics::worldTframe(robotTCP1,_state);
    cout << "Postion of end1 " << worldTTCP1.P() << " and rotation in RPY" << rw::math::RPY<double>(worldTTCP1.R()) << std::endl;
    cout << "rotation1\n" << worldTTCP1.R().e() << std::endl;

    rw::kinematics::Frame* robotTCP2 = _wc->findFrame("GraspTCP2");
    rw::math::Transform3D<double> worldTTCP2 = rw::kinematics::Kinematics::worldTframe(robotTCP2,_state);
    cout << "Postion of end2 " << worldTTCP2.P() << " and rotation in RPY" << rw::math::RPY<double>(worldTTCP2.R()) << std::endl;
    cout << "rotation2\n" << worldTTCP2.R().e() << std::endl;

    functor.robot_1_init_location= worldTTCP1.P().e().cast <double>();
    functor.robot_2_init_location= worldTTCP2.P().e().cast <double>();
    functor.grip12_transform = xyzrpyToTransformMatrix(0.0, 0.0, 0.01, 0.,0.,pi).cast <double>();
    functor.grip1_transform=xyzrpyToTransformMatrix(0.0, 0.0, 0.01, 0.,0.,pi/2.).cast <double>();
    functor.grip2_transform=xyzrpyToTransformMatrix(0.0, 0.0, 0.01, 0.,0.,-pi/2.).cast <double>();
    functor.mating_object_pos= _wc->findFrame("Bottle")->getTransform(_state).P().e().cast <double>();

    Eigen::Matrix4d dummy_pose1 = xyzrpyToTransformMatrix(0.,0.,0.0, 0., 0, pi/4).cast <double>();
    Eigen::Affine3d d1;
    d1.matrix() = dummy_pose1;

    functor.zi1 << worldTTCP1.R().e()(0,2),worldTTCP1.R().e()(1,2),worldTTCP1.R().e()(2,2);
    functor.zi2 << worldTTCP2.R().e()(0,2),worldTTCP2.R().e()(1,2),worldTTCP2.R().e()(2,2);

    std::cout<<"zi1 "<<functor.zi1(0)<<" "<<functor.zi1(1)<<" "<<functor.zi1(2)<<"\n";
    std::cout<<"zi2 "<<functor.zi2(0)<<" "<<functor.zi2(1)<<" "<<functor.zi2(2)<<"\n";

    Eigen::NumericalDiff<my_functor> numDiff(functor);
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<my_functor>,double> lm(numDiff);
    lm.parameters.maxfev = 1000;
    lm.parameters.xtol = 1.0e-10;
    std::cout <<" lm.parameters.maxfev "<< lm.parameters.maxfev << std::endl;

    int ret = lm.minimize(x);
    std::cout <<"lm.iter "<< lm.iter << std::endl;
    std::cout << "ret "<<ret << std::endl;
    std::cout <<"lm.nfev "<< lm.nfev << std::endl;
    //std::cout <<"lm.JacobianType "<< lm.JacobianType << std::endl;
    std::cout <<"lm.fjac "<< lm.fjac << std::endl;


    rw::kinematics::MovableFrame::Ptr Bottle = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    //Eigen::Matrix4d bottle_pose = xyzrpyToTransformMatrix(0.0, 0.01, 0.0, 0.,0.,pi).cast <double>();//functor.get_bottle_pose(x);


    Bottle->moveTo(
                rw::math::Transform3D<>(rw::math::Vector3D<>(functor.mating_object_pos),
                                        rw::math::RPY<>(x(0)*pi/180.,x(1)*pi/180.,x(2)*pi/180.)
                                        ), _state);
    getRobWorkStudio()->setState(_state);

    Eigen::Matrix4d bottle_pose = Bottle->getTransform(_state).e().cast <double>();
    Eigen::Matrix4d grip1_mating_pose = functor.get_gripper1_pose(bottle_pose);//
    Eigen::Matrix4d grip2_mating_pose = functor.get_gripper2_pose(bottle_pose);
    std::cout << "RPY that minimizes the differences: " << x << std::endl;
   std::cout <<"Bottle pose\n"<< bottle_pose<<"\n";
    std::cout <<"Gripper 1 pose\n"<< grip1_mating_pose<<"\n";
    std::cout <<"Gripper 2 pose\n"<< grip2_mating_pose<<"\n";
    std::cout<<"g1 transform \n"<<functor.grip1_transform<<"\n";
    std::cout<<"g2 transform \n"<<functor.grip2_transform<<"\n";
    Eigen::Affine3d grip1_mating_pose_affine,grip2_mating_pose_affine;




    Transform3D<> r1gripToTCP = Kinematics::frameTframe(robotTCP1, robot1TCP, _state);

    std::cout<<"r1gripToTCP "<<r1gripToTCP<<"\n";
    Transform3D<> r1WorldToBase = rw::kinematics::Kinematics::worldTframe(robot1base,_state);
    std::cout<<"r1WorldToBase inv "<<r1WorldToBase<<"\n";
    grip1_mating_pose_affine.matrix() = (r1WorldToBase.e().inverse()*(grip1_mating_pose*r1gripToTCP.e()));
    std::cout<<"grip1_mating_pose_affine\n"<<grip1_mating_pose_affine.translation()<<" rot "<<rw::math::RPY<double>(rw::math::Rotation3D<double>(grip1_mating_pose_affine.rotation()))<<"\n";

    Transform3D<> r2gripToTCP = Kinematics::frameTframe(robotTCP2, robot2TCP, _state);
    std::cout<<"r2gripToTCP "<<r2gripToTCP<<"\n";
    Transform3D<> r2WorldToBase = rw::kinematics::Kinematics::worldTframe(robot2base,_state);
    std::cout<<"r2WorldToBase inv "<<r2WorldToBase<<"\n";
    grip2_mating_pose_affine.matrix() = (r2WorldToBase.e().inverse()*(grip2_mating_pose*r2gripToTCP.e()));
    std::cout<<"grip1_mating_pose_affine\n"<<grip2_mating_pose_affine.translation()<<" rot "<<rw::math::RPY<double>(rw::math::Rotation3D<double>(grip2_mating_pose_affine.rotation()))<<"\n";

    const rw::invkin::JacobianIKSolver solver(_device1, _state);
    const Transform3D<> Tdesired(rw::math::Vector3D<>(0.1,0.20,0.44),//grip1_mating_pose_affine.translation()
                                 rw::math::Rotation3D<>(grip1_mating_pose_affine.rotation()));
    const std::vector<Q> solutions = solver.solve(Tdesired, _state);
    std::cout << "Found " << solutions.size() << "robot 1 IK solutions." << std::endl;

    const rw::invkin::JacobianIKSolver solver2(_device2, _state);
    const Transform3D<> Tdesired2(rw::math::Vector3D<>(0.1,0.20,0.44),//grip2_mating_pose_affine.translation()
                                 rw::math::Rotation3D<>(grip2_mating_pose_affine.rotation()));
    const std::vector<Q> solutions2 = solver.solve(Tdesired2, _state);
    std::cout << "Found " << solutions2.size() << "robot 2 IK solutions." << std::endl;

    if (solutions.size()>0){
        _device1->setQ(solutions[0],_state);
        robot_1_final_config = solutions[0];
        std::cout<<"r1 : "<<solutions[0]<<"\n";
    }
    if (solutions2.size()>0){
        _device2->setQ(solutions2[0],_state);
        robot_2_final_config = solutions2[0];
        std::cout<<"r2 : "<<solutions2[0]<<"\n";

    }
    getRobWorkStudio()->setState(_state);


    dummy_pose1 = xyzrpyToTransformMatrix(0.,0.,0.0, x(0), x(1), x(2)).cast <double>();
    d1.matrix() = dummy_pose1;
    std::cout<<"obj Z axis " << d1.rotation().coeff(0,2)<<" "<< d1.rotation().coeff(1,2)<<" "<< d1.rotation().coeff(2,2)<<"\n";

    dummy_pose1 = xyzrpyToTransformMatrix(0.,0.,0.0, x(2), x(1), x(0)).cast <double>();
    d1.matrix() = dummy_pose1;
    std::cout<<"obj Z axis " << d1.rotation().coeff(0,2)<<" "<< d1.rotation().coeff(1,2)<<" "<< d1.rotation().coeff(2,2)<<"\n";

    dummy_pose1 = xyzrpyToTransformMatrix(0.,0.,0.0, x(1), x(0), x(2)).cast <double>();
    d1.matrix() = dummy_pose1;
    std::cout<<"obj Z axis " << d1.rotation().coeff(0,2)<<" "<< d1.rotation().coeff(1,2)<<" "<< d1.rotation().coeff(2,2)<<"\n";

    dummy_pose1 = xyzrpyToTransformMatrix(0.,0.,0.0, x(2), x(0), x(1)).cast <double>();
    d1.matrix() = dummy_pose1;
    std::cout<<"obj Z axis " << d1.rotation().coeff(0,2)<<" "<< d1.rotation().coeff(1,2)<<" "<< d1.rotation().coeff(2,2)<<"\n";
    return final_R;

}

void SamplePlugin::saveTree(int robotNum)
{
    graph* ptrGraph;
    if(robotNum == 1)
        ptrGraph = &robot1Graph;
    else
        ptrGraph = &robot2Graph;
    std::ofstream xyzTree_file("xyzTree.txt"); //Save the positions of all the nodes in the tree
        for(int i = 0; i<ptrGraph->nodeVec.size();i++) {
            for(int j=0; j<3;j++){
                xyzTree_file << ptrGraph->nodeVec[i].postion[j] << " ";
            }
            xyzTree_file << '\n';
        }
        xyzTree_file.close();
}
void SamplePlugin::createTree(rw::geometry::Plane aPlane,rw::kinematics::State state, int robotNum, int size)
{
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    graph* ptrGraph;
    rw::models::SerialDevice::Ptr robot;
    rw::kinematics::MovableFrame::Ptr BottleGrip;
    rw::models::Device::Ptr gripperDevice;
    rw::kinematics::Frame* robotTCP;
    rw::kinematics::Frame* table;
    rw::common::Timer myTimer;

    if (robotNum == 1)
    {
        robotTCP = _wc->findFrame("1_UR-6-85-5-A.TCP");
        table = _wc->findFrame("Table1");
        robot = _wc->findDevice<rw::models::SerialDevice>("1_UR-6-85-5-A");
        BottleGrip = _wc->findFrame<rw::kinematics::MovableFrame>("BottleGrip1");
        gripperDevice = _wc->findDevice("1_WSG50");
        ptrGraph = &robot1Graph;
    }
    else
    {
        table = _wc->findFrame("Table2");
        robotTCP = _wc->findFrame("2_UR-6-85-5-A.TCP");
        robot = _wc->findDevice<rw::models::SerialDevice>("2_UR-6-85-5-A");
        BottleGrip = _wc->findFrame<rw::kinematics::MovableFrame>("BottleGrip2");
        gripperDevice = _wc->findDevice("2_WSG50");
        ptrGraph = &robot1Graph;
    }

    rw::math::Transform3D<double> worldTTCP;
    rw::math::Q qNew = robot->getQ(state);

    float jointConstraints[6][2] = {{3.142,-3.142},{1.570,-4.712},{3.142,-3.142},{1.570,-4.712},{3.142,-3.142},{3.142,-3.142}};

    float maxErr = 0.01;
    int maxIterations = 100;
    float maxJointStep = 0.25;
    myTimer.resetAndResume();
    for (int i = 0;i<size;i++)
    {
        if (i%100 == 0)
            cout << i << endl;
        //Add random dq
        for(int i = 0; i<6;i++){
            qNew[i] = fRand(jointConstraints[i][1],jointConstraints[i][0]);
        }
        //Find new TCP error
        robot->setQ(qNew,state);
        worldTTCP = rw::kinematics::Kinematics::frameTframe(table,robotTCP,state);
        float distance = aPlane.distance(worldTTCP.P());
        float newDistance = 0;
        int iterationCounter = 0;
        rw::math::Q qUpdate = qNew;
        while(distance > maxErr && maxIterations > iterationCounter)
        {
            iterationCounter++;
            //Random gradient decent
            for(int i = 0; i<6;i++){
                qUpdate[i] = wrapMinMax(qNew[i]+fRand(-3.142,3.142)*maxJointStep,jointConstraints[i][1],jointConstraints[i][0]);
            }
            robot->setQ(qNew,state);
            worldTTCP = rw::kinematics::Kinematics::frameTframe(table,robotTCP,state);
            newDistance = aPlane.distance(worldTTCP.P());

            if (newDistance < distance)
            {
                distance = newDistance;
                qNew = qUpdate;
            }
        }
        if (distance > maxErr || detector->inCollision(state,NULL,true))
        {
            i--;
            continue;
        }
        graphNode newNode;
        newNode.configuration = qNew;
        newNode.postion = worldTTCP.P();
        ptrGraph -> nodeVec.push_back(newNode);

    }
    cout << "Spent " << myTimer.getTimeMs()/1000.0 << "s" << endl;
}

std::vector<rw::math::Q> SamplePlugin::boost_get_path(rw::math::Q start, rw::math::Q goal)
{
  /*  using namespace boost;

    typedef adjacency_list < listS, vecS, directedS,
        no_property, property <edge_weight_t, int> > graph_t;
      typedef graph_traits < graph_t >::vertex_descriptor vertex_descriptor;
      typedef graph_traits < graph_t >::edge_descriptor edge_descriptor;
      typedef std::pair<int, int> Edge;

      const int num_nodes = 5;
      enum nodes { A, B, C, D, E };
      char name[] = "ABCDE";
      Edge edge_array[] = { Edge(A, C), Edge(B, B), Edge(B, D), Edge(B, E),
        Edge(C, B), Edge(C, D), Edge(D, E), Edge(E, A), Edge(E, B)
      };
      int weights[] = { 1, 2, 1, 2, 7, 3, 1, 1, 1 };
      int num_arcs = sizeof(edge_array) / sizeof(Edge);
    #if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
      graph_t g(num_nodes);
      property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
      for (std::size_t j = 0; j < num_arcs; ++j) {
        edge_descriptor e; bool inserted;
        tie(e, inserted) = add_edge(edge_array[j].first, edge_array[j].second, g);
        weightmap[e] = weights[j];
      }
    #else
      graph_t g(edge_array, edge_array + num_arcs, weights, num_nodes);
      property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
    #endif
      std::vector<vertex_descriptor> p(num_vertices(g));
      std::vector<int> d(num_vertices(g));
      vertex_descriptor s = vertex(A, g);

    #if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
      // VC++ has trouble with the named parameters mechanism
      property_map<graph_t, vertex_index_t>::type indexmap = get(vertex_index, g);
      dijkstra_shortest_paths(g, s, &p[0], &d[0], weightmap, indexmap,
                              std::less<int>(), closed_plus<int>(),
                              (std::numeric_limits<int>::max)(), 0,
                              default_dijkstra_visitor());
    #else
      dijkstra_shortest_paths(g, s, predecessor_map(&p[0]).distance_map(&d[0]));
    #endif

  std::cout << "distances and parents:" << std::endl;
  graph_traits < graph_t >::vertex_iterator vi, vend;
  for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
    std::cout << "distance(" << name[*vi] << ") = " << d[*vi] << ", ";
    std::cout << "parent(" << name[*vi] << ") = " << name[p[*vi]] << std::
      endl;
  }
  std::cout << std::endl;*/
}
std::vector<rw::math::Q> SamplePlugin::get_path(rw::math::Q start, rw::math::Q goal, const graph query_graph)
{

    const int n = robot2Graph.nodeVec.size();
    Eigen::SparseMatrix<float> adj;
    adj.resize(n,n);

    //typedef Eigen::Triplet<float> T;
    //std::vector<T> tripletList;
    //tripletList.reserve(n);

    int closest_start_node_index,closest_goal_node_index;

    double min_start_dist=10000,min_goal_dist=10000, cur_err;

    //find the closest start and goal Qs in graph
    for(int i =0;i<robot1Graph.nodeVec.size();i++)
    {
        cur_err=(query_graph.nodeVec[i].configuration.e()-start.e()).norm();
        if (cur_err<min_start_dist)
        {
            min_start_dist =cur_err;
            closest_start_node_index = i;
        }
        cur_err=(query_graph.nodeVec[i].configuration.e()-goal.e()).norm();
        if (cur_err<min_goal_dist)
        {
            min_goal_dist =cur_err;
            closest_goal_node_index = i;
        }
        //Eigen::VectorXd temp = query_graph.nodeVec[i].configuration.e();
        //point_t pt{temp.coeff(1),temp.coeff(2),temp.coeff(3),temp.coeff(4),temp.coeff(5),temp.coeff(6)};
        //points.push_back(pt);
    }
    for(int i =0;i<query_graph.nodeVec.size();i++)
    {
        for(int j= 0; j<query_graph.nodeVec[i].connectionVec.size(); j++)
        {
            adj(i, query_graph.nodeVec[i].connectionVec[j].index) = query_graph.nodeVec[i].connectionVec[j].distance;
            adj(query_graph.nodeVec[i].connectionVec[j].index, i) = query_graph.nodeVec[i].connectionVec[j].distance;
            //tripletList.push_back(T( i, query_graph.nodeVec[i].connectionVec[j].index,query_graph.nodeVec[i].connectionVec[j].distance));
            //tripletList.push_back(T(i, query_graph.nodeVec[i].connectionVec[j].index ,query_graph.nodeVec[i].connectionVec[j].distance));
        }
    }
    //adj.setFromTriplets(tripletList.begin(), tripletList.end());

    cout<<"\nQUERYING\n";
    float* distance;
    float* pred;
    distance = new float[n];
    pred = new float[n];

    Eigen::MatrixXf cost;
    cost.resize(n, n);

    bool* visited;
    visited = new bool[n];
    int mindistance,nextnode,i,j;

    for(i=0;i<n;i++)
       for(j=0;j<n;j++)
            if(adj.coeff(i,j)==0)
               cost(i,j)=500;
            else
               cost(i,j)=adj.coeff(i,j);

    for(i=0;i<n;i++) {
       distance[i]=cost(closest_start_node_index, i);
       pred[i]=closest_start_node_index;
       visited[i]=false;
    }

    distance[closest_start_node_index]=0;
    visited[closest_start_node_index]=true;
    int count=1;
    while(count<n-1) {
       mindistance=50000;
       for(i=0;i<n;i++)
          if(distance[i]<mindistance&&!visited[i]) {
          mindistance=distance[i];
          nextnode=i;
       }
       visited[nextnode]=true;
       for(i=0;i<n;i++)
          if(!visited[i])
       if(mindistance+cost.coeffRef(nextnode,i)<distance[i]) {
          distance[i]=mindistance+cost.coeffRef(nextnode,i);
          pred[i]=nextnode;
       }
       count++;
    }

    vector<rw::math::Q> Q_path;

   std::cout<<"\nDistance of node"<<closest_goal_node_index<<"="<<distance[i];
   std::cout<<"\nPath="<<closest_goal_node_index;
   Q_path.push_back(robot2Graph.nodeVec[i].configuration);
   j=closest_goal_node_index;
   do {
      j=pred[j];
      Q_path.push_back(robot2Graph.nodeVec[j].configuration);
      cout<<"<-"<<j;
   }while(j!=closest_start_node_index);

   //free up memory
   delete [] distance;
   delete [] pred;
   delete [] visited;

    return Q_path;
}

/* For adding connections
pointVec points;
for(int i =0;i<query_graph.nodeVec.size();i++)
{
    Eigen::VectorXd temp = query_graph.nodeVec[i].configuration.e();
    point_t pt{temp.coeff(1),temp.coeff(2),temp.coeff(3),temp.coeff(4),temp.coeff(5),temp.coeff(6)};
    points.push_back(pt);
}
//KDTree tree(points);
//vector<unsigned long int> closest_pts_indexes;//std::vector< pointIndex >
for(int i =0;i<query_graph.nodeVec.size();i++)
{
    Eigen::VectorXd curr_node = query_graph.nodeVec[i].configuration.e();
    point_t pt{curr_node.coeff(1),curr_node.coeff(2),curr_node.coeff(3),curr_node.coeff(4),curr_node.coeff(5),curr_node.coeff(6)};

    closest_pts_indexes = tree.neighborhood_indices(pt, 0.01);
    //auto closest_pts = tree.neighborhood_points(pt, 0.01);

    for ( int a : closest_pts_indexes) {
        connections connection;
        connection.cost = (curr_node-query_graph.nodeVec[a].configuration.e()).norm();
        connection.index = a;
        if ((query_graph.nodeVec[i].postion.e()-robot2Graph.nodeVec[a].postion.e()).norm()<0.01)//if workspace dist < 0.01
            query_graph.nodeVec[i].connectionVec.push_back(connection);
    }
}*/
void SamplePlugin::BottlePRM(){

    rw::kinematics::MovableFrame::Ptr Bottle = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    //Eigen::Matrix4f initTransform =Bottle_Frame->getTransform(_state).e();
    PRMNode head;
    head.id = 0;

    std::vector<PRMNode> nodePointers;
    //nodePointers.push_back(head);

    /*
    rwlibs::proximity::BasicFilterStrategy::Ptr broadphase = ownedPtr(new rwlibs::proximity::BasicFilterStrategy(_wc));
    CollisionDetector::Ptr collisionDetector = ownedPtr(new CollisionDetector(_wc, rw::proximity::ProximityStrategyYaobi::make(), broadphase));

    //Tool frame of the robot
    Frame* toolFrame = workcell->findFrame("Robot.TCP");
    //Frame of the object picked up
    Frame* objectFrame = workcell->findFrame("Object");
    //Frame of the table on which the object previously was located.
    Frame* tableFrame = workcell->findFrame("Table");

    //Remove checking between objectFrame and toolFrame
    broadphase->exclude(rw::kinematics::FramePair(objectFrame, toolFrame);
    //Add checking between the objectFrame and the tableFrame
    broadphase->include(rw::kinematics::FramePair(objectFrame, tableFrame);
    */

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(
                new rw::proximity::CollisionDetector(_wc,
                rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    int count = 0;
    float rn_x,rn_y,rn_z;
    const int n=100;
    cout<<"init adj\n";
    Eigen::SparseMatrix<float> adj(n,n);

    //float adj[n][n] = {};
    cout<<"begin while\n";;
    typedef Eigen::Triplet<double> T;
    std::vector<T> tripletList;
    tripletList.reserve(n);
    while(count < n)
    {
        do{
            rn_x = (rand() % 50)/100.0f;
            rn_y = ((rand() % 50)/100.0f);
            rn_z = ((rand() % 10)/100.0f);
            Bottle->moveTo(
                        rw::math::Transform3D<>(rw::math::Vector3D<>(rn_x,rn_y,0.21f+rn_z),
                                                rw::math::RPY<>(0,0,90*rw::math::Deg2Rad)
                                                ), _state);
        }
        while(detector->inCollision(_state,NULL,true));
        PRMNode temp;
        temp.Pos = rw::math::Vector3D<>(rn_x,rn_y,0.21f+rn_z);
        temp.id = count++;

        for(int i =0;i<nodePointers.size();i++)
        {
            //nodeDistances.push_back((temp.Pos - nodePointers[i].Pos).norm2());
            nodePointers[i].distanceFromTemp = (temp.Pos - nodePointers[i].Pos).norm2();

        }
        /*cout<<"\nBefore sort"<<endl;
        for(int i =0;i<nodePointers.size();i++)
            cout<<nodePointers[i].id<<":"<<nodePointers[i].distanceFromTemp<<" "<<nodePointers[i].Pos<<"  ";
        cout<<endl;*/
        std::sort(nodePointers.begin(), nodePointers.end(),
                       [](const auto& i, const auto& j) { return i.distanceFromTemp < j.distanceFromTemp; } );
        /*cout<<"After sort"<<endl;
        for(int i =0;i<nodePointers.size();i++)
            cout<<nodePointers[i].id<<":"<<nodePointers[i].distanceFromTemp<<"  ";
        cout<<endl;*/
        //cout<<"sorted "<<count<<"\n";



        for(int i =0;i<((nodePointers.size() > 10)? 10:nodePointers.size());i++)
        {
            //cout<<"i"<<i<<endl;
            if(nodePointers[i].distanceFromTemp<0.05)
            {
                tripletList.push_back(T(temp.id,nodePointers[i].id,(temp.Pos - nodePointers[i].Pos).norm2()));
                tripletList.push_back(T(nodePointers[i].id,temp.id,(temp.Pos - nodePointers[i].Pos).norm2()));
                //cout<<"assigned adj\n";
            }
        }
        //cout<<"pushing temp\n";
        nodePointers.push_back(temp);
        //cout<<"temp pushed\n";
        /*cout<<"After sort Edges"<<endl;
        for(int i =0;i<Edges.size();i++)
            cout<<Edges[i].ids[0]<<" "<<Edges[i].ids[1]<<":"<<Edges[i].distance<<"  ";
        cout<<endl;*/

    }
    adj.setFromTriplets(tripletList.begin(), tripletList.end());
    getRobWorkStudio()->setState(_state);

    cout<<"After sort\n";
    for(int i =0;i<nodePointers.size();i++)
        cout<<nodePointers[i].id<<":"<<nodePointers[i].distanceFromTemp<<"  ";
    cout<<endl;
    /*cout<<"adj\n";
    for(int i=0;i<n;i++)
    {

        for(int j=0;j<n;j++)
            cout<<setprecision(2)<<adj[i][j]<<" ";
        cout<<"\n";
    }*/
    //QUERY
    /*
    cout<<"\nQUERYING\n";
    int startnode = 2;
    rw::math::Vector3D<double> initPos = {0.05, 0.975, 0.247}; // FOR INV KIN: Q[6]{0.05, 0.975, 0.247, -3.142, -0.003, -1.546}
    rw::math::Vector3D<double> initRPY = {-3.142, -0.003, -1.546};

    float distance[n],pred[n];

    //https://www.tutorialspoint.com/cplusplus-program-for-dijkstra-s-shortest-path-algorithm
    Eigen::Matrix<double, n, n> cost;
    int visited[n],mindistance,nextnode,i,j;
    for(i=0;i<n;i++)
       for(j=0;j<n;j++)
            if(adj.coeff(i,j)==0)
               cost.coeffRef(i,j)=500;
            else
               cost.coeffRef(i,j)=adj.coeff(i,j);
    for(i=0;i<n;i++) {
       distance[i]=cost.coeffRef(startnode, i);
       pred[i]=startnode;
       visited[i]=0;
    }

    cout<<"cost\n";
    for(int i=0;i<n;i++)
    {

        for(int j=0;j<n;j++)
            cout<<setprecision(2)<<cost[i][j]<<" ";
        cout<<"\n";
    }

    distance[startnode]=0;
    visited[startnode]=1;
    count=1;
    while(count<n-1) {
       mindistance=50000;
       for(i=0;i<n;i++)
          if(distance[i]<mindistance&&!visited[i]) {
          mindistance=distance[i];
          nextnode=i;
       }
       visited[nextnode]=1;
       for(i=0;i<n;i++)
          if(!visited[i])
       if(mindistance+cost[nextnode][i]<distance[i]) {
          distance[i]=mindistance+cost[nextnode][i];
          pred[i]=nextnode;
       }
       count++;
    }
    for(i=0;i<n;i++)
        if(i!=startnode) {
           cout<<"\nDistance of node"<<i<<"="<<distance[i];
           cout<<"\nPath="<<i;
           j=i;
           do {
              j=pred[j];
              cout<<"<-"<<j;
           }while(j!=startnode);
        }

*/
}

void SamplePlugin::TCMP(){
    /// Find initial Q pose
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    cout << "detector " << !detector->inCollision(_state,NULL,true) << endl;
    Q HOME(6, 1.571, -1.158, -2.728, 0.771, 1.571, 0);
    _wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->moveTo(
                rw::math::Transform3D<>(rw::math::Vector3D<>(0.04, 0.835, 0.21),
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
    rw::math::Vector3D<double> initPos = {0.05, 0.975, 0.247}; // FOR INV KIN: Q[6]{0.05, 0.975, 0.247, -3.142, -0.003, -1.546}
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
    //rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    while(POSerr.norm2() > 0.05 || RPYerr.norm2() > 0.05){ //
        //Add random dq
        for(int i = 0; i<6;i++){
            qNew[i] = wrapMinMax(qVal[i]+fRand(-M_PI,M_PI)*0.05,-M_PI,M_PI);
        }

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
            cout << "update and Qval " << qVal << endl;
            getRobWorkStudio()->setState(_state);
        }
        counter++;
        if(counter % 10000000 == 0){
            cout << "Counter value " << counter << endl;
        }
    }

    cout<<"SUCCESS! CounterVal " << counter << "\t"<< qVal << endl;

    std::vector<element> Tree;

    Tree.push_back({1,qVal,worldTTCP.P(),RPYasVec});
    counter = 0;
    double maxJointStep = rw::math::Deg2Rad*10;

    //while()

}
