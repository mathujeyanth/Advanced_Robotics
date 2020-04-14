#include "SamplePlugin.hpp"


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

    _framegrabber = NULL;

    _cameras = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};
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
    WorkCell::Ptr wc = WorkCellLoader::Factory::load("/home/student/Desktop/Project_WorkCell/Scene.wc.xml");
    getRobWorkStudio()->setWorkCell(wc);

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

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame(_cameras[0]);
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap().has("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber = new GLFrameGrabber(width,height,fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber->init(gldrawer);
            }
        }

        Frame* cameraFrame25D = _wc->findFrame(_cameras25D[0]);
        if (cameraFrame25D != NULL) {
            if (cameraFrame25D->getPropertyMap().has("Scanner25D")) {
                // Read the dimensions and field of view
                double fovy;
                int width,height;
                std::string camParam = cameraFrame25D->getPropertyMap().get<std::string>("Scanner25D");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D = new GLFrameGrabber25D(width,height,fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio()->getView()->getSceneViewer();
                _framegrabber25D->init(gldrawer);
            }
        }

        _device = _wc->findDevice("UR-6-85-5-A");
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

Mat SamplePlugin::toOpenCVImage(const Image& img) {
    Mat res(img.getHeight(),img.getWidth(), CV_8SC3);
    res.data = (uchar*)img.getImageData();
    return res;
}


void SamplePlugin::btnPressed() {
    QObject *obj = sender();
    if (obj == _home)
    {
        _timer->stop();
        //rw::math::Math::seed();
        Q to(6, 1.571, -1.158, -2.728, 0.771, 1.571, 0); //From pose estimation
        _device->setQ(to,_state);
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
        std::cout << _doubleSpinBox->value() << std::endl;
        for (int i = 0;i<10;i++)
        {
            _timer->stop();
            rw::math::Math::seed();
            auto start = std::chrono::high_resolution_clock::now();
            createPathRRTConnect(_wc->findFrame<rw::kinematics::MovableFrame>("Bottle")->getTransform(_state).P(),_doubleSpinBox->value());
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
            printAblePathSize.push_back(_path.size());
            printAbleDurations.push_back(duration.count());
        }
        for (int i = 0;i<printAblePathSize.size();i++)
        {
            std::cout << printAblePathSize[i] << " ";
        }
        std::cout << "\n";
        for (int i = 0;i<printAbleDurations.size();i++)
        {
            std::cout << printAbleDurations[i] << " ";
        }
        std::cout << "\n";
        printAbleDurations.clear();
        printAblePathSize.clear();
    }
    else if(obj==_btn1){
        if (!_timer->isActive()){
            _timer->start(100); // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;

    } else if(obj==_doubleSpinBox){
        log().info() << "spin value:" << _doubleSpinBox->value() << "\n";
    }
    else if( obj==_btn_im ){
        getImage();
    }
    else if( obj==_btn_scan ){
        get25DImage();
    }


}


void SamplePlugin::get25DImage() {
    if (_framegrabber25D != NULL) {
        for( int i = 0; i < _cameras25D.size(); i ++)
        {
            // Get the image as a RW image
            Frame* cameraFrame25D = _wc->findFrame(_cameras25D[i]); // "Camera");
            _framegrabber25D->grab(cameraFrame25D, _state);

            //const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage());

            std::ofstream output(_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth() << "\n";
            output << "HEIGHT " << img->getHeight() << "\n";
            output << "POINTS " << img->getData().size() << "\n";
            output << "DATA ascii\n";
            for(const auto &p_tmp : img->getData())
            {
                rw::math::Vector3D<float> p = p_tmp;
                output << p(0) << " " << p(1) << " " << p(2) << "\n";
            }
            output.close();

        }
    }
}

void SamplePlugin::printProjectionMatrix(std::string frameName) {
    Frame* cameraFrame = _wc->findFrame(frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width,height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );

            Eigen::Matrix<double, 3, 4> KA;
            KA << fovy_pixel, 0, width / 2.0, 0,
                    0, fovy_pixel, height / 2.0, 0,
                    0, 0, 1, 0;

            log().info() << "Ïntrinsic parameters:" << std::endl;
            log().info() << KA << std::endl;


            Transform3D<> camPosOGL = cameraFrame->wTf(_state);
            Transform3D<> openGLToVis = Transform3D<>(RPY<>(-Pi, 0, Pi).toRotation3D());
            Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            log().info() << "Extrinsic parameters:" << std::endl;
            log().info() << H.e() << std::endl;
        }
    }
}

void SamplePlugin::getImage() {

    if (_framegrabber != NULL) {
        for( int i = 0; i < _cameras.size(); i ++)
        {
            printProjectionMatrix(_cameras[i]);
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame(_cameras[i]); // "Camera");
            _framegrabber->grab(cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat(rw_image->getHeight(), rw_image->getWidth(), CV_8UC3, (rw::sensor::Image*)rw_image->getImageData());

            // Convert to OpenCV image
            Mat imflip, imflip_mat;
            cv::flip(image, imflip, 1);
            cv::cvtColor( imflip, imflip_mat, COLOR_RGB2BGR );

            cv::imwrite(_cameras[i] + ".png", imflip_mat );

            // Show in QLabel
            QImage img(imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p = QPixmap::fromImage(img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap(p.scaled(maxW,maxH,Qt::KeepAspectRatio));
        }
    }
}

void SamplePlugin::timer() {
    _wc->findDevice("WSG50")-> setQ(rw::math::Q(1, 0.055),_state);
    if(0 <= _step && _step < _path.size()){
        if (_attachIdx == _step)
        {
            _device -> setQ(_attachQ,_state);
            rw::kinematics::Kinematics::gripFrame(_wc->findFrame("Bottle"),_wc->findFrame("GraspTCP"),_state);
            getRobWorkStudio()->setState(_state);
            _step++;
        }else
        {
            std::cout << _path.at(_step)(0) << " " << _path.at(_step)(1) << " " << _path.at(_step)(2) << " " << _path.at(_step)(3) << " " << _path.at(_step)(4) << " " << _path.at(_step)(5) << ";\n";
            _device->setQ(_path.at(_step),_state);
            getRobWorkStudio()->setState(_state);
            _step++;
        }

        if (_step == _path.size())
        {
            _device -> setQ(_deattachQ,_state);
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
        Bottle->moveTo(bottleStart);
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
        Bottle->moveTo(bottleStart);
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
