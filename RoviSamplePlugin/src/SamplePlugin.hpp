#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>
#include <rw/kinematics.hpp>


// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
// OpenCV 3
//#include <opencv2/opencv.hpp>

// Qt
#include <fstream>
#include <QTimer>

#include "ui_SamplePlugin.h"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>
#include <string>
#include <chrono>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;


using namespace rws;

//using namespace cv;

using namespace std::placeholders;


class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

private slots:
    void btnPressed();
    void timer();

    void stateChangedListener(const rw::kinematics::State& state);
    std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state);
    bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q);
    void createPtPPath(rw::math::Vector3D<> to);
    void createPtPiPath(rw::math::Vector3D<> to);
    void createPathRRTConnect(rw::math::Vector3D<> to, double eps);

    /// Task Constrained Motion Planning from lecture 4
    void TCMP();
    void BottlePRM();
    double fRand(double fMin, double fMax); //Random Double generator
    double wrapMax(double x, double max);
    double wrapMinMax(double x, double min, double max);
    Eigen::Matrix3f findBestHandoverOrientation();
    Eigen::Matrix4f xyzrpyToTransformMatrix(double tx, double ty, double tz, double rz, double ry, double rx);
    void createTree(rw::geometry::Plane aPlane,rw::kinematics::State state, int robotNum, int size);

    static void staticxyzrpyToTransformMatrix(Eigen::Matrix4d &_pose, double tx, double ty, double tz, double rz, double ry, double rx);
    std::vector<rw::math::Q> boost_get_path(rw::math::Q start, rw::math::Q goal);
    void saveTree(int robotNum);


private:
    //static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QTimer* _timer;
    QTimer* _timer25D;
    
    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;    

    Device::Ptr _device2;
    Device::Ptr _device1;
    QPath _path;
    int _step;
    int _attachIdx;
    Q _attachQ;
    Q _deattachQ;
    std::vector<int> printAblePathSize;
    std::vector<double> printAbleDurations;
    struct connections{
            double cost; // Difference in configuration space
            double distance; // Difference in 3D space
            int index; // index of node
        };

        struct graphNode{
            std::vector<connections> connectionVec;
            rw::math::Q configuration;
            rw::math::Vector3D<> postion;
            rw::math::Vector3D<> rotation;
        };

        struct graph{
            std::vector<graphNode> nodeVec;
        } robot1Graph,robot2Graph;
    Q robot_1_initial_config,robot_2_initial_config;
    Q robot_1_final_config,robot_2_final_config;

    std::vector<rw::math::Q> get_path(rw::math::Q start, rw::math::Q goal, const graph query_path);

};

#endif /*RINGONHOOKPLUGIN_HPP_*/
