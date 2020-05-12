#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// Includes
#include <thread>
#include <iomanip>
#include <queue>
#include <set>


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

    struct connections{
        double cost; // Difference in configuration space
        double distance; // Difference in 3D space
        int index; // index of node
        bool collisionChecked = false;
    };

    static bool lessThan(const connections a,const connections b){
            return (b.cost > a.cost);
    }

    struct graphNode{
        std::vector<connections> connectionVec;
        int index;
        int parentIdx;
        float distance;
        float heuristic;
        float cost;
        rw::math::Q configuration;
        rw::math::Vector3D<> postion;
        rw::math::Vector3D<> rotation;
    };

    struct compareNodes
    {
        bool operator()(const graphNode *a,const graphNode *b){
                    return ((b->cost) < (a->cost));
        }
    };


    struct graph{
        std::vector<graphNode> nodeVec;
    } robot1Graph,robot2Graph;

    struct D4Node{
      int x1;
      int x2;
      int y1;
      int y2;
      std::vector<int> idx;
      bool connect = false;
      float probability;
      float accumulatedProbability;
    };

    struct D4Array{

        std::vector<D4Node> Start;
        std::vector<D4Node> Goal;

    } Our4Darray;

    struct dualGraphNode{
        rw::math::Q configuration;
        int parent;
    };

    struct dualGraph{
        std::vector<dualGraphNode> r1Start;
        std::vector<dualGraphNode> r1Goal;

        std::vector<dualGraphNode> r2Start;
        std::vector<dualGraphNode> r2Goal;

    } curDualGraph;

    struct robotPtr
    {
        graph* ptrGraph;
        rw::models::SerialDevice::Ptr robot;
        rw::kinematics::MovableFrame::Ptr BottleGrip;
        rw::models::Device::Ptr gripperDevice;
        rw::kinematics::Frame* robotTCP;
        rw::kinematics::Frame* table;
    } robotPtr1, robotPtr2;

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
    double fRand(double fMin, double fMax); //Random Double generator
    double wrapMax(double x, double max);
    double wrapMinMax(double x, double min, double max);
    QPath blendPath(QPath path);
    void kuusTest();
    void planeFunc();
    void createTree(rw::geometry::Plane aPlane, rw::kinematics::State state, int robotNum, int size);
    bool RGD_New_Config(rw::geometry::Plane aPlane,rw::math::Q* q, robotPtr rPtr,rw::kinematics::State* state, float dMax);

    void lazyConnectGraph(int robotNum);
    void connectGraph(rw::kinematics::State state, int robotNum);

    bool canConnect(rw::math::Q q1, rw::math::Q q2,  robotPtr rPtr, rw::proximity::CollisionDetector::Ptr detector,rw::kinematics::State state,int splits);

    void findPath(rw::math::Vector3D<> start,rw::math::Vector3D<> goal, int robotNum);
    void lazyFindPath(rw::math::Vector3D<> start,rw::math::Vector3D<> goal, rw::kinematics::State state, int robotNum);

    vector<int> Astar(int startIdx, int goalIdx, int robotNum);

    void DualPRM(rw::math::Q goalConfRobot1, rw::math::Q goalConfRobot2,rw::kinematics::State state,int maximumIterations,float jointStep, int cellSize);
    void DualPath();
    bool DualCanConnect(rw::math::Q r1q1, rw::math::Q r1q2,rw::math::Q r2q1, rw::math::Q r2q2, rw::proximity::CollisionDetector::Ptr detector,rw::kinematics::State state,int splits);

    void dualDemo();

    void LIPpath(QPath path1, QPath path2);

    void DualShortcut(QPath r1Path, QPath r2Path, int maxIterations);

    void printDualTree();

    int randExpand(bool start);
    float weigthedNorm2(rw::math::Q q1, rw::math::Q q2);

    float costFunc(int iIdx, int jIdx,int robotNum);
    void setupRobotPtrs();
    void saveTree(int robotNum);
    QPath move(rw::math::Q From, rw::math::Q To, rw::models::SerialDevice::Ptr robot,rw::kinematics::State state);
    std::vector<rw::math::Vector3D<double>> linePath(rw::math::Vector3D<> start, rw::math::Vector3D<> end, double stepSize);

private:
    //static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QTimer* _timer;
    QTimer* _timer25D;
    float jointConstraints[6][2] = {{-3.142,3.142},{-4.712,1.570},{-3.142,3.142},{-4.712,1.570},{-3.142,3.142},{-3.142,3.142}};
    float jointWeights[6] = {4,3,2,2,1,1};
    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;    

    Device::Ptr _device2;
    Device::Ptr _device1;
    QPath _PPpath1;
    QPath _PPpath2;
    QPath _path1;
    QPath _path2;
    int _printCounter = 0;
    int _step;
    int _attachIdx;
    Q _attachQ;
    Q _deattachQ;
    std::vector<int> printAblePathSize;
    std::vector<double> printAbleDurations;
    std::vector<int> saveGraphIdx;
    std::vector<std::thread> active_threads;

    int connectionIdx[2] = {};
};

#endif /*RINGONHOOKPLUGIN_HPP_*/
