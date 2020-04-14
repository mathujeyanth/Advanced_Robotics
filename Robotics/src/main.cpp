// Source: "ReachabilityAnalysis_solution.zip" provided by Jeppe Langaa on BlackBoard

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/pathplanning.hpp>

#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>
#include <string>

USE_ROBWORK_NAMESPACE
using namespace robwork;

std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
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

int main(int argc, char** argv)
{
    //load workcell
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load("../Project_WorkCell/Scene.wc.xml");
    // find relevant frames
    rw::kinematics::MovableFrame::Ptr Bottle = wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    rw::kinematics::MovableFrame::Ptr URbase = wc->findFrame<rw::kinematics::MovableFrame>("URReference");
    rw::models::SerialDevice::Ptr robotUR5 = wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    rw::models::Device::Ptr gripperDevice = wc->findDevice("WSG50");

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    // Gripper Q to be open
    const rw::math::Q gQ(1, 0.055); // 2.886

    // get the default state
    rw::kinematics::State state = wc->getDefaultState();
    gripperDevice -> setQ(gQ,state);
    rw::math::Transform3D<> bottlePoses[3];
    bottlePoses[0] = rw::math::Transform3D<>(rw::math::Vector3D<>(-0.3,0.5,0.21),
                                             rw::math::RPY<>(0,0,90*rw::math::Deg2Rad)
                                             );
    bottlePoses[1] = rw::math::Transform3D<>(rw::math::Vector3D<>(0.3,0.5,0.21),
                                             rw::math::RPY<>(0,0,90*rw::math::Deg2Rad)
                                             );
    bottlePoses[2] = rw::math::Transform3D<>(rw::math::Vector3D<>(0.3,-0.5,0.21),
                                             rw::math::RPY<>(0,0,90*rw::math::Deg2Rad)
                                             );
    std::cout << "Reachability test \n";
    cv::Mat img = cv::Mat::zeros(13,11,CV_64F);
    cv::Mat dst(130,110,CV_64F);
    for (int x = 0;x<13;x++)
    {
        for (int y = 0;y<11;y++)
        {
            int currentValue = 0;
            float x_pos = -0.30 + x * 0.05, y_pos = -0.30 + y * 0.05;
            URbase -> moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(x_pos,y_pos,0.11f),
                                                     rw::math::RPY<>(0,0,0)
                                                     ), state);
            for (int botPos = 0;botPos<3;botPos++)
            {
                for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
                    Bottle->moveTo(bottlePoses[botPos], state);
                    std::vector<rw::math::Q> solutions = getConfigurations("Bottle", "GraspTCP", robotUR5, wc, state);
                    bool canReach = false;
                    for(unsigned int i=0; i<solutions.size(); i++){
                        // set the robot in that configuration and check if it is in collision
                        robotUR5->setQ(solutions[i], state);

                        if( !detector->inCollision(state,NULL,true) )
                        {
                            canReach = true;
                            break; // we only need one
                        }
                    }
                    if (canReach)
                        currentValue++;
                }
                for(double rollAngle=0; rollAngle<360.0; rollAngle+=10.0){ // for every degree around the roll axis
                    Bottle->moveTo(bottlePoses[botPos], state);
                    std::vector<rw::math::Q> solutions = getConfigurations("BottleTop", "GraspTCP", robotUR5, wc, state);
                    bool canReach = false;
                    for(unsigned int i=0; i<solutions.size(); i++){
                        // set the robot in that configuration and check if it is in collision
                        robotUR5->setQ(solutions[i], state);

                        if( !detector->inCollision(state,NULL,true) )
                        {
                            canReach = true;
                            break; // we only need one
                        }
                    }
                    if (canReach)
                        currentValue++;
                }
            }
            std::cout << currentValue << "\t";
            img.at<double>(x,10-y) = 1.0-(currentValue/255.0);
        }

        std::cout << "\n";
    }
    cv::resize(img,dst,dst.size(),10,10,cv::INTER_NEAREST);
    cv::imwrite("smallImg.jpg",img);
    cv::imwrite("bigImg.jpg",dst);
    cv::imshow("Map",dst);
    cv::waitKey(0);

	return 0;
}
