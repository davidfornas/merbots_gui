/**
 * Copyright (c) 2014 University of Jaume-I.
 * All rights reserved. This program and the accompanying materials
 * are made available under the terms of the GNU Public License v3.0
 * which accompanies this distribution, and is available at
 * http://www.gnu.org/licenses/gpl.html
 *
 * This file has been adapted from ROS command catkin_create_qt_pkg
 *
 * Author:
 *     Juan Carlos García
 *
 * Date February 2017
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <iostream>
#include "../include/merbots_gui/main_window.hpp"
#include "../include/merbots_gui/set_robot_pose.h"
#include "../include/merbots_gui/set_params.h"
#include <math.h>

#include <QtGui>
#include <QMessageBox>
#include <QLineEdit>
#include <QString>
#include <QtCore>
#include <QThread>
#include <QTimer>
#include <QTableWidget>
#include <QSlider>

#include <ros/package.h>
#include <sensor_msgs/image_encodings.h>


#define DEBUG			  false
#define CURRENT_THRESHOLD 1.0
#define SIMULATED_ARM	  false


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace enc = sensor_msgs::image_encodings;

namespace merbots_gui {

using namespace std;
using namespace Qt;


/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
  : QMainWindow(parent)
  , qnode(argc,argv)
{
  setWindowIcon(QIcon(":/images/icon.png"));
  ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  ui.mainTabs->setCurrentIndex(1);
  ui.graspSpecTab->setCurrentIndex(0);
  ui.poseEstimationComboBox->insertItem(0,"Get from topic"); // Index 5
  ui.poseEstimationComboBox->insertItem(0,"SuperQuadrics"); //Index 4
  ui.poseEstimationComboBox->insertItem(0,"Plane based Box"); // Index 3
  ui.poseEstimationComboBox->insertItem(0,"PCA"); //Index 2
  ui.poseEstimationComboBox->insertItem(0,"RANSAC Sphere"); //Index 1
  ui.poseEstimationComboBox->insertItem(0,"RANSAC Cylinder"); //Index 0
  ui.poseEstimationComboBox->setCurrentIndex(0);

  //Booleans control
  g500CameraEnable   = false;
  g500CameraEnable2  = true;

  //Variable to control the 'diagnostics_agg' topic
  g500DiagnosticsErrorLevel = 0;

  //Init section
  ros::init(argc,argv,"merbots_gui");
  ros::start(); 	//explicitly needed since our nodehandle is going out of scope
  if (!ros::master::check())
  {
    qDebug()<<"No ROS master found";
    showNoMasterMessage();
  }

  //Odometry tables init
  for (int i=0; i<4; i++)
  {
    for (int j=0; j<6; j++)
    {
      ui.g500OdometryTable->setItem(i, j, new QTableWidgetItem("0.0"));
    }
  }

  //Service tables init
  for (int i=0; i<3; i++)
  {
    ui.g500ServiceStatus->setItem(0, i, new QTableWidgetItem("0.0"));
  }

  ui.slewProgressBar->setAlignment(Qt::AlignCenter);
  ui.shoulderProgressBar->setAlignment(Qt::AlignCenter);
  ui.elbowProgressBar->setAlignment(Qt::AlignCenter);
  ui.jawRotateProgressBar->setAlignment(Qt::AlignCenter);
  ui.jawOpeningProgressBar->setAlignment(Qt::AlignCenter);


  //Main App connections
  QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

  //Menu objects connections
  QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  //TAB1: Image stream connections
  QObject::connect(ui.g500LoadStreamButton, SIGNAL(clicked()), this, SLOT(g500LoadStream()));
  QObject::connect(ui.g500LoadStreamButton2, SIGNAL(clicked()), this, SLOT(g500LoadStream2()));
  QObject::connect(ui.g500StopStreamButton, SIGNAL(clicked()), this, SLOT(g500StopStream()));
  QObject::connect(ui.g500StopStreamButton2, SIGNAL(clicked()), this, SLOT(g500StopStream2()));
  QObject::connect(ui.g500TopicsButton, SIGNAL(clicked()), this, SLOT(g500TopicsButtonClicked()));
  QObject::connect(ui.g500GoToPositionButton, SIGNAL(clicked()), this, SLOT(g500GoToPositionButtonClicked()));
  QObject::connect(ui.g500GoToSurfaceButton, SIGNAL(clicked()), this, SLOT(g500GoToSurface()));

  QObject::connect(ui.getGraspingPoseButton, SIGNAL(clicked()), this, SLOT(getInitGraspPose()));
  QObject::connect(ui.graspSpecTab, SIGNAL(currentChanged(int)), this, SLOT(setSpecificationMode(int)));
  QObject::connect(ui.interactiveSpecSlider1, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider2, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider3, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider4, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider5, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider6, SIGNAL(sliderMoved(int)), this, SLOT(updateInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider1, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider2, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider3, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider4, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider5, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
  QObject::connect(ui.interactiveSpecSlider6, SIGNAL(sliderReleased()), this, SLOT(updateAndResetInteractiveSpecParams()));
  QObject::connect(ui.guidedSpecSlider1, SIGNAL(sliderMoved(int)), this, SLOT(updateGuidedSpecParams()));
  QObject::connect(ui.guidedSpecSlider2, SIGNAL(sliderMoved(int)), this, SLOT(updateGuidedSpecParams()));
  QObject::connect(ui.guidedSpecSlider3, SIGNAL(sliderMoved(int)), this, SLOT(updateGuidedSpecParams()));
  QObject::connect(ui.guidedSpecSpin1, SIGNAL(valueChanged(double)), this, SLOT(updateGuidedSpecParamsSpin1()));
  QObject::connect(ui.guidedSpecSpin2, SIGNAL(valueChanged(double)), this, SLOT(updateGuidedSpecParamsSpin2()));
  QObject::connect(ui.guidedSpecSpin3, SIGNAL(valueChanged(double)), this, SLOT(updateGuidedSpecParamsSpin3()));

  QObject::connect(ui.graspNumberSpinBox, SIGNAL(valueChanged(int)), this, SLOT(updateGraspNumberSpinBox()));


  ui.interactiveSpecSlider1->setMaximum(100);
  ui.interactiveSpecSlider2->setMaximum(100);
  ui.interactiveSpecSlider3->setMaximum(100);
  ui.interactiveSpecSlider4->setMaximum(180);
  ui.interactiveSpecSlider5->setMaximum(180);
  ui.interactiveSpecSlider6->setMaximum(180);
  ui.guidedSpecSlider1->setMaximum(100);
  ui.guidedSpecSlider2->setMaximum(180);
  ui.guidedSpecSlider3->setMaximum(100);
  ui.guidedSpecSpin1->setMaximum(100);
  ui.guidedSpecSpin2->setMaximum(180);
  ui.guidedSpecSpin3->setMaximum(100);

  ui.interactiveSpecSlider1->setMinimum(-100);
  ui.interactiveSpecSlider2->setMinimum(-100);
  ui.interactiveSpecSlider3->setMinimum(-100);
  ui.interactiveSpecSlider4->setMinimum(-180);
  ui.interactiveSpecSlider5->setMinimum(-180);
  ui.interactiveSpecSlider6->setMinimum(-180);
  ui.guidedSpecSlider1->setMinimum(-100);
  ui.guidedSpecSlider2->setMinimum(-180);
  ui.guidedSpecSlider3->setMinimum(-100);
  ui.guidedSpecSpin1->setMinimum(-100);
  ui.guidedSpecSpin2->setMinimum(-180);
  ui.guidedSpecSpin3->setMinimum(-100);

  //@TODO Teleoperation mode switch; goto dredge position and execute grasp buttons...
  QObject::connect(ui.executeGraspingButton, SIGNAL(clicked()), this, SLOT(executeGrasping()));
  QObject::connect(ui.executeDredgingButton, SIGNAL(clicked()), this, SLOT(executeDredging()));
  QObject::connect(ui.addWaypointButton, SIGNAL(clicked()), this, SLOT(addWaypoint()));
  QObject::connect(ui.clearWaypointsButton, SIGNAL(clicked()), this, SLOT(clearWaypoints()));
  QObject::connect(ui.removeLastWaypointButton, SIGNAL(clicked()), this, SLOT(removeLastWaypoint()));
  QObject::connect(ui.armTopicButton, SIGNAL(clicked()), this, SLOT(armTopicButtonClicked()));

  QObject::connect(ui.setParamsButton, SIGNAL(clicked()), this, SLOT(setParamsButtonClicked()));

  //Connecting ROS callbacks
  nh = new ros::NodeHandle();
  image_transport::ImageTransport it(*nh);
  /*sub_g500Odometry      = nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this);
    sub_g500MergedBodyVel = nh->subscribe<auv_msgs::BodyVelocityReq>(ui.g500TopicMergedBodyVel->text().toUtf8().constData(), 1, &MainWindow::g500MergedBodyVelCallback, this);
    sub_g500MergedWorld   = nh->subscribe<auv_msgs::WorldWaypointReq>(ui.g500TopicMergedWorld->text().toUtf8().constData(), 1, &MainWindow::g500MergedWorldCallback, this);
  sub_g500Battery		  = nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this);
  sub_g500Runningtime	  = nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this);
  sub_g500Diagnostics	  = nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this);

    srv_g500GoTo    = nh->serviceClient<cola2_msgs::Goto>(ui.g500TopicGoToService->text().toUtf8().constData());
*/

  sub_g500Camera   = it.subscribe(ui.g500CameraTopic->text().toUtf8().constData(), 1, &MainWindow::g500CameraCallback, this);
  //sub_g500Camera   = it.subscribe(ui.g500CameraTopic->text().toUtf8().constData(), 1, &MainWindow::g500CameraCallback, this);
  /* sub_imageTopic   = it.subscribe(ui.vsCameraInput->text().toUtf8().constData(), 1, &MainWindow::vsInputImageCallback, this);
  sub_resultTopic  = it.subscribe(ui.vsResult->text().toUtf8().constData(), 1, &MainWindow::vsResultImageCallback, this);
  pub_target		 = it.advertise(ui.vsCroppedImage->text().toUtf8().constData(), 1);
*/
  sub_arm_state	 = nh->subscribe<sensor_msgs::JointState>(ui.armTopic->text().toUtf8().constData(), 1, &MainWindow::armStateCallback, this);
  sub_spec_params	 = nh->subscribe<std_msgs::Float32MultiArray>("/specification_params_to_gui", 1, &MainWindow::specParamsCallback, this);
  sub_score = nh->subscribe<std_msgs::Float32>("/score", 1, &MainWindow::scoreCallback, this);
  sub_list_size = nh->subscribe<std_msgs::Int32>("/list_size", 1, &MainWindow::listSizeCallback, this);
  sub_score_description = nh->subscribe<std_msgs::String>("/score_description", 1, &MainWindow::scoreDescriptionCallback, this);
  sub_metrics = nh->subscribe<std_msgs::String>("/metrics", 1, &MainWindow::metricsCallback, this);

  pub_spec_params	 = nh->advertise<std_msgs::Float32MultiArray>("/specification_params_to_uwsim", 1);
  pub_spec_action	 = nh->advertise<std_msgs::String>("/specification_status", 1);
  pub_dredg_action = nh->advertise<std_msgs::String>("/dredging_status", 1);
  pub_grasp_id = nh->advertise<std_msgs::Int8>("/grasp_id", 1);
  //pub_dredging     = nh->advertise<cola2_msgs::Setpoints>("/dredge_pump_setpoint", 1);

  //Timer to ensure the ROS communications
  QTimer *timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(processSpinOnce()));
  timer->start();
  QTimer *armTimer = new QTimer(this);
  connect(armTimer, SIGNAL(timeout()), this, SLOT(armPublisher()));
  armTimer->start(300);
  QTimer *dredginTimer = new QTimer(this);
  connect(dredginTimer, SIGNAL(timeout()), this, SLOT(dredgingPublisher()));
  dredginTimer->start(100);   //10Hz



  dlg = new SetRobotPoseDlg(nh, this);
  QObject::connect(dlg, SIGNAL(newRobotPose(double, double, double, double, double, double)),
                   this, SLOT(setRobotPosition(double, double, double, double, double, double)));


  dlg2 = new SetParamsDlg(nh, this);

  //Parameters to set content label to fill the label size
  ui.g500StreamView->setScaledContents(true);
  ui.g500StreamView2->setScaledContents(true);

  //Arm teleoperation init
  axisindex[0]= 0; AxisDir[0]= 1;		//Slew
  axisindex[1]= 1; AxisDir[1]= 1;		//Shoulder
  axisindex[2]= 4; AxisDir[2]= -1;	//Elbow
  axisindex[3]= 3; AxisDir[3]= -1;	//JawRotate
  axisindex[4]= 5; AxisDir[4]= 1;		//JawClose
  axisindex[5]= 2;					//JawOpen

  buttonindex[0]= 0;
  buttonindex[1]= 1;
  buttonindex[2]= 2;
  buttonindex[3]= 6;
  buttonindex[4]= 7;

  error_ = 0.035;

  current=0;
  slewLocked_=false;
  jawRotateLocked_=false;
  jawOpenLocked_=false;
  fixSlew_=false;
  park_=false;

  joy_sub_ = nh->subscribe<sensor_msgs::Joy>("/joystick_out", 10, &MainWindow::joyCallback, this);
  arm_sub_= nh->subscribe<sensor_msgs::JointState>("/arm5e/joint_state_rticks", 1, &MainWindow::armCallback, this);

  if (SIMULATED_ARM)
  {	//UWSim
    scale_ = 0.3;
    vel_pub_ = nh->advertise<sensor_msgs::JointState>("/uwsim/joint_state_command",1);
    arm_angle_sub_= nh->subscribe<sensor_msgs::JointState>("/uwsim/joint_state", 1, &MainWindow::armAngleCallback, this);
  }
  else
  {	//Real arm
    scale_ = 2500;
    vel_pub_ = nh->advertise<sensor_msgs::JointState>("/arm5e/command_ticks",1);
    arm_angle_sub_= nh->subscribe<sensor_msgs::JointState>("/arm5e/joint_state_angle", 1, &MainWindow::armAngleCallback, this);
  }

  nh->param("SlewAxis", axisindex[0], axisindex[0]);
  nh->param("ShoulderAxis", axisindex[1], axisindex[1]);
  nh->param("ElbowAxis", axisindex[2], axisindex[2]);
  nh->param("JawRotateAxis", axisindex[3], axisindex[3]);
  nh->param("JawCloseAxis", axisindex[4], axisindex[4]);
  nh->param("JawOpenAxis", axisindex[5], axisindex[5]);
  nh->param("SlewDir", AxisDir[0], AxisDir[0]);
  nh->param("ShoulderDir", AxisDir[1], AxisDir[1]);
  nh->param("ElbowDir", AxisDir[2], AxisDir[2]);
  nh->param("WristDir", AxisDir[3], AxisDir[3]);
  nh->param("JawDir", AxisDir[4], AxisDir[4]);
  nh->param("scale", scale_, scale_);
  nh->param("error", error_, error_);
  nh->param("LockSlewButton", buttonindex[0], buttonindex[0]);
  nh->param("LockJawRotateButton", buttonindex[1], buttonindex[1]);
  nh->param("LockJawOpenButton", buttonindex[2], buttonindex[2]);
  nh->param("FixSlewButton", buttonindex[3], buttonindex[3]);
  nh->param("ParkButton", buttonindex[4], buttonindex[4]);


  desired_[0]=0.0;
  desired_[1]=0.15;
  desired_[2]=1.8;
  desired_[3]=0.0;
  desired_[4]=0.36;

}


MainWindow::~MainWindow() { }

/*****************************************************************************
** ARM TELEOPERATION
*****************************************************************************/

void MainWindow::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  //sensor_msgs::JointState current_js;
  js.name.resize(0);
  js.velocity.resize(0);

  if (joy->buttons[buttonindex[0]]==1 && lastValue_[0]!=1){
    slewLocked_=slewLocked_?false:true;
    ROS_INFO("Button %d pressed, Slew locked: %s", buttonindex[0], slewLocked_?"yes":"no" );
  }

  if (joy->buttons[buttonindex[1]]==1 && lastValue_[1]!=1){
    jawRotateLocked_=jawRotateLocked_?false:true;
    ROS_INFO("Button %d pressed, Jaw rotate locked: %s", buttonindex[1], jawRotateLocked_?"yes":"no" );
  }

  if (joy->buttons[buttonindex[2]]==1 && lastValue_[2]!=1){
    jawOpenLocked_=jawOpenLocked_?false:true;
    ROS_INFO("Button %d pressed, Jaw open locked: %s", buttonindex[2], jawOpenLocked_?"yes":"no" );
  }

  if (joy->buttons[buttonindex[3]]==1 && lastValue_[3]!=1){
    fixSlew_=fixSlew_?false:true;
    if(fixSlew_) park_=false;
    ROS_INFO("Button %d pressed, Fix slew angle: %s", buttonindex[3], fixSlew_?"yes":"no" );
  }

  if (joy->buttons[buttonindex[4]]==1 && lastValue_[4]!=1){
    park_=park_?false:true;
    if(park_)fixSlew_=false;
    ROS_INFO("Button %d pressed, Park: %s", buttonindex[4], park_?"yes":"no" );
  }


  if(current<CURRENT_THRESHOLD){
    if (park_){
      int okcount=0;int speed_=scale_;
      js.name.push_back(std::string("Slew"));
      if( (angles_[0]-desired_[0]) > error_){
        js.velocity.push_back(-speed_*(angles_[0]-desired_[0]));
      }else if ( (angles_[0]-desired_[0]) < -error_){
        js.velocity.push_back(-speed_*(angles_[0]-desired_[0]));
      }else{
        js.velocity.push_back(0);okcount++;
      }
      js.name.push_back(std::string("Shoulder"));
      if( (angles_[1]-desired_[1]) > error_){
        js.velocity.push_back(-speed_*(angles_[1]-desired_[1]));
      }else if ( (angles_[1]-desired_[1]) < -error_){
        js.velocity.push_back(-speed_*(angles_[1]-desired_[1]));
      }else{
        js.velocity.push_back(0);okcount++;
      }
      js.name.push_back(std::string("Elbow"));
      if( (angles_[2]-desired_[2]) > error_){
        js.velocity.push_back(speed_*(angles_[2]-desired_[2]));//This speed has the sign changed
      }else if ( (angles_[2]-desired_[2]) < -error_){
        js.velocity.push_back(speed_*(angles_[2]-desired_[2]));
      }else{
        js.velocity.push_back(0);okcount++;
      }
      js.name.push_back(std::string("JawRotate"));
      if( (angles_[3]-desired_[3]) > error_){
        js.velocity.push_back(-speed_/4*(angles_[3]-desired_[3]));
      }else if ( (angles_[3]-desired_[3]) < -error_){
        js.velocity.push_back(-speed_/4*(angles_[3]-desired_[3]));
      }else{
        js.velocity.push_back(0);okcount++;
      }
      js.name.push_back(std::string("JawOpening"));
      if( (angles_[4]-desired_[4]) > error_){
        js.velocity.push_back(-speed_*(angles_[4]-desired_[4]));
      }else if ( (angles_[4]-desired_[4]) < -error_){
        js.velocity.push_back(-speed_*(angles_[4]-desired_[4]));
      }else{
        js.velocity.push_back(0);okcount++;
      }
      if(okcount==5){
        park_=false;
        ROS_INFO("Parked with a precision of %3f", error_);
      }
    }else if (fixSlew_){
      js.name.push_back(std::string("Slew"));
      if(angles_[0]>-error_ && angles_[0]<error_){//Near 0
        fixSlew_=false;
        ROS_INFO("Slew is at 0 angles (+-%f)", error_);
        js.velocity.push_back(0);
      }else{
        if(angles_[0]>0)
          js.velocity.push_back(-2250);
        else
          js.velocity.push_back(2250);
      }
      js.name.push_back(std::string("Shoulder"));
      js.velocity.push_back(0);
      js.name.push_back(std::string("Elbow"));
      js.velocity.push_back(0);
      js.name.push_back(std::string("JawRotate"));
      js.velocity.push_back(0);
      js.name.push_back(std::string("JawOpening"));
      js.velocity.push_back(0);
    }else{
      js.name.push_back(std::string("Slew"));
      if(joy->axes[axisindex[0]]>0.2 || joy->axes[axisindex[0]]<-0.2){
        js.velocity.push_back(scale_*joy->axes[axisindex[0]]*AxisDir[0]*(slewLocked_?0.0:1.0));
      }
      else{
        js.velocity.push_back(0);
      }
      js.name.push_back(std::string("Shoulder"));
      js.velocity.push_back(scale_*joy->axes[axisindex[1]]*AxisDir[1]);

      js.name.push_back(std::string("Elbow"));

      js.velocity.push_back(scale_*joy->axes[axisindex[2]]*AxisDir[2]);

      js.name.push_back(std::string("JawRotate"));

      js.velocity.push_back(0.3*scale_*joy->axes[axisindex[3]]*AxisDir[3]*(jawRotateLocked_?0.0:1.0));

      js.name.push_back(std::string("JawOpening"));
      if (joy->axes[axisindex[4]] < joy->axes[axisindex[5]]) {
        if(joy->axes[axisindex[4]]!=0)
          js.velocity.push_back(scale_*((joy->axes[axisindex[4]]-1)/2)*AxisDir[4]);
        else
          js.velocity.push_back(0);
      } else if (joy->axes[axisindex[4]] > joy->axes[axisindex[5]]) {
        if(joy->axes[axisindex[5]]!=0)
          js.velocity.push_back(-scale_*((joy->axes[axisindex[5]]-1)/2)*AxisDir[4]*(jawOpenLocked_?0.0:1.0));
        else
          js.velocity.push_back(0);
      } else {
        js.velocity.push_back(0);
      }
    }
  }
  else{
    js.name.push_back(std::string("Slew"));
    js.velocity.push_back(0);
    js.name.push_back(std::string("Shoulder"));
    js.velocity.push_back(0);
    js.name.push_back(std::string("Elbow"));
    js.velocity.push_back(0);
    js.name.push_back(std::string("JawRotate"));
    js.velocity.push_back(0);
    js.name.push_back(std::string("JawOpening"));
    js.velocity.push_back(0);
  }

  //vel_pub_.publish(js);

  lastValue_[0]=joy->buttons[buttonindex[0]];
  lastValue_[1]=joy->buttons[buttonindex[1]];
  lastValue_[2]=joy->buttons[buttonindex[2]];
  lastValue_[3]=joy->buttons[buttonindex[3]];
  lastValue_[4]=joy->buttons[buttonindex[4]];
}

void MainWindow::armPublisher()
{
  if (ui.armTeleopCheckbox->isChecked())
    vel_pub_.publish(js);
}


void MainWindow::dredgingPublisher()
{/*
    cola2_msgs::Setpoints dredgingMsg;
    dredgingMsg.setpoints.resize(0);

    double value = round( ui.dredgingSpinBox->value() * -10.0 ) / 10.0;
    dredgingMsg.setpoints.push_back(value);
    pub_dredging.publish(dredgingMsg);*/

}


void MainWindow::armCallback(const sensor_msgs::JointState::ConstPtr& mes){
  current=mes->effort[0];
}

void MainWindow::armAngleCallback(const sensor_msgs::JointState::ConstPtr& mes){
  for(int i=0;i<5;i++) angles_[i]=mes->position[i];
}

void MainWindow::g500CameraCallback(const sensor_msgs::Image::ConstPtr& msg)
{
  QImage dest (msg->data.data(), msg->width, msg->height, QImage::Format_Indexed8);//_RGB888);
  g500Image = dest.copy();
  g500Pixmap = QPixmap::fromImage(g500Image);
  //if (g500CameraEnable)
  //	ui.g500StreamView->setPixmap(g500Pixmap);
  if (g500CameraEnable2)
    ui.g500StreamView2->setPixmap(g500Pixmap);
  //ui.g500StreamView2->se
}


/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}


void MainWindow::processSpinOnce()
{
  //We need to exectue the SpinOnce with the timer
  ros::spinOnce();
}


void MainWindow::g500TopicsButtonClicked()
{/*
  qDebug()<<"g500TopicsButton clicked: reconnecting all the G500 topics...";
  sub_g500Odometry.shutdown();
    sub_g500MergedBodyVel.shutdown();
    sub_g500MergedWorld.shutdown();
  sub_g500Battery.shutdown();
  sub_g500Runningtime.shutdown();
  sub_g500Diagnostics.shutdown();
  srv_g500GoTo.shutdown();
    sub_g500Camera.shutdown();
  qDebug()<<"g500 topics have been shutdown";
  sub_g500Odometry	  = nh->subscribe<auv_msgs::NavSts>(ui.g500TopicOdometry->text().toUtf8().constData(), 1, &MainWindow::g500OdometryCallback, this);
    sub_g500MergedBodyVel = nh->subscribe<auv_msgs::BodyVelocityReq>(ui.g500TopicMergedBodyVel->text().toUtf8().constData(), 1, &MainWindow::g500MergedBodyVelCallback, this);
    sub_g500MergedWorld   = nh->subscribe<auv_msgs::WorldWaypointReq>(ui.g500TopicMergedWorld->text().toUtf8().constData(), 1, &MainWindow::g500MergedWorldCallback, this);
  sub_g500Battery		  = nh->subscribe<cola2_msgs::BatteryLevel>(ui.g500TopicBatteryLevel->text().toUtf8().constData(), 1, &MainWindow::g500BatteryCallback, this);
  sub_g500Runningtime	  = nh->subscribe<cola2_msgs::TotalTime>(ui.g500TopicRunningTime->text().toUtf8().constData(), 1, &MainWindow::g500RunningTimeCallback, this);
  sub_g500Diagnostics	  = nh->subscribe<diagnostic_msgs::DiagnosticArray>(ui.g500TopicDiagnostics->text().toUtf8().constData(), 1, &MainWindow::g500DiagnosticsCallback, this);
  srv_g500GoTo 		  = nh->serviceClient<cola2_msgs::Goto>(ui.g500TopicGoToService->text().toUtf8().constData());
    image_transport::ImageTransport it(*nh);
    sub_g500Camera   = it.subscribe(ui.g500CameraTopic->text().toUtf8().constData(), 1, &MainWindow::g500CameraCallback, this);
  qDebug()<<"g500 topics have been reconnected";*/
}

void MainWindow::armTopicButtonClicked()
{
  qDebug()<<"armTopicsButton clicked: reconnecting all the G500 topics";
  sub_arm_state.shutdown();
  qDebug()<<"arm topic has been shutdown";
  sub_arm_state	= nh->subscribe<sensor_msgs::JointState>(ui.armTopic->text().toUtf8().constData(), 1, &MainWindow::armStateCallback, this);
  qDebug()<<"arm topic has been reconnected";
}


void MainWindow::g500LoadStream()
{

  g500CameraEnable = true;
}


void MainWindow::g500LoadStream2()
{

  g500CameraEnable2 = true;
}


void MainWindow::g500StopStream()
{
  g500CameraEnable = false;
  ui.g500StreamView->clear();
  ui.g500StreamView->setText("G500 camera input");
}

void MainWindow::g500StopStream2()
{
  g500CameraEnable2 = false;
  ui.g500StreamView2->clear();
  ui.g500StreamView2->setText("G500 camera input");
}

void MainWindow::g500GoToPositionButtonClicked()
{
  //This shows a QDialog to enter manually the robot desired position or get it from UWSim (ToDo)
  dlg->show();
}

void MainWindow::setParamsButtonClicked()
{
  dlg2->show();
}

void MainWindow::setRobotPosition(double xValueSrv, double yValueSrv, double zValueSrv, double rollValueSrv, double pitchValueSrv, double yawValueSrv)
{/*
    qDebug() << "Sending the G500 to the new position...";
    qDebug() << xValueSrv << " " << yValueSrv << " " << zValueSrv << " " << rollValueSrv << " " << pitchValueSrv << " " << yawValueSrv;

    QMessageBox msgBox;
    cola2_msgs::Goto srv;

    srv.request.position.x      = xValueSrv;
    srv.request.position.y      = yValueSrv;
    srv.request.position.z      = zValueSrv;
    srv.request.disable_axis.x  = false;
    srv.request.disable_axis.y  = true;
    srv.request.disable_axis.z  = false;
    srv.request.disable_axis.roll  = true;
    srv.request.disable_axis.pitch = true;
    srv.request.disable_axis.yaw   = false;
    srv.request.blocking        = false;
    srv.request.keep_position   = false;
    srv.request.position_tolerance.x = 0.4;
    srv.request.position_tolerance.y = 0.4;
    srv.request.position_tolerance.z = 0.4;
    srv.request.altitude_mode   = false;
    srv.request.priority        = 10;
    srv.request.reference       = srv.request.REFERENCE_NED;

    if(srv_g500GoTo.call(srv))
    {
        qDebug() << "Service call success.";
        msgBox.setText("Service call successfully.");
    }
    else
    {
        qDebug() << "Service call failed.";
        msgBox.setText("Service call failed.");
    }
    msgBox.exec();*/
}

void MainWindow::g500GoToSurface()
{/*
  qDebug() << "Sending the G500 to the surface...";

  cola2_msgs::Goto srv;
    QMessageBox msgBox;

    srv.request.position.z       = 0;
    srv.request.disable_axis.x   = true;
    srv.request.disable_axis.y   = true;
    srv.request.disable_axis.z   = false;
    srv.request.disable_axis.yaw = false;
    srv.request.blocking 		 = false;
    srv.request.keep_position	 = false;
    srv.request.position_tolerance.x = 0.4;
    srv.request.position_tolerance.y = 0.4;
    srv.request.position_tolerance.z = 0.4;
    srv.request.altitude_mode	 = false;
    srv.request.priority		 = 10;
    srv.request.reference 		 = srv.request.REFERENCE_NED;

    if(srv_g500GoTo.call(srv))
    {
        qDebug() << "Service call success.";
        msgBox.setText("Service call successfully.");
    }
    else
    {
        qDebug() << "Service call failed.";
        msgBox.setText("Service call failed.");
    }
    msgBox.exec();*/
}

void MainWindow::getInitGraspPose(){
  std_msgs::String msg;
  switch( ui.poseEstimationComboBox->currentIndex() ){
  case 0:
    msg.data = "initFromRansac";
    break;
  case 1:
    msg.data = "initFromSphere";
    break;
  case 2:
    msg.data = "initFromPCA";
    break;
  case 3:
    msg.data = "initFromBox";
    break;
  case 4:
    msg.data = "initFromSQ";
    break;
  case 5:
    msg.data = "initFromPose";
    break;
  }
  pub_spec_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::setSpecificationMode( int tab ){

  std_msgs::String msg;
  if( tab == 0 ){
    msg.data = "guided";
  }else{
    msg.data = "interactive";
  }
  pub_spec_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::updateInteractiveSpecParams(){
  std_msgs::Float32MultiArray msg;
  //Scale params. From int (deg fractions) to rad.
  msg.data.push_back(ui.interactiveSpecSlider1->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider2->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider3->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider4->value()*3.14/180/4);
  msg.data.push_back(ui.interactiveSpecSlider5->value()*3.14/180/4);
  msg.data.push_back(ui.interactiveSpecSlider6->value()*3.14/180/4);
  msg.data.push_back(ui.gripperOpeningSlider->value());
  pub_spec_params.publish(msg);
  ros::spinOnce();
}

void MainWindow::executeGrasping(){
  std_msgs::String msg;
  msg.data = "execute";
  pub_spec_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::executeDredging(){
  std_msgs::String msg;
  msg.data = "send_first";
  pub_dredg_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::addWaypoint(){
  std_msgs::String msg;
  msg.data = "add";
  pub_dredg_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::clearWaypoints(){
  std_msgs::String msg;
  msg.data = "clear";
  pub_dredg_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::removeLastWaypoint(){
  std_msgs::String msg;
  msg.data = "remove_last";
  pub_dredg_action.publish(msg);
  ros::spinOnce();
}

void MainWindow::updateAndResetInteractiveSpecParams(){
  std_msgs::Float32MultiArray msg;
  //Scale params. From int (deg fractions) to rad.
  msg.data.push_back(ui.interactiveSpecSlider1->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider2->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider3->value()/300.0);
  msg.data.push_back(ui.interactiveSpecSlider4->value()*3.14/180/4);
  msg.data.push_back(ui.interactiveSpecSlider5->value()*3.14/180/4);
  msg.data.push_back(ui.interactiveSpecSlider6->value()*3.14/180/4);
  pub_spec_params.publish(msg);

  std_msgs::String str_msg;
  str_msg.data = "markerReset";
  pub_spec_action.publish(str_msg);

  ros::spinOnce();

  ui.interactiveSpecSlider1->setValue(0);
  ui.interactiveSpecSlider2->setValue(0);
  ui.interactiveSpecSlider3->setValue(0);
  ui.interactiveSpecSlider4->setValue(0);
  ui.interactiveSpecSlider5->setValue(0);
  ui.interactiveSpecSlider6->setValue(0);
}

void MainWindow::updateGuidedSpecParams(){

  ui.guidedSpecSpin1->setValue(ui.guidedSpecSlider1->value());
  ui.guidedSpecSpin2->setValue(ui.guidedSpecSlider2->value());
  ui.guidedSpecSpin3->setValue(ui.guidedSpecSlider3->value());

  std_msgs::Float32MultiArray msg;
  msg.data.push_back(ui.guidedSpecSlider1->value());
  msg.data.push_back(ui.guidedSpecSlider2->value());
  msg.data.push_back(ui.guidedSpecSlider3->value());
  msg.data.push_back(ui.gripperOpeningSlider->value());
  pub_spec_params.publish(msg);
  ros::spinOnce();
}

void MainWindow::updateGuidedSpecParamsSpin1(){

  ui.guidedSpecSlider1->setValue(ui.guidedSpecSpin1->value());
  publishSliders();

}
void MainWindow::updateGuidedSpecParamsSpin2(){

  ui.guidedSpecSlider2->setValue(ui.guidedSpecSpin2->value());
  publishSliders();

}
void MainWindow::updateGuidedSpecParamsSpin3(){

  ui.guidedSpecSlider3->setValue(ui.guidedSpecSpin3->value());
  publishSliders();

}

void MainWindow::updateGraspNumberSpinBox(){

  grasp_id_changed = true;
  grasp_id_changed2 = true;
  score_description = ui.graspScoreDetail->toPlainText();
  metrics = ui.graspMetrics->toPlainText();

  std_msgs::Int8 msg;
  msg.data = ui.graspNumberSpinBox->value();
  pub_grasp_id.publish( msg );
  ros::spinOnce();

}



void MainWindow::publishSliders(){

  std_msgs::Float32MultiArray msg;
  msg.data.push_back(ui.guidedSpecSlider1->value());
  msg.data.push_back(ui.guidedSpecSlider2->value());
  msg.data.push_back(ui.guidedSpecSlider3->value());
  msg.data.push_back(ui.gripperOpeningSlider->value());
  pub_spec_params.publish(msg);
  ros::spinOnce();

}

/*****************************************************************************
** Implemenation [Callbacks]
*****************************************************************************/

float MainWindow::rad2grad(double rads)
{
  //Converts radians to grads
  float roundedRad = roundf( (rads*180/M_PI) * 100) /100;
  return roundedRad;
}

float MainWindow::trun2dec(double value)
{
  //Converts doubles to float with 2 decimals
  float roundedValue = roundf(value * 100) / 100;
  return roundedValue;
}

void MainWindow::specParamsCallback(const std_msgs::Float32MultiArrayConstPtr& specificationParams){
  //Used to receive defaults.
  if(specificationParams->data.size()==4){
    ui.guidedSpecSlider1->setValue( specificationParams->data[0] );
    ui.guidedSpecSlider2->setValue( specificationParams->data[1] );
    ui.guidedSpecSlider3->setValue( specificationParams->data[2] );
    ui.gripperOpeningSlider->setValue( specificationParams->data[3] );

    ui.guidedSpecSpin2->setValue(ui.guidedSpecSlider2->value());
    ui.guidedSpecSpin1->setValue(ui.guidedSpecSlider1->value());
    ui.guidedSpecSpin3->setValue(ui.guidedSpecSlider3->value());
  }
}

void MainWindow::scoreCallback(const std_msgs::Float32::ConstPtr& msg){

  std::ostringstream stringStream;
  stringStream << msg->data;
  ui.graspOverallScore->setText(QString::fromStdString( stringStream.str() ));
}

void MainWindow::listSizeCallback(const std_msgs::Int32::ConstPtr& msg){
  ui.graspNumberSpinBox->setMaximum(msg->data-1);
}


void MainWindow::scoreDescriptionCallback(const std_msgs::String::ConstPtr& msg){
  QString s;
  s = QString::fromStdString(msg->data);
  if( grasp_id_changed && score_description != s ){
    grasp_id_changed = false;
    ui.graspScoreDetail->setText(s);
  }
}

void MainWindow::metricsCallback(const std_msgs::String::ConstPtr& msg){
  QString s;
  s = QString::fromStdString(msg->data);
  if( grasp_id_changed2 && metrics != s ){
    grasp_id_changed2 = false;
    ui.graspMetrics->setText(s);
  }
}

void MainWindow::armStateCallback(const sensor_msgs::JointState::ConstPtr& armStateMsg)
{
  QString armEffort = QString::number(armStateMsg->effort[0]);
  ui.armCurrentLimitLabel->setText("Arm current limit: " + armEffort.mid(0,5));

  double value = armStateMsg->position[0];
  if (value < 0)
    value = (value / -1.571) * 100;
  else
    value = (value / 0.549) * 100;
  ui.slewProgressBar->setValue(value);
  if (value < 60)
    ui.slewProgressBar->setStyleSheet("::chunk {background-color: green}");
  else
    if (value < 80)
      ui.slewProgressBar->setStyleSheet("::chunk {background-color: orange}");
    else
      ui.slewProgressBar->setStyleSheet("::chunk {background-color: red}");

  value = (armStateMsg->position[1] / 1.587) * 100;
  ui.shoulderProgressBar->setValue(value);
  if (value < 60)
    ui.shoulderProgressBar->setStyleSheet("::chunk {background-color: green}");
  else
    if (value < 80)
      ui.shoulderProgressBar->setStyleSheet("::chunk {background-color: orange}");
    else
      ui.shoulderProgressBar->setStyleSheet("::chunk {background-color: red}");

  value = (armStateMsg->position[2] / 2.153) * 100;
  ui.elbowProgressBar->setValue(value);
  if (value < 60)
    ui.elbowProgressBar->setStyleSheet("::chunk {background-color: green}");
  else
    if (value < 80)
      ui.elbowProgressBar->setStyleSheet("::chunk {background-color: orange}");
    else
      ui.elbowProgressBar->setStyleSheet("::chunk {background-color: red}");

  value = ((armStateMsg->position[3] + M_PI/2) /M_PI) * 100;
  ui.jawRotateProgressBar->setValue(value);
  if (value < 60)
    ui.jawRotateProgressBar->setStyleSheet("::chunk {background-color: green}");
  else
    if (value < 80)
      ui.jawRotateProgressBar->setStyleSheet("::chunk {background-color: orange}");
    else
      ui.jawRotateProgressBar->setStyleSheet("::chunk {background-color: red}");

  value = (armStateMsg->position[4] / 1.338) * 100;
  ui.jawOpeningProgressBar->setValue(value);
  if (value < 60)
    ui.jawOpeningProgressBar->setStyleSheet("::chunk {background-color: green}");
  else
    if (value < 80)
      ui.jawOpeningProgressBar->setStyleSheet("::chunk {background-color: orange}");
    else
      ui.jawOpeningProgressBar->setStyleSheet("::chunk {background-color: red}");

}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."),tr("<h2>Manipulation GUI</h2> \
                                              <p>Copyright IRSLab - Universitat Jaume I (<a href='mailto:irslab@uji.es'>irslab@uji.es</a>)</p> \
                                              <p>This GUI will control the most relevant options to manipulate objects.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::closeEvent(QCloseEvent *event)
{

  QMainWindow::closeEvent(event);
}

}  // namespace merbots_gui

