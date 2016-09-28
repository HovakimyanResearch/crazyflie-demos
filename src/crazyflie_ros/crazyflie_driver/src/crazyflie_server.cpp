//#define USE_Add_Logging
#define USE_KF
#define dist_thrshld  0.3

#include "ros/ros.h"
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"  //YHJ <- by Arun
#include "geometry_msgs/TransformStamped.h" //YHJ: the vicon_bridge sends the msg type, TransformStamped.
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt16.h"  //YHJ <- by Arun
#include "std_msgs/Int16.h" //YHJ
#include "std_msgs/Int32.h" //YHJ
#include <Eigen/Dense>   // YHJ: This is for matrix arithmatics.
#include <cmath>


//#include <regex>
#include <thread>
#include <mutex>

#include <crazyflie_cpp/Crazyflie.h>
using namespace Eigen; // YHJ: This is for matrix arithmatics.

constexpr double pi() { return std::atan(1)*4; }

double degToRad(double deg) {
    return deg / 180.0 * pi();
}

double radToDeg(double rad) {
    return rad * 180.0 / pi();
}

class CrazyflieROS
{
public:
  CrazyflieROS(
    const std::string& link_uri,
    const std::string& tf_prefix,
    float roll_trim,
    float pitch_trim,
    bool enable_logging,
    bool enable_parameters,
    std::vector<crazyflie_driver::LogBlock>& log_blocks,
    bool use_ros_time,
    bool enable_logging_imu,
    bool enable_logging_temperature,
    bool enable_logging_magnetic_field,
    bool enable_logging_pressure,
    bool enable_logging_battery)
    : m_cf(link_uri)
    , m_tf_prefix(tf_prefix)
    , m_isEmergency(false)
    , m_isViconInitialized(false) //YHJ
    , m_reboot_reqested(false) //YHJ
    , m_roll_trim(roll_trim)
    , m_pitch_trim(pitch_trim)
    , m_enableLogging(enable_logging)
    , m_enableParameters(enable_parameters)
    , m_logBlocks(log_blocks)
    , m_use_ros_time(use_ros_time)
    , m_enable_logging_imu(enable_logging_imu)
    , m_enable_logging_temperature(enable_logging_temperature)
    , m_enable_logging_magnetic_field(enable_logging_magnetic_field)
    , m_enable_logging_pressure(enable_logging_pressure)
    , m_enable_logging_battery(enable_logging_battery)
    , m_serviceEmergency()
    , m_serviceUpdateParams()
    , m_subscribeCmdVel()
    , m_subscribeViconPos() //YHJ: Subscribing a Topic from Vion Bridge
    , m_pubImu()
    , m_pubTemp()
    , m_pubMag()
    , m_pubPressure()
    , m_pubBattery()
    //YHJ begin
    //YHJ:  I added some debugging topics which publish logging variables defined in the crazyflie-firmware.
#ifdef USE_Add_Logging
    , m_pubDebug_1()
    , m_pubDebug_2()
    , m_pubDebug_3()
    , m_pubDebug_4()
    , m_pubDebug_5()
    , m_pubDebug_6()
    , m_pubDebug_7()
    , m_pubDebug_8()
    , m_pubDebug_9()
    , m_pubDebug_10()
    , m_pubDebug_11()
    , m_pubDebug_12()
    , m_pubDebug_13()
    , m_pubDebug_14()
    , m_pubDebug_15()
    , m_pubDebug_16()
    , m_pubDebug_17()
    , m_pubDebug_18()
    , m_pubDebug_19()
    , m_pubDebug_20()
    , m_pubDebug_21()
    , m_pubDebug_22()
#endif /* USE_Add_Logging */
    //YHJ end


    , m_pubRssi()
    , m_sentSetpoint(false)
    , m_previousTime(ros::Time::now()) // YHJ: This is used to calculate dt for Kalman Filter.
  {
    ros::NodeHandle n;
//YHJ begin
    m_subscribeCmdPath = n.subscribe(tf_prefix + "/cmd_path", 1, &CrazyflieROS::cmdPathChanged, this);
   // m_subscribeCmdVel = n.subscribe(tf_prefix + "/cmd_vel", 1, &CrazyflieROS::cmdVelChanged, this);
   // m_subscribeJoyVel = n.subscribe(tf_prefix + "/joy", 1, &CrazyflieROS::cmdJoyChanged, this);
    m_subscribeViconPos = n.subscribe("/vicon" + tf_prefix + tf_prefix, 1, &CrazyflieROS::viconPosChanged, this);
    // YHJ: The vicon bridge publish topic in this form, '/vicion/percy/percy'.
    // tf_prefix should be the name of object in Vicon Tracker.
    m_serviceInitializeViconPos = n.advertiseService(tf_prefix + "/initialize_vicon_pos", &CrazyflieROS::viconPosInitialized, this);
    m_reboot = n.advertiseService(tf_prefix + "/reboot", &CrazyflieROS::reboot, this);
//YHJ end    
    m_serviceEmergency = n.advertiseService(tf_prefix + "/emergency", &CrazyflieROS::emergency, this);
    m_serviceUpdateParams = n.advertiseService(tf_prefix + "/update_params", &CrazyflieROS::updateParams, this);

    if (m_enable_logging_imu) {
      m_pubImu = n.advertise<sensor_msgs::Imu>(tf_prefix + "/imu", 10);
    }
    if (m_enable_logging_temperature) {
      m_pubTemp = n.advertise<sensor_msgs::Temperature>(tf_prefix + "/temperature", 10);
    }
    if (m_enable_logging_magnetic_field) {
      m_pubMag = n.advertise<sensor_msgs::MagneticField>(tf_prefix + "/magnetic_field", 10);
    }
    if (m_enable_logging_pressure) {
      m_pubPressure = n.advertise<std_msgs::Float32>(tf_prefix + "/pressure", 10);
    }
    if (m_enable_logging_battery) {
      m_pubBattery = n.advertise<std_msgs::Float32>(tf_prefix + "/battery", 10);
    }
#ifdef USE_Add_Logging
    //YHJ begin
    // YHJ: These are debuging varialbes added.
    m_pubDebug_1  = n.advertise<std_msgs::Float32>(tf_prefix + "/position/x_tgt", 10);
    m_pubDebug_2  = n.advertise<std_msgs::Float32>(tf_prefix + "/position/y_tgt", 10);
    m_pubDebug_3  = n.advertise<std_msgs::Float32>(tf_prefix + "/position/z_tgt", 10);
    m_pubDebug_4  = n.advertise<std_msgs::Float32>(tf_prefix + "/position/x_est", 10);
    m_pubDebug_5  = n.advertise<std_msgs::Float32>(tf_prefix + "/position/y_est", 10);
    m_pubDebug_6  = n.advertise<std_msgs::Float32>(tf_prefix + "/position/z_est", 10); 
    m_pubDebug_7  = n.advertise<std_msgs::Float32>(tf_prefix + "/velocity/x_tgt", 10);
    m_pubDebug_8  = n.advertise<std_msgs::Float32>(tf_prefix + "/velocity/y_tgt", 10);
    m_pubDebug_9  = n.advertise<std_msgs::Float32>(tf_prefix + "/velocity/z_tgt", 10);
    m_pubDebug_10  = n.advertise<std_msgs::Float32>(tf_prefix + "/velocity/x_est", 10);
    m_pubDebug_11  = n.advertise<std_msgs::Float32>(tf_prefix + "/velocity/y_est", 10);
    m_pubDebug_12  = n.advertise<std_msgs::Float32>(tf_prefix + "/velocity/z_est", 10); 
    m_pubDebug_13  = n.advertise<std_msgs::Int16>(tf_prefix + "/sys_id/motor_roll", 10);
    m_pubDebug_14  = n.advertise<std_msgs::Int16>(tf_prefix + "/sys_id/motor_pitch", 10);
    m_pubDebug_15  = n.advertise<std_msgs::Int16>(tf_prefix + "/sys_id/motor_yaw", 10);
    m_pubDebug_16  = n.advertise<std_msgs::Float32>(tf_prefix + "/sys_id/imu_rate_x", 10);
    m_pubDebug_17  = n.advertise<std_msgs::Float32>(tf_prefix + "/sys_id/imu_rate_y", 10);
    m_pubDebug_18  = n.advertise<std_msgs::Float32>(tf_prefix + "/sys_id/imu_rate_z", 10); 
    m_pubDebug_19  = n.advertise<std_msgs::Int32>(tf_prefix + "/motor/m1", 10); 
    m_pubDebug_20  = n.advertise<std_msgs::Int32>(tf_prefix + "/motor/m2", 10); 
    m_pubDebug_21  = n.advertise<std_msgs::Int32>(tf_prefix + "/motor/m3", 10); 
    m_pubDebug_22  = n.advertise<std_msgs::Int32>(tf_prefix + "/motor/m4", 10); 
    //YHJ end
#endif /* USE_Add_Logging */
    m_pubRssi = n.advertise<std_msgs::Float32>(tf_prefix + "/rssi", 10);

    for (auto& logBlock : m_logBlocks)
    {
      m_pubLogDataGeneric.push_back(n.advertise<crazyflie_driver::GenericLogData>(tf_prefix + "/" + logBlock.topic_name, 10));
    }

    std::thread t(&CrazyflieROS::run, this);
    t.detach();
  }

private:
  struct logImu {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
  } __attribute__((packed));

  struct log2 {
    float mag_x;
    float mag_y;
    float mag_z;
    float baro_temp;
    float baro_pressure;
    float pm_vbat;
  } __attribute__((packed));

#ifdef USE_Add_Logging
  //YHJ begin
  // YHJ: These are for debuging varialbes added.
  struct log3 {
    float debug_1;
    float debug_2;
    float debug_3;
    float debug_4;
    float debug_5;
    float debug_6;
  } __attribute__((packed));

  struct log4 {
    float debug_7;
    float debug_8;
    float debug_9;
    float debug_10;
    float debug_11;
    float debug_12;
  } __attribute__((packed));

  struct log5 {
    int16_t debug_13;
    int16_t debug_14;
    int16_t debug_15;
    float debug_16;
    float debug_17;
    float debug_18;
  } __attribute__((packed));

  struct log6 {
    int32_t debug_19;
    int32_t debug_20;
    int32_t debug_21;
    int32_t debug_22;
  } __attribute__((packed));
   // YHJ end
#endif /* USE_Add_Logging */

 
  //YHJ begin
  // State varibale in Kalman Filtering
  // State estimate <- this initial value can be tuned if you need.
  float state_x_hat_k[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

  // Sigma matrix <- this initial value can be tuned if you need.
  float sigma_k[36] = {
    1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f,
    0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f
  };

  // To calculate dt in each time step.
  ros::Time m_previousTime;
  //YHJ end

  // To find initial offset
  float x_initial = 0.0f;
  float y_initial = 0.0f;
  float z_initial = 0.0f;

private:

  bool emergency(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_FATAL("Emergency requested!");
    m_isEmergency = true;

    return true;
  }

  //YHJ begin

  bool viconPosInitialized(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_INFO("vicon position initialization requested!");
    m_isViconInitialized = true;

    return true;
  }

  bool reboot(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    ROS_INFO("reboot!");
    m_reboot_reqested = true;

    return true;
  }

  //YHJ end


  template<class T, class U>
  void updateParam(uint8_t id, const std::string& ros_param) {
      U value = 1;
      ros::param::get(ros_param, value);
      m_cf.setParam<T>(id, (T)value);
  }

  bool updateParams(
    crazyflie_driver::UpdateParams::Request& req,
    crazyflie_driver::UpdateParams::Response& res)
  {
    ROS_INFO("Update parameters");

    for (auto&& p : req.params) {
      std::string ros_param = "/" + m_tf_prefix + "/" + p;
      ROS_INFO_STREAM(p);
      ROS_INFO_STREAM(ros_param); //YHJ test
      size_t pos = p.find("/");
      std::string group(p.begin(), p.begin() + pos);
      std::string name(p.begin() + pos + 1, p.end());


      auto entry = m_cf.getParamTocEntry(group, name);
      if (entry)
      {
        switch (entry->type) {
          case Crazyflie::ParamTypeUint8:
            updateParam<uint8_t, int>(entry->id, ros_param);
            //ROS_INFO("request value: %ld", (long int)req.value);
            //ROS_INFO("case1, rosparam: %s", ros_param);
            break;
          case Crazyflie::ParamTypeInt8:
            updateParam<int8_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint16:
            updateParam<uint16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt16:
            updateParam<int16_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeUint32:
            updateParam<uint32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeInt32:
            updateParam<int32_t, int>(entry->id, ros_param);
            break;
          case Crazyflie::ParamTypeFloat:
            updateParam<float, float>(entry->id, ros_param);
            break;
        }
      }
      else {
        ROS_ERROR("Could not find param %s/%s", group.c_str(), name.c_str());
      }
    }
    return true;
  }

    void cmdPathChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      float tgtX = msg->linear.x;
      float tgtY = msg->linear.y;
      float tgtZ = msg->linear.z;
      float tgtvX = msg->angular.x;
      float tgtvY = msg->angular.y;
      float tgtvZ = msg->angular.z;

      m_cf.sendPFcmd(tgtX, tgtY, tgtZ, tgtvX, tgtvY, tgtvZ);
      ROS_INFO("PF Cmd  tgtX:%f, tgtY:%f, tgtZ:%f, tgtvX:%f, tgtvY:%f, tgtvZ:%f is sent.", tgtX, tgtY, tgtZ, tgtvX, tgtvY, tgtvZ); //YHJ

      m_sentSetpoint = true;
    }
  }

  void cmdJoyChanged(
    const sensor_msgs::Joy::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      float roll = -10.0f*(msg->axes[0]) + m_roll_trim;
      float pitch = -(10.0f*(msg->axes[1]) + m_pitch_trim);
      float yawrate = 10.0f*(msg->axes[2]);
      uint16_t thrust = std::max<uint16_t>(std::min<float>(60000*(msg->axes[3]+1.0), 120000.0), 0.0);

      m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
      //ROS_INFO("SetPoint Sent.");
      m_sentSetpoint = true;
    }
  }

    void cmdVelChanged(
    const geometry_msgs::Twist::ConstPtr& msg)
  {
    if (!m_isEmergency) {
      float roll = msg->linear.y + m_roll_trim;
      float pitch = - (msg->linear.x + m_pitch_trim);
      float yawrate = msg->angular.z;
      uint16_t thrust = std::min<uint16_t>(std::max<float>(msg->linear.z, 0.0), 60000);

      m_cf.sendSetpoint(roll, pitch, yawrate, thrust);
      //ROS_INFO("SetPoint Sent.");
      m_sentSetpoint = true;
    }
  }

// YHJ begin



  //YHJ: The following function is called when it receives vicon_data.
  //So I implemented a standard discrete Kalman filtering with
  //a single integrator model.
 
  void viconPosChanged(
    const geometry_msgs::TransformStamped::ConstPtr& msg)
  { 
    if (m_isViconInitialized) {

      x_initial = msg->transform.translation.x;
      y_initial = msg->transform.translation.y;
      z_initial = msg->transform.translation.z;

      m_isViconInitialized = false;
    }

    if (m_reboot_reqested){
        m_cf.reboot(); 
        m_reboot_reqested = false;
    }


    if (!m_isEmergency) {

#ifdef USE_KF

      float x = msg->transform.translation.x - x_initial; 
      float y = msg->transform.translation.y - y_initial; 
      float z = msg->transform.translation.z - z_initial;

      /*

      if(fabs(x - state_x_hat_k[0]) >= dist_thrshld || fabs(y - state_x_hat_k[1]) >= dist_thrshld || fabs(z - state_x_hat_k[2]) >= dist_thrshld){
        x = state_x_hat_k[0];
        y = state_x_hat_k[1];
        z = state_x_hat_k[2];
      }
      */

      ros::Time time = ros::Time::now();
      float time_now = time.toSec();

      // Calculatnig dt, time-step
      // ros::Time time = ros::Time::now();
      float dt = time.toSec() - m_previousTime.toSec(); // <- sample time

      // Using Eigen, put measurment(vicon_data) in matix form.
      MatrixXf vec_y_k(3,1);
      vec_y_k(0,0) = x;
      vec_y_k(1,0) = y;
      vec_y_k(2,0) = z;


      // Matrix Fk <- the model(transitional) matrix
      //= | 1 0 0 dt 0 0 |
      //  | 0 1 0 0 dt 0 |
      //  | 0 0 1 0  0 dt|
      //  | 0 0 0 1  0 0 |
      //  | 0 0 0 0  1 0 |
      //  | 0 0 0 0  0 1 |

      MatrixXf mat_F_k(6,6); //Using Eigen
      mat_F_k.setIdentity();
      mat_F_k.topRightCorner(3, 3) = MatrixXf::Identity(3, 3)*dt;
      

      // Matrix Hk_transpose <- the mesurment matrix
      //= | 1 0 0 0 0 0 |
      //  | 0 1 0 0 0 0 |
      //  | 0 0 1 0 0 0 |
      MatrixXf mat_H_T_k(3,6); //Using Eigen
      mat_H_T_k.setZero();
      mat_H_T_k(0,0) = 1.0f;
      mat_H_T_k(1,1) = 1.0f;
      mat_H_T_k(2,2) = 1.0f;


      // Matrix Qk <- process noise covariance matrix, this is a tunning parameter.
      //= | 1 0 0 0 0 0 |               
      //  | 0 1 0 0 0 0 |
      //  | 0 0 1 0 0 0 | * q
      //  | 0 0 0 1 0 0 |
      //  | 0 0 0 0 1 0 |
      //  | 0 0 0 0 0 1 |
      MatrixXf mat_Q_k(6,6);
      mat_Q_k = MatrixXf::Identity(6, 6)*1.0;  //<- you can increase this for greater filtering of process noise.


      // Matrix Rk  <- sensor noise covariance matrix, this is a tunning parameter.
      //= | 1 0 0 |
      //  | 0 1 0 | * r
      //  | 0 0 1 | 
      MatrixXf mat_R_k(3,3);
      mat_R_k = MatrixXf::Identity(3, 3)*100.0; //<- you can increase this for greater filtering of sensor noise. 

      /// Input data ///
      
      // Initialize vector_state_x_hat_k
      float *ptr_state_x_hat_k;
      ptr_state_x_hat_k = state_x_hat_k;
      MatrixXf vec_state_x_hat_k = Map<MatrixXf>(ptr_state_x_hat_k, 6,1);

      // Initialize matrix_sigma_k
      float *ptr_sigma_k;
      ptr_sigma_k = sigma_k;
      MatrixXf mat_sigma_k = Map<MatrixXf>(ptr_sigma_k, 6,6);

      // Calculate gain matrix K
      MatrixXf mat_K_k(6,3);
      MatrixXf mat_H_sigma_H_R(3,3);
      mat_H_sigma_H_R = mat_H_T_k * mat_sigma_k * mat_H_T_k.transpose() + mat_R_k;
      mat_K_k = mat_F_k * mat_sigma_k * mat_H_T_k.transpose() * mat_H_sigma_H_R.inverse();

      // Update mat_sigma_k
      mat_sigma_k = mat_F_k * ( mat_sigma_k - mat_sigma_k * mat_H_T_k.transpose() * mat_H_sigma_H_R.inverse() * mat_H_T_k * mat_sigma_k ) * mat_F_k.transpose() + mat_Q_k;

      // The Estimate
      vec_state_x_hat_k = mat_F_k * vec_state_x_hat_k + mat_K_k * (vec_y_k - mat_H_T_k * vec_state_x_hat_k);

      // Update the Sigma array and the Estimate array
      Map<MatrixXf>( ptr_state_x_hat_k, vec_state_x_hat_k.rows(), vec_state_x_hat_k.cols() ) = vec_state_x_hat_k;
      Map<MatrixXf>( ptr_sigma_k, mat_sigma_k.rows(), mat_sigma_k.cols() ) = mat_sigma_k;

      x = state_x_hat_k[0];
      y = state_x_hat_k[1];
      z = state_x_hat_k[2];

      float vx = state_x_hat_k[3];
      float vy = state_x_hat_k[4];
      float vz = state_x_hat_k[5];

      //ROS_INFO("KF est x:%f, y:%f, z:%f, vx:%f, vy:%f, vz:%f, dt:%f .", x, y, z, vx, vy, vz, dt); //YHJ

      m_previousTime = time;

      m_cf.sendViconKFPos(x, y, z, vx, vy, vz);
      ROS_INFO("ViconPosition %f, %f, %f, %f, %f, %f are sent.", x, y, z, vx, vy, vz); //YHJ
      m_sentSetpoint = true; 
#else
      float x = msg->transform.translation.x - x_initial; 
      float y = msg->transform.translation.y - y_initial; 
      float z = msg->transform.translation.z - z_initial;
      m_cf.sendViconKFPos(x, y, z, 0.0, 0.0, 0.0);
      ROS_INFO("ViconPosition %f, %f, %f, 0, 0, 0 are sent.", x, y, z); //YHJ
      m_sentSetpoint = true; 

#endif /*USE_KF*/

    }
  }
// YHJ end

  void run()
  {
    m_cf.reboot();

    m_cf.logReset();

    std::function<void(float)> cb_lq = std::bind(&CrazyflieROS::onLinkQuality, this, std::placeholders::_1);
    m_cf.setLinkQualityCallback(cb_lq);

    auto start = std::chrono::system_clock::now();

    if (m_enableParameters)
    {
      ROS_INFO("Requesting parameters...");
      m_cf.requestParamToc();
      for (auto iter = m_cf.paramsBegin(); iter != m_cf.paramsEnd(); ++iter) {
        auto entry = *iter;
        std::string paramName = "/" + m_tf_prefix + "/" + entry.group + "/" + entry.name;
        switch (entry.type) {
          case Crazyflie::ParamTypeUint8:
            ros::param::set(paramName, m_cf.getParam<uint8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt8:
            ros::param::set(paramName, m_cf.getParam<int8_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint16:
            ros::param::set(paramName, m_cf.getParam<uint16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt16:
            ros::param::set(paramName, m_cf.getParam<int16_t>(entry.id));
            break;
          case Crazyflie::ParamTypeUint32:
            ros::param::set(paramName, (int)m_cf.getParam<uint32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeInt32:
            ros::param::set(paramName, m_cf.getParam<int32_t>(entry.id));
            break;
          case Crazyflie::ParamTypeFloat:
            ros::param::set(paramName, m_cf.getParam<float>(entry.id));
            break;
        }
      }
    }

    std::unique_ptr<LogBlock<logImu> > logBlockImu;
    std::unique_ptr<LogBlock<log2> > logBlock2;

#ifdef USE_Add_Logging
    // YHJ begin
    // YHJ: this is for monitoring debuging variables

    std::unique_ptr<LogBlock<log3> > logBlock3;
    std::unique_ptr<LogBlock<log4> > logBlock4;
    std::unique_ptr<LogBlock<log5> > logBlock5;
    std::unique_ptr<LogBlock<log6> > logBlock6;
    
    // YHJ end
#endif /* USE_Add_Logging */

    std::vector<std::unique_ptr<LogBlockGeneric> > logBlocksGeneric(m_logBlocks.size());
    if (m_enableLogging) {

      std::function<void(const crtpPlatformRSSIAck*)> cb_ack = std::bind(&CrazyflieROS::onEmptyAck, this, std::placeholders::_1);
      m_cf.setEmptyAckCallback(cb_ack);

      ROS_INFO("Requesting Logging variables...");
      m_cf.requestLogToc();

      if (m_enable_logging_imu) {
        std::function<void(uint32_t, logImu*)> cb = std::bind(&CrazyflieROS::onImuData, this, std::placeholders::_1, std::placeholders::_2);

        logBlockImu.reset(new LogBlock<logImu>(
          &m_cf,{
            {"acc", "x"},
            {"acc", "y"},
            {"acc", "z"},
            {"gyro", "x"},
            {"gyro", "y"},
            {"gyro", "z"},
          }, cb));
        logBlockImu->start(10); // 100ms
      }

      if (   m_enable_logging_temperature
          || m_enable_logging_magnetic_field
          || m_enable_logging_pressure
          || m_enable_logging_battery)
      {
        std::function<void(uint32_t, log2*)> cb2 = std::bind(&CrazyflieROS::onLog2Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock2.reset(new LogBlock<log2>(
          &m_cf,{
            {"mag", "x"},
            {"mag", "y"},
            {"mag", "z"},
            {"baro", "temp"},
            {"baro", "pressure"},
            {"pm", "vbat"},
          }, cb2));
        logBlock2->start(10); // 100ms
      }

#ifdef USE_Add_Logging
      //YHJ begin
      //YHJ: you can select logging variables in the following.
      //YHJ: I think that this is very useful for debugging.
      //YHJ: I don't know why but it works upto 6 float numbers.
      //YHJ: Adding another float returns error.

      if (true)
      {
        std::function<void(uint32_t, log3*)> cb3 = std::bind(&CrazyflieROS::onLog3Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock3.reset(new LogBlock<log3>(
          &m_cf,{
            {"setpoint_tgt_value", "tgt_x"},
            {"setpoint_tgt_value", "tgt_y"},
            {"setpoint_tgt_value", "tgt_z"},
            {"crtlmeasure", "pos_x"},
            {"crtlmeasure", "pos_y"},
            {"crtlmeasure", "pos_z"},
          }, cb3));
        logBlock3->start(10); // 100ms
      }

      if (true)
      {
        std::function<void(uint32_t, log4*)> cb4 = std::bind(&CrazyflieROS::onLog4Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock4.reset(new LogBlock<log4>(
          &m_cf,{
            {"setpoint_tgt_value", "tgt_vx"},
            {"setpoint_tgt_value", "tgt_vy"},
            {"setpoint_tgt_value", "tgt_vz"},
            {"crtlmeasure", "vel_x"},
            {"crtlmeasure", "vel_y"},
            {"crtlmeasure", "vel_z"},
          }, cb4));
        logBlock4->start(10); // 100ms
      }

      if (true)
      {
        std::function<void(uint32_t, log5*)> cb5 = std::bind(&CrazyflieROS::onLog5Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock5.reset(new LogBlock<log5>(
          &m_cf,{
            {"motor_cmd", "roll"},
            {"motor_cmd", "pitch"},
            {"motor_cmd", "yaw"},
            {"gyro", "x"},
            {"gyro", "y"},
            {"gyro", "z"},
          }, cb5));
        logBlock5->start(10); // 100ms
      }

      if (true)
      {
        std::function<void(uint32_t, log6*)> cb6 = std::bind(&CrazyflieROS::onLog6Data, this, std::placeholders::_1, std::placeholders::_2);

        logBlock6.reset(new LogBlock<log6>(
          &m_cf,{
            {"motor", "m1"},
            {"motor", "m2"},
            {"motor", "m3"},
            {"motor", "m4"},
          }, cb6));
        logBlock5->start(10); // 100ms
      }
      //YHJ end
#endif /* USE_Add_Logging */

      // custom log blocks
      size_t i = 0;
      for (auto& logBlock : m_logBlocks)
      {
        std::function<void(uint32_t, std::vector<double>*, void* userData)> cb =
          std::bind(
            &CrazyflieROS::onLogCustom,
            this,
            std::placeholders::_1,
            std::placeholders::_2,
            std::placeholders::_3);

        logBlocksGeneric[i].reset(new LogBlockGeneric(
          &m_cf,
          logBlock.variables,
          (void*)&m_pubLogDataGeneric[i],
          cb));
        logBlocksGeneric[i]->start(logBlock.frequency / 10);
        ++i;
      }


    }

    ROS_INFO("Ready...");
    auto end = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsedSeconds = end-start;
    ROS_INFO("Elapsed: %f s", elapsedSeconds.count());

    // Send 0 thrust initially for thrust-lock
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }

    while(!m_isEmergency) {
      // make sure we ping often enough to stream data out
      if (m_enableLogging && !m_sentSetpoint) {
        m_cf.sendPing();
      }
      m_sentSetpoint = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // Make sure we turn the engines off
    for (int i = 0; i < 100; ++i) {
       m_cf.sendSetpoint(0, 0, 0, 0);
    }
    
  }

  void onImuData(uint32_t time_in_ms, logImu* data) {
    if (m_enable_logging_imu) {
      sensor_msgs::Imu msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      msg.orientation_covariance[0] = -1;

      // measured in deg/s; need to convert to rad/s
      msg.angular_velocity.x = degToRad(data->gyro_x);
      msg.angular_velocity.y = degToRad(data->gyro_y);
      msg.angular_velocity.z = degToRad(data->gyro_z);

      // measured in mG; need to convert to m/s^2
      msg.linear_acceleration.x = data->acc_x * 9.81;
      msg.linear_acceleration.y = data->acc_y * 9.81;
      msg.linear_acceleration.z = data->acc_z * 9.81;

      m_pubImu.publish(msg);
    }
  }

  void onLog2Data(uint32_t time_in_ms, log2* data) {

    if (m_enable_logging_temperature) {
      sensor_msgs::Temperature msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";
      // measured in degC
      msg.temperature = data->baro_temp;
      m_pubTemp.publish(msg);
    }

    if (m_enable_logging_magnetic_field) {
      sensor_msgs::MagneticField msg;
      if (m_use_ros_time) {
        msg.header.stamp = ros::Time::now();
      } else {
        msg.header.stamp = ros::Time(time_in_ms / 1000.0);
      }
      msg.header.frame_id = m_tf_prefix + "/base_link";

      // measured in Tesla
      msg.magnetic_field.x = data->mag_x;
      msg.magnetic_field.y = data->mag_y;
      msg.magnetic_field.z = data->mag_z;
      m_pubMag.publish(msg);
    }

    if (m_enable_logging_pressure) {
      std_msgs::Float32 msg;
      // hPa (=mbar)
      msg.data = data->baro_pressure;
      m_pubPressure.publish(msg);
    }

    if (m_enable_logging_battery) {
      std_msgs::Float32 msg;
      // V
      msg.data = data->pm_vbat;
      float v_battery = data->pm_vbat;
      /*if(v_battery < 3.3) {
        ROS_FATAL("Battery is Too Low");
      }*/
      m_pubBattery.publish(msg);
    }
  }

#ifdef USE_Add_Logging
//YHJ begin
//YHJ: for adding that logging variables.
  void onLog3Data(uint32_t time_in_ms, log3* data) {

      std_msgs::Float32 msg;

      msg.data = data->debug_1;
      m_pubDebug_1.publish(msg);

      msg.data = data->debug_2;
      m_pubDebug_2.publish(msg);

      msg.data = data->debug_3;
      m_pubDebug_3.publish(msg);

      msg.data = data->debug_4;
      m_pubDebug_4.publish(msg);

      msg.data = data->debug_5;
      m_pubDebug_5.publish(msg);

      msg.data = data->debug_6;
      m_pubDebug_6.publish(msg);
    
  }

    void onLog4Data(uint32_t time_in_ms, log4* data) {

      std_msgs::Float32 msg;

      msg.data = data->debug_7;
      m_pubDebug_7.publish(msg);

      msg.data = data->debug_8;
      m_pubDebug_8.publish(msg);

      msg.data = data->debug_9;
      m_pubDebug_9.publish(msg);

      msg.data = data->debug_10;
      m_pubDebug_10.publish(msg);

      msg.data = data->debug_11;
      m_pubDebug_11.publish(msg);

      msg.data = data->debug_12;
      m_pubDebug_12.publish(msg);
    
  }

    void onLog5Data(uint32_t time_in_ms, log5* data) {

      std_msgs::Int16 msg_i;
      std_msgs::Float32 msg_f;

      msg_i.data = data->debug_13;
      m_pubDebug_13.publish(msg_i);

      msg_i.data = data->debug_14;
      m_pubDebug_14.publish(msg_i);

      msg_i.data = data->debug_15;
      m_pubDebug_15.publish(msg_i);

      msg_f.data = data->debug_16;
      m_pubDebug_16.publish(msg_f);

      msg_f.data = data->debug_17;
      m_pubDebug_17.publish(msg_f);

      msg_f.data = data->debug_18;
      m_pubDebug_18.publish(msg_f);
    
  }

   void onLog6Data(uint32_t time_in_ms, log6* data) {

      std_msgs::Int32 msg_i;
      //std_msgs::Float32 msg_f;

      msg_i.data = data->debug_19;
      m_pubDebug_19.publish(msg_i);

      msg_i.data = data->debug_20;
      m_pubDebug_20.publish(msg_i);

      msg_i.data = data->debug_21;
      m_pubDebug_21.publish(msg_i);

      msg_i.data = data->debug_22;
      m_pubDebug_22.publish(msg_i);

  }

//YHJ end
#endif /* USE_Add_Logging */

  void onLogCustom(uint32_t time_in_ms, std::vector<double>* values, void* userData) {

    ros::Publisher* pub = reinterpret_cast<ros::Publisher*>(userData);

    crazyflie_driver::GenericLogData msg;
    if (m_use_ros_time) {
      msg.header.stamp = ros::Time::now();
    } else {
      msg.header.stamp = ros::Time(time_in_ms / 1000.0);
    }
    msg.header.frame_id = m_tf_prefix + "/base_link";
    msg.values = *values;

    pub->publish(msg);
  }

  void onEmptyAck(const crtpPlatformRSSIAck* data) {
      std_msgs::Float32 msg;
      // dB
      msg.data = data->rssi;
      m_pubRssi.publish(msg);
  }

  void onLinkQuality(float linkQuality) {
      if (linkQuality < 0.7) {
        ROS_WARN("Link Quality low (%f)", linkQuality);
      }
  }

private:
  Crazyflie m_cf;
  std::string m_tf_prefix;
  bool m_isEmergency;
  bool m_isViconInitialized; //YHJ
  bool m_reboot_reqested; //YHJ
  float m_roll_trim;
  float m_pitch_trim;
  bool m_enableLogging;
  bool m_enableParameters;
  std::vector<crazyflie_driver::LogBlock> m_logBlocks;
  bool m_use_ros_time;
  bool m_enable_logging_imu;
  bool m_enable_logging_temperature;
  bool m_enable_logging_magnetic_field;
  bool m_enable_logging_pressure;
  bool m_enable_logging_battery;

  ros::ServiceServer m_serviceEmergency;
  ros::ServiceServer m_serviceUpdateParams;
  ros::ServiceServer m_serviceInitializeViconPos; //YHJ
  ros::ServiceServer m_reboot; //YHJ
  ros::Subscriber m_subscribeCmdVel;
  ros::Subscriber m_subscribeJoyVel;
  ros::Subscriber m_subscribeCmdPath;
  ros::Subscriber m_subscribeViconPos;  //YHJ
  ros::Publisher m_pubImu;
  ros::Publisher m_pubTemp;
  ros::Publisher m_pubMag;
  ros::Publisher m_pubPressure;
  ros::Publisher m_pubBattery;
#ifdef USE_Add_Logging
  //YHJ begin
  ros::Publisher m_pubDebug_1;
  ros::Publisher m_pubDebug_2;
  ros::Publisher m_pubDebug_3;
  ros::Publisher m_pubDebug_4;
  ros::Publisher m_pubDebug_5;
  ros::Publisher m_pubDebug_6;
  ros::Publisher m_pubDebug_7;
  ros::Publisher m_pubDebug_8;
  ros::Publisher m_pubDebug_9;
  ros::Publisher m_pubDebug_10;
  ros::Publisher m_pubDebug_11;
  ros::Publisher m_pubDebug_12;
  ros::Publisher m_pubDebug_13;
  ros::Publisher m_pubDebug_14;
  ros::Publisher m_pubDebug_15;
  ros::Publisher m_pubDebug_16;
  ros::Publisher m_pubDebug_17;
  ros::Publisher m_pubDebug_18;
  ros::Publisher m_pubDebug_19;
  ros::Publisher m_pubDebug_20;
  ros::Publisher m_pubDebug_21;
  ros::Publisher m_pubDebug_22;
  //YHJ end
  #endif /* USE_Add_Logging */
  ros::Publisher m_pubRssi;
  std::vector<ros::Publisher> m_pubLogDataGeneric;

  bool m_sentSetpoint;

};

bool add_crazyflie(
  crazyflie_driver::AddCrazyflie::Request  &req,
  crazyflie_driver::AddCrazyflie::Response &res)
{
  ROS_INFO("Adding %s as %s with trim(%f, %f). Logging: %d, Parameters: %d, Use ROS time: %d",
    req.uri.c_str(),
    req.tf_prefix.c_str(),
    req.roll_trim,
    req.pitch_trim,
    req.enable_parameters,
    req.enable_logging,
    req.use_ros_time);

  // Leak intentionally
  CrazyflieROS* cf = new CrazyflieROS(
    req.uri,
    req.tf_prefix,
    req.roll_trim,
    req.pitch_trim,
    req.enable_logging,
    req.enable_parameters,
    req.log_blocks,
    req.use_ros_time,
    req.enable_logging_imu,
    req.enable_logging_temperature,
    req.enable_logging_magnetic_field,
    req.enable_logging_pressure,
    req.enable_logging_battery);

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "crazyflie_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_crazyflie", add_crazyflie);
  ros::spin();

  return 0;
}
