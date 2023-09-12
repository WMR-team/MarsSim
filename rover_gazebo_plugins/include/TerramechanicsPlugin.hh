#ifndef _TERRAMECHANICSPLUGIN_HH_
#define _TERRAMECHANICSPLUGIN_HH_

#include <thread>
#include <queue>

#include <string>
// gazebo
// #include <sdf/sdf.hh>
#include <sdf/Element.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math.hh>
#include <ignition/math/Matrix4.hh>
// Eigen 
#include <Eigen/Core>
#include <Eigen/Geometry>

//ROS
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <ros/subscribe_options.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Wrench.h>
#include <std_msgs/Float64MultiArray.h>
// #include <dynamic_reconfigure/server.h>
// #include <rover_gazebo_plugins/DynamicTerrainConfig.h>
// #include <dynamic_reconfigure/Config.h>
// #include "dynamic_params/terrain_params.h"

namespace gazebo
{
	class TerramachanicsPlugin : public ModelPlugin
	{
        public: virtual ~TerramachanicsPlugin();
        public: TerramachanicsPlugin();
        //加载SDF文件，仿真时的主函数
        public: void virtual Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        public: void GetSDFParam(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        public: void InitTerrain();
        public: void virtual InitSubPub();
        //接收速度控制指令
        public: void OnRosMsg(const std_msgs::Float64ConstPtr &msg);
        //仿真迭代求解函数
        public: void OnUpdate();
        //结构体：定义车轮所受六维力
        public: struct FT
            {
                double Force[3];
                double Torque[3];
            };
        
        public: FT force_torque;
        public: ignition::math::Vector3d wheel_force;
        public: ignition::math::Vector3d wheel_torque;
        public: physics::JointWrench wrech;
        public: ignition::math::Vector3d force;
        public: ignition::math::Vector3d torque;
        
        //车轮六维力解算
        public: FT CalculationWheelForce(double Terrain_Data[], int iter);
        public: void virtual WheelTerrainInteraction(double theta11);
        public: void StaticModel(ignition::math::Matrix3d Rot_0_2);

        //结构体：定义轮地接触点处的地形高度值和地面力学参数值
        public: struct NodeZandTerrainParams
            {
                double NodeZ;
                double *TerrainParams;
            };
        //获取轮地接触点处的地形高度值和地面力学参数值
        public: NodeZandTerrainParams GetTouchNodes(double px, double py, double Terrain_Data[]);
        //结构体：轮地接触面法向量和平面上的一个点
        public: struct NormalandNode
            {
                ignition::math::Vector3d NormalVector;
                ignition::math::Vector3d NodePosition;
            };
        //计算轮地接触面法向量，同时返回平面上的一个点
        public: NormalandNode CalculateTouchArea(double WheelPos[3], double Terrain_Data[], double theta1_in, double phi_yaw);
        public: NormalandNode CalculateTouchArea(ignition::math::Matrix4d TransMat, double Terrain_Data[], double theta1_in);
        //计算车轮滑转率
        public: double CalculateSlip(double Angv, double Local_Lin_Velocity[3]);
        //计算车轮滑转率正负
        public: double CalculateS_flag(double s);
        //计算车轮侧偏角
        public: double CalculateBeta(double Local_Lin_Velocity[3]);
        //计算车轮侧偏角正负
        public: double CalculateBeta_flag(double beta);
        //计算车轮沉陷量
        public: double CalculationSinkage(ignition::math::Vector3d WheelPos, ignition::math::Vector3d P1, ignition::math::Vector3d Area, double sinkage_last);
        //计算车轮进入角
        public: double CalculationTheta1(double z_sinkage0, double theta1_last);
        //计算车轮等效半径
        public: double CalculationEqualRadius(double slip);
        //轮地接触时的阻尼(不太清楚)
        public: double LimitDampingCoef(double Linv[3]);
        //限制最大法向力
        public: double LimitSustainForce(double Fn_in);
        //计算考虑滑转时的车轮等效半径
        public: double CalculateRj(double s);
        //计算车体静止时车轮的摩擦力方向
        public: ignition::math::Vector3d CalculateFrictionDirectionNew(ignition::math::Vector3d normalVector, ignition::math::Vector3d WheelPos0, ignition::math::Vector3d WheelPos1);
        //
        public: void QueueThread();
        public: void QueueThread2();

        public: void GetSimulationParam();

        // void dynamic_callback(rover_gazebo_plugins::DynamicTerrainConfig &config);
        // void OnCallback(const dynamic_params::terrain_paramsConstPtr &msg);

        // public: void 
        //定义仿真所需的参数
        public:
            /// \brief 
            physics::ModelPtr model;

            /// \brief 
            event::ConnectionPtr updateConnection;

            /// \brief 
            physics::JointPtr WheelJoint;

            /// \brief 
            physics::LinkPtr WheelLink, BaseLink;

            /// \brief 
            std::vector<double> terrain_data;
            std::string robot_namespace_;

            /// \brief 
            bool enable_plugin = true;

            double c1 = 0.5;
            double c2 = -0.3;
            double c3 = -0.2;

            // double c_T1 = 0.2135;
            double c_T1 = 0.214;
            // double c_d1 = -0.378846*1.65 ;
            double c_d1 = -0.626;
            // double c_d2 = 0.616310 / 2;
            double c_d2 = 0.308;
            double c_d3 = -0.44800;
            // double c_d3 = -0.5;

            double Kc;
            double Kphi;
            double c;
            double phi;

            double K;
            double n0;
            double n1;

            double Force_T[3] = {0, 0, 0};
            double Torque_T[3] = {0, 0, 0};
            double Force_W[3] = {0, 0, 0};
            double Torque_W[3] = {0, 0, 0};

            std::unique_ptr<ros::NodeHandle> rosNode;
            ros::Subscriber rosSub;
            ros::Publisher rosPub_z;
            ros::Publisher rosPub_s;
            ros::Publisher rosPub_beta;
            ros::Publisher rosPub_FT;
            ros::CallbackQueue rosQueue;
            std::thread rosQueueThread;
            ros::Subscriber rosSub2;
            ros::CallbackQueue rosQueue2;
            std::thread rosQueueThread2;
            double theta_p;
            double control_v=0;
            std::string TerrainMapFileName;
            std::ifstream TerrainMapFile;
            /// \brief 
            /// \param[in]
            ///
            std::vector<double> TerrainDataVec;

            int SimulateCount=0;
            
            double SteerAngle=0;
            ignition::math::Vector3d WheelPos;
            ignition::math::Vector3d NormalVector;
            ignition::math::Vector3d NodePosition;
            double theta1_LastStep=0.01;
            ignition::math::Vector3d Velocity_Local;
            double AngleVelocity;
            double s;
            double beta;
            double sinkage;
            double sinkage_LastStep=0;
            double theta1;
            ignition::math::Vector3d WheelPos_Org;
            double Force_Local[3];
            double Force_Local_LastStep[3] = {0,0,0};
            std::string SubTopic;
            std::string Sinkage_PubTopic;
            std::string Slip_PubTopic;
            std::string WheelName;
            double Velocity_Local_Real[3];
            double x_step;
            double y_step;
            
            double x_0, y_0;
            int Node_Coef;
            int PublishRate;
            double sinkage_step=0.01;
            double sinkage_max=0.15;
            double FirstDampCoef=5000;
            double SecondDampCoef=5000;
            double Fn_Max=5000;
            double theta1_step=0.01;
            std_msgs::Float32 Sinkage_msg;
            std_msgs::Float32 Slip_msg;
            std_msgs::Float32 Beta_msg;
            geometry_msgs::Wrench Wheel_FT_msg;

            // default paramters
            double FN_AVE=220;                       
            double cmd_vel=0;

            double r = 0.14;
            double b = 0.15;
            double h = 0.01;
            // const double lambda = 0.65;
            // double h_lug = 0.01;
            // change for debug
            int PrintStep = 1000;
            int DTM_params = 10;
            double *TerrainData;

            int FrictionFlag = 0;
            double A, B, C;
            double sigma_m, tao_m, theta2, thetam;

            double x_max;
            double x_min;
            double y_max;
            double y_min;
            int x_grids;
            int y_grids;

            int TerrainParamsNum = 7;

            std::queue<double> vel_q;
            ignition::math::Vector3d forcelocal;
            ignition::math::Vector3d forceworld;
            ignition::math::Vector3d torquelocal;
            ignition::math::Vector3d torqueworld;
    };
}

#endif