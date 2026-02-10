#include <random>
#include <iostream>
#include <gazebo/gazebo_client.hh>
#include "ros/callback_queue.h"
#include "ros/subscription_callback_helper.h"
#include "std_msgs/Bool.h"
#include <gazebo/rendering/rendering.hh>
#include <ros/package.h>

#include "TerramechanicsPlugin.hh"

namespace gazebo
{
    TerramachanicsPlugin::TerramachanicsPlugin() {}
    TerramachanicsPlugin::~TerramachanicsPlugin()
    {
        delete[] TerrainData;
    }
    void TerramachanicsPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        // 读取sdf文件中的参数
        GetSDFParam(_model, _sdf);

        if (!ros::isInitialized())
        {
            int argc = 0;
            char **argv = NULL;
            ros::init(argc, argv, "terramechanics_node", ros::init_options::NoSigintHandler);
        }

        this->rosNode.reset(new ros::NodeHandle(this->robot_namespace_));

        GetSimulationParam();
        InitTerrain();

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&TerramachanicsPlugin::OnUpdate, this));

        InitSubPub();
    }
    void TerramachanicsPlugin::GetSDFParam(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->robot_namespace_ = "";
        if (_sdf->HasElement("robotNamespace"))
            this->robot_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
        this->model = _model;
        this->BaseLink = this->model->GetLink(
            _sdf->GetElement("base_link")->Get<std::string>());
        this->WheelLink = this->model->GetLink(
            _sdf->GetElement("wheel_link")->Get<std::string>());
        this->WheelJoint = this->model->GetJoint(
            _sdf->GetElement("wheel_joint")->Get<std::string>());

        // std::string WheelName = _sdf->GetElement("wheel_link")->Get<std::string>();

        this->SubTopic = _sdf->GetElement("sub_topic")->Get<std::string>();
        this->r = _sdf->GetElement("wheel_r")->Get<double>();
        this->b = _sdf->GetElement("wheel_b")->Get<double>();
        this->h = _sdf->GetElement("wheel_h")->Get<double>();
        this->FN_AVE = _sdf->GetElement("FN_AVE")->Get<double>();
        this->WheelName = _sdf->GetElement("wheel_link")->Get<std::string>();
    }
    void TerramachanicsPlugin::InitTerrain()
    {
        TerrainData = new double[x_grids*y_grids*DTM_params];

        // Resolve relative TerrainMapFileName against MarsSim repo root.
        // YAML now may provide: "rover_gazebo/data/simulated_terrain_data.txt"
        std::string dtmPath = TerrainMapFileName;
        const bool isAbs =
            (!dtmPath.empty() && (dtmPath[0] == '/' /* unix */));

        if (!isAbs)
        {
            // rover_gazebo_plugins is under <MarsSim>/rover_gazebo_plugins
            const std::string pluginPkgPath = ros::package::getPath("rover_gazebo_plugins");
            if (!pluginPkgPath.empty())
            {
                // <MarsSim> = <MarsSim>/rover_gazebo_plugins/..
                dtmPath = pluginPkgPath + "/../" + dtmPath;
            }
        }

        TerrainMapFile.open(dtmPath);
        if (!TerrainMapFile.is_open())
        {
            // Fallback: try the raw value (in case user expects CWD-based relative path)
            TerrainMapFile.clear();
            TerrainMapFile.open(TerrainMapFileName);
        }

        if (!TerrainMapFile.is_open())
        {
            std::cerr
                << "[TerramechanicsPlugin] Failed to open TerrainMapFileName.\n"
                << "  param value: " << TerrainMapFileName << "\n"
                << "  resolved:    " << dtmPath << "\n";
            return; // keep plugin alive; forces will effectively be invalid
        }

        // Optionally store resolved absolute-ish path for internal use (does not write back to YAML)
        TerrainMapFileName = dtmPath;

        std::string data;
        getline(TerrainMapFile, data);
        x_0 = std::stod(data);
        y_0 = x_0;
        // getline(TerrainMapFile, data);
        // y_0 = std::stod(data);
        getline(TerrainMapFile, data);
        x_step = std::stod(data);
        // getline(TerrainMapFile, data);
        // y_step = std::stod(data);
        y_step = x_step;
        getline(TerrainMapFile, data);
        Node_Coef = std::stoi(data);

        while (getline(TerrainMapFile, data))
        {
            this->TerrainDataVec.push_back(std::stod(data));
        }
        for (int i = 0; i < TerrainDataVec.size(); i++)
        {
            this->TerrainData[i] = TerrainDataVec[i];
        }
    }
    void TerramachanicsPlugin::InitSubPub()
    {
        ros::SubscribeOptions so =
            ros::SubscribeOptions::create<std_msgs::Float64>(
                SubTopic,
                1,
                boost::bind(&TerramachanicsPlugin::OnRosMsg, this, _1),
                ros::VoidPtr(),
                &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);
        this->rosQueueThread = std::thread(std::bind(&TerramachanicsPlugin::QueueThread, this));
        rosPub_z = this->rosNode->advertise<std_msgs::Float32>(this->WheelName+"/sinkage", 20);
        rosPub_s = this->rosNode->advertise<std_msgs::Float32>(this->WheelName+"/slip", 20);
        rosPub_beta = this->rosNode->advertise<std_msgs::Float32>(this->WheelName+"/beta", 20);
        rosPub_FT = this->rosNode->advertise<geometry_msgs::Wrench>(this->WheelName+"/force_torque",20);
    }
    void TerramachanicsPlugin::OnRosMsg(const std_msgs::Float64ConstPtr &msg)
    {
        cmd_vel = msg->data;
    }
    void TerramachanicsPlugin::OnUpdate()
    {
        if(enable_plugin)
        {
            SimulateCount++;
            force_torque = TerramachanicsPlugin::CalculationWheelForce(this->TerrainData, SimulateCount);

            wheel_force = ignition::math::Vector3d(force_torque.Force[0], force_torque.Force[1], force_torque.Force[2]);
            this->WheelLink->AddForce(wheel_force);

            wheel_torque = ignition::math::Vector3d(force_torque.Torque[0], force_torque.Torque[1], force_torque.Torque[2]);
            this->WheelLink->AddTorque(wheel_torque);
            int PublishStep = 1000/PublishRate;
            if (SimulateCount%PublishStep ==0)
            {
                wrech = this->WheelJoint->GetForceTorque(0);
                force = wrech.body1Force;
                torque = wrech.body1Torque;
                Wheel_FT_msg.force.x = -force.X();
                Wheel_FT_msg.force.y = -force.Y();
                Wheel_FT_msg.force.z = -force.Z();
                Wheel_FT_msg.torque.x = -torque.X();
                Wheel_FT_msg.torque.y = -torque.Y();
                Wheel_FT_msg.torque.z = -torque.Z();

                rosPub_z.publish(Sinkage_msg);
                rosPub_s.publish(Slip_msg);
                rosPub_beta.publish(Beta_msg);
                rosPub_FT.publish(Wheel_FT_msg);
            }
        }

    }
    TerramachanicsPlugin::FT TerramachanicsPlugin::CalculationWheelForce(double Terrain_Data[], int iter)
    {
        if (iter % PrintStep == 0)
            std::cout << "=======================" << WheelName << "======================" << std::endl;
        FT ForceTorque;
        static double FriDirection[3];


        // if (FrictionFlag == 0)
        // {
        //     deltajLength = 0;
        //     ignition::math::Pose3d WheelPose = this->WheelLink->WorldPose();
        //     WheelPos_Org = WheelPose.Pos();
        // }
        // AngleVelocity = cmd_vel;
        AngleVelocity = this->WheelJoint->GetVelocity(0);
        // cmd_vel=AngleVelocity;

        ignition::math::Vector3d Velocity_Local_wheel = this->WheelLink->RelativeLinearVel();
        ignition::math::Vector3d Pose_base = this->BaseLink->RelativePose().Pos();
        ignition::math::Pose3d Pose_wheel = this->WheelLink->RelativePose();
        ignition::math::Matrix3d RotMat(Pose_wheel.Rot());
        ignition::math::Vector3d VL = RotMat * Velocity_Local_wheel;
        Velocity_Local = VL;

        ignition::math::Pose3d WheelPose = this->WheelLink->WorldPose();
        WheelPos = WheelPose.Pos();
        //建立车轮坐标系及其转换矩阵
        ignition::math::Vector3d translate_coord = this->WheelLink->WorldPose().Pos();
        ignition::math::Vector3d wheel_v = this->WheelLink->WorldLinearVel();
        ignition::math::Vector3d wheel_x_axis = this->BaseLink->WorldPose().Rot().XAxis();
        ignition::math::Vector3d wheel_y_axis = this->BaseLink->WorldPose().Rot().YAxis();
        ignition::math::Vector3d wheel_z_axis = this->BaseLink->WorldPose().Rot().ZAxis();

        ignition::math::Matrix3d Rot_0_1;
        Rot_0_1.Axes(wheel_x_axis, wheel_y_axis, wheel_z_axis);
        ignition::math::Matrix4d Trans_0_1;
        Trans_0_1 = Rot_0_1;
        Trans_0_1.SetTranslation(translate_coord);

        ignition::math::Pose3d base_pose = this->model->WorldPose();
        // double phi_yaw = base_pose.Rot().GetYaw();
        double phi_yaw;
        ignition::math::Matrix3d RotMat_w(WheelPose.Rot());

        if (RotMat_w(0, 1) < 0)
        {
            phi_yaw = acos(RotMat_w(1, 1) / sqrt(RotMat_w(1, 1) * RotMat_w(1, 1) + RotMat_w(0, 1) * RotMat_w(0, 1)));
        }
        else
        {
            phi_yaw = -acos(RotMat_w(1, 1) / sqrt(RotMat_w(1, 1) * RotMat_w(1, 1) + RotMat_w(0, 1) * RotMat_w(0, 1)));
        }

        // if (iter % PrintStep == 0)
        // {
        //     std::cout << "VL="<< VL << std::endl;
        //     std::cout << "phi_yaw=" << WheelPose.Rot().GetYaw() << std::endl;
        //     std::cout << "wheel_v=" <<wheel_v << std::endl;
        // }

        if (fabs(SteerAngle) < 0.001)
            SteerAngle = 0;

        NormalandNode NN = CalculateTouchArea(Trans_0_1, Terrain_Data, theta1_LastStep);
        NormalVector = NN.NormalVector;
        NodePosition = NN.NodePosition;

        // 建立轮地接触坐标系及其转换矩阵
        ignition::math::Vector3d wheel_terrain_z_axis = NormalVector;
        wheel_terrain_z_axis = wheel_terrain_z_axis.Normalize();
        ignition::math::Vector3d wheel_terrain_y_axis = (wheel_terrain_z_axis.Cross(wheel_x_axis)).Normalize();
        ignition::math::Vector3d wheel_terrain_x_axis = (wheel_terrain_y_axis.Cross(wheel_terrain_z_axis)).Normalize();
        ignition::math::Matrix3d Rot_0_2;
        Rot_0_2.Axes(wheel_terrain_x_axis, wheel_terrain_y_axis, wheel_terrain_z_axis);
        ignition::math::Matrix4d Trans_0_2;
        Trans_0_2 = Rot_0_2;
        Trans_0_2.SetTranslation(translate_coord);

        // //求解局部车轮速度
        Velocity_Local_Real[0] = wheel_v.Dot(wheel_x_axis);
        Velocity_Local_Real[1] = wheel_v.Dot(wheel_y_axis);
        Velocity_Local_Real[2] = wheel_v.Dot(wheel_z_axis);

        // double s_flag;
        // double beta_flag;
        s = CalculateSlip(AngleVelocity, Velocity_Local_Real);
        Slip_msg.data = s;
        if (iter % PrintStep == 0)
        {
            std::cout << "slip= " << s << "\t";
            // std::cout << "phi_yaw= " << phi_yaw << std::endl;
        }

        beta = CalculateBeta(Velocity_Local_Real);
        Beta_msg.data = beta;

        sinkage = CalculationSinkage(WheelPos, NodePosition, NormalVector, sinkage_LastStep);
        Sinkage_msg.data = sinkage;
        if (iter % PrintStep == 0)
            std::cout << "sinkage=" << sinkage << std::endl;
        theta1 = CalculationTheta1(sinkage, theta1_LastStep);

        double n = n0 + n1 * fabs(s);
        if (sinkage <= 0.00002)
        {
            sinkage = 0;
            Force_W[0] = 0;
            Force_W[1] = 0;
            Force_W[2] = 0;
            Torque_W[0] = 0;
            Torque_W[1] = 0;
            Torque_W[2] = 0;

            sinkage_LastStep = 0.0002f;
            theta1_LastStep = 0.01f;
        }
        else
        {
            double Dz = LimitDampingCoef(Velocity_Local_Real);
            double Rs = CalculateRj(s);
            theta2 = c3 * theta1;
            // double wheel_terrain_pitch = wheel_terrain_x_axis[2]/fabs(wheel_terrain_x_axis[2]+1e-6)*acos(sqrt(wheel_terrain_x_axis[0] * wheel_terrain_x_axis[0] + wheel_terrain_x_axis[1] * wheel_terrain_x_axis[1]));
            // theta2 = -wheel_terrain_pitch;
            // if(theta2>0)
            //     theta2 = 0;
            // double thetam = ((c1 + c2 * s) * theta1 + theta2) / 2;
            thetam = (c1 + c2 * s) * theta1;
            // thetam = (theta1) / 2;

            double theta_m2 = thetam - theta2;
            double theta_1m = theta1 - thetam;
            double theta_12 = theta1 - theta2;

            double A_1 = (cos(thetam) - cos(theta2)) / theta_m2;
            double A_2 = (cos(thetam) - cos(theta1)) / theta_1m;
            A = A_1 + A_2;
            double B_1 = (sin(thetam) - sin(theta2)) / theta_m2;
            double B_2 = (sin(thetam) - sin(theta1)) / theta_1m;
            B = B_1 + B_2;
            C = theta_12 / 2;

            double theta11 = acos(r * cos(theta1) / Rs);
            sigma_m = (Kc / b + Kphi) * (pow(r, (n))) * pow((cos(thetam) - cos(theta1)), (n));
            // std::cout<< this->WheelName << "sigma_m=" << sigma_m << std::endl;
            if (fabs(AngleVelocity) >= 0.01)
            {
                WheelTerrainInteraction(theta11);
            }
            else
            {
                StaticModel(Rot_0_2);
            }

            forceworld = Rot_0_2 * forcelocal;
            Force_W[0] = forceworld.X();
            Force_W[1] = forceworld.Y();
            Force_W[2] = forceworld.Z();
            if (isnanf(Force_W[0]))
            {
                exit(0);
            }

            torqueworld = Rot_0_2 * torquelocal;
            Torque_W[0] = torqueworld.X();
            Torque_W[1] = torqueworld.Y();
            Torque_W[2] = torqueworld.Z();

            // if (iter % PrintStep == 0)
            //     std::cout << "FX=" << FX_Local << "  slip_flag=" << s_flag << std::endl;
            sinkage_LastStep = sinkage;
            theta1_LastStep = theta1;
            Force_Local_LastStep[2] = Force_Local[2];
        }
        ////////////////////////////////////返回相关参数//////////////////////////////
        std::normal_distribution<double> rand_num(0, 0.00000001);
        std::default_random_engine gen;
        ForceTorque.Force[0] = Force_W[0] * (1 + rand_num(gen));
        ForceTorque.Force[1] = Force_W[1] * (1 + rand_num(gen));
        ForceTorque.Force[2] = Force_W[2] * (1 + rand_num(gen));
        ForceTorque.Torque[0] = Torque_W[0] * (1 + rand_num(gen));
        ForceTorque.Torque[1] = Torque_W[1] * (1 + rand_num(gen));
        ForceTorque.Torque[2] = Torque_W[2] * (1 + rand_num(gen));

        if (iter % PrintStep == 0)
        {

            std::cout << "F_T_x"
                      << ":" << forcelocal.X() << "\t";
            std::cout << "F_W_x"
                      << ":" << Force_W[0] << std::endl;
            std::cout << "F_T_y"
                      << ":" << forcelocal.Y() << "\t";
            std::cout << "F_W_y"
                      << ":" << Force_W[1] << std::endl;
            std::cout << "F_T_z"
                      << ":" << forcelocal.Z() << "\t";
            std::cout << "F_W_z"
                      << ":" << Force_W[2] << std::endl;

            std::cout << "T_T_X"
                      << ":" << torquelocal.X() << "\t";
            std::cout << "T_W_X"
                      << ":" << Torque_W[0] << std::endl;
            std::cout << "T_T_Y"
                      << ":" << torquelocal.Y() << "\t";
            std::cout << "T_W_Y"
                      << ":" << Torque_W[1] << std::endl;
            std::cout << "T_T_Z"
                      << ":" << torquelocal.Z() << "\t";
            std::cout << "T_W_Z"
                      << ":" << Torque_W[2] << std::endl;
            // std::cout << "####################" << std::endl;
        }
        return ForceTorque;
    }

    void TerramachanicsPlugin::WheelTerrainInteraction(double theta11)
    {
        double FX_Local;
        double FY_Local;
        double FZ_Local;
        double MX_Local;
        double MY_Local;
        double MZ_Local;
        double jx, jy;
        double exp_jk_x, exp_jk_y;
        double Dz = LimitDampingCoef(Velocity_Local_Real);
        double s_flag = CalculateS_flag(s);
        double beta_flag = CalculateBeta_flag(beta);
        double rs = r + 0.5 * h;
        // double j = rs * ((theta11 - thetam) - (1 - fabs(s)) * (sin(theta11) - sin(thetam)));
        // double one_jdk = 1 - exp(-j / K);
        // double tao_m = (c + sigma_m * tan(phi)) * one_jdk;

        if (s_flag >= 0)
        {
            jx = rs * ((theta11 - thetam) - (1 - s) * (sin(theta11) - sin(thetam)));
            jy = r * (1 - s) * (theta11 - thetam) * tan(fabs(beta));
        }
        else
        {
            jx = -rs * ((theta11 - thetam) - (sin(theta11) - sin(thetam)) / (1+0.9*s));
            jy = r * (theta11 - thetam) * tan(fabs(beta)) / (1+0.99*s);
            // jx = rs * ((theta11 - thetam) - (1 - s) * (sin(theta11) - sin(thetam)));
            // jy = r * (1 - s) * (theta11 - thetam) * tan(fabs(beta));
        }

        exp_jk_x = 1 - exp(-jx / K);
        exp_jk_y = 1 - exp(-jy / K);
        tao_m = (c + sigma_m * tan(phi)) * exp_jk_x;

        // std::cout << "exp_jk_x=" << exp_jk_x << std::endl;
        // std::cout << "exp_jk_y=" << exp_jk_y << std::endl;

        double Fs = r * b * C * (c + sigma_m * tan(0.5 * phi)) * exp_jk_y;

        double Fn1 = r * b * A * sigma_m + rs * b * tao_m * B;
        double Fn = LimitSustainForce(Fn1);

        double coef_tempT = 1 + (c_T1 * ((FN_AVE - Fn) / FN_AVE));
        double Mr1 = rs * rs * C * (b * c + coef_tempT * Fn * tan(phi) / (r * A)) * exp_jk_x;
        double Mr2 = 1 + rs * B * tan(phi) * exp_jk_x / (r * A);
        double Mr = Mr1 / Mr2;
        // Mr = r*C*r*b*tao_m;

        double coef_s = c_d1 + c_d2 * s;
        double coef_t = c_d3 * ((FN_AVE - Fn) / FN_AVE);

        double Fdp1 = (A / C + B * B / (A * C)) * Mr / rs - B * Fn / A;
        double Fdp = Fdp1 * ((1 + coef_s) * (1 + coef_t));
        // Fdp = r * b * A * tao_m - rs * b * sigma_m * B;

        // Fn = r * b * A * sigma_m + r * b * tao_m * B;
        // Fdp = r * b * A * tao_m - r * b * sigma_m * B;
        // Mr = r*r*b*C*tao_m;

        FX_Local = Fdp * s_flag;
        if (s_flag < 0)
        {
            if (FX_Local > 0)
            {
                FX_Local = 0;
            }
            FX_Local = FX_Local - 200 * fabs(Velocity_Local_Real[0]);
        }
        if (s_flag >= 0)
        {
            if (FX_Local < -100)
            {
                FX_Local = -100;
            }
        }
        FY_Local = Fs;
        if(Velocity_Local_Real[1]>=0)
        {
            FY_Local = -FY_Local;
        }
        if (AngleVelocity < -0.0)
        {
            // FY_Local = -FY_Local;
            Mr = -Mr;
        }
        double FY_sub = 200 * Velocity_Local_Real[1];
        if (fabs(FY_sub) > 1000)
        {
            if (FY_sub > 0)
            {
                FY_sub = 1000;
            }
            else
            {
                FY_sub = -1000;
            }
        }
        double FZ_sub = Dz * Velocity_Local_Real[2];
        if (fabs(FZ_sub) > Dz * 5)
        {
            if (FZ_sub > 0)
            {
                FZ_sub = Dz * 5;
            }
            else
            {
                FZ_sub = -Dz * 5;
            }
        }
        FY_Local = FY_Local - FY_sub;
        FZ_Local = 1.0 * Fn - FZ_sub;

        MX_Local = 1.0 * r * FY_Local;
        MY_Local = -Mr;
        // double yita2;
        //yita2 = SteerAngle / (2 * M_PI);
        MZ_Local = sin(thetam) * r * FY_Local; //+ yita2 * b * FX_Local;

        if (AngleVelocity < -0.00)
        {
            // if (iter % PrintStep == 0)
            //     std::cout << "车轮反转!" << std::endl;
            FX_Local = -FX_Local;
            MZ_Local = -MZ_Local;
        }
        if (AngleVelocity * Velocity_Local_Real[0]<0)
        {
            FX_Local -= 200*Velocity_Local_Real[0];
        }
        FrictionFlag = 0;

        forcelocal.X() = FX_Local;
        forcelocal.Y() = FY_Local;
        forcelocal.Z() = FZ_Local;
        torquelocal.X() = MX_Local;
        torquelocal.Y() = MY_Local;
        torquelocal.Z() = MZ_Local;
    }

    void TerramachanicsPlugin::StaticModel(ignition::math::Matrix3d Rot_0_2)
    {
        double FX_Local;
        double FY_Local;
        double FZ_Local;
        double MX_Local;
        double MY_Local;
        double MZ_Local;
        static double deltajLength;
        ignition::math::Vector3d Friction;
        double Dz = LimitDampingCoef(Velocity_Local_Real);
        double theta_12 = theta1 - theta2;
        // std::cout<<"STATIC"<<std::endl;

        double Fn1 = r * b * A * sigma_m /*+rs*b*tao_m*B*/;
        double Fn = LimitSustainForce(Fn1);

        // ignition::math::Vector3d deltaj = CalculateFrictionDirectionNew(NormalVector, WheelPos_Org, WheelPos);
        ignition::math::Vector3d deltaj = WheelPos-WheelPos_Org;
        deltajLength = deltaj.Length();
        ignition::math::Vector3d FriDirectionV;
        if (deltajLength > 0.0001)
        {
            FriDirectionV = -deltaj / deltajLength;
        }

        double FrictionForce;
        FrictionForce = (r * b * theta_12 * c + Fn * tan(phi)) * (1 - exp(-deltajLength / K));
        Friction = FrictionForce * FriDirectionV;

        ignition::math::Vector3d FrctionL;
        FrctionL = Rot_0_2.Inverse() * Friction;
        double FX_sub = 2000 * Velocity_Local_Real[0];
        if (fabs(FX_sub) > 1000)
        {
            if (FX_sub > 0)
            {
                FX_sub = 1000;
            }
            else
            {
                FX_sub = -1000;
            }
        }
        double FY_sub = 10000 * Velocity_Local_Real[1];
        if (fabs(FY_sub) > 1000)
        {
            if (FY_sub > 0)
            {
                FY_sub = 1000;
            }
            else
            {
                FY_sub = -1000;
            }
        }
        double FZ_sub = Dz * Velocity_Local_Real[2];
        if (fabs(FZ_sub) > Dz * 5)
        {
            if (FZ_sub > 0)
            {
                FZ_sub = Dz * 5;
            }
            else
            {
                FZ_sub = -Dz * 5;
            }
        }

        FX_Local = FrctionL.X() - FX_sub;
        FY_Local = FrctionL.Y() - FY_sub;
        FZ_Local = 1.0 * Fn - FZ_sub;

        MX_Local = 1.0 * r * FY_Local;
        MY_Local = -(r * FX_Local + 0 * r * FZ_Local);
        MZ_Local = sin(thetam) * r * FY_Local + 0 * b * FX_Local;
        WheelPos_Org = WheelPos;
        FrictionFlag = 1;

        forcelocal.X() = FX_Local;
        forcelocal.Y() = FY_Local;
        forcelocal.Z() = FZ_Local;
        torquelocal.X() = MX_Local;
        torquelocal.Y() = MY_Local;
        torquelocal.Z() = MZ_Local;
    }

    TerramachanicsPlugin::NodeZandTerrainParams TerramachanicsPlugin::GetTouchNodes(double px, double py, double Terrain_Data[])
    {
        if (px > x_min && px < x_max && py > y_min && y_max)
        {
            double pz_node;

            int px_num = int(floor((px - x_min) / x_step));
            int py_num = int(floor((py - y_min) / y_step));

            double delta_x0 = px - (x_min + px_num * x_step);
            double delta_x1 = x_step - delta_x0;
            double delta_y0 = py - (y_min + py_num * y_step);
            double delta_y1 = y_step - delta_y0;
            double u0 = delta_x1 * delta_y1 / (x_step * y_step);
            double u1 = delta_x1 * delta_y0 / (x_step * y_step);
            double u2 = delta_x0 * delta_y1 / (x_step * y_step);
            double u3 = delta_x0 * delta_y0 / (x_step * y_step);
            int Count0 = Node_Coef * px_num + py_num;
            int Count1 = Node_Coef * px_num + py_num + 1;
            int Count2 = Node_Coef * (px_num + 1) + py_num;
            int Count3 = Node_Coef * (px_num + 1) + py_num + 1;
            double pz_node0 = Terrain_Data[DTM_params * Count0 + 2];
            double pz_node1 = Terrain_Data[DTM_params * Count1 + 2];
            double pz_node2 = Terrain_Data[DTM_params * Count2 + 2];
            double pz_node3 = Terrain_Data[DTM_params * Count3 + 2];
            pz_node = u0 * pz_node0 + u1 * pz_node1 + u2 * pz_node2 + u3 * pz_node3;
            double Kc_arr[4], Kphi_arr[4], n0_arr[4], n1_arr[4], c_arr[4], phi_arr[4], K_arr[4];
            int Count[4] = {Count0, Count1, Count2, Count3};
            double TerrainParams[TerrainParamsNum][4];
            for (int point = 0; point < 4; point++)
            {
                for (int j = 0; j < TerrainParamsNum; j++)
                {
                    TerrainParams[j][point] = Terrain_Data[DTM_params * Count[point] + 3 + j];
                }
            }

            NodeZandTerrainParams NTP;
            NTP.TerrainParams = new double[TerrainParamsNum];
            for (int i = 0; i < TerrainParamsNum; i++)
            {
                NTP.TerrainParams[i] = u0 * TerrainParams[i][0] + u1 * TerrainParams[i][1] + u2 * TerrainParams[i][2] + u3 * TerrainParams[i][3];
            }
            NTP.NodeZ = pz_node;

            return NTP;
        }
        else
        {
            NodeZandTerrainParams NTP;
            NTP.NodeZ = -100;
            return NTP;
        }
    }
    TerramachanicsPlugin::NormalandNode TerramachanicsPlugin::CalculateTouchArea(ignition::math::Matrix4d TransMat, double Terrain_Data[], double theta1_in)
    {
        ignition::math::Vector3d PW1(r * sin(theta1_in), b / 2, -r * cos(theta1_in));
        ignition::math::Vector3d PW2(r * sin(theta1_in), -b / 2, -r * cos(theta1_in));
        ignition::math::Vector3d PW3(-r * sin(theta1_in), 0, -r * cos(theta1_in));
        ignition::math::Vector3d PW1_0 = TransMat * PW1;
        ignition::math::Vector3d PW2_0 = TransMat * PW2;
        ignition::math::Vector3d PW3_0 = TransMat * PW3;
        ignition::math::Vector3d PA, PB, PC;
        PA = PW1_0;
        PB = PW2_0;
        PC = PW3_0;

        NodeZandTerrainParams NTP1 = GetTouchNodes(PA.X(), PA.Y(), Terrain_Data);
        NodeZandTerrainParams NTP2 = GetTouchNodes(PB.X(), PB.Y(), Terrain_Data);
        NodeZandTerrainParams NTP3 = GetTouchNodes(PC.X(), PC.Y(), Terrain_Data);
        double pz1_node = NTP1.NodeZ;
        double pz2_node = NTP2.NodeZ;
        double pz3_node = NTP3.NodeZ;
        PA.Z() = NTP1.NodeZ;
        PB.Z() = NTP2.NodeZ;
        PC.Z() = NTP3.NodeZ;

        NormalandNode NN;
        NN.NormalVector = (PB - PC).Cross(PA - PC);
        if (NN.NormalVector.Z() < 0)
        {
            NN.NormalVector = -NN.NormalVector;
        }

        NN.NodePosition = PA;

        Kc = (NTP1.TerrainParams[0] + NTP2.TerrainParams[0] + NTP3.TerrainParams[0]) / 3;
        Kphi = (NTP1.TerrainParams[1] + NTP2.TerrainParams[1] + NTP3.TerrainParams[1]) / 3;
        n0 = (NTP1.TerrainParams[2] + NTP2.TerrainParams[2] + NTP3.TerrainParams[2]) / 3;
        n1 = (NTP1.TerrainParams[3] + NTP2.TerrainParams[3] + NTP3.TerrainParams[3]) / 3;
        c = (NTP1.TerrainParams[4] + NTP2.TerrainParams[4] + NTP3.TerrainParams[4]) / 3;
        phi = (NTP1.TerrainParams[5] + NTP2.TerrainParams[5] + NTP3.TerrainParams[5]) / 3;
        K = (NTP1.TerrainParams[6] + NTP2.TerrainParams[6] + NTP3.TerrainParams[6]) / 3;
        delete[] NTP1.TerrainParams;
        delete[] NTP2.TerrainParams;
        delete[] NTP3.TerrainParams;
        return NN;
    }
    double TerramachanicsPlugin::CalculateSlip(double Angv, double Local_Lin_Velocity[3])
    {

        double s;
        double del_v;
        double vel_real = Local_Lin_Velocity[0];
        if (Angv > 0)
        {
            del_v = (r + 0.5 * h) * Angv - vel_real;
        }
        else
        {
            del_v = fabs((r + 0.5 * h) * Angv) - fabs(vel_real);
        }

        if (fabs(del_v) <= 0.001 || fabs(Angv) < 0.001)
        {
            s = 0;
        }
        else if (del_v > 0.001)
        {
            s = 1 - fabs(vel_real / ((r + 0.5 * h) * Angv));
        }
        else if (del_v < -0.001)
        {
            s = fabs((r + 0.5 * h) * Angv) / fabs(vel_real) - 1;
        }

        if (s > 1)
            s = 1;
        if (s < -1)
            s = -1;
        if (Angv * vel_real < 0)
        {
            s = 1;
        }
        return s;
    }
    double TerramachanicsPlugin::CalculateS_flag(double s)
    {
        double s_flag;
        if (s >= 0)
        {
            s_flag = 1;
        }
        else
        {
            s_flag = -1;
        }
        return s_flag;
    }
    double TerramachanicsPlugin::CalculateBeta(double Local_Lin_Velocity[3])
    {

        double beta;
        if (fabs(Local_Lin_Velocity[1]) <= 0.001)
        {
            beta = 0;
        }
        else
        {
            // if (Local_Lin_Velocity[1] >= 0)
            // beta = acos(Local_Lin_Velocity[0] / sqrt(Local_Lin_Velocity[0] * Local_Lin_Velocity[0] + Local_Lin_Velocity[1] * Local_Lin_Velocity[1]));
            beta = acos(-1) / 2 - atan(Local_Lin_Velocity[0] / Local_Lin_Velocity[1]);
            if (beta > acos(-1) / 2)
            {
                beta -= acos(-1);
            }
            // else
            // beta = -acos(Local_Lin_Velocity[0] / sqrt(Local_Lin_Velocity[0] * Local_Lin_Velocity[0] + Local_Lin_Velocity[1] * Local_Lin_Velocity[1]));
        }

        if (fabs(beta) < 0.01)
        {
            beta = 0;
        }
        return beta;
    }
    double TerramachanicsPlugin::CalculateBeta_flag(double beta)
    {

        double beta_flag;
        if (beta > 0)
        {
            beta_flag = -1;
        }
        else if (beta == 0)
        {
            beta_flag = 1;
        }
        else
        {
            beta_flag = 1;
        }
        return beta_flag;
    }
    double TerramachanicsPlugin::CalculationSinkage(ignition::math::Vector3d WheelPos, ignition::math::Vector3d P1, ignition::math::Vector3d Area, double sinkage_last)
    {
        double sinkage = r - Area.Dot(WheelPos - P1) / Area.Length();
        if (sinkage < 0)
        {
            sinkage = 0;
        }
        else
        {
            if (fabs(sinkage - sinkage_last) > sinkage_step)
            {
                if (sinkage > sinkage_last)
                {
                    sinkage = sinkage_last + sinkage_step;
                }
                else
                {
                    sinkage = sinkage_last - sinkage_step;
                }
            }
            if (sinkage > sinkage_max)
            {
                sinkage = sinkage_max;
            }
        }
        return sinkage;
    }
    double TerramachanicsPlugin::CalculationTheta1(double z_sinkage0, double theta1_last)
    {
        double theta1_out;
        double theta1_now;
        theta1_now = acos((r - z_sinkage0) / r);
        // if (fabs(theta1_now - theta1_last) > theta1_step)
        // {
        //     if (theta1_now > theta1_last)
        //     {
        //         theta1_out = theta1_last + theta1_step;
        //     }
        //     else
        //     {
        //         theta1_out = theta1_last - theta1_step;
        //     }
        // }
        // else
        // {
        //     theta1_out = theta1_now;
        // }
        return theta1_now;
    }
    // double TerramachanicsPlugin::CalculationEqualRadius(double slip)
    // {
    //     double RRR_out;
    //     double r;
    //     r = R + h;
    //     if (slip >= 0.05)
    //     {
    //         RRR_out = R - 3 * (slip - 0.05) * h_lug;
    //         if (RRR_out < r)
    //         {
    //             RRR_out = r;
    //         }
    //     }
    //     else
    //     {
    //         RRR_out = R;
    //     }
    //     return RRR_out;
    // }

    double TerramachanicsPlugin::LimitDampingCoef(double Linv[3])
    {
        double DampCoef;
        if (Linv[2] > 0)
        {
            DampCoef = FirstDampCoef;
        }
        else
        {
            DampCoef = SecondDampCoef;
        }
        return DampCoef;
    }
    double TerramachanicsPlugin::LimitSustainForce(double Fn_in)
    {
        double Fn_out;
        if (Fn_in > Fn_Max)
        {
            Fn_out = Fn_Max;
        }
        else
        {
            Fn_out = Fn_in;
        }
        return Fn_out;
    }
    double TerramachanicsPlugin::CalculateRj(double s)
    {
        double Rj;
        if (s < 0.15)
            Rj = r;
        else if (s > 0.5)
            Rj = r + h;
        else
            Rj = r + h * (s - 0.15) / (0.5 - 0.15);
        return Rj;
    }
    ignition::math::Vector3d TerramachanicsPlugin::CalculateFrictionDirectionNew(ignition::math::Vector3d normalVector, ignition::math::Vector3d WheelPos0, ignition::math::Vector3d WheelPos1)
    {
        ignition::math::Vector3d j, wheelDistance, normalVectorLength;
        wheelDistance = WheelPos1 - WheelPos0;
        normalVectorLength = normalVector.Length();
        double TempData = wheelDistance.Dot(normalVector);
        j = wheelDistance - normalVector * TempData / (normalVectorLength * normalVectorLength);
        return j;
    }
    void TerramachanicsPlugin::QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok())
        {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

    void TerramachanicsPlugin::GetSimulationParam()
    {
        while (!rosNode->hasParam("/zhurong/c1"))
        {
            std::cout<<"wait"<<std::endl;
        }
        if (!rosNode->hasParam("/zhurong/enable_plugin"))
        {
            ROS_WARN("No param named 'enable_plugin'");
        }
        else
        {
            rosNode->getParam("/zhurong/enable_plugin", enable_plugin);
        }
        if (!rosNode->hasParam("/zhurong/c1"))
        {
            ROS_WARN("No param named 'c1'");
        }
        else
        {
            rosNode->getParam("/zhurong/c1", c1);
        }

        if (!rosNode->hasParam("/zhurong/c2"))
        {
            ROS_WARN("No param named 'c2'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/c2", c2);
        }

        if (!rosNode->hasParam("/zhurong/c3"))
        {
            ROS_WARN("No param named 'c3'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/c3", c3);
        }

        if (!rosNode->hasParam("/zhurong/c_T1"))
        {
            ROS_WARN("No param named 'c_T1'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/c_T1", c_T1);
        }
        if (!rosNode->hasParam("/zhurong/c_d1"))
        {
            ROS_WARN("No param named 'c_d1'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/c_d1", c_d1);
        }
        if (!rosNode->hasParam("/zhurong/c_d2"))
        {
            ROS_WARN("No param named 'c_d2'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/c_d2", c_d2);
        }
        if (!rosNode->hasParam("/zhurong/c_d3"))
        {
            ROS_WARN("No param named 'c_d3'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/c_d3", c_d3);
        }

        if (!rosNode->hasParam("/zhurong/sinkage_max"))
        {
            ROS_WARN("No param named 'sinkage_max'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/sinkage_max", sinkage_max);
        }
        if (!rosNode->hasParam("/zhurong/FirstDampCoef"))
        {
            ROS_WARN("No param named 'FirstDampCoef'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/FirstDampCoef", FirstDampCoef);
        }
        if (!rosNode->hasParam("/zhurong/SecondDampCoef"))
        {
            ROS_WARN("No param named 'SecondDampCoef'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/SecondDampCoef", SecondDampCoef);
        }

        if (!rosNode->hasParam("/zhurong/PrintStep"))
        {
            ROS_WARN("No param named 'PrintStep'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/PrintStep", PrintStep);
        }

        if (!rosNode->hasParam("/zhurong/DTM_params"))
        {
            ROS_WARN("No param named 'DTM_params'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/DTM_params", DTM_params);
        }
        if (!rosNode->hasParam("/zhurong/TerrainParamsNum"))
        {
            ROS_WARN("No param named 'TerrainParamsNum'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/TerrainParamsNum", TerrainParamsNum);
        }

        if (!rosNode->hasParam("/zhurong/x_min"))
        {
            ROS_WARN("No param named 'x_min'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/x_min", x_min);
        }
        if (!rosNode->hasParam("/zhurong/x_max"))
        {
            ROS_WARN("No param named 'x_max'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/x_max", x_max);
        }
        if (!rosNode->hasParam("/zhurong/y_min"))
        {
            ROS_WARN("No param named 'y_min'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/y_min", y_min);
        }
        if (!rosNode->hasParam("/zhurong/y_max"))
        {
            ROS_WARN("No param named 'y_max'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/y_max", y_max);
        }
        if (!rosNode->hasParam("/zhurong/x_grids"))
        {
            ROS_WARN("No param named 'x_grids'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/x_grids", x_grids);
        }
        if (!rosNode->hasParam("/zhurong/y_grids"))
        {
            ROS_WARN("No param named 'y_grids'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/y_grids", y_grids);
        }

        if (!rosNode->hasParam("/zhurong/TerrainMapFileName"))
        {
            ROS_WARN("No param named 'TerrainMapFileName'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/TerrainMapFileName", TerrainMapFileName);
        }

        if (!rosNode->hasParam("/zhurong/PublishRate"))
        {
            ROS_WARN("No param named 'PublishRate'");
        }
        else
        {
            this->rosNode->getParam("/zhurong/PublishRate", PublishRate);
        }

        ROS_WARN("Load all parameters! ");
    }

    GZ_REGISTER_MODEL_PLUGIN(TerramachanicsPlugin)
} // namespace gazebo