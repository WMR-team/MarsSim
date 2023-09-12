#include "TerramechanicsPlugin_new.hh"
#include <math.h>
namespace gazebo
{
    TerramachanicsPlugin_new::TerramachanicsPlugin_new() {}
    TerramachanicsPlugin_new::~TerramachanicsPlugin_new()
    {
        delete[] TerrainData;
    }

    void TerramachanicsPlugin_new::WheelTerrainInteraction(double theta11)
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
        beta = 0.99*beta;
        double beta_flag = CalculateBeta_flag(beta);
        // double rs = r + 0.5 * h;
        // std::cout<<"s_flag: "<<s_flag<<"\t";

        if (s_flag >= 0)
        {
            jx = r * ((theta11 - thetam) - (1 - s) * (sin(theta11) - sin(thetam)));
            jy = r * (1 - s) * (theta11 - thetam) * tan(fabs(beta));
            exp_jk_x = 1 - exp(-jx / K);
        }
        else
        {
            jx = r * ((theta11 - thetam) - (sin(theta11) - sin(thetam)) / (1 + 0.99 * s));
            jy = r * (theta11 - thetam) * tan(fabs(beta)) / (1 + 0.99 * s);
            exp_jk_x = tanh(1 - exp(-jx / K));
            
            // jx = rs * ((theta11 - thetam) - (1 - s) * (sin(theta11) - sin(thetam)));
            // jy = r * (1 - s) * (theta11 - thetam) * tan(fabs(beta));
        }

        // exp_jk_x = 1 - exp(-jx / K);
        exp_jk_y = 1 - exp(-jy / K);
        tao_m = (c + sigma_m * tan(phi)) * exp_jk_x;
        // std::cout<< this->WheelName << "tao_m: " << tao_m << std::endl;

        double Nq = exp((1.5 * M_PI - phi) * tan(phi)) / (2 * cos(M_PI / 4 + phi / 2) * cos(M_PI / 4 + phi / 2));
        double Nc = (Nq - 1) / tan(phi);
        double Nr = 2 * (Nq + 1) * tan(phi) / (1 + 0.4 * sin(4 * phi));
        double Nphi = tan(M_PI / 4 + phi / 2) * tan(M_PI / 4 + phi / 2);
        double aa = rho * g * Nr;
        double bb = c * Nc;

        double Fu = r * b * C * (c + sigma_m * tan(phi)) * exp_jk_y;
        double Fs = 2.0 / 3.0 * r * r * (cos(theta1) - 1) * sin(theta1) * (r * aa * (cos(theta1) - 1) - 1.5 * b);
        double Fy = Fu + Fs * sin(fabs(beta));
        // std::cout<<"s_flag: "<<s_flag<<"\t";
        // std::cout<<"Fs: "<<Fs<<"\t";
        // std::cout<<"Fu: "<<Fu<<"\t";
        // std::cout<<"jy: "<<jy<<"\t";
        // std::cout<<"Fu: "<<Fu<<std::endl;

        double v_limg = fabs(AngleVelocity*3);
        if (v_limg>=1)
        {
            v_limg = 1;
        }

        double Fg = b * (0.5 * rho * g * h * h * Nphi + 2 * c * h * sqrt(Nphi)) * v_limg;
        // std::cout << h << std::endl;
        // std::cout << Nphi << std::endl;

        // double wheel_angle = this->WheelJoint->Position(1);
        // std::cout<<wheel_angle<<std::endl;
        double wheel_angle = 0;

        double beta_g = wheel_angle / theta_g - floor(wheel_angle / theta_g);

        // double beta_g = 0;
        int i_g = 0;
        if (beta_g < theta1)
        {
            angle_arr[i_g] = beta_g;
        }
        double beta_temp = beta_g + theta_g;
        while (beta_temp < theta1)
        {
            i_g++;
            angle_arr[i_g] = beta_temp;
            beta_temp += theta_g;
        }

        beta_temp = beta_g - theta_g;
        while (beta_temp > theta2)
        {
            i_g++;
            angle_arr[i_g] = beta_temp;
            beta_temp -= theta_g;
        }
        double F_gs = 0;
        double F_gc = 0;
        double F_ga = Fg * (i_g + 1);
        for (int i = 0; i < i_g; i++)
        {
            F_gs += Fg * sin(angle_arr[i]);
            F_gc += Fg * cos(angle_arr[i]);
        }

        // if (SimulateCount==0)
        // {
        //     F_ga_f = F_ga;
        //     F_gs_f = F_gs;
        //     F_gc_f = F_gc;
        // }
        
        F_ga = F_ga*Fg_ratio + (1-Fg_ratio)*F_ga_f;
        F_gs = F_gs*Fg_ratio + (1-Fg_ratio)*F_gs_f;
        F_gc = F_gc*Fg_ratio + (1-Fg_ratio)*F_gc_f;
        F_ga_f = F_ga;
        F_gs_f = F_gs;
        F_gc_f = F_gc;

        // double Mr = s_flag* (r * C * r * b * tao_m + F_ga*(r+0.67*h));
        double Mr = (r * C * r * b * tao_m + F_ga*(r+0.67*h));

        // double Fn1 = r * b * A * sigma_m + s_flag * r * b * tao_m * B + F_gs;
        double Fn1 = r * b * A * sigma_m + r * b * tao_m * B + F_gs;

        double Fn = LimitSustainForce(Fn1);
        

        // double Fdp = s_flag * r * b * A * tao_m - r * b * sigma_m * B + s_flag * F_gc;
        double Fdp = r * b * A * tao_m - r * b * sigma_m * B + s_flag * F_gc;

        // double coef_tempT = 1 + (c_T1 * ((FN_AVE - Fn) / FN_AVE));
        // double Mr1 = rs * rs * C * (b * c + coef_tempT * Fn * tan(phi) / (r * A)) * exp_jk_x;
        // double Mr2 = 1 + rs * B * tan(phi) * exp_jk_x / (r * A);
        // double Mr = Mr1 / Mr2;
        // Mr = r*C*r*b*tao_m;

        // double coef_s = c_d1 + c_d2 * s;
        // double coef_t = c_d3 * ((FN_AVE - Fn) / FN_AVE);

        // double Fdp1 = (A / C + B * B / (A * C)) * Mr / rs - B * Fn / A;
        // double Fdp = Fdp1 * ((1 + coef_s) * (1 + coef_t));

        // Fdp = r * b * A * tao_m - rs * b * sigma_m * B;

        // Fn = r * b * A * sigma_m + r * b * tao_m * B;
        // Fdp = r * b * A * tao_m - r * b * sigma_m * B;
        // Mr = r*r*b*C*tao_m;

        FX_Local = Fdp;
        // if (s_flag < 0)
        // {
        //     if (FX_Local > 0)
        //     {
        //         FX_Local = 0;
        //     }
        //     FX_Local = FX_Local - 200 * fabs(Velocity_Local_Real[0]);
        // }
        // if (s_flag >= 0)
        // {
        //     if (FX_Local < -100)
        //     {
        //         FX_Local = -100;
        //     }
        // }
        FY_Local = Fy;
        if (AngleVelocity < -0.0)
        {
            FX_Local = -FX_Local;
            // FY_Local = -FY_Local;
            Mr = -Mr;
        }
        if(Velocity_Local_Real[1]>=0)
        {
            FY_Local = -FY_Local;
        }
        // std::cout<<"FY_Local: "<<FY_Local<<std::endl;
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
        FZ_Local = Fn - FZ_sub;

        MX_Local = r * FY_Local;
        MY_Local = -Mr;

        MZ_Local = sin(thetam) * r * FY_Local;
        if (AngleVelocity < -0.00)
        {
            MZ_Local = -MZ_Local;
        }
        if (AngleVelocity * Velocity_Local_Real[0] < 0)
        {
            FX_Local -= 10000 * Velocity_Local_Real[0];
            // std::cout<<Velocity_Local_Real[0]<<std::endl;
        }

        FrictionFlag = 0;

        forcelocal.X() = FX_Local;
        forcelocal.Y() = FY_Local;
        forcelocal.Z() = FZ_Local;
        torquelocal.X() = MX_Local;
        torquelocal.Y() = MY_Local;
        torquelocal.Z() = MZ_Local;

        // std::cout << "F_T_x"
        //             << ":" << FX_Local << "\t";
        // std::cout << "T_T_y"
        //             << ":" << MY_Local << std::endl;
    }

    GZ_REGISTER_MODEL_PLUGIN(TerramachanicsPlugin_new)
} // namespace gazebo