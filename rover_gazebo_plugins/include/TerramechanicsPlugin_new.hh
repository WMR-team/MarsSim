#ifndef _TERRAMECHANICSPLUGIN_NEW_HH_
#define _TERRAMECHANICSPLUGIN_NEW_HH_

#include "TerramechanicsPlugin.hh"


namespace gazebo
{
	class TerramachanicsPlugin_new : public TerramachanicsPlugin
	{
        public: virtual ~TerramachanicsPlugin_new();
        public: TerramachanicsPlugin_new();
        public: void virtual WheelTerrainInteraction(double theta11);
        public:
            double rho = 1400;
            double g = 9.8;
            const int num_g = 20;
            double theta_g = 2*M_PI/num_g;
            double angle_arr[20] = {0};
            double F_ga_f=0;
            double F_gs_f=0;
            double F_gc_f=0;
            double Fg_ratio = 0.03;
    };
}

#endif