// The following class defines the necessary members and methods required for estimating angle of attack and angle of sideslip using a complementary filter
#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include<math.h>

class Complementary_Filter
{
    private:
    float T;    //  Sampling Time
    float Tau;  //  Time-Constant
    float Z;    //  Z=2*Tau/T
    const float g; // 32.2 ft/s^2
    // Independent variables
    float v, Theta, Phi, ny, p, r;  //  v (velocity in Yb - ft/s), Theta (Pitch Attitude - rad), Phi (Roll Attitude - rad), ny (Acceleration in Yb), p (Pitch Rate about Xb - rad/s), r (Yaw Rate about Zb - rad/s)
    float sine_G;   //  sine_G = h_dot/V_dot
    float V_1, V_2; // Total Velocity in Body Axis - ft/s
	float V_dot_1, V_dot_2;
    float h_1, h_2; // Altitude - ft
	float h_dot_1, h_dot_2; // Change of Altitude - ft/s
	float Beta_dot_i_1, Beta_dot_i_2;
    // Depedent variables
    float Beta_i_0, Beta_i_1, Beta_i_2;
    float Beta_if_0, Beta_if_1, Beta_if_2;
    float Beta_v_0, Beta_v_1, Beta_v_2;
    float Beta_vf_0, Beta_vf_1, Beta_vf_2;
	float Beta_cf_1, Beta_cf_2;
    float Alpha_i_0, Alpha_i_1, Alpha_i_2;
    float Alpha_if_0, Alpha_if_1, Alpha_if_2;
    float Alpha_v_0, Alpha_v_1, Alpha_v_2;
    float Alpha_vf_0, Alpha_vf_1, Alpha_vf_2;
    float Alpha_cf_2;

    // Estimates Beta_dot based on the given inputs
    inline float Beta_dot_Inertial(float g, float V, float Beta, float V_dot, float ny, float Theta, float Phi, float r, float Alpha, float p)
    {   return (g/V*(ny+cos(Theta)*sin(Phi))-Beta*V_dot/V*-r+Alpha*p);  }

    inline float Alpha_Inertial(float sine_G, float Theta, float Beta, float Phi)
    {   return ((-sine_G+sin(Theta)-Beta*sin(Phi))/cos(Phi));   }

    inline float integrate(float un,float un1,float yn)
    {   return ((T*(un1+un)+2*yn)/2);    }

    inline float differentiate(float un, float un1, float yn)
    {   return ((2*(un1-un)-T*yn)/T);   }

    inline float low_pass(float un, float un1, float un2, float yn, float yn1)
    {   return ((un2+2*un1+un-yn1*2*(1+Z)*(1-Z)-yn*(1-Z))/(1+Z)/(1+Z)); }

    inline float high_pass(float un, float un1, float un2, float yn, float yn1)
    {   return (2*Z*(un2*(1/2*Z+1)-un1*Z+un*(1/2*Z-1)-yn1*2*(1+Z)*(1-Z)-yn*(1-Z)/(1-Z))/(1+Z)/(1+Z));   }

    public:
    // Class Constructor
    complementary_filter(float T = 0.015, float Tau = 3):
    T(T), Tau(Tau), g(32.2), v(0.0), Theta(0.0), Phi(0.0), ny(0.0), p(0.0), r(0.0),
    sine_G(0.0), V_1(0.0), V_2(0.0), V_dot_1(0.0), V_dot_2(0.0), h_1(0.0), h_2(0.0),
	h_dot_1(0.0), h_dot_2(0.0), Beta_dot_i_1(0.0), Beta_dot_i_2(0.0), Beta_i_0(0.0),
    Beta_i_1(0.0), Beta_i_2(0.0), Beta_if_0(0.0), Beta_if_1(0.0), Beta_if_2(0.0),
    Beta_v_0(0.0), Beta_v_1(0.0), Beta_v_2(0.0), Beta_vf_0(0.0), Beta_vf_1(0.0),
    Beta_vf_2(0.0), Beta_cf_1(0.0), Beta_cf_2(0.0), Alpha_i_0(0.0), Alpha_i_1(0.0),
    Alpha_i_2(0.0), Alpha_if_0(0.0), Alpha_if_1(0.0), Alpha_if_2(0.0), Alpha_v_0(0.0),
    Alpha_v_1(0.0), Alpha_v_2(0.0), Alpha_vf_0(0.0), Alpha_vf_1(0.0), Alpha_vf_2(0.0),
    Alpha_cf_2(0.0)
    {Z = 2*Tau/T;}

    // Set values the first time
    void init(float Theta_l, float Phi_l, float ny_l, float r_l, float p_l, float v_l, float V_1_l, float h_1_l)
    {
        Theta = Theta_l; Phi = Phi_l; ny = ny_l; r = r_l; p = p_l; v = v_l; V_1 = V_1_l; h_1 = h_1_l;
        Alpha_i_1 = Alpha_Inertial(sine_G, Theta, Beta_i_1, Phi);
        Beta_dot_i_1 = Beta_dot_Inertial(g, V_1, Beta_i_1, V_dot_1, ny, Theta, Phi, r, Alpha_i_1, p);
        Beta_v_1 = atan(v/V_1);
    }

    void update()
    {
        // Update for High-Pass Filter
        h_1 = h_2;
		V_1 = V_2;
		h_dot_1 = h_dot_2;
        Beta_i_0 = Beta_i_1;
        Beta_i_1 = Beta_i_2;
        Beta_if_0 = Beta_if_1;
        Beta_if_1 = Beta_if_2;
        // Update for Low-Pass Filter
        Beta_v_0 = Beta_v_1;
        Beta_v_1 = Beta_v_2;
        Beta_vf_0 = Beta_vf_1;
        Beta_vf_1 = Beta_vf_2;
        // Update for Beta Complementary Filter
        Beta_cf_1 = Beta_cf_2;
        // Update for Alpha High-Pass Computations
        Alpha_i_0 = Alpha_i_1;
        Alpha_i_1 = Alpha_i_2;
        Alpha_if_0 = Alpha_if_1;
        Alpha_if_1 = Alpha_if_2;
        // Update for Alpha Low-Pass Filter
        Alpha_v_0 = Alpha_v_1;
        Alpha_v_1 = Alpha_v_2;
        Alpha_vf_0 = Alpha_vf_1;
        Alpha_vf_1 = Alpha_vf_2;
    }

	// set_n_time
    void runFilter(float Theta_l, float Phi_l, float ny_l, float r_l, float p_l, float v_l, float V_2_l, float h_2_l)
    {
        Theta = Theta_l; Phi = Phi_l; ny = ny_l; r = r_l; p = p_l; v = v_l; V_2 = V_2_l; h_2 = h_2_l;
        h_dot_2 = differentiate(h_1, h_2, h_dot_1);
		V_dot_2 = differentiate(V_1, V_2, V_dot_1);
		sine_G = h_dot_2/V_dot_2;
		Alpha_i_2 = Alpha_Inertial(sine_G, Theta, Beta_cf_1, Phi);
		Beta_dot_i_2 = Beta_dot_Inertial(g, V_2, Beta_i_2, V_dot_2, ny, Theta, Phi, r, Alpha_i_2, p);
		Beta_i_2 = integrate(Beta_dot_i_1, Beta_dot_i_2, Beta_i_1);
        // High-Pass Filter
        Beta_if_2 = high_pass(Beta_i_0, Beta_i_1, Beta_i_2, Beta_if_0, Beta_if_1);
        // Vane Computations
        Beta_v_2 = atan(v/V_2);
        // Low-Pass Filter
        Beta_vf_2 = low_pass(Beta_v_0, Beta_v_1, Beta_v_2, Beta_vf_0, Beta_vf_1);
        // Complementary Filter (Final)
        Beta_cf_2 = Beta_if_2 + Beta_vf_2;
        // Apha(Inertial) Computations
        Alpha_i_2 = Alpha_Inertial(sine_G, Theta, Beta_cf_2, Phi);
        Alpha_if_2 = high_pass(Alpha_i_0, Alpha_i_1, Alpha_i_2, Alpha_if_0, Alpha_if_1);
        // Alpha(Vane) Computations
        Alpha_vf_2 = low_pass(Alpha_v_0, Alpha_v_1, Alpha_v_2, Alpha_vf_0, Alpha_vf_1);
        // Complementary Filter
        Alpha_cf_2 = Alpha_i_2 + Alpha_vf_2;
        update();
    }

    float getBeta()
    {   return Beta_cf_2;   }

    float getAlpha()
    {   return Alpha_cf_2;  }
};

#endif
