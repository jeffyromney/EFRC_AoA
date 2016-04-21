// The following class defines the necessary members and methods required for estimating angle of attack and angle of sideslip using a complementary filter
#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include<math.h>

class complementary_filter
{
    private:
    
    float T; //Sampling Time (s)
    float Tau; // Time-Constant (s)
    float Z; // Z = 2*Tau/T
    float g; // Gravitational Acceleration, g = 32.174 ft/s2
    float h[2]; // Altitude, ft
    float V[2]; // Velocity, ft/s
    float B_dot_i[2];
    float A_dot_i[2];
    
    // Dependent Variables:
    float B_i[3], B_if[3], B_v[3], B_vf[3], B_cf[2];
    float A_i[3], A_if[3], A_v[3], A_vf[3], A_cf[2];
    
	// Computational Functions
    inline float Beta_dot_i(float g, float V, float ny, float Theta, float Phi, float Beta, float V_dot, float r, float Alpha, float p)
    {   return g/V*(ny+cos(Theta)*sin(Phi))-Beta*V_dot/V-r+Alpha*p;  }
    
    inline float Alpha_dot_i(float g, float V, float nz, float Theta, float Phi, float Alpha, float V_dot, float q, float Beta, float p)
    {   return g/V*(nz+cos(Theta)*cos(Phi))-Alpha*V_dot/V+q-Beta*p;   }

    inline float Alpha_i(float sine_G, float Theta, float Beta, float Phi)
    {   return (-sine_G+sin(Theta)-Beta*sin(Phi))/cos(Phi);   }

    inline float Integrate(float un0, float un1, float yn0, float T)
    {   return (T*(un1+un0)+2*yn0)/2;    }

    inline float Differentiate(float un0, float un1, float T)
    {   return (un1-un0)/T;   }

    inline float Low_Pass(float un0, float un1, float un2, float yn0, float yn1, float Z)
    {   return (un2+2*un1+un0-yn1*2*(1+Z)*(1-Z)-yn0*(1-Z)*(1-Z))/(1+Z)/(1+Z); }

    inline float High_Pass(float un0, float un1, float un2, float yn0, float yn1, float Z)
    {   return (2*Z*(un2*(1/2*Z+1)-un1*Z+un0*(1/2*Z-1))-yn1*2*(1+Z)*(1-Z)-yn0*(1-Z)*(1-Z))/(1+Z)/(1+Z);   }
    
    public:
    
    // Constructor
    complementary_filter(float T = 0.015, float Tau = 3): 
    T(T), Tau(Tau), g(32.174)
    {Z = 2*Tau/T;}
    
    // Initialize
    int init(float Alt, float Beta_v, float Alpha_v, float Theta, float Phi, float Vel, float ny, float r, float p, float nz, float q)
    {
        float V_dot;
    	if(Vel>0.0 && cos(Phi)!=0.0)
    	{
          	//printf("BeginInit: h[0]=%f h[1]=%f V[0]=%f V[1]=%f B_dot_i[0]=%f B_dot_i[1]=%f B_i[0]=%f B_i[1]=%f B_i[2]=%f B_if[0]=%f B_if[1]=%f B_if[2]=%f B_v[0]=%f B_v[1]=%f B_v[2]=%f  B_vf[0]=%f B_vf[1]=%f B_vf[2]=%f B_cf[0]=%f B_cf[1]=%f \n ", h[0], h[1], V[0], V[1], B_dot_i[0], B_dot_i[1], B_i[0], B_i[1], B_i[2], B_if[0], B_if[1], B_if[2], B_v[0], B_v[1], B_v[2], B_vf[0], B_vf[1], B_vf[2], B_cf[0], B_cf[1]);            
            h[0] = Alt;
            V[0] = Vel;
			B_v[0] = Beta_v; 
            B_v[1] = Beta_v;
            B_vf[0] = Beta_v; 
            B_vf[1] = Beta_v;
            B_i[0] = Beta_v; 
            B_i[1] = Beta_v; 
            B_i[2] = Beta_v; // B_i[2] is over-written during runtime
            B_if[0] = 0; 
            B_if[1] = 0;
            B_cf[0] = Beta_v; 
            B_cf[1] = Beta_v; // B_cf[1] is over-written during runtime
            A_v[0] = Alpha_v; 
            A_v[1] = Alpha_v;
            A_vf[0] = Alpha_v; 
            A_vf[1] = Alpha_v;
            A_i[0] = Alpha_v; 
            A_i[1] = Alpha_v; 
            A_i[2] = Alpha_v; // A_i[2] is over-written during runtime
            A_if[0] = 0; 
            A_if[1] = 0;
            A_cf[0] = Alpha_v; 
            A_cf[1] = Alpha_v; // A_cf[1] is over-written during runtime
            V_dot = 0;
            B_dot_i[0] = Beta_dot_i(g, Vel, ny, Theta, Phi, B_cf[1], V_dot, r, A_i[1], p);
            A_dot_i[0] = Alpha_dot_i(g, Vel, nz, Theta, Phi, A_v[1], V_dot, q, B_i[1], p);
          	//printf("EndInit: h[0]=%f h[1]=%f V[0]=%f V[1]=%f B_dot_i[0]=%f B_dot_i[1]=%f B_i[0]=%f B_i[1]=%f B_i[2]=%f B_if[0]=%f B_if[1]=%f B_if[2]=%f B_v[0]=%f B_v[1]=%f B_v[2]=%f  B_vf[0]=%f B_vf[1]=%f B_vf[2]=%f B_cf[0]=%f B_cf[1]=%f \n ", h[0], h[1], V[0], V[1], B_dot_i[0], B_dot_i[1], B_i[0], B_i[1], B_i[2], B_if[0], B_if[1], B_if[2], B_v[0], B_v[1], B_v[2], B_vf[0], B_vf[1], B_vf[2], B_cf[0], B_cf[1]);                    	
            return 1;
		}
		else
		return 0;
    }
    
    void update()
    {
        h[0] = h[1];
        V[0] = V[1];
        B_dot_i[0] = B_dot_i[1];
        B_i[0] = B_i[1];
        B_i[1] = B_i[2];
        B_if[0] = B_if[1];
        B_if[1] = B_if[2];
        B_v[0] = B_v[1];
        B_v[1] = B_v[2];
        B_vf[0] = B_vf[1];
        B_vf[1] = B_vf[2];
        B_cf[0] = B_cf[1];
        A_dot_i[0] = A_dot_i[1];
        A_i[0] = A_i[1];
        A_i[1] = A_i[2];
        A_if[0] = A_if[1];
        A_if[1] = A_if[2];
        A_v[0] = A_v[1];
        A_v[1] = A_v[2];
        A_vf[0] = A_vf[1];
        A_vf[1] = A_vf[2];
        A_cf[0] = A_cf[1];
    }

    void runFilter(float smplng_tm, float Beta_v, float Alpha_v, float Alt, float Vel, float Theta, float ny, float Phi, float r, float p, float nz, float q)
    {
        float h_dot, V_dot, sine_G;
        
        if(Vel>0.0 && cos(Phi)!=0.0)
        {
            //printf("BeginrunFilter: h[0]=%f h[1]=%f V[0]=%f V[1]=%f B_dot_i[0]=%f B_dot_i[1]=%f B_i[0]=%f B_i[1]=%f B_i[2]=%f B_if[0]=%f B_if[1]=%f B_if[2]=%f B_v[0]=%f B_v[1]=%f B_v[2]=%f  B_vf[0]=%f B_vf[1]=%f B_vf[2]=%f B_cf[0]=%f B_cf[1]=%f \n ", h[0], h[1], V[0], V[1], B_dot_i[0], B_dot_i[1], B_i[0], B_i[1], B_i[2], B_if[0], B_if[1], B_if[2], B_v[0], B_v[1], B_v[2], B_vf[0], B_vf[1], B_vf[2], B_cf[0], B_cf[1]);       
            T = (smplng_tm>0)? smplng_tm : T;
            Z = 2*Tau/T;
            A_v[2] = Alpha_v;
            B_v[2] = Beta_v;
            h[1] = Alt;
            V[1] = Vel;
            h_dot = Differentiate(h[0],h[1],T);
            V_dot = Differentiate(V[0],V[1],T);
            sine_G = h_dot/Vel;
            sine_G = sine_G>1 ? 1 : (sine_G<-1 ? -1 : sine_G);
            A_i[2] = Alpha_i(sine_G, Theta, B_cf[0], Phi);
            B_dot_i[1] = Beta_dot_i(g, Vel, ny, Theta, Phi, B_cf[0], V_dot, r, A_i[2], p);
            B_i[2] = Integrate(B_dot_i[0], B_dot_i[1], B_i[1], T);
            B_if[2] = High_Pass(B_i[0], B_i[1], B_i[2], B_if[0], B_if[1], Z);
            B_vf[2] = Low_Pass(B_v[0], B_v[1], B_v[2], B_vf[0], B_vf[1], Z);
            B_cf[1] = B_vf[2] + B_if[2];
            A_dot_i[1] = Alpha_dot_i(g, Vel, nz, Theta, Phi, A_cf[0], V_dot, q, B_cf[1], p);
            A_i[2] = Integrate(A_dot_i[0], A_dot_i[1], A_i[1], T);
            A_if[2] = High_Pass(A_i[0], A_i[1], A_i[2], A_if[0], A_if[1], Z);
            A_vf[2] = Low_Pass(A_v[0], A_v[1], A_v[2], A_vf[0], A_vf[1], Z);
            A_cf[1] = A_vf[2] + A_if[2];
            update();
            //printf("EndrunFilter: h[0]=%f h[1]=%f V[0]=%f V[1]=%f B_dot_i[0]=%f B_dot_i[1]=%f B_i[0]=%f B_i[1]=%f B_i[2]=%f B_if[0]=%f B_if[1]=%f B_if[2]=%f B_v[0]=%f B_v[1]=%f B_v[2]=%f  B_vf[0]=%f B_vf[1]=%f B_vf[2]=%f B_cf[0]=%f B_cf[1]=%f \n ", h[0], h[1], V[0], V[1], B_dot_i[0], B_dot_i[1], B_i[0], B_i[1], B_i[2], B_if[0], B_if[1], B_if[2], B_v[0], B_v[1], B_v[2], B_vf[0], B_vf[1], B_vf[2], B_cf[0], B_cf[1]);
         }
    }
    
    // Get Alpha-Inertial
    float getAlphai()
    {
        return A_i[2];
    }
        
    // Get Alpha-Filtered
    float getAlphacf()
    {
        return A_cf[1];
    }
    
    // Get Beta-Inertial
    float getBetai()
    {
        return B_i[2];
    }
    
    // Get Beta-Filtered
    float getBetacf()
    {
        return B_cf[1];
    }
};

#endif
