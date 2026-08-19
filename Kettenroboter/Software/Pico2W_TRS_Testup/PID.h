#ifndef PID_h
#define PID_h
#include <float.h>

class PID{
	public:
		PID(bool constrain_out);
		bool constrain_output;
		float calculate_out(float e_new, float timediff);
		void set_parameters(float k_p, float k_i, float k_d);
		void set_antiwindup(bool setting);
		float y;
	private: // alles floats
		float e_last;
		float k_i;
		float k_d;
		float k_p;
		float s;
		float y_constrained; 
		bool anti_windup = true;
		//bool signofy;
		const float max_v = FLT_MAX;
		const float min_v = FLT_MIN;
};
#endif