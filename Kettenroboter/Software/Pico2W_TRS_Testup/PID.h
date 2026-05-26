#ifndef PID_h
#define PID_h
#include <float.h>
class PID{
	public:
		PID(bool constrain_out);
		bool constrain_output;
		float calculate_out(float e_new, float timediff);
		void set_parameters(float k_p, float k_i, float k_d);
		
		
	private: // alles floats
		float e_last;
		float k_i;
		float k_d;
		float k_p;
		float s;
		float y;
		float y_constrained; 
		const float max_v = FLT_MAX;
};
#endif