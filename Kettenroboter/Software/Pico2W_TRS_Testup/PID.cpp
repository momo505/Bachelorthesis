#include "Arduino.h"
#include "PID.h"
PID::PID(bool constrain_out) {
	constrain_output = constrain_out;
	e_last=0;
	s=0;
}

float PID::calculate_out(float e_new, float timediff){
	float inverse_tdiff = 1/timediff;
	y = k_p * e_new;
	y += k_d * inverse_tdiff * (e_new - e_last);
	e_last = e_new;
	s += e_new*timediff;
	y += k_i * s;
	y_constrained = y; // will be implemented next
	if(constrain_output){return y_constrained;}
	return y;
}

void PID::set_parameters(float newk_p, float newk_i, float newk_d){
	k_p = newk_p;
	k_i = newk_i;
	k_d = newk_d;
}