#include "Arduino.h"
#include "PID.h"
#include "my_serial.h"
PID::PID(bool constrain_out) {
	constrain_output = constrain_out;
	e_last=0;
	s=0;
}

float PID::calculate_out(float e_new, float timediff){
	if(timediff > 0){
		float inverse_tdiff = 1/timediff;
		y = k_p * e_new;
		y += k_d * inverse_tdiff * (e_new - e_last);
		e_last = e_new; //Zwischenspeichern für Differenz beim nächsten Mal
		s += e_new*timediff;
		if(anti_windup && (s>300)){
			s=300;
		}
		y += k_i * s;
		y_constrained = constrain(y, 0, 243); // will be implemented next
		if(constrain_output){return y_constrained;}
	}else{ y = 0;}
	return y;
}

void PID::set_parameters(float newk_p, float newk_i, float newk_d){
	k_p = newk_p;
	k_i = newk_i;
	k_d = newk_d;
}

void PID::set_antiwindup(bool setting){
	anti_windup = setting;
}