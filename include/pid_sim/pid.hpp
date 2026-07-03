#pragma once

namespace pid_sim {
	// PID controller with anti-windup on the integral term.
	struct PID {
		float Kp, Ki, Kd;
		float error[3] {}; // [0]=latest, [1]=prev, [2]=oldest
		float Iout = 0, max_iout;

		PID(float Kp, float Ki, float Kd, float max_iout = 50) : Kp(Kp), Ki(Ki), Kd(Kd), max_iout(max_iout) {}

		// PID control signal (P + I + D).
		float calculate(float target, float actual);
		// Bang-bang control: full force toward target.
		float bangbang(float target, float actual);
	
	};
}