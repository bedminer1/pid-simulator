#pragma once

namespace pid_sim {
	struct PID {
		float Kp, Ki, Kd;
		float error[3] {}; // most to least recent
		float Iout = 0, max_iout;

		PID(float Kp, float Ki, float Kd, float max_iout = 50) : Kp(Kp), Ki(Ki), Kd(Kd), max_iout(max_iout) {}

		float calculate(float target, float actual);
		float bangbang(float target, float actual);
		
	};
}