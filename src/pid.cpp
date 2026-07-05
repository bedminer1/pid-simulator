#include "pid_sim/pid.hpp"
#include "pid_sim/math_utils.hpp"
#include <limits>

namespace pid {
	// PID update: error history → P + I (with anti-windup) + D → control signal.
float PID::calculate(float target, float actual) {
		error[2] = error[1];
		error[1] = error[0];
		error[0] = target - actual;

		float dt = 0.1f;

		float P = Kp * error[0];
		Iout += Ki * error[0] * dt;
		Iout = pid_sim::clamp(Iout, -max_iout, max_iout);
		float D = Kd * (error[0] - error[1]) / dt;

		return P + Iout + D;
	}

	// Bang-bang: full positive or negative force — no proportional band.
float PID::bangbang(float target, float actual) {
		error[2] = error[1];
		error[1] = error[0];
		error[0] = target - actual;

		if (error[0] > 0) {
			return std::numeric_limits<float>::max();
		}

		return -(std::numeric_limits<float>::max() - 1);
	}
};