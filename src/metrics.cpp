#include "pid_sim/metrics.hpp"

namespace pid_sim {
	StepMetrics compute_metrics(const std::vector<float> &position, float target, float settle_band_pct) {
		return StepMetrics {
			.overshoot_pct = 1.0f,
			.rise_time = 1.0f,
			.settling_time = 1.0f,
			.steady_state_error = 1.0f
		};
	};
};