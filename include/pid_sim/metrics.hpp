#pragma once

#include <vector>

namespace pid_sim {
	struct StepMetrics {
		float rise_time;
		float overshoot_pct;
		float settling_time;
		float steady_state_error;

	};
	StepMetrics compute_metrics(const std::vector<float>& position,
		float target, float settle_band_pct = 2.0f);
}