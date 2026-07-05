#include "pid_sim/plant.hpp"
#include "pid_sim/math_utils.hpp"
#include <cstdlib>

namespace plant {
	// Advances the 1D mass-drag plant by dt, with noise and force clamping.
void Plant::update(float force, float dt) {
		force = pid_sim::clamp(force, -max_force, max_force);
		// ranging +- 0.25
		float noise = ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
		// a = F/m - drag*V + noise
		float accel = force / mass - drag * velocity + noise;
		velocity += accel * dt;
		position += velocity * dt;
	};
}