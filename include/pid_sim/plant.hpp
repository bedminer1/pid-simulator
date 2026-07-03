#pragma once

namespace pid_sim {
	// 1D physics model: mass + drag + noise.
	struct Plant {
		float velocity = 0, position = 0;
		float mass, drag;
		float max_force = 200;

		Plant(float mass, float drag) : mass(mass),  drag(drag) {}

		// Advances the simulation by dt seconds.
		void update(float force, float dt);
	};
}