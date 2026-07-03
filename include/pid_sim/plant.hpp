#pragma once

#include <cstdlib>

namespace pid_sim {
	struct Plant {
		float velocity = 0, position = 0;
		float mass, drag;
		float max_force = 200;

		Plant(float mass, float drag) : mass(mass),  drag(drag) {}

		void update(float force, float dt);
	};
}