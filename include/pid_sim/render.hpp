#pragma once

namespace render {
	// Snapshot of simulation state for rendering.
	struct RenderData {
		int step;
		float Kp, Ki, Kd;
		float target, actual, output;
	};

	// Startup header: car info + usage.
	void splash(float mass, float drag);
	// One frame of the ASCII bar-chart animation.
	void render_frame(RenderData data, int width = 40);
	// Settling/timeout summary printed after the sim ends.
	void announce_end(bool settled, int step, float dt, float threshold, float error);
};