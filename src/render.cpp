#include "pid_sim/render.hpp"
#include <cstdio>

namespace render {
	// Clears screen and prints the car info + usage header.
	void splash(float mass, float drag) {
	  	printf("\033[2J\033[H");
	    printf(" Car: mass=%.1f kg  drag=%.2f\n", mass, drag);
	    printf(" Usage: ./pid-simulator [Kp Ki Kd] [mass drag]\n");
	    printf("   eg:  ./pid-simulator 2.0 0.5 0.3\n");
	    printf("        ./pid-simulator 2.0 0.5 0.3 2.4 0.02\n");
	    printf(" Press Ctrl+C to exit\n");
	}
	 
	// Draws one animation frame: gains bar → target/actual bar chart.
	void render_frame(RenderData data, int width) {
		printf("\033[6;1H");
	 	printf(" Kp=%.2f  Ki=%.2f  Kd=%.2f  |  step %3d\n\n", data.Kp, data.Ki, data.Kd, data.step);
	    printf(" target ──●──  │  actual ──◯──\n");
	    printf("─────────────────────────────────────────────────\n");

	    for (int i = 0; i < width; i++) {
	        bool is_target = (int)(data.target / 200.0f * width) == i;
	        bool is_actual = (int)(data.actual / 200.0f * width) == i;
	        if (is_target && is_actual) printf("╳");
	        else if (is_target) printf("●");
	        else if (is_actual) printf("◯");
	        else printf("·");
	    }
	    printf("\n");
		printf("─────────────────────────────────────────────────\n");
	}

	// Prints the settling summary (or timeout) after simulation ends.
	void announce_end(bool settled, int step, float dt, float threshold, float error) {
		if (!settled) {
			printf("Didn't settle within 500 steps\n");
			printf("Done. Final error: %.2f\n", error);
			return;
		}
	 	printf("\nSettled at step %d (error < %.1f for 10 steps)\n", step - 9, threshold);
        printf("Time to stability: %.1fs\n", step * dt);
        printf("Done. Final error: %.2f\n", error);
	}
};
