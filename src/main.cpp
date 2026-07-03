#include <cstdlib>
#include <cstdio>
#include <chrono>
#include <thread>
#include "pid_sim/plant.hpp"
#include "pid_sim/pid.hpp"

struct RenderConfig {
    int width = 40;
};

struct RenderData {
    float target_pos, actual_pos, output;
    float Kp, Ki, Kd;
    int step;
};

RenderData capture(const pid_sim::PID& pid, const pid_sim::Plant& plant, float target, float output, int step) {
    return {
        .target_pos = target,
        .actual_pos = plant.position,
        .output = output,
        .Kp = pid.Kp, .Ki = pid.Ki, .Kd = pid.Kd,
        .step = step,
    };
}

void render(const RenderConfig& cfg, const RenderData& data) {
	printf("\033[H");
    printf(" Kp=%.2f  Ki=%.2f  Kd=%.2f  |  step %3d\n\n", data.Kp, data.Ki, data.Kd, data.step);
    printf(" target ──●──  │  actual ──◯──\n");
    printf("─────────────────────────────────────────────────\n");

    for (int i = 0; i < cfg.width; i++) {
        bool is_target = (int)(data.target_pos / 200.0f * cfg.width) == i;
        bool is_actual = (int)(data.actual_pos / 200.0f * cfg.width) == i;
        if (is_target && is_actual) printf("╳");
        else if (is_target) printf("●");
        else if (is_actual) printf("◯");
        else printf("·");
    }
    printf("\n");
    printf("─────────────────────────────────────────────────\n");
}

int main(int argc, char** argv) {
    float Kp = 1.0, Ki = 0.0, Kd = 0.0;
    float mass = 0, drag = 0;

    if (argc >= 4) {
        Kp  = atof(argv[1]);
        Ki  = atof(argv[2]);
        Kd  = atof(argv[3]);
    }
    if (argc >= 6) {
        mass = atof(argv[4]);
        drag = atof(argv[5]);
    }

    srand(time(0));

    if (mass == 0) mass = 1.0f + ((float)rand() / RAND_MAX) * 4.0f;
    if (drag == 0) drag = 0.01f + ((float)rand() / RAND_MAX) * 0.49f;

    printf("\033[2J\033[H");
    printf(" Car: mass=%.1f kg  drag=%.2f\n", mass, drag);
    printf(" Usage: ./pid-simulator [Kp Ki Kd] [mass drag]\n");
    printf("   eg:  ./pid-simulator 2.0 0.5 0.3\n");
    printf("        ./pid-simulator 2.0 0.5 0.3 2.4 0.02\n");
    printf(" Press Ctrl+C to exit\n");
    std::this_thread::sleep_for(std::chrono::seconds(2));

    pid_sim::Plant car(mass, drag);
    pid_sim::PID pid(Kp, Ki, Kd);
    float target = 100.0f;
    float dt = 0.1f;
    int width = 40;
    int settled_count = 0;

    RenderConfig cfg {};

    float settle_threshold = 1.0f;

    for (int step = 0; step < 500; step++) {
        float output = pid.calculate(target, car.position);
        // float output = pid.bangbang(target, car.position);
        car.update(output, dt);
        RenderData data = capture(pid, car, target, output, step);
        render(cfg, data);

        float error = target - car.position;
        if (error < 0) error = -error;
        if (error < settle_threshold) {
            settled_count++;
            if (settled_count >= 10) {
                printf("\nSettled at step %d (error < %f for 10 steps)\n", step - 9, settle_threshold);
                printf("Time to stability: %.1fs\n", step * dt);
                break;
            }
        } else {
            settled_count = 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    printf("\nDone. Final error: %.2f\n", target - car.position);
    return 0;
}
