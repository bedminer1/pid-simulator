#include <cmath>
#include <cstdlib>
#include <chrono>
#include <thread>
#include <vector>

#include "pid_sim/plant.hpp"
#include "pid_sim/pid.hpp"
#include "pid_sim/render.hpp"
#include "pid_sim/metrics.hpp"

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

    render::splash(mass, drag);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    plant::Plant car(mass, drag);
    pid::PID pid(Kp, Ki, Kd);
    float target = 100.0f;
    std::vector<float> positions {};
    
    int step = 0;
    float dt = 0.1f;
    int settled_count = 0;
    bool settled = false;

    float settle_threshold = 1.0f;

    // Simulation loop: sense → control → act → render.
    for (; step < 500; ++step) {
        float output = pid.calculate(target, car.position);
        // float output = pid.bangbang(target, car.position);
        car.update(output, dt);
        render::render_frame({.step = step, .Kp = pid.Kp, .Ki = pid.Ki, .Kd = pid.Kd,
            .target = target, .actual = car.position, .output = output});

        float error = fabs(target - car.position);
        if (error < settle_threshold) {
            settled_count++;
            if (settled_count >= 10) {
           		settled = true;
                break;
            }
        } else {
            settled_count = 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    pid_sim::StepMetrics m = pid_sim::compute_metrics(positions, target, dt);
    printf("rise=%.2fs overshoot=%.1f%% settle=%.2fs sse=%.3f\n",
        m.rise_time, m.overshoot_pct, m.settling_time, m.steady_state_error);
    render::announce_end(settled, step, dt, settle_threshold, target - car.position);

    return 0;
}
