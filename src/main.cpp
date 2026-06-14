#include <cstdio>
#include <cmath>
#include <random>
#include <thread>
#include <chrono>
#include <cstdlib>

struct Plant {
    float position = 0, velocity = 0;
    float mass, drag;

    Plant(float mass, float drag) : mass(mass), drag(drag) {}

    void update(float force, float dt) {
        float noise = ((float)rand() / RAND_MAX - 0.5f) * 0.5f;
        float accel = force / mass - drag * velocity + noise;
        velocity += accel * dt;
        position += velocity * dt;
    }
};

struct PID {
    float Kp, Ki, Kd;
    float error[3] = {}, Iout = 0, max_iout;

    PID(float Kp, float Ki, float Kd, float max_iout = 50)
        : Kp(Kp), Ki(Ki), Kd(Kd), max_iout(max_iout) {}

    float calculate(float set, float ref) {
        error[2] = error[1];
        error[1] = error[0];
        error[0] = set - ref;

        float P = Kp * error[0];
        Iout += Ki * error[0] * 0.1f;
        if (Iout > max_iout) Iout = max_iout;
        if (Iout < -max_iout) Iout = -max_iout;
        float D = Kd * (error[0] - error[1]) / 0.1f;

        return P + Iout + D;
    }
};

void render(float target, float actual, float output, int step, float Kp, float Ki, float Kd, int width) {
    printf("\033[H");
    printf(" Kp=%.2f  Ki=%.2f  Kd=%.2f  |  step %3d\n\n", Kp, Ki, Kd, step);
    printf(" target ──●──  │  actual ──◯──\n");
    printf("────────────────┼────────────────────────────────\n");

    for (int i = 0; i < width; i++) {
        bool is_target = (int)(target / 200.0f * width) == i;
        bool is_actual = (int)(actual / 200.0f * width) == i;
        if (is_target && is_actual) printf("╳");
        else if (is_target) printf("●");
        else if (is_actual) printf("◯");
        else printf("·");
    }
    printf("\n");
    printf("────────────────┼────────────────────────────────\n");
    printf("               │  ");
    for (int i = 0; i < width; i++) {
        if ((int)(output / 200.0f * width) == i) printf("|");
        else printf(" ");
    }
    printf("  output\n");
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

    Plant car(mass, drag);
    PID pid(Kp, Ki, Kd);
    float target = 100.0f;
    float dt = 0.1f;
    int width = 40;
    int settled_count = 0;

    for (int step = 0; step < 500; step++) {
        float output = pid.calculate(target, car.position);
        car.update(output, dt);
        render(target, car.position, output, step, Kp, Ki, Kd, width);

        float error = target - car.position;
        if (error < 0) error = -error;
        if (error < 1.0f) {
            settled_count++;
            if (settled_count >= 10) {
                printf("\nSettled at step %d (error < 1.0 for 10 steps)\n", step - 9);
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
