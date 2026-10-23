#include "sparcbot/config/robot_config.hpp"
#include "sparcbot/hardware/motor.hpp"
#include "sparcbot/hardware/robot_motors.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <vector>
#include <stdexcept>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

// ============================================================
// GLOBAL
// ============================================================

static volatile std::sig_atomic_t keep_running = 1;

void signalHandler(int)
{
    keep_running = 0;
}

// ============================================================
// CONFIGURATION
// ============================================================

constexpr double CPR = 500.0;

// ---------- Breakaway search ----------

// First find the approximate region quickly.
constexpr double BREAKAWAY_COARSE_START = 0.06;
constexpr double BREAKAWAY_COARSE_END = 0.80;
constexpr double BREAKAWAY_COARSE_STEP = 0.02;

// Then refine to 0.01 resolution.
constexpr double BREAKAWAY_FINE_STEP = 0.01;

// Number of successful starts required at final candidate.
constexpr int BREAKAWAY_VERIFY_TRIALS = 3;

// Maximum time motor may remain powered during one startup attempt.
constexpr int BREAKAWAY_MAX_PULSE_MS = 150;

// Encoder checked this often.
constexpr int BREAKAWAY_SAMPLE_MS = 5;

// Two counts = 1.44 degrees of wheel rotation.
constexpr long long BREAKAWAY_MIN_COUNTS = 2;

// Rest between startup attempts.
constexpr int REST_MS = 500;


// ---------- Stop-duty search ----------

// Start slightly above measured breakaway.
constexpr double STOP_START_MARGIN = 0.04;

// Reduce command this much at each step.
constexpr double STOP_STEP = 0.01;

// Time to observe each running-duty level.
constexpr int STOP_STEP_MS = 120;

// Minimum encoder movement during one stop interval.
constexpr long long STOP_MIN_COUNTS = 2;

// Repeat complete stop experiment this many times.
constexpr int STOP_VERIFY_TRIALS = 3;


// ============================================================
// DATA STRUCTURES
// ============================================================

struct StartupTrial {
    bool moved = false;
    long long powered_delta = 0;
    long long passive_delta = 0;
    long long max_progress = 0;
    double detection_time_s = 0.0;
};

struct StopTrial {
    double stop_duty = -1.0;
};

struct DirectionResult {
    double left_breakaway = -1.0;
    double left_stop = -1.0;

    double right_breakaway = -1.0;
    double right_stop = -1.0;
};

// ============================================================
// UTILITIES
// ============================================================

bool sleepInterruptible(int ms)
{
    while (keep_running && ms > 0) {
        const int chunk = std::min(ms, 5);
        std::this_thread::sleep_for(std::chrono::milliseconds(chunk));
        ms -= chunk;
    }

    return keep_running;
}

std::string wheelName(bool test_left)
{
    return test_left ? "LEFT" : "RIGHT";
}

std::string directionName(int direction)
{
    return direction > 0 ? "FORWARD" : "REVERSE";
}

void coastAll(RobotMotors& motors)
{
    motors.coast();
}

// ============================================================
// COMMAND ONE MOTOR ONLY
// ============================================================

void commandSingleWheel(RobotMotors& motors, bool test_left, double command)
{
    if (test_left) {
        motors.right().coast();
        motors.left().setCommand(command);
    }
    else {
        motors.left().coast();
        motors.right().setCommand(command);
    }
}

// ============================================================
// BREAKAWAY: ONE STARTUP ATTEMPT
// ============================================================

StartupTrial runStartupTrial(RobotMotors& motors, bool test_left, int direction, double duty)
{
    coastAll(motors);

    if (!sleepInterruptible(REST_MS)) {
        return {};
    }

    const RobotMotorState start_state = motors.getState();

    const long long powered_start = test_left ? start_state.left.counts : start_state.right.counts;
    const long long passive_start = test_left ? start_state.right.counts : start_state.left.counts;

    commandSingleWheel(motors, test_left, direction * duty);

    const auto start_time = std::chrono::steady_clock::now();

    long long max_progress = 0;
    bool moved = false;
    double detection_time_s = 0.0;

    for (int elapsed_ms = 0;
         elapsed_ms < BREAKAWAY_MAX_PULSE_MS && keep_running;
         elapsed_ms += BREAKAWAY_SAMPLE_MS) {

        sleepInterruptible(BREAKAWAY_SAMPLE_MS);

        const RobotMotorState state = motors.getState();
        const long long powered_now = test_left ? state.left.counts : state.right.counts;

        const long long signed_progress = direction * (powered_now - powered_start);
        max_progress = std::max(max_progress, signed_progress);

        if (max_progress >= BREAKAWAY_MIN_COUNTS) {
            moved = true;

            const auto detection_time = std::chrono::steady_clock::now();
            detection_time_s = std::chrono::duration<double>(detection_time - start_time).count();

            break;
        }
    }

    coastAll(motors);

    const RobotMotorState end_state = motors.getState();

    const long long powered_end = test_left ? end_state.left.counts : end_state.right.counts;
    const long long passive_end = test_left ? end_state.right.counts : end_state.left.counts;

    StartupTrial result;
    result.moved = moved;
    result.powered_delta = powered_end - powered_start;
    result.passive_delta = passive_end - passive_start;
    result.max_progress = max_progress;
    result.detection_time_s = detection_time_s;

    return result;
}

// ============================================================
// BREAKAWAY: TEST ONE DUTY
// ============================================================

bool verifyBreakawayDuty(RobotMotors& motors,
                         bool test_left,
                         int direction,
                         double duty,
                         int trials)
{
    int successes = 0;

    std::cout << std::fixed << std::setprecision(2)
              << "  duty " << duty << " : ";

    for (int trial = 1; trial <= trials && keep_running; ++trial) {
        const StartupTrial result = runStartupTrial(motors, test_left, direction, duty);

        if (result.moved) {
            ++successes;
            std::cout << "PASS";
        }
        else {
            std::cout << "FAIL";
        }

        std::cout << "(dN=" << result.powered_delta;

        if (result.moved) {
            std::cout << ", t=" << std::setprecision(3)
                      << result.detection_time_s << "s";
        }

        std::cout << ")";

        if (trial < trials) {
            std::cout << " | ";
        }
    }

    std::cout << "  [" << successes << "/" << trials << "]\n";

    return successes == trials;
}

// ============================================================
// BREAKAWAY: COARSE + FINE SEARCH
// ============================================================

double characterizeBreakaway(RobotMotors& motors, bool test_left, int direction)
{
    std::cout << "\n========================================\n"
              << wheelName(test_left) << " "
              << directionName(direction)
              << " BREAKAWAY\n"
              << "========================================\n";

    std::cout << "Coarse search:\n";

    double first_moving_duty = -1.0;
    double previous_duty = BREAKAWAY_COARSE_START;

    for (double duty = BREAKAWAY_COARSE_START;
         duty <= BREAKAWAY_COARSE_END + 1e-9 && keep_running;
         duty += BREAKAWAY_COARSE_STEP) {

        const StartupTrial trial = runStartupTrial(motors, test_left, direction, duty);

        std::cout << std::fixed << std::setprecision(2)
                  << "  duty " << duty
                  << " : " << (trial.moved ? "MOVE" : "NO MOVE")
                  << "   powered dN=" << trial.powered_delta
                  << " passive dN=" << trial.passive_delta;

        if (trial.moved) {
            std::cout << "   detect=" << std::setprecision(3)
                      << trial.detection_time_s << " s";
        }

        std::cout << '\n';

        if (trial.moved) {
            first_moving_duty = duty;
            break;
        }

        previous_duty = duty;
    }

    if (first_moving_duty < 0.0) {
        std::cout << "\nNo breakaway found up to "
                  << BREAKAWAY_COARSE_END << ".\n";

        return -1.0;
    }

    // Search between last failure and first success.
    const double fine_start =
        std::max(BREAKAWAY_COARSE_START,
                 previous_duty + BREAKAWAY_FINE_STEP);

    std::cout << "\nFine search:\n";

    double candidate = first_moving_duty;

    for (double duty = fine_start;
         duty <= first_moving_duty + 1e-9 && keep_running;
         duty += BREAKAWAY_FINE_STEP) {

        const StartupTrial trial = runStartupTrial(motors, test_left, direction, duty);

        std::cout << std::fixed << std::setprecision(2)
                  << "  duty " << duty
                  << " : " << (trial.moved ? "MOVE" : "NO MOVE")
                  << "   dN=" << trial.powered_delta;

        if (trial.moved) {
            std::cout << "   detect=" << std::setprecision(3)
                      << trial.detection_time_s << " s";
        }

        std::cout << '\n';

        if (trial.moved) {
            candidate = duty;
            break;
        }
    }

    // --------------------------------------------------------
    // Reliability verification
    // --------------------------------------------------------

    std::cout << "\nReliability verification at duty "
              << std::fixed << std::setprecision(2)
              << candidate << ":\n";

    if (verifyBreakawayDuty(
            motors,
            test_left,
            direction,
            candidate,
            BREAKAWAY_VERIFY_TRIALS)) {

        std::cout << "\n*** RELIABLE BREAKAWAY = "
                  << candidate << " ***\n";

        return candidate;
    }

    // If candidate is marginal, increase by 0.01 until reliable.
    std::cout << "\nCandidate was not reliable. Increasing duty:\n";

    for (double duty = candidate + BREAKAWAY_FINE_STEP;
         duty <= BREAKAWAY_COARSE_END + 1e-9 && keep_running;
         duty += BREAKAWAY_FINE_STEP) {

        if (verifyBreakawayDuty(
                motors,
                test_left,
                direction,
                duty,
                BREAKAWAY_VERIFY_TRIALS)) {

            std::cout << "\n*** RELIABLE BREAKAWAY = "
                      << duty << " ***\n";

            return duty;
        }
    }

    return -1.0;
}

// ============================================================
// STOP TEST: ONE COMPLETE DOWNWARD TRIAL
// ============================================================

StopTrial runStopTrial(RobotMotors& motors,
                       bool test_left,
                       int direction,
                       double breakaway_duty)
{
    coastAll(motors);

    if (!sleepInterruptible(REST_MS)) {
        return {};
    }

    // Give a small margin above breakaway so the wheel definitely starts.
    const double start_duty =
        std::min(1.0, breakaway_duty + STOP_START_MARGIN);

    commandSingleWheel(motors, test_left, direction * start_duty);

    // Confirm startup using encoder movement.
    const RobotMotorState start_state = motors.getState();
    const long long start_count =
        test_left ? start_state.left.counts : start_state.right.counts;

    bool started = false;

    for (int elapsed_ms = 0;
         elapsed_ms < BREAKAWAY_MAX_PULSE_MS && keep_running;
         elapsed_ms += BREAKAWAY_SAMPLE_MS) {

        sleepInterruptible(BREAKAWAY_SAMPLE_MS);

        const RobotMotorState state = motors.getState();
        const long long current_count =
            test_left ? state.left.counts : state.right.counts;

        const long long progress =
            direction * (current_count - start_count);

        if (progress >= BREAKAWAY_MIN_COUNTS) {
            started = true;
            break;
        }
    }

    if (!started) {
        coastAll(motors);

        std::cout << "    WARNING: wheel did not start at "
                  << start_duty << '\n';

        return {};
    }

    // IMPORTANT:
    // Do NOT coast between these steps.
    // The purpose is to determine how low the command can become
    // while an already-moving wheel continues moving.

    for (double duty = start_duty;
         duty >= 0.0 && keep_running;
         duty -= STOP_STEP) {

        const RobotMotorState before = motors.getState();
        const long long count_before =
            test_left ? before.left.counts : before.right.counts;

        commandSingleWheel(motors, test_left, direction * duty);

        sleepInterruptible(STOP_STEP_MS);

        const RobotMotorState after = motors.getState();
        const long long count_after =
            test_left ? after.left.counts : after.right.counts;

        const long long progress =
            direction * (count_after - count_before);

        std::cout << std::fixed << std::setprecision(2)
                  << "    duty " << duty
                  << " -> dN=" << progress << '\n';

        if (progress < STOP_MIN_COUNTS) {
            coastAll(motors);

            StopTrial result;
            result.stop_duty = std::max(0.0, duty);

            return result;
        }
    }

    coastAll(motors);

    StopTrial result;
    result.stop_duty = 0.0;

    return result;
}

// ============================================================
// STOP DUTY: REPEATED MEASUREMENT
// ============================================================

double characterizeStopDuty(RobotMotors& motors,
                            bool test_left,
                            int direction,
                            double breakaway_duty)
{
    if (breakaway_duty < 0.0) {
        return -1.0;
    }

    std::cout << "\n========================================\n"
              << wheelName(test_left) << " "
              << directionName(direction)
              << " STOP DUTY\n"
              << "========================================\n";

    double sum = 0.0;
    int valid_trials = 0;

    for (int trial = 1;
         trial <= STOP_VERIFY_TRIALS && keep_running;
         ++trial) {

        std::cout << "\n  Stop trial "
                  << trial << "/"
                  << STOP_VERIFY_TRIALS << '\n';

        const StopTrial result =
            runStopTrial(
                motors,
                test_left,
                direction,
                breakaway_duty);

        if (result.stop_duty >= 0.0) {
            std::cout << "  -> stop duty = "
                      << std::fixed
                      << std::setprecision(2)
                      << result.stop_duty
                      << '\n';

            sum += result.stop_duty;
            ++valid_trials;
        }
    }

    if (valid_trials == 0) {
        return -1.0;
    }

    const double average =
        sum / static_cast<double>(valid_trials);

    std::cout << "\nAverage stop duty = "
              << std::fixed
              << std::setprecision(3)
              << average << '\n';

    return average;
}

// ============================================================
// CHARACTERIZE ONE DIRECTION
// ============================================================

DirectionResult characterizeDirection(RobotMotors& motors, int direction)
{
    DirectionResult result;

    std::cout << "\n\n"
              << "########################################\n"
              << "       "
              << directionName(direction)
              << " CHARACTERIZATION\n"
              << "########################################\n";

    result.left_breakaway =
        characterizeBreakaway(
            motors,
            true,
            direction);

    if (!keep_running) return result;

    result.left_stop =
        characterizeStopDuty(
            motors,
            true,
            direction,
            result.left_breakaway);

    if (!keep_running) return result;

    result.right_breakaway =
        characterizeBreakaway(
            motors,
            false,
            direction);

    if (!keep_running) return result;

    result.right_stop =
        characterizeStopDuty(
            motors,
            false,
            direction,
            result.right_breakaway);

    coastAll(motors);

    return result;
}

// ============================================================
// PRINT RESULT
// ============================================================

void printDirectionResult(const DirectionResult& result, int direction)
{
    std::cout << "\n\n"
              << "========================================\n"
              << "       "
              << directionName(direction)
              << " FINAL RESULTS\n"
              << "========================================\n"
              << std::fixed
              << std::setprecision(3);

    std::cout << "LEFT\n"
              << "  breakaway = " << result.left_breakaway << '\n'
              << "  stop      = " << result.left_stop << '\n';

    if (result.left_breakaway >= 0.0 &&
        result.left_stop >= 0.0) {

        std::cout << "  hysteresis = "
                  << result.left_breakaway -
                         result.left_stop
                  << '\n';
    }

    std::cout << "\nRIGHT\n"
              << "  breakaway = "
              << result.right_breakaway
              << '\n'
              << "  stop      = "
              << result.right_stop
              << '\n';

    if (result.right_breakaway >= 0.0 &&
        result.right_stop >= 0.0) {

        std::cout << "  hysteresis = "
                  << result.right_breakaway -
                         result.right_stop
                  << '\n';
    }

    std::cout << "========================================\n";
}

// ============================================================
// STEADY-STATE PWM SWEEP (ONE WHEEL AT A TIME)
// ============================================================

struct SweepPoint { double speed_m_s; double duty; };

// Conservative unloaded diagnostic. Increase only after validating response.
constexpr double SWEEP_START = 0.12;
constexpr double SWEEP_END = 0.90;
constexpr double SWEEP_STEP = 0.03;
constexpr int SWEEP_SETTLE_MS = 350;
constexpr int SWEEP_MEASURE_MS = 650;
constexpr double WHEEL_RADIUS_M = 0.050;

// Prevent an exception or early return from leaving the motors commanded.
struct MotorCoastGuard {
    RobotMotors& motors;
    ~MotorCoastGuard() { motors.coast(); }
};

long long wheelCounts(const RobotMotorState& state, bool left) {
    return left ? state.left.counts : state.right.counts;
}

void printFeedforwardFit(const std::vector<SweepPoint>& points,
                         bool left, int direction) {
    // Least-squares |PWM| = static + velocity_gain * |velocity|.
    if (points.size() < 3) {
        std::cout << "Insufficient valid samples for fit.\n";
        return;
    }
    double sx=0, sy=0, sxx=0, sxy=0;
    for (const auto& p : points) {
        sx += p.speed_m_s;
        sy += p.duty;
        sxx += p.speed_m_s * p.speed_m_s;
        sxy += p.speed_m_s * p.duty;
    }
    const double n = static_cast<double>(points.size());
    const double den = n*sxx - sx*sx;
    if (std::abs(den) < 1e-8) {
        std::cout << "Insufficient velocity spread for fit.\n";
        return;
    }
    const double kv = (n*sxy - sx*sy)/den;
    const double us = (sy - kv*sx)/n;
    std::cout << "\n" << wheelName(left) << " " << directionName(direction)
              << " fit: |PWM| = " << us << " + " << kv << " * |v_m_s|\n";
    if (us < 0 || kv <= 0) {
        std::cout << "WARNING: Nonphysical fit. Check slip, measurement range, and data.\n";
    } else {
        std::cout << "Suggested " << (direction > 0 ? "forward" : "reverse")
                  << "_static: " << us << "\n"
                  << "Suggested " << (direction > 0 ? "forward" : "reverse")
                  << "_velocity_gain: " << kv << "\n";
    }
}

void runSteadySweep(RobotMotors& motors) {
    MotorCoastGuard guard{motors};
    std::ofstream csv("motor_pwm_sweep.csv");
    if (!csv) throw std::runtime_error("Cannot open motor_pwm_sweep.csv");
    csv << "wheel,direction,duty,velocity_m_s,encoder_delta,measure_s\n";
    std::cout << "\nSTEADY PWM SWEEP: max duty " << SWEEP_END
              << "; one wheel at a time. Robot must be secured with wheels clear.\n";
    for (bool left : {true, false}) {
        for (int direction : {+1, -1}) {
            std::vector<SweepPoint> points;
            for (int step=0; keep_running; ++step) {
                const double duty = SWEEP_START + step*SWEEP_STEP;
                if (duty > SWEEP_END + 1e-9) break;
                coastAll(motors);
                if (!sleepInterruptible(REST_MS)) break;
                commandSingleWheel(motors, left, direction*duty);
                if (!sleepInterruptible(SWEEP_SETTLE_MS)) break;
                const auto start = std::chrono::steady_clock::now();
                const auto count0 = wheelCounts(motors.getState(), left);
                if (!sleepInterruptible(SWEEP_MEASURE_MS)) break;
                const auto count1 = wheelCounts(motors.getState(), left);
                const double elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now()-start).count();
                coastAll(motors);
                const long long delta = count1-count0;
                const double velocity = (static_cast<double>(delta)/CPR)
                    * (2.0*3.14159265358979323846*WHEEL_RADIUS_M)/elapsed;
                csv << wheelName(left) << ',' << directionName(direction) << ','
                    << direction*duty << ',' << velocity << ',' << delta << ',' << elapsed << '\n';
                csv.flush();
                std::cout << wheelName(left) << ' ' << directionName(direction)
                          << " duty=" << direction*duty << " speed=" << velocity << " m/s\n";
                if (direction*velocity > 0.01) points.push_back({std::abs(velocity),duty});
                else if (direction*velocity < -0.01) {
                    std::cout << "ERROR: encoder sign inconsistent; stopping sweep.\n";
                    return;
                }
            }
            if (!keep_running) return;
            printFeedforwardFit(points, left, direction);
        }
    }
    std::cout << "\nSaved motor_pwm_sweep.csv\n";
}

// ============================================================
// MAIN
// ============================================================

int main()
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    const MotorConfig left = sparcbot::config::leftMotor();
    const MotorConfig right = sparcbot::config::rightMotor();

    try {
        RobotMotors motors(left, right);

        coastAll(motors);

        std::cout
            << "\n"
            << "========================================\n"
            << " SPARCBot Breakaway / Stop Characterizer\n"
            << "========================================\n\n"
            << "Run with robot ON THE FLOOR.\n\n"
            << "Only one motor is powered at a time.\n"
            << "The other wheel remains coasting.\n"
            << "Commands are intentionally short to minimize\n"
            << "robot movement.\n\n"
            << "Ctrl+C can be used to stop the program.\n\n";

        while (keep_running) {
            std::cout
                << "----------------------------------------\n"
                << "f : characterize FORWARD\n"
                << "r : characterize REVERSE\n"
                << "a : characterize BOTH directions\n"
                << "s : steady-state PWM sweep + CSV + feedforward fits\n"
                << "q : quit\n"
                << "----------------------------------------\n"
                << "> "
                << std::flush;

            char command;
            std::cin >> command;

            if (!std::cin) {
                break;
            }

            if (command == 'q' || command == 'Q') {
                break;
            }

            if (command == 'f' || command == 'F') {
                std::cout
                    << "\nMake sure the robot has clear space.\n"
                    << "Press ENTER to start FORWARD characterization...";

                std::cin.ignore(
                    std::numeric_limits<std::streamsize>::max(),
                    '\n');

                std::cin.get();

                const DirectionResult result =
                    characterizeDirection(motors, +1);

                printDirectionResult(result, +1);
            }

            else if (command == 'r' || command == 'R') {
                std::cout
                    << "\nMake sure the robot has clear space.\n"
                    << "Press ENTER to start REVERSE characterization...";

                std::cin.ignore(
                    std::numeric_limits<std::streamsize>::max(),
                    '\n');

                std::cin.get();

                const DirectionResult result =
                    characterizeDirection(motors, -1);

                printDirectionResult(result, -1);
            }

            else if (command == 's' || command == 'S') {
                std::cout << "STOP the ROS driver first. Secure robot, elevate wheels,\n"
                          << "keep clear of moving parts. Type y to confirm: ";
                char confirm = 'n';
                std::cin >> confirm;
                if (confirm == 'y' || confirm == 'Y') runSteadySweep(motors);
            }

            else if (command == 'a' || command == 'A') {
                std::cout
                    << "\nThis will characterize BOTH directions.\n"
                    << "Make sure the robot has clear space.\n"
                    << "Press ENTER to begin...";

                std::cin.ignore(
                    std::numeric_limits<std::streamsize>::max(),
                    '\n');

                std::cin.get();

                const DirectionResult forward =
                    characterizeDirection(motors, +1);

                if (!keep_running) break;

                const DirectionResult reverse =
                    characterizeDirection(motors, -1);

                if (!keep_running) break;

                printDirectionResult(forward, +1);
                printDirectionResult(reverse, -1);

                std::cout
                    << "\n\n"
                    << "========================================\n"
                    << "       FINAL FEEDFORWARD VALUES\n"
                    << "========================================\n"
                    << std::fixed
                    << std::setprecision(3)
                    << "LEFT\n"
                    << "  forward_breakaway = "
                    << forward.left_breakaway << '\n'
                    << "  reverse_breakaway = "
                    << reverse.left_breakaway << "\n\n"
                    << "RIGHT\n"
                    << "  forward_breakaway = "
                    << forward.right_breakaway << '\n'
                    << "  reverse_breakaway = "
                    << reverse.right_breakaway << '\n'
                    << "========================================\n";
            }

            else {
                std::cout << "Use f, r, a, s, or q.\n";
            }

            coastAll(motors);
        }

        coastAll(motors);

        std::cout << "\nDone.\n";
    }

    catch (const std::exception& e) {
        std::cerr << "\nERROR: "
                  << e.what()
                  << '\n';

        return 1;
    }

    return 0;
}