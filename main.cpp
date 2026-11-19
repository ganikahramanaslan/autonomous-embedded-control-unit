
/*
========================================================
 PROJECT: AUTONOMOUS EMBEDDED CONTROL UNIT (AECU)
 MODULE : Embedded Control Core
 LANGUAGE: C++17
 VERSION : 1.0.0

 FEATURES:
 - Sensor simulation
 - Signal filtering
 - PID controller
 - Fault detection
 - Safety state management
 - System telemetry

 NOTE:
 Desktop simulation prototype.
 Not certified for automotive or safety-critical use.
========================================================
*/

#include <iostream>
#include <cmath>
#include <array>
#include <algorithm>
#include <iomanip>
#include <string>

// SYSTEM CONFIGURATION

constexpr float DT = 0.01f;
constexpr float TARGET_TEMPERATURE = 65.0f;
constexpr float MAX_TEMPERATURE = 100.0f;

enum class SystemState {
    INIT,
    RUNNING,
    WARNING,
    SAFE_MODE
};

enum class FaultCode {
    NONE,
    SENSOR_FAILURE,
    OVER_TEMPERATURE,
    SYSTEM_FAILURE
};

// SENSOR SIMULATOR

class SensorManager {
private:
    float temperature = 25.0f;
    float voltage = 12.0f;

public:
    void update(int step) {

        temperature =
            25.0f + 0.05f * step
            + 2.0f * std::sin(step * 0.1f);

        voltage =
            12.0f + 0.2f * std::sin(step * 0.05f);
    }

    float getTemperature() const {
        return temperature;
    }

    float getVoltage() const {
        return voltage;
    }
};

// SIGNAL FILTER

class MovingAverage {
private:
    static constexpr int SIZE = 5;

    std::array<float, SIZE> buffer{};
    int index = 0;
    int count = 0;

public:
    float process(float input) {

        buffer[index] = input;

        index = (index + 1) % SIZE;

        if (count < SIZE) {
            count++;
        }

        float sum = 0.0f;

        for (int i = 0; i < count; i++) {
            sum += buffer[i];
        }

        return sum / count;
    }
};

// PID CONTROLLER

class PIDController {
private:
    float kp = 2.0f;
    float ki = 0.4f;
    float kd = 0.1f;

    float integral = 0.0f;
    float previousError = 0.0f;

public:
    float compute(float target, float actual) {

        float error = target - actual;

        integral += error * DT;

        integral = std::clamp(
            integral, -100.0f, 100.0f
        );

        float derivative =
            (error - previousError) / DT;

        previousError = error;

        float output =
            kp * error +
            ki * integral +
            kd * derivative;

        return std::clamp(output, 0.0f, 100.0f);
    }

    void reset() {
        integral = 0.0f;
        previousError = 0.0f;
    }
};

// FAULT MANAGER

class FaultManager {
public:
    FaultCode check(float temperature,
                    float voltage) {

        if (!std::isfinite(temperature) ||
            !std::isfinite(voltage)) {
            return FaultCode::SENSOR_FAILURE;
        }

        if (temperature > MAX_TEMPERATURE) {
            return FaultCode::OVER_TEMPERATURE;
        }

        if (voltage < 9.0f || voltage > 15.0f) {
            return FaultCode::SYSTEM_FAILURE;
        }

        return FaultCode::NONE;
    }
};

// SAFETY CONTROLLER

class SafetyController {
private:
    SystemState state = SystemState::INIT;

public:
    void update(FaultCode fault) {

        if (fault == FaultCode::OVER_TEMPERATURE ||
            fault == FaultCode::SYSTEM_FAILURE ||
            fault == FaultCode::SENSOR_FAILURE) {

            state = SystemState::SAFE_MODE;
        }
        else {
            state = SystemState::RUNNING;
        }
    }

    SystemState getState() const {
        return state;
    }
};

// MAIN APPLICATION

int main() {

    SensorManager sensors;
    MovingAverage filter;
    PIDController pid;
    FaultManager faultManager;
    SafetyController safety;

    std::cout
        << "AECU EMBEDDED CONTROL SYSTEM\n"
        << "----------------------------------------\n";

    std::cout
        << "TIME\tTEMP\tVOLTAGE\tPWM\tSTATE\n";

    for (int step = 0; step < 500; step++) {

        // 1. SENSOR ACQUISITION
        sensors.update(step);

        float rawTemperature =
            sensors.getTemperature();

        float voltage =
            sensors.getVoltage();

        // 2. SIGNAL PROCESSING
        float filteredTemperature =
            filter.process(rawTemperature);

        // 3. FAULT DETECTION
        FaultCode fault =
            faultManager.check(
                filteredTemperature,
                voltage
            );

        // 4. SAFETY MANAGEMENT
        safety.update(fault);

        // 5. CONTROL ALGORITHM
        float pwm = 0.0f;

        if (safety.getState() == SystemState::RUNNING) {

            pwm = pid.compute(
                TARGET_TEMPERATURE,
                filteredTemperature
            );
        }
        else {
            pid.reset();
        }

        // 6. TELEMETRY
        std::string stateName =
            safety.getState() == SystemState::RUNNING
            ? "RUNNING"
            : "SAFE_MODE";

        std::cout
            << std::fixed << std::setprecision(2)
            << step * DT << "\t"
            << filteredTemperature << "\t"
            << voltage << "\t"
            << pwm << "\t"
            << stateName << "\n";
    }

    std::cout << "\nSimulation completed.\n";

    return 0;
}
