# PID Controller Documentation

This section provides a comprehensive overview of the custom PID controller implemented in `PID.cpp`/`PID.h`. The controller supports proportional–integral–derivative control with advanced features such as feedforward, anti-windup, derivative filtering, and dead zone compensation.

## Class Overview

```cpp
class PID {
public:
    PID(float Kp, float Ki, float Kd,
        float dt,
        float integratorMin, float integratorMax,
        int outputMin, int outputMax);

    int compute(float setpoint, float measurement);
    void reset();

    // Configuration
    void setTunings(float Kp, float Ki, float Kd);
    void setWindupLimits(float integratorMin, float integratorMax);
    void setOutputLimits(int outputMin, int outputMax);
    void setInterval(float dt);
    void setDeadZone(int deadZone);
    void setDerivativeFilterAlpha(float alpha);
    void setFeedforwardParams(float a, float b);
    void enableFeedforward(bool enable);

    bool isSaturated() const;

private:
    // PID coefficients
    float _Kp, _Ki, _Kd;
    // Time step
    float _dt;
    // Integrator state and limits
    float _integrator, _intMin, _intMax;
    // Previous measurement and error
    float _prevMeas, _prevError;
    // Derivative filter state and coefficient
    float _dFiltered, _dFilterAlpha;
    // Output limits and dead zone
    int _outMin, _outMax;
    int _deadZone;
    // Feedforward parameters
    float _ffA, _ffB;
    bool _useFeedforward;

public:
    // Last computed error and output
    float error;
    int output;
};
```

## Features & Algorithms

### 1. Proportional Term (P)

* **Equation**: `P = Kp * error`
* **Description**: Provides immediate response proportional to the error between setpoint and measurement.

### 2. Integral Term (I) with Anti-Windup

* **Equation**: `I_temp = integrator + Ki * error * dt`
* The integrator only updates if the unconstrained control signal `u_un` is within output limits, preventing windup.
* **Clamping**: After update, `integrator = constrain(I_temp, intMin, intMax)`.

### 3. Derivative Term (D) with Low-Pass Filter

* **Derivative of Measurement**: `dMeas = (measurement - prevMeas) / dt`
* **Filtered**: `_dFiltered = alpha * dMeas + (1 - alpha) * _dFiltered`
* **Term**: `D = -Kd * _dFiltered`
* **Benefit**: Reduces sensitivity to measurement noise.

### 4. Dead Zone Compensation

* If the raw control signal is non-zero, a fixed `deadZone` offset is added to overcome mechanical stiction.

### 5. Feedforward Control

* **Equation**: `FF = ffA * setpoint + ffB` (optional)
* **Usage**: Combines model-based feedforward term with PID for faster dynamic response.

### 6. Output Limiting & Saturation Detection

* Final output is clamped between `[outputMin, outputMax]`.
* `isSaturated()` returns `true` if output is at either limit.

## Usage Example

```cpp
// Create PID: Kp=1.0, Ki=0.5, Kd=0.1, dt=0.01s, integrator [-10,10], output [0, 4095]
PID pid(1.0f, 0.5f, 0.1f, 0.01f, -10.0f, 10.0f, 0, 4095);

// In control loop
float setpoint = 100.0f;     // target RPM
float measurement = getRPM(); // actual RPM
int command = pid.compute(setpoint, measurement);
applyMotorCommand(command);

// Check for saturation
if (pid.isSaturated()) {
    // handle actuator limit condition
}
```

## Configuration Methods

| Method                            | Description                                               |
| --------------------------------- | --------------------------------------------------------- |
| `setTunings(Kp, Ki, Kd)`          | Update PID coefficients.                                  |
| `setWindupLimits(min, max)`       | Set integrator anti-windup clamps.                        |
| `setOutputLimits(min, max)`       | Set actuator output range.                                |
| `setInterval(dt)`                 | Adjust control loop time step.                            |
| `setDeadZone(dz)`                 | Define dead zone offset for non-zero commands.            |
| `setDerivativeFilterAlpha(alpha)` | Tune derivative filter (0=no filter, 1=noisy derivative). |
| `setFeedforwardParams(a, b)`      | Configure linear feedforward: `a*setpoint + b`.           |
| `enableFeedforward(bool)`         | Enable or disable feedforward term.                       |

---

*This PID implementation is tailored for embedded motor control, balancing responsiveness with robustness against noise and actuator constraints.*
