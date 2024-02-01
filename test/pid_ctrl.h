#ifndef LIB_SRC_PID_CTRL_H_
#define LIB_SRC_PID_CTRL_H_

#include <Arduino.h>

typedef struct
{
  float k_p;  //<  proportional factor
  float k_i;  //<  integral factor
  float k_d;  //<  derivative factor
} pid_params;

/**
 * @brief PIDControl class to control an actuator
 */
class PIDControl{
 public:
  PIDControl();
  ~PIDControl();

  /**
   * @brief Adjust heating with PID-controller
   * 
   * @param pid - const pointer to pid_params_object
   * @param error - error = v_target - v_current
   * @param dt - timestep delta t
   */
  void pid_ctrl(const pid_params *pid, float error, float dt);

  /**
   * @brief reset interal pid parameters
   */
  void reset();

  /**
   * @brief Setup PID parameters.
   *        Generated control actions lie within [min, max].
   * 
   * @param pin - output pin to use
   * @param min - minimum allowed control action
   * @param max - maximum allowed control action
   * @param f - pwm frequency
   */
  void setup_pid(uint8_t pin, float min = 10, float max = 250, float f = 10.0f);

 private:
  float _get_pid_ctrl(const pid_params *params,
                      const float *error,
                      const float *dt);

  uint8_t _pin;
  float _min;
  float _max;
  float _prev_ctrl;
  float _error[2];
};
#endif  // LIB_SRC_PID_CTRL_H_
