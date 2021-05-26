/** @file ps_pwm.h
 * @brief Driver for the MCPWM hardware modules on the Espressif ESP32
 * or ESP32-S3 SoC for generating a Phase-Shift-PWM waveform between
 * two pairs of hardware pins. (Not compatible with ESP32-S2)
 * 
 * Application in power electronics, e.g. Zero-Voltage-Switching (ZVS)
 * Full-Bridge-, Dual-Active-Bridge- and LLC converters.
 * 
 *
 * @note This depends on the ESP-IDF SDK source files.
 *
 * 2021-05-21 Ulrich Lukas
 */
#ifndef PS_PWM_H__
#define PS_PWM_H__

#include <stdbool.h> 
#include "driver/mcpwm.h"

// Set log level to ESP_LOG_INFO for production!
#define PS_PWM_LOG_LEVEL ESP_LOG_INFO

//#define GPIO_SYNC0_IN   2   //Set GPIO 02 as SYNC0

// Unscaled input clock frequency
#define MCPWM_INPUT_CLK 160000000 // 160 MHz
// Hardware prescaler factor for input clock.
// Dead time generators are configured to run on this scaled clock signal
// Valid values are 1...255
#define BASE_CLK_PRESCALE_DEFAULT 1
// Hardware prescaler factor for timer operator sub-modules
// Valid values are 1...255
#define TIMER_CLK_PRESCALE_DEFAULT 1
// Minimum timer counter TOP value / timer resolution. Used for calculation of
// frequency_min value and subsequent range checking of frequency setpoint
static const uint16_t period_min = 4;

#ifdef __cplusplus
extern "C" {
#endif

/** Static structure holds the inter-dependent setpoint values for PWM timing.
 * Frequency in Hz. Phase-shift in percent of a half timer period.
 * Dead time settings for both MCPWM hardware modules are defined as lead and
 * lag bridge-leg low-side output rising and falling edge dead-times in seconds
 */
typedef struct {
    // Frequency setpoint
    float frequency;
    // Phase shift setpoint
    float ps_duty;
    // Lead leg, dead time for rising edge (up_ctr_mode)
    // or both edges (up_down_ctr_mode)
    float lead_red;
    // Falling edge dead time for up_ctr_mode, not defined for up_down_ctr_mode
    float lead_fed;
    // All the same for lagging leg
    float lag_red;
    float lag_fed;
    // true if output is enabled
    bool output_enabled;
} pspwm_setpoint_t;

/** State of the timer counter clock prescaler.
 * MCPWM_INPUT_CLK is 160MHz.
 * This is divided by base_clk_prescale to yield base_clock.
 * This is in turn divided by timer_clk_prescale to yield timer_clk.
 * 
 * @note These settings are common for both PWM generators.
 */
typedef struct {
    uint8_t base_clk_prescale;
    uint8_t timer_clk_prescale;
    float base_clk;
    float timer_clk;
} pspwm_clk_conf_t;

/** Limiting values for frequency and dead-time settings.
 * These are set by the initialiser and prescaler setter functions
 * contained herein and should be treated read-only
 */
typedef struct {
    // Minimum and maximum allowed frequency setpoints
    float frequency_min;
    float frequency_max;
    // Dead time for each bridge leg must be smaller than this value,
    // both in sum and also both individually. (Minimum is zero.)
    float dt_sum_max;
} pspwm_setpoint_limits_t;

/********************************************************************//**
 *    FULL-SPEED-MODE, 4x INDIVIDUAL DEAD-TIME, HW-DEAD-TIME-MODULE
 ************************************************************************
 * @brief Set up the PS-PWM generator module for up-counting mode,
 * allowing individual dead-time values for rising and falling edges
 * for both lead and lag output pairs.
 * 
 * To achieve this, this uses the hardware dead-band generator while
 * also calculating and setting complementary timing values for
 * the PWM operator compare registers.
 * 
 * Combined output waveform of the phase-shifted full-bridge
 * is DC-free symmetric nevertheless.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param gpio_lead_a: GPIO number leading leg low_side
 * @param gpio_lead_b: GPIO number leading leg high_side
 * @param gpio_lag_a: GPIO number lagging leg low_side
 * @param gpio_lag_b: GPIO number lagging leg high_side
 * @param frequency: Frequency of the non-rectified waveform in Hz,
 * @param ps_duty: Duty cycle of the rectified waveform (0..1)
 * @param lead_red: dead time value for rising edge, leading leg
 * @param lead_fed: dead time value for falling edge, leading leg
 * @param lag_red: dead time value for rising edge, lagging leg
 * @param lag_fed: dead time value for falling edge, lagging leg
 * @param output_enabled: initial output state (true <==> ON)
 * @param disable_action_lead_leg: Choice of actions when lead bridge leg is disabled
 *                                 (See typedef for mcpwm_action_on_pwmxa_t)
 * @param disable_action_lag_leg: Same for lag leg
 */
esp_err_t pspwm_init(mcpwm_unit_t mcpwm_num,
                     int gpio_lead_a,
                     int gpio_lead_b,
                     int gpio_lag_a,
                     int gpio_lag_b,
                     float frequency,
                     float ps_duty,
                     float lead_red, float lead_fed,
                     float lag_red, float lag_fed,
                     bool output_enabled,
                     mcpwm_action_on_pwmxa_t disable_action_lead_leg,
                     mcpwm_action_on_pwmxa_t disable_action_lag_leg);

/** @brief Shortcut version of pspwm_init() with identical
 * rising and falling edge dead times applied for each of lead and lag leg.
 * 
 * This is also call compatible with the up_down_ctr_mode API
 * (which is disabled by default, see further down this file).
 */
esp_err_t pspwm_init_symmetrical(mcpwm_unit_t mcpwm_num,
                                 int gpio_lead_a,
                                 int gpio_lead_b,
                                 int gpio_lag_a,
                                 int gpio_lag_b,
                                 float frequency,
                                 float ps_duty,
                                 float lead_dt,
                                 float lag_dt,
                                 bool output_enabled,
                                 mcpwm_action_on_pwmxa_t disable_action_lead_leg,
                                 mcpwm_action_on_pwmxa_t disable_action_lag_leg);

/** @brief Set frequency when running PS-PWM generator in up-counting mode
 * @note
 * This does not alter prescaler settings.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param frequency: Frequency of the non-rectified waveform in Hz,
 */
esp_err_t pspwm_set_frequency(mcpwm_unit_t mcpwm_num,
                              float frequency);

/** @brief Set deadtime values individually for leading leg rising and
 * falling edge as well as for lagging leg rising and falling edge
 * for all four PWM outputs.
 * 
 * @param  mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param  lead_red: dead time value for rising edge, leading leg
 * @param  lead_fed: dead time value for falling edge, leading leg
 * @param  lag_red: dead time value for rising edge, lagging leg
 * @param  lag_fed: dead time value for falling edge, lagging leg
 */
esp_err_t pspwm_set_deadtimes(mcpwm_unit_t mcpwm_num,
                              float lead_red,
                              float lead_fed,
                              float lag_red,
                              float lag_fed);

/** @brief Shortcut version of pspwm_set_deadtimes() with identical
 * rising and falling edge dead times applied for each of lead and lag leg.
 * 
 * This is also call compatible with the up_down_ctr_mode API (different file).
 */
esp_err_t pspwm_set_deadtimes_symmetrical(mcpwm_unit_t mcpwm_num,
                                          float lead_dt,
                                          float lag_dt);

/** @brief Set PS-PWM phase shift between lead and lag leg output pairs
 * 
 * This is expressed in units of one where 1.0 means 180° phase-shift
 * for the output pairs, giving maximum duty cycle after rectification.
 * @note The value is calculated based on current value of the
 *       PWM timer "period" register.
 *
 * @param  mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param  ps_duty: Duty cycle of the rectified waveform (0..1)
 */
esp_err_t pspwm_set_ps_duty(mcpwm_unit_t mcpwm_num,
                            float ps_duty);


/*****************************************************************
 *                         COMMON SETUP
 *****************************************************************/
/** @brief Returns true while the hardware fault shutdown pin is active
 * i.e. for as long as the failure is still present.
 * 
 */
bool pspwm_get_hw_fault_shutdown_present(mcpwm_unit_t mcpwm_num);

/** @brief Returns true when the hardware fault shutdown pin has been activated.
 * 
 * The state remains true (shutdown activated) as long as hw status has
 * not been cleared by a call to pspwm_clear_hw_fault_shutdown_occurred().
 */
bool pspwm_get_hw_fault_shutdown_occurred(mcpwm_unit_t mcpwm_num);

/** @brief Resets the fault shutdown active flag without re-enabling the output
 * 
 * If a hardware shutdown occurred, this flag must be reset first
 * in order to re-enable the output. This acts as a safety feature.
 */
void pspwm_clear_hw_fault_shutdown_occurred(mcpwm_unit_t mcpwm_num);

/** @brief Disable PWM output immediately by software-triggering the one-shot
 * fault input of the "trip-zone" fault handler module.
 * 
 * This sets the PWM output pins to predefined levels TRIPZONE_ACTION_PWMxA
 * and TRIPZONE_ACTION_PWMxB, which can be configured in the header file.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 */
esp_err_t pspwm_disable_output(mcpwm_unit_t mcpwm_num);

/** @brief (Re-)enable PWM output by clearing fault handler one-shot trigger
 * after software-triggering a re-sync to the initial phase setpoint.
 * 
 * If a hardware shutdown occurred, the shutdown flag must be reset first
 * by calling pspwm_clear_hw_fault_shutdown_occurred() in order to be able to 
 * re-enable the output. This acts as a safety feature.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 */
esp_err_t pspwm_resync_enable_output(mcpwm_unit_t mcpwm_num);

/** @brief Enable hardware fault shutdown ("tripzone") input on given GPIO pin.
 * 
 * This registers the fault handler FH0 signal with the specified PWM unit.
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param gpio_fault_shutdown: GPIO pin number for shutdown input
 * @param fault_pin_active_level: Logic level setting the fault condition
 *                                [MCPWM_LOW_LEVEL_TGR | MCPWM_HIGH_LEVEL_TGR]
 */
esp_err_t pspwm_enable_hw_fault_shutdown(mcpwm_unit_t mcpwm_num,
                                         int gpio_fault_shutdown,
                                         mcpwm_fault_input_level_t fault_pin_active_level);

/** @brief Disable hardware fault shutdown pin, resetting the GPIO to default state.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param gpio_fault_shutdown: GPIO pin number for shutdown input
 */
esp_err_t pspwm_disable_hw_fault_shutdown(mcpwm_unit_t mcpwm_num,
                                          int gpio_fault_shutdown);

/** @brief Return a pointer to PSPWM stage runtime setpoints.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param setpoint: Pointer to a struct instance of pspwm_setpoint_t
 */
esp_err_t pspwm_get_setpoint_ptr(mcpwm_unit_t mcpwm_num,
                                 pspwm_setpoint_t** setpoint);

/** @brief Return a pointer to PSPWM stage setpoint limits as
 * calculated from clock configuration.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param setpoint_limits: Pointer to a struct instance of pspwm_setpoint_limits_t
 */
esp_err_t pspwm_get_setpoint_limits_ptr(mcpwm_unit_t mcpwm_num,
                                        pspwm_setpoint_limits_t** setpoint_limits);

/** @brief Return a pointer to PSPWM stage clock configuration.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param clk_conf: Pointer to a struct instance of pspwm_setpoint_t
 */
esp_err_t pspwm_get_clk_conf_ptr(mcpwm_unit_t mcpwm_num,
                                 pspwm_clk_conf_t** clk_conf);


// Disabled by default, included as reference or to be enabled on demand
#ifdef PSPWM_USE_UP_DOWN_CTR_MODE_API
/*************************************************************//**
 * @brief Set up the PS-PWM generator module for up-down-counting mode, which
 * assures identical ON times for high side and low side outputs of each leg.
 * 
 * Individual dead-times both half-bridge PWM outputs are still possible.
 * 
 * This PWM generation mode does not use the dead-band generator hardware.
 * Instead, the dead-time for each two outputs of a half-bridge is configured
 * by running the main timers in up-down-counting mode and using both compare
 * registers for each timer to generate two symmetric outputs.
 * 
 * Because of the up/down-counting mode, maximum output frequency is half of
 * the value which is possible when using the hardware dead-band generator.
 * 
 * @note When using up/down mode implemented here, it is NOT SAFE to change the
 * frequency, phase shift duty or dead-time setpoints during operation as
 * timer compare events could be missed, causing invalid output waveforms
 * for up to one timer period.
 * THIS WILL CAUSE SHORT-CIRCUITS for bridge output stages!
 * See ps_pwm.c for implementation using the dead-time genrator to avoid this.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param gpio_lead_a: GPIO number leading leg low_side
 * @param gpio_lead_b: GPIO number leading leg high_side
 * @param gpio_lag_a: GPIO number lagging leg low_side
 * @param gpio_lag_b: GPIO number lagging leg high_side
 * @param frequency: Frequency of the non-rectified waveform in Hz,
 * @param ps_duty: Duty cycle of the rectified waveform (0..1)
 * @param lead_dt: leading bridge-leg dead-time in sec (0..),
 * @param lag_dt: lagging bridge-leg dead-time in sec (0..)
 * @param output_enabled: initial output state (true <==> ON)
 * @param disable_action_lead_leg: Choice of actions when lead bridge leg is disabled
 *                                 (See typedef for mcpwm_action_on_pwmxa_t)
 * @param disable_action_lag_leg: Same for lag leg
 */
esp_err_t pspwm_up_down_ctr_mode_init(mcpwm_unit_t mcpwm_num,
                                      int gpio_lead_a,
                                      int gpio_lead_b,
                                      int gpio_lag_a,
                                      int gpio_lag_b,
                                      float frequency,
                                      float ps_duty,
                                      float lead_dt,
                                      float lag_dt,
                                      bool output_enabled,
                                      mcpwm_action_on_pwmxa_t disable_action_lead_leg,
                                      mcpwm_action_on_pwmxa_t disable_action_lag_leg);

/** @brief Set frequency (and update dead-time values) for both output pairs
 * signals of the phase-shift-PWM when using the timer in up/down counting mode.
 * 
 * Because of the up/down-counting mode, maximum output frequency is half of
 * the value which is possible when using the hardware dead-band generator.
 * 
 * @note
 * This does not alter prescaler settings.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param frequency: Frequency of the non-rectified waveform in Hz,
 */
esp_err_t pspwm_up_down_ctr_mode_set_frequency(mcpwm_unit_t mcpwm_num,
                                               float frequency);

/** @brief Set dead-time values for both output pairs of the phase-shift-PWM
 * when using the timer in up/down counting mode.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param lead_dt: leading bridge-leg dead-time in sec (0..),
 * @param lag_dt: lagging bridge-leg dead-time in sec (0..)
 */
esp_err_t pspwm_up_down_ctr_mode_set_deadtimes(mcpwm_unit_t mcpwm_num,
                                               float lead_dt,
                                               float lag_dt);

/** @brief Set PS-PWM phase shift between the two output pairs based on the
 * current period time setting as stored in the PWM hardware "period" register.
 * 
 * @param mcpwm_num: PWM unit number (enum, MCPWM_UNIT_0 = 0, MCPWM_UNIT_1 = 1),
 * @param ps_duty: Duty cycle of the rectified waveform (0..1)
 */
esp_err_t pspwm_up_down_ctr_mode_set_ps_duty(mcpwm_unit_t mcpwm_num,
                                             float ps_duty);

#endif // PSPWM_USE_UP_DOWN_CTR_MODE_API

#ifdef __cplusplus
}
#endif

#endif  /* PS_PWM_H__ */
