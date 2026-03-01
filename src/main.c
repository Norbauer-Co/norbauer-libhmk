/*
 * This program is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 */

#include "advanced_keys.h"
#include "commands.h"
#include "crc32.h"
#include "deferred_actions.h"
#include "eeconfig.h"
#include "hardware/hardware.h"
#include "hid.h"
#include "layout.h"
#include "matrix.h"
#include "tusb.h"
#include "wear_leveling.h"
#include "xinput.h"

#define STARTUP_SETTING_KEY 0
#define STARTUP_HOLD_CHECK_SAMPLES 20
#define STARTUP_HOLD_CHECK_DELAY_MS 1
#define STARTUP_HOLD_REQUIRED_SAMPLES 32
#define STARTUP_RELEASE_STABLE_DELAY_MS 20
#define STARTUP_ADC_WARMUP_TIMEOUT_MS 50
#define STARTUP_HOLD_THRESHOLD 2200

static bool startup_is_setting_key_pressed(void) {
  const uint16_t adc = analog_read(STARTUP_SETTING_KEY);
#if defined(MATRIX_INVERT_ADC_VALUES)
  const uint16_t adjusted_adc = ADC_MAX_VALUE - adc;
#else
  const uint16_t adjusted_adc = adc;
#endif
  return adjusted_adc >= STARTUP_HOLD_THRESHOLD;
}

static void startup_handle_profile_tone_setting(void) {
  // Let DMA ADC values settle after analog init.
  for (uint8_t i = 0; i < STARTUP_ADC_WARMUP_TIMEOUT_MS; i++) {
    if (analog_read(STARTUP_SETTING_KEY) != 0)
      break;
    timer_delay(1);
  }

  uint8_t pressed_samples = 0;
  for (uint8_t i = 0; i < STARTUP_HOLD_CHECK_SAMPLES; i++) {
    pressed_samples += startup_is_setting_key_pressed();
    timer_delay(STARTUP_HOLD_CHECK_DELAY_MS);
  }

  if (pressed_samples < STARTUP_HOLD_REQUIRED_SAMPLES)
    return;

  // Distinct fast tone for toggling profile startup/runtime tones.
  for (uint8_t i = 0; i < 2; i++) {
    timer_buzzer_start();
    timer_delay(20);
    timer_buzzer_stop();
    if (i < 1)
      timer_delay(20);
  }

  eeconfig_options_t options = eeconfig->options;
  options.profile_tones_disabled = !options.profile_tones_disabled;
  EECONFIG_WRITE(options, &options);

  while (startup_is_setting_key_pressed())
    timer_delay(STARTUP_HOLD_CHECK_DELAY_MS);
  timer_delay(STARTUP_RELEASE_STABLE_DELAY_MS);
}

static void startup_play_profile_buzzer(uint8_t profile) {
  if (eeconfig->options.profile_tones_disabled)
    return;

  if (profile == 0) {
    timer_buzzer_start();
    timer_delay(1000);
    timer_buzzer_stop();
  } else if (profile == 1) {
    for (uint8_t i = 0; i < 3; i++) {
      timer_buzzer_start();
      timer_delay(100);
      timer_buzzer_stop();
      if (i < 2)
        timer_delay(100);
    }
  }
}

int main(void) {
  // Initialize the hardware
  board_init();
  timer_init();
  crc32_init();
  flash_init();

  // Initialize the persistent configuration
  wear_leveling_init();
  eeconfig_init();

  // Initialize the core modules
  analog_init();
  startup_handle_profile_tone_setting();
  matrix_init();
  hid_init();
  deferred_action_init();
  advanced_key_init();
  xinput_init();
  layout_init();
  command_init();

  tud_init(BOARD_TUD_RHPORT);

  startup_play_profile_buzzer(eeconfig->current_profile);
  
  while (1) {
    tud_task();

    analog_task();
    matrix_scan();
    layout_task();
    xinput_task();
  }

  return 0;
}
