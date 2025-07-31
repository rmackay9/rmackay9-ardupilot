/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  battery BMS includes a button which, when pressed, shows the state of charge percentage using LEDs
 */
#include "AP_Periph.h"

#if AP_PERIPH_BATTERY_BMS_ENABLED

#include "stdio.h"

extern const AP_HAL::HAL &hal;

BatteryBMS::BatteryBMS(void)
{
    // Initialize button state variables
    button_last_state = true;  // Assuming button is pulled up (true = not pressed)
    button_press_start_ms = 0;
    button_press_handled = false;

    // initialise LEDs
#if HAL_GPIO_LED_ON != 0
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED1, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED2, HAL_GPIO_OUTPUT);
    /*hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED3, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED4, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED5, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED6, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED7, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_LED8, HAL_GPIO_OUTPUT);*/
    
    // Configure button as input with pullup
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_BTN1, HAL_GPIO_INPUT);
    hal.gpio->write(HAL_GPIO_PIN_BMS_BTN1, 1); // Enable pullup
#endif
}

void BatteryBMS::update(void)
{
#ifdef HAL_GPIO_PIN_BMS_BTN1
    handle_button_press();
#endif

    // display startup patterns if LEDs are present
    // exit immediately if LED patterns have already been completed
    if (init_done) {
        return;
    }
    // advance init stage
    init_stage++;
    switch (init_stage) {
    case 1:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED1, HAL_GPIO_LED_ON);
        break;
    case 2:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED2, HAL_GPIO_LED_ON);
        break;
    case 3:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED3, HAL_GPIO_LED_ON);
        break;
    case 4:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED4, HAL_GPIO_LED_ON);
        break;
    case 5:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED5, HAL_GPIO_LED_ON);
        break;
    case 6:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED6, HAL_GPIO_LED_ON);
        break;
    case 7:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED7, HAL_GPIO_LED_ON);
        break;
    case 8:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED8, HAL_GPIO_LED_ON);
        break;
    case 9:
    default:
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED1, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED2, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED3, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED4, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED5, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED6, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED7, HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_PIN_BMS_LED8, HAL_GPIO_LED_OFF);
        init_done = true;  // all LEDs have been lit, so we can stop
        break;
    }
}

#ifdef HAL_GPIO_PIN_BMS_BTN1
void BatteryBMS::handle_button_press(void)
{
    // Read current button state (assuming active low - pressed = 0)
    bool button_raw_state = hal.gpio->read(HAL_GPIO_PIN_BMS_BTN1);
    bool button_current_state = (button_raw_state == 0); // pressed = true
    uint32_t now_ms = AP_HAL::millis();
    
    // Button just pressed (transition from released to pressed)
    if (button_current_state && !button_last_state) {
        button_press_start_ms = now_ms;
        button_press_handled = false;
        printf("BMS: Button pressed!\n");
    }
    
    // Button is currently pressed - check for long press
    if (button_current_state && !button_press_handled) {
        uint32_t press_duration = now_ms - button_press_start_ms;
        
        // Long press detected (2+ seconds)
        if (press_duration >= LONG_PRESS_THRESHOLD_MS) {
            button_press_handled = true;
            printf("BMS: LONG PRESS DETECTED! (2+ seconds)\n");
        }
    }
    
    // Button just released (transition from pressed to released)
    if (!button_current_state && button_last_state) {
        uint32_t press_duration = now_ms - button_press_start_ms;
        
        // Short press detected (less than 2 seconds and not already handled as long press)
        if (press_duration < LONG_PRESS_THRESHOLD_MS && !button_press_handled) {
            printf("BMS: SHORT PRESS DETECTED! (%d ms)\n", press_duration);
        }
        
        // If it was a long press
        if (button_press_handled) {
            printf("BMS: Long press released after %d ms\n", press_duration);
        }
        
        button_press_handled = false;
    }

    // show battery percentage while button is pressed
    if (button_current_state) {
        printf("BMS: Battery percentage\n");
    }

    // Update last state for next iteration
    button_last_state = button_current_state;
}
#endif

#endif  // AP_PERIPH_BATTERY_BMS_ENABLED
