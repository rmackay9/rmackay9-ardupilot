#pragma once

#if AP_PERIPH_BATTERY_BMS_ENABLED

class BatteryBMS {
public:
    friend class AP_Periph_FW;
    BatteryBMS(void);

    void update(void);

private:

    // handle button press
    void handle_button_press(void);

    // startup LED variables
    uint8_t init_stage; // current stage of the startup LED sequence
    bool init_done;     // true once the startup LED sequence has completed

    // Button handling variables
    bool button_last_state;
    uint32_t button_press_start_ms;
    bool button_press_handled;
    static const uint32_t LONG_PRESS_THRESHOLD_MS = 2000; // 2 seconds for long press
};

#endif // AP_PERIPH_BATTERY_TAG_ENABLED

