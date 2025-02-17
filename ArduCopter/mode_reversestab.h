#pragma once

#include "mode.h"

class ModeReverseStab : public Mode {

public:
    ModeReverseStab(void);

    bool init(bool ignore_checks) override;
    void run() override;

    bool requires_GPS() const override { return false; }
    bool has_manual_throttle() const override { return true; }
    bool allows_arming(AP_Arming::Method method) const override { return true; };
    bool is_autopilot() const override { return false; }
    bool has_user_takeoff(bool must_navigate) const override { return false; }
    bool allows_flip() const override { return true; }

protected:

    const char *name() const override { return "REVERSESTAB"; }
    const char *name4() const override { return "RSTB"; }
};
