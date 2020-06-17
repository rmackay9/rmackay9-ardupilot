#pragma once

#include <AP_Param/AP_Param.h>

#define AP_NAKEKF_SOURCE_COUNT 3    // three banks of sources

class AP_NavEKF_Source
{

public:
    // Constructor
    AP_NavEKF_Source();

    /* Do not allow copies */
    AP_NavEKF_Source(const AP_NavEKF_Source &other) = delete;
    AP_NavEKF_Source &operator=(const AP_NavEKF_Source&) = delete;

    enum class SourceXY {
        NONE = 0,
        // BARO = 1 (not applicable)
        // RANGEFINDER = 2 (not applicable)
        GPS = 3,
        BEACON = 4,
        OPTFLOW = 5,
        EXTNAV = 6,
        WHEEL_ENCODER = 7
    };

    enum class SourceZ {
        NONE = 0,
        BARO = 1,
        RANGEFINDER = 2,
        GPS = 3,
        BEACON = 4,
        // OPTFLOW = 5 (not applicable)
        EXTNAV = 6
    };

    enum class SourceYaw {
        NONE = 0,
        COMPASS = 1,
        EXTERNAL = 2,
        EXTERNAL_COMPASS_FALLBACK = 3
    };

    // initialisation
    void init();

    // get current position source
    SourceXY getPosXYSource() const { return _active_source.initialised ? _active_source.posxy : (SourceXY)_source[0].posxy.get(); }
    SourceZ getPosZSource() const { return _active_source.initialised ? _active_source.posz : (SourceZ)_source[0].posz.get() ; }

    // set position and velocity sources to either 0=primary, 1=secondary, 2=tertiary
    void setPosVelXYZSource(uint8_t source_idx);

    // get/set velocity source
    SourceXY getVelXYSource() const { return _active_source.initialised ? _active_source.velxy : (SourceXY)_source[0].velxy.get(); }
    SourceZ getVelZSource() const { return _active_source.initialised ? _active_source.velz : (SourceZ)_source[0].velz.get(); }
    void setVelZSource(SourceZ source) { _active_source.velz = source; }

    // get yaw source
    SourceYaw getYawSource() const { return _active_source.initialised ? _active_source.yaw : (SourceYaw)_source[0].yaw.get(); }

    // align position of inactive sources to ahrs
    void align_inactive_sources();

    // sensor specific helper functions

    // true if any source is GPS
    bool usingGPS() const;

    // true if any primary source parameters have been configured (used for parameter conversion)
    bool params_configured_in_storage() const;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const;

    static const struct AP_Param::GroupInfo var_info[];

private:

    // get source by index (0, 1 or 2)
    SourceXY getPosXYSourceByIndex(uint8_t idx) const;
    SourceZ getPosZSourceByIndex(uint8_t idx) const;
    SourceXY getVelXYSourceByIndex(uint8_t idx) const;
    SourceZ getVelZSourceByIndex(uint8_t idx) const;
    SourceYaw getYawSourceByIndex(uint8_t idx) const;

    // Parameters
    struct {
        AP_Int8 posxy;  // xy position source
        AP_Int8 velxy;  // xy velocity source
        AP_Int8 posz;   // position z (aka altitude or height) source
        AP_Int8 velz;   // velocity z source
        AP_Int8 yaw;    // yaw source
    } _source[AP_NAKEKF_SOURCE_COUNT];

    // active sources
    struct {
        bool initialised;   // true once init has been run
        SourceXY posxy;     // current xy position source
        SourceZ posz;       // current z position source
        SourceXY velxy;     // current xy velocity source
        SourceZ velz;       // current z velocity source
        SourceYaw yaw;      // current yaw source
    } _active_source;
};
