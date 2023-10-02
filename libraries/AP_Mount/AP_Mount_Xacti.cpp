#include "AP_Mount_Xacti.h"

#if HAL_MOUNT_XACTI_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <com/xacti/GnssStatusReq.hpp>
#include <com/xacti/GimbalAttitudeStatus.hpp>
#include <AP_RTC/AP_RTC.h>

extern const AP_HAL::HAL& hal;

// DroneCAN Frontend Registry Binder
UC_REGISTRY_BINDER(XactiGimbalAttitudeStatusCb, com::xacti::GimbalAttitudeStatus);
UC_REGISTRY_BINDER(XactiGnssStatusReqCb, com::xacti::GnssStatusReq);

#define LOG_TAG "Mount"
#define XACTI_MSG_SEND_MIN_MS 20                    // messages should not be sent to camera more often than 20ms
#define XACTI_ZOOM_RATE_UPDATE_INTERVAL_MS  500     // zoom rate control increments zoom by 10% up or down every 0.5sec
#define XACTI_STATUS_REQ_INTERVAL_MS 3000           // request status every 3 seconds
#define XACTI_SET_PARAM_QUEUE_SIZE 3                // three set-param requests may be queued

#define AP_MOUNT_XACTI_DEBUG 0
#define debug(fmt, args ...) do { if (AP_MOUNT_XACTI_DEBUG) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xacti: " fmt, ## args); } } while (0)

bool AP_Mount_Xacti::_subscribed = false;
AP_Mount_Xacti::DetectedModules AP_Mount_Xacti::_detected_modules[];
HAL_Semaphore AP_Mount_Xacti::_sem_registry;
const char* AP_Mount_Xacti::send_text_prefix = "Xacti:";
const char* AP_Mount_Xacti::sensor_mode_str[] = { "RGB", "IR", "PIP", "NDVI" };
const char* AP_Mount_Xacti::_param_names[] = {"SingleShot", "Recording", "FocusMode", "SensorMode", "DigitalZoomMagnification", "FirmwareVersion", "Status", "DateTime"};

// Constructor
AP_Mount_Xacti::AP_Mount_Xacti(class AP_Mount &frontend, class AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance)
{
    register_backend();

    param_int_cb = FUNCTOR_BIND_MEMBER(&AP_Mount_Xacti::handle_param_get_set_response_int, bool, AP_UAVCAN*, const uint8_t, const char*, int32_t &);
    param_string_cb = FUNCTOR_BIND_MEMBER(&AP_Mount_Xacti::handle_param_get_set_response_string, bool, AP_UAVCAN*, const uint8_t, const char*, AP_UAVCAN::string &);
    param_save_cb = FUNCTOR_BIND_MEMBER(&AP_Mount_Xacti::handle_param_save_response, void, AP_UAVCAN*, const uint8_t, bool);

    // static assert that Param enum matches parameter names array
    static_assert((uint8_t)AP_Mount_Xacti::Param::LAST+1 == ARRAY_SIZE(AP_Mount_Xacti::_param_names), "AP_Mount_Xacti::_param_names array must match Param enum");
}

// init - performs any required initialisation for this instance
void AP_Mount_Xacti::init()
{
    // instantiate parameter queue, return on failure so init fails
    _set_param_int32_queue = new ObjectArray<SetParamQueueItem>(XACTI_SET_PARAM_QUEUE_SIZE);
    if (_set_param_int32_queue == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s init failed", send_text_prefix);
        return;
    }
    _initialised = true;
}

// update mount position - should be called periodically
void AP_Mount_Xacti::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // return immediately if any message sent is unlikely to be processed
    const uint32_t now_ms = AP_HAL::millis();
    if (!is_safe_to_send(now_ms)) {
        return;
    }

    // get firmware version
    if (request_firmware_version(now_ms)) {
        return;
    }

    // set date and time
    if (_firmware_version.received && set_datetime(now_ms)) {
        return;
    }

    // request status
    if (request_status(now_ms)) {
        return;
    }

    // process queue of set parameter items
    if (process_set_param_int32_queue()) {
        return;
    }

    // periodically send copter attitude and GPS status
    if (send_copter_att_status(now_ms)) {
        // if message sent avoid sending other messages
        return;
    }

    // update zoom rate control
    if (update_zoom_rate_control(now_ms)) {
        // if message sent avoid sending other messages
        return;
    }

    // update based on mount mode
    switch (get_mode()) {
        // move mount to a "retracted" position.  To-Do: remove support and replace with a relaxed mode?
        case MAV_MOUNT_MODE_RETRACT: {
            const Vector3f &angle_bf_target = _params.retract_angles.get();
            send_target_angles(ToRad(angle_bf_target.y), ToRad(angle_bf_target.z), false);
            break;
        }

        // move mount to a neutral position, typically pointing forward
        case MAV_MOUNT_MODE_NEUTRAL: {
            const Vector3f &angle_bf_target = _params.neutral_angles.get();
            send_target_rates(ToRad(angle_bf_target.y), ToRad(angle_bf_target.z), false);
            break;
        }

        // point to the angles given by a mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            switch (mavt_target.target_type) {
            case MountTargetType::ANGLE:
                send_target_angles(mavt_target.angle_rad.pitch, mavt_target.angle_rad.yaw, mavt_target.angle_rad.yaw_is_ef);
                break;
            case MountTargetType::RATE:
                send_target_rates(mavt_target.rate_rads.pitch, mavt_target.rate_rads.yaw, mavt_target.rate_rads.yaw_is_ef);
                break;
            }
            break;

        // RC radio manual angle control, but with stabilization from the AHRS
        case MAV_MOUNT_MODE_RC_TARGETING: {
            // update targets using pilot's rc inputs
            MountTarget rc_target {};
            if (get_rc_rate_target(rc_target)) {
                send_target_rates(rc_target.pitch, rc_target.yaw, rc_target.yaw_is_ef);
            } else if (get_rc_angle_target(rc_target)) {
                send_target_angles(rc_target.pitch, rc_target.yaw, rc_target.yaw_is_ef);
            }
            break;
        }

        // point mount to a GPS point given by the mission planner
        case MAV_MOUNT_MODE_GPS_POINT: {
            MountTarget angle_target_rad {};
            if (get_angle_target_to_roi(angle_target_rad)) {
                send_target_angles(angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        case MAV_MOUNT_MODE_HOME_LOCATION: {
            MountTarget angle_target_rad {};
            if (get_angle_target_to_home(angle_target_rad)) {
                send_target_angles(angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        case MAV_MOUNT_MODE_SYSID_TARGET:{
            MountTarget angle_target_rad {};
            if (get_angle_target_to_sysid(angle_target_rad)) {
                send_target_angles(angle_target_rad.pitch, angle_target_rad.yaw, angle_target_rad.yaw_is_ef);
            }
            break;
        }

        default:
            // we do not know this mode so raise internal error
            INTERNAL_ERROR(AP_InternalError::error_t::flow_of_control);
            break;
    }
}

// return true if healthy
bool AP_Mount_Xacti::healthy() const
{
    // unhealthy until gimbal has been found and replied with firmware version and no motor errors
    if (!_initialised || !_firmware_version.received || _motor_error) {
        return false;
    }

    // unhealthy if attitude information NOT received recently
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - _last_current_attitude_quat_ms > 1000) {
        return false;
    }

    // if we get this far return healthy
    return true;
}

// take a picture.  returns true on success
bool AP_Mount_Xacti::take_picture()
{
    // fail if camera errored
    if (_camera_error) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s unable to take pic", send_text_prefix);
        return false;
    }

    return set_param_int32(Param::SingleShot, 0);
}

// start or stop video recording.  returns true on success
// set start_recording = true to start record, false to stop recording
bool AP_Mount_Xacti::record_video(bool start_recording)
{
    return set_param_int32(Param::Recording, start_recording ? 1 : 0);
}

// zoom in, out or hold
// zoom out = -1, hold = 0, zoom in = 1
bool AP_Mount_Xacti::set_zoom_step(int8_t zoom_step)
{
    // zoom rate
    if (zoom_step == 0) {
        // stop zooming
        _zoom_rate_control.enabled = false;
    } else {
        // zoom in or out
        _zoom_rate_control.enabled = true;
        _zoom_rate_control.increment = (zoom_step < 0) ? -100 : 100;
    }
    return true;
}

// set focus in, out or hold.  returns true on success
// focus in = -1, focus hold = 0, focus out = 1
bool AP_Mount_Xacti::set_manual_focus_step(int8_t focus_step)
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // convert focus type and value to parameter value
    // focus rate and percentage control not supported so simply switch to manual focus
    // FocusMode of 0:Manual Focus
    uint8_t focus_param_value = 0;

    // set FocusMode parameter
    return set_param_int32(Param::FocusMode, focus_param_value);
}

// auto focus.  returns true on success
bool AP_Mount_Xacti::set_auto_focus()
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // FocusMode of 1:Single AutoFocus, 2:Continuous AutoFocus
    uint8_t focus_param_value = 2;

    // set FocusMode parameter
    return set_param_int32(Param::FocusMode, focus_param_value);
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_Xacti::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat = _current_attitude_quat;
    return true;
}

// send target pitch and yaw rates to gimbal
// yaw_is_ef should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_Xacti::send_target_rates(float pitch_rads, float yaw_rads, bool yaw_is_ef)
{
    // send gimbal rate target to gimbal
    send_gimbal_control(3, degrees(pitch_rads) * 100, degrees(yaw_rads) * 100);
}

// send target pitch and yaw angles to gimbal
// yaw_is_ef should be true if yaw_rad target is an earth frame angle, false if body_frame
void AP_Mount_Xacti::send_target_angles(float pitch_rad, float yaw_rad, bool yaw_is_ef)
{
    // convert yaw to body frame
    const float yaw_bf_rad = yaw_is_ef ? wrap_PI(yaw_rad - AP::ahrs().yaw) : yaw_rad;

    // send angle target to gimbal
    send_gimbal_control(2, degrees(pitch_rad) * 100, degrees(yaw_bf_rad) * 100);
}

// subscribe to Xacti DroneCAN messages
void AP_Mount_Xacti::subscribe_msgs(AP_UAVCAN* ap_dronecan)
{
    // return immediately if DroneCAN is unavailable
    if (ap_dronecan == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "%s DroneCAN subscribe failed", send_text_prefix);
        return;
    }

    _subscribed = true;

    auto* node = ap_dronecan->get_node();

    uavcan::Subscriber<com::xacti::GimbalAttitudeStatus, XactiGimbalAttitudeStatusCb> *xacti_gimbal_attitude_status_listener;
    xacti_gimbal_attitude_status_listener = new uavcan::Subscriber<com::xacti::GimbalAttitudeStatus, XactiGimbalAttitudeStatusCb>(*node);
    if (xacti_gimbal_attitude_status_listener == nullptr) {
        AP_BoardConfig::allocation_error("gimbal_attitude_status_sub");
        _subscribed = false;
    }

    // register method to handle incoming message
    const int xacti_gimbal_attitude_status_listener_res = xacti_gimbal_attitude_status_listener->start(XactiGimbalAttitudeStatusCb(ap_dronecan, &handle_gimbal_attitude_status));
    if (xacti_gimbal_attitude_status_listener_res < 0) {
        AP_BoardConfig::allocation_error("Xacti: subscriber start problem");
        _subscribed = false;
        return;
    }

    uavcan::Subscriber<com::xacti::GnssStatusReq, XactiGnssStatusReqCb> *xacti_gnss_status_req_listener;
    xacti_gnss_status_req_listener = new uavcan::Subscriber<com::xacti::GnssStatusReq, XactiGnssStatusReqCb>(*node);
    if (xacti_gnss_status_req_listener == nullptr) {
        AP_BoardConfig::allocation_error("gnss_status_req_sub");
        _subscribed = false;
    }

    // register method to handle incoming message
    const int xacti_gnss_status_req_listener_res = xacti_gnss_status_req_listener->start(XactiGnssStatusReqCb(ap_dronecan, &handle_gnss_status_req));
    if (xacti_gnss_status_req_listener_res < 0) {
        AP_BoardConfig::allocation_error("Xacti: subscriber start problem");
        _subscribed = false;
        return;
    }

    // debug
    if (_subscribed) {
        gcs().send_text(MAV_SEVERITY_CRITICAL, "Xacti: DroneCAN subscribe succeeded!");
    }
}

// register backend in detected modules array used to map DroneCAN port and node id to backend
void AP_Mount_Xacti::register_backend()
{
    WITH_SEMAPHORE(_sem_registry);

    // add this backend to _detected_modules array
    _detected_modules[_instance].driver = this;

    // return if devid is zero meaning this backend has not yet been associated with a mount
    const uint32_t devid = (uint32_t)_params.dev_id.get();
    if (devid == 0) {
        return;
    }

    // get DroneCan port from device id
    const uint8_t can_driver_index = AP_HAL::Device::devid_get_bus(devid);
    const uint8_t can_num_drivers = AP::can().get_num_drivers();
    for (uint8_t i = 0; i < can_num_drivers; i++) {
        AP_UAVCAN *ap_dronecan = AP_UAVCAN::get_uavcan(i);
        if (ap_dronecan != nullptr && ap_dronecan->get_driver_index() == can_driver_index) {
            _detected_modules[_instance].ap_dronecan = ap_dronecan;
        }
    }

    // get node_id from device id
    _detected_modules[_instance].node_id = AP_HAL::Device::devid_get_address(devid);
}

// find backend associated with the given dronecan port and node_id.  also associates backends with zero node ids
// returns pointer to backend on success, nullptr on failure
AP_Mount_Xacti* AP_Mount_Xacti::get_dronecan_backend(AP_UAVCAN* ap_dronecan, uint8_t node_id)
{
    WITH_SEMAPHORE(_sem_registry);

    // exit immediately if DroneCAN is unavailable or invalid node id
    if (ap_dronecan == nullptr || node_id == 0) {
        return nullptr;
    }

    // search for backend with matching dronecan port and node id
    for (uint8_t i = 0; i < ARRAY_SIZE(_detected_modules); i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].ap_dronecan == ap_dronecan &&
            _detected_modules[i].node_id == node_id ) {
            return _detected_modules[i].driver;
        }
    }

    // if we got this far, this dronecan port and node id are not associated with any backend
    // associate with first backend with node id of zero
    for (uint8_t i = 0; i < ARRAY_SIZE(_detected_modules); i++) {
        if (_detected_modules[i].driver != nullptr &&
            _detected_modules[i].node_id == 0) {
                _detected_modules[i].ap_dronecan = ap_dronecan;
                _detected_modules[i].node_id = node_id;
                const auto dev_id = AP_HAL::Device::make_bus_id(AP_HAL::Device::BUS_TYPE_UAVCAN,
                                                                ap_dronecan->get_driver_index(),
                                                                node_id, 0);
                _detected_modules[i].driver->set_dev_id(dev_id);
                return _detected_modules[i].driver;
        }
    }

    return nullptr;
}

void AP_Mount_Xacti::handle_gimbal_attitude_status(AP_UAVCAN* ap_dronecan, uint8_t node_id, const XactiGimbalAttitudeStatusCb &cb)
{
    // fetch the matching backend driver, node id and gimbal id backend instance
    AP_Mount_Xacti* driver = get_dronecan_backend(ap_dronecan, node_id);
    if (driver == nullptr) {
        return;
    }

    // convert body-frame Euler angles to Quaternion.  Note yaw direction is reversed from normal
    driver->_current_attitude_quat.from_euler(radians(cb.msg->gimbal_roll * 0.01), radians(cb.msg->gimbal_pitch * 0.01), radians(-cb.msg->gimbal_yaw * 0.01));
    driver->_last_current_attitude_quat_ms = AP_HAL::millis();

    // debug
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Xacti: r:%d p:%d y:%d", (int)(cb.msg->gimbal_roll * 0.01), (int)(cb.msg->gimbal_pitch * 0.01), (int)(cb.msg->gimbal_yaw * 0.01));
}

void AP_Mount_Xacti::handle_gnss_status_req(AP_UAVCAN* ap_dronecan, uint8_t node_id, const XactiGnssStatusReqCb &cb)
{
    // fetch the matching backend driver, node id and gimbal id backend instance
    AP_Mount_Xacti* driver = get_dronecan_backend(ap_dronecan, node_id);
    if (driver == nullptr) {
        return;
    }

    // trigger sending xacti specific gnss status message
    ap_dronecan->trigger_send_gnss_status(cb.msg->requirement);    
}

// handle param get/set response
bool AP_Mount_Xacti::handle_param_get_set_response_int(AP_UAVCAN* ap_dronecan, uint8_t node_id, const char* name, int32_t &value)
{
    // display errors
    const char* err_prefix_str = "Xacti: failed to";
    if (strcmp(name, get_param_name_str(Param::SingleShot)) == 0) {
        if (value < 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s take pic", err_prefix_str);
        }
        return false;
    }
    if (strcmp(name, get_param_name_str(Param::Recording)) == 0) {
        if (value < 0) {
            _recording_video = false;
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s record", err_prefix_str);
        } else {
            _recording_video = (value == 1);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s recording %s", send_text_prefix, _recording_video ? "ON" : "OFF");
        }
        return false;
    }
    if (strcmp(name, get_param_name_str(Param::FocusMode)) == 0) {
        if (value < 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s change focus", err_prefix_str);
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %s focus", send_text_prefix, value == 0 ? "manual" : "auto");
        }
        return false;
    }
    if (strcmp(name, get_param_name_str(Param::SensorMode)) == 0) {
        if (value < 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s change lens", err_prefix_str);
        } else if ((uint32_t)value < ARRAY_SIZE(sensor_mode_str)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s %s", send_text_prefix, sensor_mode_str[(uint8_t)value]);
        }
        return false;
    }
    if (strcmp(name, get_param_name_str(Param::DigitalZoomMagnification)) == 0) {
        if (value < 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s change zoom", err_prefix_str);
            // disable zoom rate control (if active) to avoid repeated failures
            _zoom_rate_control.enabled = false;
        } else if (value >= 100 && value <= 1000) {
            _last_zoom_param_value = value;
        }
        return false;
    }
    // unhandled parameter get or set
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s get/set %s res:%ld", send_text_prefix, name, (long int)value);
    return false;
}

// handle param get/set response
bool AP_Mount_Xacti::handle_param_get_set_response_string(AP_UAVCAN* ap_dronecan, uint8_t node_id, const char* name, AP_UAVCAN::string &value)
{
    if (strcmp(name, get_param_name_str(Param::FirmwareVersion)) == 0) {
        _firmware_version.received = true;
        const uint8_t len = MIN(value.len, ARRAY_SIZE(_firmware_version.str)-1);
        memcpy(_firmware_version.str, (const char*)value.data, len);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Mount: Xacti fw:%s", _firmware_version.str);

        // firmware str from gimbal is of the format YYMMDD[b]xx.  Convert to uint32 for reporting to GCS
        if (len >= 9) {
            const char major_str[3] = {_firmware_version.str[0], _firmware_version.str[1], 0};
            const char minor_str[3] = {_firmware_version.str[2], _firmware_version.str[3], 0};
            const char patch_str[3] = {_firmware_version.str[4], _firmware_version.str[5], 0};
            const char dev_str[3] = {_firmware_version.str[7], _firmware_version.str[8], 0};
            const uint8_t major_ver_num = atoi(major_str) & 0xFF;
            const uint8_t minor_ver_num = atoi(minor_str) & 0xFF;
            const uint8_t patch_ver_num = atoi(patch_str) & 0xFF;
            const uint8_t dev_ver_num = atoi(dev_str) & 0xFF;
            _firmware_version.mav_ver = UINT32_VALUE(dev_ver_num, patch_ver_num, minor_ver_num, major_ver_num);
        }
        return false;
    } else if (strcmp(name, get_param_name_str(Param::DateTime)) == 0) {
        // display when time and date have been set
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s datetime set %s", send_text_prefix, (const char*)value.data);
        return false;
    } else if (strcmp(name, get_param_name_str(Param::Status)) == 0) {
        // check for expected length
        const char* error_str = "error";
        if (value.len != sizeof(_status)) {
            INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
            return false;
        }

        // backup error status and copy to structure
        const uint32_t last_error_status = _status.error_status;
        memcpy(&_status, value.data, value.len);

        // report change in status
        uint32_t changed_bits = last_error_status ^ _status.error_status;
        const char* ok_str = "OK";
        if (changed_bits & (uint32_t)ErrorStatus::TIME_NOT_SET) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s time %sset", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::TIME_NOT_SET ? "not " : "");
            if (_status.error_status & (uint32_t)ErrorStatus::TIME_NOT_SET) {
                // try to set time again
                _datetime.set = false;
            }
        }
        if (changed_bits & (uint32_t)ErrorStatus::MEDIA_ERROR) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s media %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::MEDIA_ERROR ? error_str : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::LENS_ERROR) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s lens %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::LENS_ERROR ? error_str : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::MOTOR_INIT_ERROR) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s motor %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::MOTOR_INIT_ERROR ? "init error" : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::MOTOR_OPERATION_ERROR) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s motor op %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::MOTOR_OPERATION_ERROR ? error_str : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::GIMBAL_CONTROL_ERROR) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s control %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::GIMBAL_CONTROL_ERROR ? error_str : ok_str);
        }
        if (changed_bits & (uint32_t)ErrorStatus::TEMP_WARNING) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s temp %s", send_text_prefix, _status.error_status & (uint32_t)ErrorStatus::TEMP_WARNING ? "warning" : ok_str);
        }

        // set motor error for health reporting
        _motor_error = _status.error_status & ((uint32_t)ErrorStatus::MOTOR_INIT_ERROR | (uint32_t)ErrorStatus::MOTOR_OPERATION_ERROR | (uint32_t)ErrorStatus::GIMBAL_CONTROL_ERROR);
        _camera_error = _status.error_status & ((uint32_t)ErrorStatus::LENS_ERROR | (uint32_t)ErrorStatus::MEDIA_ERROR);
        return false;
    }

    // unhandled parameter get or set
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "%s get/set string %s res:%s", send_text_prefix, name, (const char*)value.data);
    return false;
}

void AP_Mount_Xacti::handle_param_save_response(AP_UAVCAN* ap_dronecan, const uint8_t node_id, bool success)
{
    // display failure to save parameter
    if (!success) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s CAM%u failed to set param", send_text_prefix, (int)_instance+1);
    }
}

// get parameter name for a particular param enum value
const char* AP_Mount_Xacti::get_param_name_str(Param param) const
{
    // check to avoid reading beyond end of array.  This should never happen
    if ((uint8_t)param > ARRAY_SIZE(_param_names)) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return nullptr;
    }
    return _param_names[(uint8_t)param];
}

// helper function to set integer parameters
bool AP_Mount_Xacti::set_param_int32(Param param, int32_t param_value)
{
    if (_set_param_int32_queue == nullptr) {
        return false;
    }

    // set param request added to queue to be sent.  throttling requests improves reliability
    return _set_param_int32_queue->push(SetParamQueueItem{param, param_value});
}

// helper function to set string parameters
bool AP_Mount_Xacti::set_param_string(Param param, const AP_UAVCAN::string& param_value)
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // convert param to string
    const char* param_name_str = get_param_name_str(param);
    if (param_name_str == nullptr) {
        return false;
    }

    if (_detected_modules[_instance].ap_dronecan->set_parameter_on_node(_detected_modules[_instance].node_id, param_name_str, param_value, &param_string_cb)) {
        last_send_getset_param_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

// helper function to get string parameters
bool AP_Mount_Xacti::get_param_string(Param param)
{
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // convert param to string
    const char* param_name_str = get_param_name_str(param);
    if (param_name_str == nullptr) {
        return false;
    }

    if (_detected_modules[_instance].ap_dronecan->get_parameter_on_node(_detected_modules[_instance].node_id, param_name_str, &param_string_cb)) {
        last_send_getset_param_ms = AP_HAL::millis();
        return true;
    }
    return false;
}

// process queue of set parameter items
bool AP_Mount_Xacti::process_set_param_int32_queue()
{
    if ((_set_param_int32_queue == nullptr) || (_detected_modules[_instance].ap_dronecan == nullptr)) {
        return false;
    }

    SetParamQueueItem param_to_set;
    if (_set_param_int32_queue->pop(param_to_set)) {
        // convert param to string
        const char* param_name_str = get_param_name_str(param_to_set.param);
        if (param_name_str == nullptr) {
            return false;
        }
        if (_detected_modules[_instance].ap_dronecan->set_parameter_on_node(_detected_modules[_instance].node_id, param_name_str, param_to_set.value, &param_int_cb)) {
            last_send_getset_param_ms = AP_HAL::millis();
            return true;
        }
        return false;
    }
    return false;
}

// send gimbal control message via DroneCAN
// mode is 2:angle control or 3:rate control
// pitch_cd is pitch angle in centi-degrees or pitch rate in cds
// yaw_cd is angle in centi-degrees or yaw rate in cds
void AP_Mount_Xacti::send_gimbal_control(uint8_t mode, int16_t pitch_cd, int16_t yaw_cd)
{
    // exit immediately if no DroneCAN port
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return;
    }

    // send at no faster than 5hz
    const uint32_t now_ms = AP_HAL::native_millis();
    if (now_ms - last_send_gimbal_control_ms < 200) {
        return;
    }
    last_send_gimbal_control_ms = now_ms;

    // send xacti specific gimbal control message
    _detected_modules[_instance].ap_dronecan->set_xacti_gimbal_control(mode, pitch_cd, yaw_cd);
}

// send copter attitude status message to gimbal.  now_ms is current system time
// returns true if sent so that we avoid immediately trying to also send other messages
bool AP_Mount_Xacti::send_copter_att_status(uint32_t now_ms)
{
    // exit immediately if no DroneCAN port
    if (_detected_modules[_instance].ap_dronecan == nullptr) {
        return false;
    }

    // send at no faster than 5hz
    if (now_ms - last_send_copter_att_status_ms < 100) {
        return false;
    }
    last_send_copter_att_status_ms = now_ms;

    // trigger sending vehicle attitude message
    _detected_modules[_instance].ap_dronecan->trigger_send_xacti_copter_att_status();
    return true;
}

// update zoom rate controller.  now_ms is current system time
// returns true if sent so that we avoid immediately trying to also send other messages
bool AP_Mount_Xacti::update_zoom_rate_control(uint32_t now_ms)
{
    // return immediately if zoom rate control is not enabled
    if (!_zoom_rate_control.enabled) {
        return false;
    }

    // update only every 0.5 sec
    if (now_ms - _zoom_rate_control.last_update_ms < XACTI_ZOOM_RATE_UPDATE_INTERVAL_MS) {
        return false;
    }
    _zoom_rate_control.last_update_ms = now_ms;

    // increment zoom
    const uint16_t zoom_value = _last_zoom_param_value + _zoom_rate_control.increment;

    // if reached limit then disable zoom
    if ((zoom_value < 100) || (zoom_value > 1000)) {
        _zoom_rate_control.enabled = false;
        return false;
    }

    // send desired zoom to camera
    return set_param_int32(Param::DigitalZoomMagnification, zoom_value);
}

// request firmware version. now_ms should be current system time (reduces calls to AP_HAL::millis)
// returns true if sent so that we avoid immediately trying to also send other messages
bool AP_Mount_Xacti::request_firmware_version(uint32_t now_ms)
{
    // return immediately if already have version or no dronecan
    if (_firmware_version.received) {
        return false;
    }

    // send request once per second until received
    if (now_ms - _firmware_version.last_request_ms < 1000) {
        return false;
    }
    _firmware_version.last_request_ms = now_ms;
    return get_param_string(Param::FirmwareVersion);
}

// set date and time.  now_ms is current system time
bool AP_Mount_Xacti::set_datetime(uint32_t now_ms)
{
    // return immediately if gimbal's date/time has been set
    if (_datetime.set) {
        return false;
    }

    // attempt to set datetime once per second until received
    if (now_ms - _datetime.last_request_ms < 1000) {
        return false;
    }
    _datetime.last_request_ms = now_ms;

    // get date and time
    uint16_t year;
    uint8_t month, day, hour, min, sec;
    if (!AP::rtc().get_date_and_time_utc(year, month, day, hour, min, sec)) {
        return false;
    }

    // date time is of the format YYYYMMDDHHMMSS (14 bytes)
    // convert month from 0~11 to 1~12 range
    AP_UAVCAN::string datetime_string {};
    const int num_bytes = snprintf((char *)datetime_string.data, sizeof(AP_UAVCAN::string::data), "%04u%02u%02u%02u%02u%02u",
                             (unsigned)year, (unsigned)month+1, (unsigned)day, (unsigned)hour, (unsigned)min, (unsigned)sec);
    // sanity check bytes to be sent
    if (num_bytes != 14) {
        INTERNAL_ERROR(AP_InternalError::error_t::invalid_arg_or_result);
        return false;
    }
    datetime_string.len = num_bytes;
    _datetime.set = set_param_string(Param::DateTime, datetime_string);
    if (!_datetime.set) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s failed to set date/time", send_text_prefix);
    }

    return _datetime.set;
}

// request status. now_ms should be current system time (reduces calls to AP_HAL::millis)
// returns true if sent so that we avoid immediately trying to also send other messages
bool AP_Mount_Xacti::request_status(uint32_t now_ms)
{
    // return immediately if 3 seconds has not passed
    if (now_ms - _status_report.last_request_ms < XACTI_STATUS_REQ_INTERVAL_MS) {
        return false;
    }

    _status_report.last_request_ms = now_ms;
    return get_param_string(Param::Status);
}

// check if safe to send message (if messages sent too often camera will not respond)
// now_ms should be current system time (reduces calls to AP_HAL::millis)
bool AP_Mount_Xacti::is_safe_to_send(uint32_t now_ms) const
{
    // check time since last attitude sent
    if (now_ms - last_send_copter_att_status_ms < XACTI_MSG_SEND_MIN_MS) {
        return false;
    }

    // check time since last angle target sent
    if (now_ms - last_send_gimbal_control_ms < XACTI_MSG_SEND_MIN_MS) {
        return false;
    }

    // check time since last set param message sent
    if (now_ms - last_send_getset_param_ms < XACTI_MSG_SEND_MIN_MS) {
        return false;
    }

    return true;
}

#endif // HAL_MOUNT_XACTI_ENABLED
