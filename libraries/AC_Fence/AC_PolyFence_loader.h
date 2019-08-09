#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Common/Location.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

enum class AC_PolyFenceType {
    END_OF_STORAGE    = 99,
    POLYGON_INCLUSION = 98,
    POLYGON_EXCLUSION = 97,
    CIRCLE_EXCLUSION  = 96,
    RETURN_POINT = 95,
    CIRCLE_INCLUSION  = 94,
};

// a FenceItem is just a means of passing data about an item into
// and out of the polyfence loader.  It uses a AC_PolyFenceType to
// indicate the item type, assuming each fence type is made up of
// only one sort of item.
// FIXME: make this a union to save space (or a sublcass thing)
class AC_PolyFenceItem {
public:
    AC_PolyFenceType type;
    // FIXME: make this a Vector2l
    Vector2l loc;
    uint8_t vertex_count;
    float radius;
};

class AC_PolyFence_loader
{

public:

    AC_PolyFence_loader(AP_Int8 &total) :
        _total(total) {}

    AC_PolyFence_loader(const AC_PolyFence_loader &other) = delete;
    AC_PolyFence_loader &operator=(const AC_PolyFence_loader&) = delete;

    /// returns pointer to array of polygon points and num_points is
    /// filled in with the total number.  This does not include the
    /// return point or the closing point.  This is the inclusion fence
    Vector2f* get_boundary_points(uint16_t& num_points) const;

    // methods primarily for MissionItemProtocol_Fence to use:
    // return the total number of points stored
    uint16_t num_stored_items() const { return _eeprom_item_count; }
    bool get_item(const uint16_t seq, AC_PolyFenceItem &item) WARN_IF_UNUSED;
    // bool replace_item(const uint16_t seq, const AC_PolyFenceItem &item) WARN_IF_UNUSED;
    // bool append_item(const AC_PolyFenceItem &item) WARN_IF_UNUSED;
    bool truncate(uint8_t num) WARN_IF_UNUSED;

    ///
    /// exclusion zones
    ///
    /// returns number of polygon exclusion zones defined
    uint16_t get_exclusion_polygon_count() const {
        return _num_loaded_exclusion_boundaries;
    }

    /// returns pointer to array of exclusion polygon points and num_points is filled in with the number of points in the polygon
    /// points are offsets from EKF origin in NE frame
    Vector2f* get_exclusion_polygon(uint16_t index, uint16_t &num_points) const;

    /// return system time of last update to the exclusion polygon points
    uint32_t get_exclusion_polygon_update_ms() const {
        return _load_time_ms;
    }

    ///
    /// mavlink
    ///

    /// handler for polygon fence messages with GCS
    void handle_msg(class GCS_MAVLINK &link, const mavlink_message_t& msg);

    bool breached() WARN_IF_UNUSED;
    //   returns true if location is outside the boundary
    bool breached(const Location& loc) WARN_IF_UNUSED;

    //   returns true if boundary is valid
    bool valid_const() const WARN_IF_UNUSED { return _boundary != nullptr; }

    //   returns true if boundary is valid
    bool valid() WARN_IF_UNUSED;

    // returns the system in in millis when the boundary was last updated
    uint32_t update_ms() const {
        return _update_ms;
    }

    bool get_stored_return_point(Vector2f &ret) const WARN_IF_UNUSED {
        if (_loaded_return_point == nullptr) {
            return false;
        }
        ret = *_loaded_return_point;
        return true;
    }

    bool loaded() const WARN_IF_UNUSED;

    uint16_t eeprom_item_count() {
        return _eeprom_item_count;
    }

    // maximum number of fence points we can store in eeprom
    uint16_t max_items() const;

    bool validate_fence(const AC_PolyFenceItem *new_items, uint16_t count) const WARN_IF_UNUSED;
    bool write_fence(const AC_PolyFenceItem *new_items, uint16_t count)  WARN_IF_UNUSED;

private:

    /// load polygon points stored in eeprom into boundary array and
    /// perform validation.  returns true if load successfully
    /// completed
    bool load_from_eeprom() WARN_IF_UNUSED;
    void unload();
    bool format() WARN_IF_UNUSED;

    bool get_return_point(Vector2l &ret) const WARN_IF_UNUSED;

    bool breached(const Vector2f& location)  WARN_IF_UNUSED;

    // load boundary point from eeprom, returns true on successful load
    bool load_point_from_eeprom(uint16_t i, Vector2l& point) WARN_IF_UNUSED;

    // update the validity flag:
    bool calculate_boundary_valid() const WARN_IF_UNUSED;

    bool _inclusion_fence_valid;

    class FenceIndex {
    public:
        AC_PolyFenceType type;
        uint16_t count;
        uint16_t storage_offset;
    };
    uint16_t index_fence_count(const AC_PolyFenceType type);

    Vector2f *_boundary;          // array of boundary points.  Note: point 0 is the return point
    // FIXME:
    static FenceIndex *_boundary_index;   // array specifying type of each boundary point
    uint8_t _boundary_num_points; // number of points in the boundary array (should equal _total parameter after load has completed)
    static uint16_t _num_fences;

    uint32_t        _update_ms;   // system time of last update to the boundary in storage
    uint32_t _load_time_ms;

    void void_index() {
        free(_boundary_index);
        _boundary_index = nullptr;
    }

    bool check_indexed() WARN_IF_UNUSED;

    AP_Int8 &_total;

    template<typename T>
    bool calculate_centroid(T *points, uint16_t count, T &centroid) WARN_IF_UNUSED;
    // if no return point is present, and the boundary is complete,
    // fill the return point with the centroid of the boundary both
    // within eeprom and in points

    FenceIndex *find_first_fence(const AC_PolyFenceType type) const;
    FenceIndex *get_or_create_include_fence();
    FenceIndex *get_or_create_return_point();
    void create_return_point();

    uint16_t eos_offset;

    bool formatted_for_new_storage() const WARN_IF_UNUSED;
    bool format_for_new_storage() WARN_IF_UNUSED;
    bool contains_compatible_fence() const WARN_IF_UNUSED;

    bool find_index_for_seq(const uint16_t seq, const FenceIndex *&entry, uint16_t &i) const WARN_IF_UNUSED;
    bool find_storage_offset_for_seq(const uint16_t seq, uint16_t &offset, AC_PolyFenceType &type, uint16_t &vertex_count_offset) const WARN_IF_UNUSED;

    void handle_msg_fetch_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg);
    void handle_msg_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg);

    static const uint8_t new_fence_storage_magic = 235; // FIXME: ensure this is out-of-band for old lat/lon point storage

    // pointer into the boundary point array where the inclusion fence
    // can be found, and the number of points in that boundary:
    // FIXME: allow any number of these to be uploaded
    Vector2f *loaded_inclusion_boundary;
    uint8_t loaded_inclusion_point_count;

    // pointer into the boundary point array where the return point
    // can be found:
    Vector2f *_loaded_return_point;

    // pointers into the boundary point array where exclusion polygons
    // can be found:
    class ExclusionBoundary {
    public:
        Vector2f *points; // pointer into the _boundary array
        uint8_t count; // count of points in the boundary
    };
    ExclusionBoundary *_loaded_exclusion_boundary;
    uint8_t _num_loaded_exclusion_boundaries;

    class ExclusionCircle {
    public:
        Vector2f loc;
        float radius;
    };
    ExclusionCircle *_loaded_circle_exclusion_boundary;
    uint8_t _num_loaded_circle_exclusion_boundaries;

    class InclusionCircle {
    public:
        Vector2f loc;
        float radius;
    };
    InclusionCircle *_loaded_circle_inclusion_boundary;
    uint8_t _num_loaded_circle_inclusion_boundaries;

    bool _load_attempted;


    bool convert_to_new_storage() WARN_IF_UNUSED;
    bool read_scaled_latlon_from_storage(const Location &origin, uint16_t &read_offset, Vector2f &dest) WARN_IF_UNUSED;
    bool read_polygon_from_storage(const Location &origin,
                                   uint16_t &read_offset,
                                   const uint8_t vertex_count,
                                   Vector2f *&next_storage_point) WARN_IF_UNUSED;

    bool write_type_to_storage(uint16_t &offset, AC_PolyFenceType type) WARN_IF_UNUSED;
    bool write_latlon_to_storage(uint16_t &offset, const Vector2l &latlon) WARN_IF_UNUSED;
    bool write_fenceitem_to_storage(uint16_t &offset, const AC_PolyFenceItem &item) WARN_IF_UNUSED;
    bool read_latlon_from_storage(uint16_t &read_offset, Vector2l &latlon) const WARN_IF_UNUSED;

    bool scan_eeprom(void (*callback)(AC_PolyFenceType type, uint16_t offset)) WARN_IF_UNUSED;
    static void scan_eeprom_count_fences(const AC_PolyFenceType type, uint16_t read_offset);
    static void scan_eeprom_index_fences(const AC_PolyFenceType type, uint16_t read_offset);
    bool count_eeprom_fences() WARN_IF_UNUSED;
    bool index_eeprom() WARN_IF_UNUSED;
    bool create_compatible_fence() WARN_IF_UNUSED;

    static uint16_t _eeprom_fence_count;
    static uint16_t _eeprom_item_count;
};
