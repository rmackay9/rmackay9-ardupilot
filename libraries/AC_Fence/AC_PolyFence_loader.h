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
    RETURN_POINT      = 95,
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
    /// return point or the closing point.  These points come from the
    /// *first* inclusion fence
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
    bool valid_const() const WARN_IF_UNUSED { return _loaded_offsets_from_origin != nullptr; }

    //   returns true if boundary is valid
    bool valid() WARN_IF_UNUSED;

    // returns the system in in millis when the boundary was last updated
    uint32_t update_ms() const {
        return _update_ms;
    }

    bool loaded() const WARN_IF_UNUSED;

    // eeprom_item_count - returns the number of items currently
    // stored in fence storage
    uint16_t eeprom_item_count() {
        return _eeprom_item_count;
    }

    // maximum number of fence points we can store in eeprom
    uint16_t max_items() const;

    bool validate_fence(const AC_PolyFenceItem *new_items, uint16_t count) const WARN_IF_UNUSED;
    bool write_fence(const AC_PolyFenceItem *new_items, uint16_t count)  WARN_IF_UNUSED;

private:

    // breached(Vector2f&) - returns true of location breaches any fence
    bool breached(const Vector2f& location)  WARN_IF_UNUSED;

    /*
     * Fence storage Index related functions
     */
    // FenceIndex - a class used to store information about a fence in
    // fence storage.
    class FenceIndex {
    public:
        AC_PolyFenceType type;
        uint16_t count;
        uint16_t storage_offset;
    };
    // index_fence_count - returns the number of fences of type
    // currently in the index
    uint16_t index_fence_count(const AC_PolyFenceType type);

    void void_index() {
        free(_index);
        _index = nullptr;
    }

    bool check_indexed() WARN_IF_UNUSED;

    // find_first_fence - return first fence in index of specific type
    FenceIndex *find_first_fence(const AC_PolyFenceType type) const;

    // find_index_for_seq - returns true if seq is contained within a
    // fence.  If it is, entry will be the relevant FenceIndex.  i
    // will be the offset within _loaded_offsets_from_origin where the
    // first point in the fence is found CHECKME
    bool find_index_for_seq(const uint16_t seq, const FenceIndex *&entry, uint16_t &i) const WARN_IF_UNUSED;
    // find_storage_offset_for_seq - uses the index to return an
    // offset into storage for an item
    bool find_storage_offset_for_seq(const uint16_t seq, uint16_t &offset, AC_PolyFenceType &type, uint16_t &vertex_count_offset) const WARN_IF_UNUSED;


    /*
     * storage-related methods - dealing with fence_storage
     */

    // new_fence_storage_magic - magic number indicating fence storage
    // has been formatted for use by polygon fence storage code.
    // FIXME: ensure this is out-of-band for old lat/lon point storage
    static const uint8_t new_fence_storage_magic = 235;

    uint32_t        _update_ms;   // system time of last update to the boundary in storage

    template<typename T>
    bool calculate_centroid(T *points, uint16_t count, T &centroid) WARN_IF_UNUSED;

    // eos_offset - stores the offset in storage of the end-of-storage
    // marker.  Used by low-level manipulation code to extend storage
    uint16_t eos_offset;

    // formatted - returns true if the fence storage space seems to be
    // formatted for new-style fence storage
    bool formatted() const WARN_IF_UNUSED;
    // format - format the storage space for use by
    // the new polyfence code
    bool format() WARN_IF_UNUSED;


    /*
     * Loaded Fence functionality
     *
     * methods and members to do with fences stored in memory.  The
     * locations are translated into offset-from-origin-in-metres
     */

    // load polygon points stored in eeprom into boundary array and
    // perform validation.  returns true if load successfully
    // completed
    bool load_from_eeprom() WARN_IF_UNUSED;
    void unload();

    // pointer into the boundary point array where the return point
    // can be found:
    Vector2f *_loaded_return_point;

    class InclusionBoundary {
    public:
        Vector2f *points; // pointer into the _loaded_offsets_from_origin array
        uint8_t count; // count of points in the boundary
    };
    InclusionBoundary *_loaded_inclusion_boundary;
    uint8_t _num_loaded_inclusion_boundaries;

    class ExclusionBoundary {
    public:
        Vector2f *points; // pointer into the _loaded_offsets_from_origin array
        uint8_t count; // count of points in the boundary
    };
    ExclusionBoundary *_loaded_exclusion_boundary;
    uint8_t _num_loaded_exclusion_boundaries;

    // _loaded_offsets_from_origin - stores x/y offset-from-origin
    // coordinate pairs.  Various items store their locations in this
    // allocation - the polygon boundaries and the return point, for
    // example.
    Vector2f *_loaded_offsets_from_origin;

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

    // _load_attempted - true if we have attempted to load the fences
    // from storage into _loaded_circle_exclusion_boundary,
    // _loaded_offsets_from_origin etc etc
    bool _load_attempted;

    // _load_time_ms - from millis(), system time when fence load last
    // attempted
    uint32_t _load_time_ms;

    // read_scaled_latlon_from_storage - reads a latitude/longitude
    // from offset in permanent storage, transforms them into an
    // offset-from-origin and deposits the result into dest.
    // read_offset is increased by the storage space used by the
    // latitude/longitude
    bool read_scaled_latlon_from_storage(const Location &origin,
                                         uint16_t &read_offset,
                                         Vector2f &dest) WARN_IF_UNUSED;
    // read_polygon_from_storage - reads vertex_count
    // latitude/longitude points from offset in permanent storage,
    // transformst them into an offset-from-origin and deposits the
    // results into next_storage_point.
    bool read_polygon_from_storage(const Location &origin,
                                   uint16_t &read_offset,
                                   const uint8_t vertex_count,
                                   Vector2f *&next_storage_point) WARN_IF_UNUSED;


    /*
     * Upgrade functions
     */
    // convert_to_new_storage - will attempt to change a pre-existing
    // stored fence to the new storage format (so people don't lose
    // their fences when upgrading)
    bool convert_to_new_storage() WARN_IF_UNUSED;
    // load boundary point from eeprom, returns true on successful load
    bool load_point_from_eeprom(uint16_t i, Vector2l& point) WARN_IF_UNUSED;


    /*
     * FENCE_POINT protocol compatability
     */
    void handle_msg_fetch_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg);
    void handle_msg_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg);
    // contains_compatible_fence - returns true if the permanent fence
    // storage contains fences that are compatible with the old
    // FENCE_POINT protocol.
    bool contains_compatible_fence() const WARN_IF_UNUSED;
    // create_compatible_fence - erases the current fences and
    // installs a fence suitable for storing a fence supplied with the
    // FENCE_POINT protocol
    bool create_compatible_fence() WARN_IF_UNUSED;

    // get_or_create_include_fence - returns a point to an include
    // fence to be used for the FENCE_POINT-supplied polygon.  May
    // format the storage appropriately.
    FenceIndex *get_or_create_include_fence();
    // get_or_create_include_fence - returns a point to a return point
    // to be used for the FENCE_POINT-supplied return point.  May
    // format the storage appropriately.
    FenceIndex *get_or_create_return_point();

    // primitives to write parts of fencepoints out:
    bool write_type_to_storage(uint16_t &offset, AC_PolyFenceType type) WARN_IF_UNUSED;
    bool write_latlon_to_storage(uint16_t &offset, const Vector2l &latlon) WARN_IF_UNUSED;
    bool read_latlon_from_storage(uint16_t &read_offset, Vector2l &latlon) const WARN_IF_UNUSED;

    // methods to write specific types of fencepoint out:
    bool write_returnpoint_to_storage(uint16_t &offset, const Vector2l &loc);
    bool write_eos_to_storage(uint16_t &offset);

    // get_return_point - returns latitude/longitude of return point
    bool get_return_point(Vector2l &ret) WARN_IF_UNUSED;

    // _total - reference to FENCE_TOTAL parameter.  This is used
    // solely for compatability with the FENCE_POINT protocol
    AP_Int8 &_total;


    // scan_eeprom - a method that traverses the fence storage area,
    // calling the supplied callback for each fence found.  If the
    // scan fails (for example, the storage is corrupt), then this
    // method will return false.
    bool scan_eeprom(void (*callback)(AC_PolyFenceType type, uint16_t offset)) WARN_IF_UNUSED;
    // scan_eeprom_count_fences - a static function designed to be
    // massed to scan_eeprom which counts the number of fences and
    // fence items present.  The results of this counting appear in _eeprom_fence_count and _eeprom_item_count
    static void scan_eeprom_count_fences(const AC_PolyFenceType type, uint16_t read_offset);
    static uint16_t _eeprom_fence_count;
    static uint16_t _eeprom_item_count;

    // scan_eeprom_index_fences - a static function designed to be
    // passed to scan_eeprom.  _index must be a pointer to
    // memory sufficient to hold information about all fences present
    // in storage - so it is expected that scan_eeprom_count_fences
    // has been used to count those fences and the allocation already
    // made.  After this method has been called _index will
    // be filled with information about the fences in the fence
    // storage - type, item counts and storage offset.
    static void scan_eeprom_index_fences(const AC_PolyFenceType type, uint16_t read_offset);
    // array specifying type of each fence in storage (and a count of
    // items in that fence)
    static FenceIndex *_index;
    // _num_fences - count of the number of fences in _index.  This
    // should be equal to _eeprom_fence_count
    static uint16_t _num_fences;

    // count_eeprom_fences - refresh the count of fences in permanent storage
    bool count_eeprom_fences() WARN_IF_UNUSED;
    // index_eeprom - (re)allocate and fill in _index
    bool index_eeprom() WARN_IF_UNUSED;
};
