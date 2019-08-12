#include "AC_PolyFence_loader.h"

#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

#define POLYFENCE_LOADER_DEBUGGING 1

#if POLYFENCE_LOADER_DEBUGGING
#define debug(fmt, args ...)  do {::fprintf(stderr,"%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); gcs().send_text(MAV_SEVERITY_WARNING, fmt, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

uint16_t AC_PolyFence_loader::_eeprom_fence_count;
uint16_t AC_PolyFence_loader::_eeprom_item_count;
AC_PolyFence_loader::FenceIndex *AC_PolyFence_loader::_boundary_index;
uint16_t AC_PolyFence_loader::_num_fences;

extern const AP_HAL::HAL& hal;

static const StorageAccess fence_storage(StorageManager::StorageFence);

bool AC_PolyFence_loader::find_index_for_seq(const uint16_t seq, const FenceIndex *&entry, uint16_t &i) const
{
    if (_boundary_index == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Should not have been called");
#endif
        return false;
    }

    if (seq > _eeprom_item_count) {
        return false;
    }

    i = 0;
    for (uint16_t j=0; j<_num_fences; j++) {
        entry = &_boundary_index[j];
        if (seq < i + entry->count) {
            return true;
        }
        i += entry->count;
    }
    return false;
}

bool AC_PolyFence_loader::find_storage_offset_for_seq(const uint16_t seq, uint16_t &offset, AC_PolyFenceType &type, uint16_t &vertex_count_offset) const
{
    if (_boundary_index == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Should not have been called");
#endif
        return false;
    }

    uint16_t i=0;
    const FenceIndex *entry = nullptr;
    if (!find_index_for_seq(seq, entry, i)) {
        return false;
    }

    if (entry == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("entry is nullptr");
#endif
        return false;
    }

    const uint16_t delta = seq - i;

    offset = entry->storage_offset;
    type = entry->type;
    offset++; // type
    switch (type) {
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (delta != 0) {
            abort();
        }
#endif
        break;
    case AC_PolyFenceType::POLYGON_INCLUSION:
    case AC_PolyFenceType::POLYGON_EXCLUSION:
        vertex_count_offset = offset;
        offset += 1; // the count of points in the fence
        offset += (delta * 8);
        break;
    case AC_PolyFenceType::RETURN_POINT:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (delta != 0) {
            abort();
        }
#endif
        break;
    case AC_PolyFenceType::END_OF_STORAGE:
        return false;
    }
    return true;
}

bool AC_PolyFence_loader::get_item(const uint16_t seq, AC_PolyFenceItem &item)
{
    if (!check_indexed()) {
        return false;
    }
    if (_boundary_index == nullptr) {
        // no fences
        return false;
    }

    uint16_t vertex_count_offset = 0; // initialised to make compiler happy
    uint16_t offset;
    AC_PolyFenceType type;
    if (!find_storage_offset_for_seq(seq, offset, type, vertex_count_offset)) {
        return false;
    }

    item.type = type;

    switch (type) {
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        item.radius = fence_storage.read_uint32(offset);
        offset += 4;
        break;
    case AC_PolyFenceType::POLYGON_INCLUSION:
    case AC_PolyFenceType::POLYGON_EXCLUSION:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        item.vertex_count = fence_storage.read_uint8(vertex_count_offset);
        break;
    case AC_PolyFenceType::RETURN_POINT:
        if (!read_latlon_from_storage(offset, item.loc)) {
            return false;
        }
        break;
    case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("read end-of-storage when I should never");
#endif
        return false;
    }
    return true;
}

bool AC_PolyFence_loader::write_type_to_storage(uint16_t &offset, const AC_PolyFenceType type)
{
    fence_storage.write_uint8(offset, (uint8_t)type);
    offset++;
    return true;
}

bool AC_PolyFence_loader::write_latlon_to_storage(uint16_t &offset, const Vector2l &latlon)
{
    fence_storage.write_uint32(offset, latlon[0]);
    offset += 4;
    fence_storage.write_uint32(offset, latlon[1]);
    offset += 4;
    return true;
}

bool AC_PolyFence_loader::read_latlon_from_storage(uint16_t &read_offset, Vector2l &ret) const
{
    ret.x = fence_storage.read_uint32(read_offset);
    read_offset += 4;
    ret.y = fence_storage.read_uint32(read_offset);
    read_offset += 4;
    return true;
}

bool AC_PolyFence_loader::truncate(uint8_t num)
{
    if (!check_indexed()) {
        return false;
    }
    if (_boundary_index == nullptr) {
        return num == 0;
    }

    uint16_t i=0;
    const FenceIndex *entry = nullptr;
    if (!find_index_for_seq(num, entry, i)) {
        return false;
    }
 
    uint16_t offset = entry->storage_offset;
    if (!write_eos_to_storage(offset)) {
        return false;
    }
    _total.set_and_save(num+2); // FIXME
    void_index();
    return true;
}

// load boundary point from eeprom, returns true on successful load
// only used for converting from old storage to new storage
bool AC_PolyFence_loader::load_point_from_eeprom(uint16_t i, Vector2l& point)
{
    // sanity check index
    if (i >= max_items()) {
        return false;
    }

    // read fence point
    point.x = fence_storage.read_uint32(i * sizeof(Vector2l));
    point.y = fence_storage.read_uint32(i * sizeof(Vector2l) + sizeof(uint32_t));
    return true;
}

// returns true if a polygonal include fence could be returned
bool AC_PolyFence_loader::valid()
{
    if (!load_from_eeprom()) {
        return false;
    }
    return _num_loaded_inclusion_boundaries != 0;
}

bool AC_PolyFence_loader::breached()
{
    // check if vehicle is outside the polygon fence
    Vector2f position;
    if (!AP::ahrs().get_relative_position_NE_origin(position)) {
        // we have no idea where we are; can't breach the fence
        return false;
    }

    position = position * 100.0f;  // m to cm
    return breached(position);
}

bool AC_PolyFence_loader::breached(const Location& loc)
{
    Vector2f posNE;
    if (!loc.get_vector_xy_from_origin_NE(posNE)) {
        // not breached if we don't now where we are
        return false;
    }
    return breached(posNE);
}

// if the inclusion boundary is complete, fill the return point with
// the centroid of the inclusion boundary
template<typename T>
bool AC_PolyFence_loader::calculate_centroid(T *points, uint16_t count, T &centroid)
{
    if (points == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        abort();
#endif
        return false;
    }
    if (count < 3) {
        return false;
    }
    float min_lat = points[0].x;
    float min_lon = points[0].y;
    float max_lat = min_lat;
    float max_lon = min_lon;
    for (uint8_t i=1; i<count; i++) {
        const float lat = points[i].x;
        const float lon = points[i].y;
        if (lat < min_lat) {
            min_lat = lat;
        }
        if (lat > max_lat) {
            max_lat = lat;
        }
        if (lon < min_lon) {
            min_lon = lon;
        }
        if (_boundary[i][1] > max_lon) {
            max_lon = lon;
        }
    }
    centroid = {
        ((min_lat+max_lat)/2),
        ((min_lon+max_lon)/2)
    };
    return true;
}


// check if a location (expressed as either floats or long ints) is within the boundary
//   returns true if location is outside the boundary
bool AC_PolyFence_loader::breached(const Vector2f& location)
{
    if (!load_from_eeprom()) { // FIXME: don't do this here
        return false;
    }

    // check we are inside each inclusion zone:
    for (uint8_t i=0; i<_num_loaded_inclusion_boundaries; i++) {
        const InclusionBoundary &boundary = _loaded_inclusion_boundary[i];
        if (Polygon_outside(location, boundary.points, boundary.count)) {
            return true;
        }
    }

    // check we are outside each exclusion zone:
    for (uint8_t i=0; i<_num_loaded_exclusion_boundaries; i++) {
        const ExclusionBoundary &boundary = _loaded_exclusion_boundary[i];
        if (!Polygon_outside(location, boundary.points, boundary.count)) {
            return true;
        }
    }

    // check circular excludes
    for (uint8_t i=0; i<_num_loaded_circle_exclusion_boundaries; i++) {
        const ExclusionCircle &circle = _loaded_circle_exclusion_boundary[i];
        const Vector2f diff_cm = location - circle.loc;
        const float diff_cm_squared = diff_cm.length_squared();
        if (diff_cm_squared < sq(circle.radius*100.0f)) {
            return true;
        }
    }

    // check circular includes
    for (uint8_t i=0; i<_num_loaded_circle_inclusion_boundaries; i++) {
        const InclusionCircle &circle = _loaded_circle_inclusion_boundary[i];
        const Vector2f diff_cm = location - circle.loc;
        const float diff_cm_squared = diff_cm.length_squared();
        if (diff_cm_squared > sq(circle.radius*100.0f)) {
            return true;
        }
    }

    // no fence breached
    return false;
}

bool AC_PolyFence_loader::formatted() const
{
    return (fence_storage.read_uint8(0) == new_fence_storage_magic);
}

uint16_t AC_PolyFence_loader::max_items() const
{
    // this is 84 items on PixHawk
    return MIN(255U, fence_storage.size() / sizeof(Vector2l));
}

bool AC_PolyFence_loader::format()
{
    uint16_t offset = 0;
    fence_storage.write_uint32(offset, 0);
    fence_storage.write_uint8(offset, new_fence_storage_magic);
    offset += 4;
    void_index();
    return write_eos_to_storage(offset);
}

bool AC_PolyFence_loader::convert_to_new_storage()
{
    // sanity check total
    _total = constrain_int16(_total, 0, max_items());
    // FIXME: ensure the fence was closed and don't load it if it was not
    if (_total < 5) {
        // fence was invalid.  Just format it and move on
        return format();
    }

    const uint32_t array_size = _total * sizeof(Vector2l);
    if (hal.util->available_memory() < 100U + array_size) {
        return false;
    }

    Vector2l *_tmp_boundary = (Vector2l *)malloc(array_size);
    if (_tmp_boundary == nullptr) {
        return false;
    }

    // load each point from eeprom
    bool ret = false;
    for (uint16_t index=0; index<_total; index++) {
        // load boundary point as lat/lon point
        if (!load_point_from_eeprom(index, _tmp_boundary[index])) {
            goto out;
        }
    }

    // now store:
    if (!format()) {
        goto out;
    }
    {
        uint16_t offset = 4; // skip magic
        if (!write_returnpoint_to_storage(offset, _tmp_boundary[0])) {
            goto out;
        }
        fence_storage.write_uint8(offset, (uint8_t)AC_PolyFenceType::POLYGON_INCLUSION);
        offset++;
        fence_storage.write_uint8(offset, (uint8_t)_total-2);
        offset++;
        for (uint8_t i=1; i<_total-1; i++) {
            if (!write_latlon_to_storage(offset, _tmp_boundary[i])) {
                goto out;
            }
        }

        if (!write_eos_to_storage(offset)) {
            goto out;
        }
    }

    ret = true;

out:
    free(_tmp_boundary);
    return ret;
}

bool AC_PolyFence_loader::read_scaled_latlon_from_storage(const Location &origin, uint16_t &read_offset, Vector2f &dest)
{
    Location tmp_loc;
    tmp_loc.lat = fence_storage.read_uint32(read_offset);
    read_offset += 4;
    tmp_loc.lng = fence_storage.read_uint32(read_offset);
    read_offset += 4;
    dest = origin.get_distance_NE(tmp_loc) * 100.0f;
    return true;
}

bool AC_PolyFence_loader::read_polygon_from_storage(const Location &origin, uint16_t &read_offset, const uint8_t vertex_count, Vector2f *&next_storage_point)
{
    for (uint8_t i=0; i<vertex_count; i++) {
        // read and convert to lat/lon
        if (!read_scaled_latlon_from_storage(origin, read_offset, *next_storage_point)) {
            return false;
        }
        next_storage_point++;
    }
    return true;
}

bool AC_PolyFence_loader::scan_eeprom(void (*callback)(AC_PolyFenceType type, uint16_t offset))
{
    uint16_t read_offset = 0; // skipping reserved first 4 bytes
    const uint32_t magic = fence_storage.read_uint32(read_offset);
    if ((magic & 0xff) != new_fence_storage_magic) {
        gcs().send_text(MAV_SEVERITY_WARNING, "fence magic mismatch (%u) (%u)", magic, new_fence_storage_magic);
        return false;
    }
    read_offset+=4;
    bool all_done = false;
    while (!all_done) {
        if (read_offset > fence_storage.size()) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("did not find end-of-storage-marker before running out of space");
#endif
            return false;
        }
        const AC_PolyFenceType type = (AC_PolyFenceType)fence_storage.read_uint8(read_offset);
        // validate what we've just pulled back from storage:
        switch (type) {
        case AC_PolyFenceType::END_OF_STORAGE:
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
        case AC_PolyFenceType::RETURN_POINT:
            break;
        default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Fence corrupt (offset=%u)", read_offset);
#endif
            gcs().send_text(MAV_SEVERITY_WARNING, "Fence corrupt");
            return false;
        }

        callback(type, read_offset);
        read_offset++;
        switch (type) {
        case AC_PolyFenceType::END_OF_STORAGE:
            eos_offset = read_offset-1;
            all_done = true;
            break;
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION: {
            const uint8_t vertex_count = fence_storage.read_uint8(read_offset);
            read_offset += 1; // for the count we just read
            read_offset += vertex_count*8;
            break;
        }
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION: {
            read_offset += 8; // for latlon
            read_offset += 4; // for radius
            break;
        }
        case AC_PolyFenceType::RETURN_POINT:
            read_offset += 8; // for latlon
            break;
        }
    }
    return true;
}

// note read_offset here isn't const and ALSO is not a reference
void AC_PolyFence_loader::scan_eeprom_count_fences(const AC_PolyFenceType type, uint16_t read_offset)
{
    if (type == AC_PolyFenceType::END_OF_STORAGE) {
        return;
    }
    _eeprom_fence_count++;
    switch (type) {
    case AC_PolyFenceType::END_OF_STORAGE:
        break;
    case AC_PolyFenceType::POLYGON_EXCLUSION:
    case AC_PolyFenceType::POLYGON_INCLUSION: {
        const uint8_t vertex_count = fence_storage.read_uint8(read_offset+1); // skip type
        _eeprom_item_count += vertex_count;
        break;
    }
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
    case AC_PolyFenceType::RETURN_POINT:
        _eeprom_item_count++;
        break;
    }
}

bool AC_PolyFence_loader::count_eeprom_fences()
{
    _eeprom_fence_count = 0;
    _eeprom_item_count = 0;
    const bool ret = scan_eeprom(scan_eeprom_count_fences);
    return ret;
}

void AC_PolyFence_loader::scan_eeprom_index_fences(const AC_PolyFenceType type, uint16_t read_offset)
{
    if (type == AC_PolyFenceType::END_OF_STORAGE) {
        return;
    }
    FenceIndex &index = _boundary_index[_num_fences++];
    index.type = type;
    index.storage_offset = read_offset;
    switch (type) {
    case AC_PolyFenceType::END_OF_STORAGE:
        break;
    case AC_PolyFenceType::POLYGON_EXCLUSION:
    case AC_PolyFenceType::POLYGON_INCLUSION: {
        const uint8_t vertex_count = fence_storage.read_uint8(read_offset+1);
        index.count = vertex_count;
        break;
    }
    case AC_PolyFenceType::CIRCLE_INCLUSION:
    case AC_PolyFenceType::CIRCLE_EXCLUSION:
        index.count = 1;
        break;
    case AC_PolyFenceType::RETURN_POINT:
        index.count = 1;
        break;
    }
}

bool AC_PolyFence_loader::index_eeprom()
{
    if (!formatted()) {
        if (!convert_to_new_storage()) {
            return false;
        }
    }

    if (!count_eeprom_fences()) {
        return false;
    }
    if (_eeprom_fence_count == 0) {
        _load_attempted = false;
        return true;
    }

    free(_boundary_index);
    _boundary_index = nullptr;
    if (_eeprom_fence_count) {
        const uint32_t index_allocation_size = _eeprom_fence_count*sizeof(FenceIndex);
        gcs().send_text(MAV_SEVERITY_INFO, "Fence: Allocating %u bytes for index", index_allocation_size);
        _boundary_index = (FenceIndex*)malloc(index_allocation_size);
        if (_boundary_index == nullptr) {
            return false;
        }
    }

    _num_fences = 0;
    if (!scan_eeprom(scan_eeprom_index_fences)) {
        free(_boundary_index);
        _boundary_index = nullptr;
        return false;
    }

    if (_num_fences != _eeprom_fence_count) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("indexed fences not equal to eeprom fences");
#endif
    }

    _load_attempted = false;

    return true;
}

bool AC_PolyFence_loader::check_indexed()
{
   if (_boundary_index != nullptr) {
       return true;
   }

   return index_eeprom();
}

void AC_PolyFence_loader::unload()
{
    free(_boundary);
    _boundary = nullptr;

    free(_loaded_inclusion_boundary);
    _loaded_inclusion_boundary = nullptr;
    _num_loaded_inclusion_boundaries = 0;

    free(_loaded_exclusion_boundary);
    _loaded_exclusion_boundary = nullptr;
    _num_loaded_exclusion_boundaries = 0;

    free(_loaded_circle_inclusion_boundary);
    _loaded_circle_inclusion_boundary = nullptr;
    _num_loaded_circle_inclusion_boundaries = 0;

    free(_loaded_circle_exclusion_boundary);
    _loaded_circle_exclusion_boundary = nullptr;
    _num_loaded_circle_exclusion_boundaries = 0;

    _loaded_return_point = nullptr;
    // FIXME: other loaded artificacts?
}

// return the number of fences of type type in the index:
uint16_t AC_PolyFence_loader::index_fence_count(const AC_PolyFenceType type)
{
    uint16_t ret = 0;
    for (uint8_t i=0; i<_eeprom_fence_count; i++) {
        const FenceIndex &index = _boundary_index[i];
        if (index.type == type) {
            ret++;
        }
    }
    return ret;
}

bool AC_PolyFence_loader::load_from_eeprom()
{
    if (!check_indexed()) {
        return false;
    }

    if (_load_attempted) {
        return _boundary != nullptr;
    }
    _load_time_ms = 0;

    struct Location ekf_origin{};
    if (!AP::ahrs().get_origin(ekf_origin)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "fence load requires origin");
        return false;
    }

    _load_attempted = true;

    // find indexes of each fence:
    uint16_t read_offset = 0; // skipping reserved first 4 bytes
    const uint32_t magic = fence_storage.read_uint32(read_offset);
    if ((magic & 0xff) != new_fence_storage_magic) {
        gcs().send_text(MAV_SEVERITY_WARNING, "fence magic mismatch (%u) (%u)", magic, new_fence_storage_magic);
        return false;
    }

    unload();

    if (_eeprom_item_count == 0) {
        _load_time_ms = AP_HAL::millis();
        return true;
    }

    { // allocate general boundary point array:
        // FIXME: should not be _eeprom_item_count, should be the sum of the number of points in the return point, inclusion fences and exclusion fences
        const uint32_t allocation_size = _eeprom_item_count * sizeof(Vector2f);
        _boundary = (Vector2f*)malloc(allocation_size);
        gcs().send_text(MAV_SEVERITY_WARNING, "Fence: Allocating %u bytes for points", allocation_size);
        if (_boundary == nullptr) {
            unload();
            return false;
        }
    }

    // FIXME: find some way of factoring out all of these allocation routines.

    { // allocate storage for inclusion polyfences:
        const uint32_t allocation_size = index_fence_count(AC_PolyFenceType::POLYGON_INCLUSION) * sizeof(InclusionBoundary);
        gcs().send_text(MAV_SEVERITY_WARNING, "Fence: Allocating %u bytes for inc. fences", allocation_size);
        _loaded_inclusion_boundary = (InclusionBoundary*)malloc(allocation_size);
        if (_loaded_inclusion_boundary == nullptr) {
            unload();
            return false;
        }
    }

    { // allocate storage for exclusion polyfences:
        const uint32_t allocation_size = index_fence_count(AC_PolyFenceType::POLYGON_EXCLUSION) * sizeof(ExclusionBoundary);
        gcs().send_text(MAV_SEVERITY_WARNING, "Fence: Allocating %u bytes for exc. fences", allocation_size);
        _loaded_exclusion_boundary = (ExclusionBoundary*)malloc(allocation_size);
        if (_loaded_exclusion_boundary == nullptr) {
            unload();
            return false;
        }
    }

    { // allocate storage for circular inclusion fences:
        const uint32_t allocation_size = index_fence_count(AC_PolyFenceType::CIRCLE_INCLUSION) * sizeof(InclusionCircle);
        gcs().send_text(MAV_SEVERITY_WARNING, "Fence: Allocating %u bytes for circ. inc. fences", allocation_size);
        _loaded_circle_inclusion_boundary = (InclusionCircle*)malloc(allocation_size);
        if (_loaded_circle_inclusion_boundary == nullptr) {
            unload();
            return false;
        }
    }

    { // allocate storage for circular exclusion fences:
        const uint32_t allocation_size = index_fence_count(AC_PolyFenceType::CIRCLE_EXCLUSION) * sizeof(ExclusionCircle);
        gcs().send_text(MAV_SEVERITY_WARNING, "Fence: Allocating %u bytes for circ. exc. fences", allocation_size);
        _loaded_circle_exclusion_boundary = (ExclusionCircle*)malloc(allocation_size);
        if (_loaded_circle_exclusion_boundary == nullptr) {
            unload();
            return false;
        }
    }

    Vector2f *next_storage_point = _boundary;

    // use index to load fences from eeprom
    bool storage_valid = true;
    for (uint8_t i=0; i<_eeprom_fence_count; i++) {
        if (!storage_valid) {
            break;
        }
        const FenceIndex &index = _boundary_index[i];
        uint16_t storage_offset = index.storage_offset;
        storage_offset += 1; // skip type
        switch (index.type) {
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("indexed end of storage found");
#endif
            storage_valid = false;
            break;
        case AC_PolyFenceType::POLYGON_INCLUSION: {
            // FIXME: consider factoring this with the EXCLUSION case
            InclusionBoundary &boundary = _loaded_inclusion_boundary[_num_loaded_inclusion_boundaries];
            boundary.points = next_storage_point;
            boundary.count = index.count;
            storage_offset += 1; // skip vertex count
            if (!read_polygon_from_storage(ekf_origin, storage_offset, index.count, next_storage_point)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "AC_Fence: polygon read failed");
                storage_valid = false;
                break;
            }
            _num_loaded_inclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::POLYGON_EXCLUSION: {
            ExclusionBoundary &boundary = _loaded_exclusion_boundary[_num_loaded_exclusion_boundaries];
            boundary.points = next_storage_point;
            boundary.count = index.count;
            storage_offset += 1; // skip vertex count
            if (!read_polygon_from_storage(ekf_origin, storage_offset, index.count, next_storage_point)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "AC_Fence: polygon read failed");
                storage_valid = false;
                break;
            }
            _num_loaded_exclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::CIRCLE_EXCLUSION: {
            ExclusionCircle &circle = _loaded_circle_exclusion_boundary[_num_loaded_circle_exclusion_boundaries];
            if (!read_scaled_latlon_from_storage(ekf_origin, storage_offset, circle.loc)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            // now read the radius
            circle.radius = fence_storage.read_uint32(storage_offset);
            _num_loaded_circle_exclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::CIRCLE_INCLUSION: {
            InclusionCircle &circle = _loaded_circle_inclusion_boundary[_num_loaded_circle_inclusion_boundaries];
            if (!read_scaled_latlon_from_storage(ekf_origin, storage_offset, circle.loc)) {
                gcs().send_text(MAV_SEVERITY_WARNING, "AC_Fence: latlon read failed");
                storage_valid = false;
                break;
            }
            // now read the radius
            circle.radius = fence_storage.read_uint32(storage_offset);
            _num_loaded_circle_inclusion_boundaries++;
            break;
        }
        case AC_PolyFenceType::RETURN_POINT:
            if (_loaded_return_point != nullptr) {
                gcs().send_text(MAV_SEVERITY_WARNING, "PolyFence: Multiple return points found");
                storage_valid = false;
                break;
            }
            _loaded_return_point = next_storage_point;
            if (!read_scaled_latlon_from_storage(ekf_origin, storage_offset, *next_storage_point)) {
                storage_valid = false;
                gcs().send_text(MAV_SEVERITY_WARNING, "PolyFence: latlon read failed");
                break;
            }
            next_storage_point++;
            break;
        }
    }

    if (!storage_valid) {
        unload();
        return false;
    }

    _load_time_ms = AP_HAL::millis();

    return true;
}

Vector2f* AC_PolyFence_loader::get_exclusion_polygon(uint16_t index, uint16_t &num_points) const
{
    if (index > _num_loaded_exclusion_boundaries) {
        return nullptr;
    }
    const ExclusionBoundary &boundary = _loaded_exclusion_boundary[index];
    num_points = boundary.count;

    return boundary.points;
}

bool AC_PolyFence_loader::validate_fence(const AC_PolyFenceItem *new_items, uint16_t count) const
{
    // validate the fence items...
    AC_PolyFenceType expecting_type = AC_PolyFenceType::END_OF_STORAGE;
    uint16_t expected_type_count = 0;
    uint16_t orig_expected_type_count = 0;
    bool seen_return_point = false;
    for (uint16_t i=0; i<count; i++) {
        switch (new_items[i].type) {
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            if (new_items[i].vertex_count == 0) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Zero-vertex-count-polygon");
                return false;
            }
            if (expected_type_count == 0) {
                expected_type_count = new_items[i].vertex_count;
                orig_expected_type_count = expected_type_count;
                expecting_type = new_items[i].type;
            } else if (new_items[i].vertex_count != orig_expected_type_count) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Unexpected vertex count want=%u got=%u\n", orig_expected_type_count, new_items[i].vertex_count);
            }
            break;
        default:
            break;
        }
        bool validate_latlon = false;
        if (expected_type_count) {
            if (new_items[i].type != expecting_type) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Received incorrect vertex type (want=%u got=%u)", (unsigned)expecting_type, (unsigned)new_items[i].type);
                return false;
            }
            expected_type_count--;
        }
        switch (new_items[i].type) {
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("passed in an END_OF_STORAGE");
#endif
            return false;
        case AC_PolyFenceType::POLYGON_INCLUSION:
            validate_latlon = true;
            break;
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            validate_latlon = true;
            break;
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            if (new_items[i].radius <= 0) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Non-positive circle radius");
                return false;
            }
            validate_latlon = true;
            break;
        case AC_PolyFenceType::RETURN_POINT:
            // spec says only one return point allowed
            if (seen_return_point) {
                return false;
            }
            seen_return_point = true;
            validate_latlon = true;
            // TODO: ensure return point is within all fences and
            // outside all exclusion zones
            break;
        }
        if (validate_latlon) {
            if (!check_latlng(new_items[i].loc[0], new_items[i].loc[1])) {
                gcs().send_text(MAV_SEVERITY_WARNING, "Bad lat or lon");
                return false;
            }
        }
    }

    if (expected_type_count) {
        gcs().send_text(MAV_SEVERITY_INFO, "Incorrect item count");
        return false;
    }

    return true;
}

bool AC_PolyFence_loader::write_fence(const AC_PolyFenceItem *new_items, uint16_t count)
{
    if (!validate_fence(new_items, count)) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Fence validation failed");
        return false;
    }

    if (!format()) {
        return false;
    }

    uint8_t total_vertex_count = 0;
    uint16_t offset = 4; // skipping magic
    uint8_t vertex_count = 0;
    for (uint16_t i=0; i<count; i++) {
        const AC_PolyFenceItem new_item = new_items[i];
        switch (new_item.type) {
        case AC_PolyFenceType::POLYGON_INCLUSION:
        case AC_PolyFenceType::POLYGON_EXCLUSION:
            if (vertex_count == 0) {
                // write out new polygon count
                vertex_count = new_item.vertex_count;
                total_vertex_count += vertex_count;
                if (!write_type_to_storage(offset, new_item.type)) {
                    return false;
                }
                fence_storage.write_uint8(offset, vertex_count);
                offset++;
            }
            vertex_count--;
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            break;
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("asked to store end-of-storage marker");
#endif
            return false;
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            if (!write_type_to_storage(offset, new_item.type)) {
                return false;
            }
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            // store the radius
            fence_storage.write_uint32(offset, new_item.radius);
            offset += 4;
            break;
        case AC_PolyFenceType::RETURN_POINT:
            if (!write_type_to_storage(offset, new_item.type)) {
                return false;
            }
            if (!write_latlon_to_storage(offset, new_item.loc)) {
                return false;
            }
            break;
        }
    }
    if (!write_eos_to_storage(offset)) {
        return false;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (!index_eeprom()) {
        AP_HAL::panic("Failed to index eeprom");
    }
    gcs().send_text(MAV_SEVERITY_DEBUG, "Fence Indexed OK");
#endif

    void_index();

    // this may be completely bogus total.  If we are storing an
    // advanced fence then the old protocol which relies on this value
    // will error off if the GCS tries to fetch points.  This number
    // should be correct for a "compatible" fence, however.
    uint16_t new_total = 0;
    if (total_vertex_count == 0) {
        new_total = 0;
    } else if (total_vertex_count < 3) {
        // we fake things up in FENCE_POINT fetch and send
        new_total = 5;
    } else {
        new_total = total_vertex_count+2;
    }
    _total.set_and_save(new_total);

    return true;
}


bool AC_PolyFence_loader::contains_compatible_fence() const
{
    // must contain a single inclusion fence with an optional return point
    if (_boundary_index == nullptr) {
        // this indicates no boundary points present
        return true;
    }
    bool seen_return_point = false;
    bool seen_poly_inclusion = false;
    for (uint16_t i=0; i<_num_fences; i++) {
        switch (_boundary_index[i].type) {
        case AC_PolyFenceType::END_OF_STORAGE:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("end-of-storage marker found in loaded list");
#endif
            return false;
        case AC_PolyFenceType::POLYGON_INCLUSION:
            if (seen_poly_inclusion) {
                return false;
            }
            seen_poly_inclusion = true;
            break;
        case AC_PolyFenceType::POLYGON_EXCLUSION:
        case AC_PolyFenceType::CIRCLE_INCLUSION:
        case AC_PolyFenceType::CIRCLE_EXCLUSION:
            return false;
        case AC_PolyFenceType::RETURN_POINT:
            if (seen_return_point) {
                return false;
            }
            seen_return_point = true;
            break;
        }
    }
    return true;
}


bool AC_PolyFence_loader::get_return_point(Vector2l &ret)
{
    if (!check_indexed()) {
        return false;
    }

    const FenceIndex *rp = find_first_fence(AC_PolyFenceType::RETURN_POINT);
    if (rp != nullptr) {
        uint16_t read_offset = rp->storage_offset + 1;
        return read_latlon_from_storage(read_offset, ret);
    }

    const FenceIndex *inc = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
    if (inc == nullptr) {
        return false;
    }

    // we found an inclusion fence but not a return point.  Calculate
    // and return the centroid.  Note that this may not actually be
    // inside all inclusion fences...
    uint16_t offset = inc->storage_offset;
    if ((AC_PolyFenceType)fence_storage.read_uint8(offset) != AC_PolyFenceType::POLYGON_INCLUSION) {
        AP_HAL::panic("wrong type at offset");
        return false;
    }
    offset++;
    const uint8_t count = fence_storage.read_uint8(offset);
    if (count < 3) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("invalid count found");
#endif
        return false;
    }
    offset++;
    Vector2l min_loc;
    if (!read_latlon_from_storage(offset, min_loc)) {
        return false;
    }
    if (min_loc.is_zero()) {
        return false;
    }
    Vector2l max_loc = min_loc;
    for (uint8_t i=1; i<count; i++) {
        Vector2l new_loc;
        if (!read_latlon_from_storage(offset, new_loc)) {
            return false;
        }
        if (new_loc.is_zero()) {
            return false;
        }
        if (new_loc.x < min_loc.x) {
            min_loc.x = new_loc.x;
        }
        if (new_loc.y < min_loc.y) {
            min_loc.y = new_loc.y;
        }
    }

    ret.x = ((min_loc.x+max_loc.x)/2);
    ret.y = ((min_loc.y+max_loc.y)/2);

    return true;
}

AC_PolyFence_loader::FenceIndex *AC_PolyFence_loader::find_first_fence(const AC_PolyFenceType type) const
{
    if (_boundary_index == nullptr) {
        return nullptr;
    }
    for (uint8_t i=0; i<_num_fences; i++) {
        if (_boundary_index[i].type == type) {
            return &_boundary_index[i];
        }
    }
    return nullptr;
}

void AC_PolyFence_loader::handle_msg_fetch_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg)
{
    if (!check_indexed()) {
        return;
    }
    if (!contains_compatible_fence()) {
        link.send_text(MAV_SEVERITY_WARNING, "Vehicle contains advanced fences");
        return;
    }

    if (_total != 0 && _total < 5) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid FENCE_TOTAL");
        return;
    }

    mavlink_fence_fetch_point_t packet;
    mavlink_msg_fence_fetch_point_decode(&msg, &packet);

    // FIXME: when writing the new-style mission ensure we update _total!
    if (packet.idx >= _total) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, index past total(%u >= %u)", packet.idx, _total.get());
        return;
    }

    mavlink_fence_point_t ret_packet{};
    ret_packet.target_system = msg.sysid;
    ret_packet.target_component = msg.compid;
    ret_packet.idx = packet.idx;
    ret_packet.count = _total;

    if (packet.idx == 0) {
        // return point
        Vector2l ret;
        if (get_return_point(ret)) {
            ret_packet.lat = ret.x * 1.0e-7f;
            ret_packet.lng = ret.y * 1.0e-7f;
        } else {
            link.send_text(MAV_SEVERITY_WARNING, "Failed to get return point");
        }
    } else {
        // find the inclusion fence:
        const FenceIndex *inclusion_fence = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
        if (inclusion_fence == nullptr) {
            // nothing stored yet; just send back zeroes
            ret_packet.lat = 0;
            ret_packet.lng = 0;
        } else {
            uint8_t fencepoint_offset; // 1st idx is return point
            if (packet.idx == _total-1) {
                // the is the loop closure point - send the first point again
                fencepoint_offset = 0;
            } else {
                fencepoint_offset = packet.idx - 1;
            }
            if (fencepoint_offset >= inclusion_fence->count) {
                // we haven't been given a value for this item yet; we will return zeroes
            } else {
                uint16_t storage_offset = inclusion_fence->storage_offset;
                storage_offset++; // skip over type
                storage_offset++; // skip over count
                storage_offset += 8*fencepoint_offset; // move to point we're interested in
                Vector2l bob;
                if (!read_latlon_from_storage(storage_offset, bob)) {
                    link.send_text(MAV_SEVERITY_WARNING, "Fence read failed");
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
                    AP_HAL::panic("read failure");
#endif
                    return;
                }
                ret_packet.lat = bob[0] * 1.0e-7f;
                ret_packet.lng = bob[1] * 1.0e-7f;
            }
        }
    }

    link.send_message(MAVLINK_MSG_ID_FENCE_POINT, (const char*)&ret_packet);
}

bool AC_PolyFence_loader::write_returnpoint_to_storage(uint16_t &offset, const Vector2l &loc)
{
    if (!write_type_to_storage(offset, AC_PolyFenceType::RETURN_POINT)) {
        return false;
    }
    if (!write_latlon_to_storage(offset, loc)) {
        return false;
    }
    return true;
}

bool AC_PolyFence_loader::write_eos_to_storage(uint16_t &offset)
{
    if (!write_type_to_storage(offset, AC_PolyFenceType::END_OF_STORAGE)) {
        return false;
    }
    eos_offset = offset - 1; // should point to the marker
    return true;
}

AC_PolyFence_loader::FenceIndex *AC_PolyFence_loader::get_or_create_return_point()
{
    if (!check_indexed()) {
        return nullptr;
    }
    FenceIndex *return_point = find_first_fence(AC_PolyFenceType::RETURN_POINT);
    if (return_point != nullptr) {
        return return_point;
    }

    // if the inclusion fence exists we will move it forward in
    // storage to avoid having to continually shift the return point
    // forward as we receive fence points
    uint16_t offset;
    const FenceIndex *inclusion_fence = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
    if (inclusion_fence != nullptr) {
        offset = inclusion_fence->storage_offset;
        // the "9"s below represent the size of a return point in storage
        for (uint8_t i=0; i<inclusion_fence->count; i++) {
            const uint16_t point_storage_offset = offset + 2 + (inclusion_fence->count-1-i) * 8;
            Vector2l latlon;
            uint16_t tmp_read_offs = point_storage_offset;
            if (!read_latlon_from_storage(tmp_read_offs, latlon)) {
                return nullptr;
            }
            uint16_t write_offset = point_storage_offset + 9;
            if (!write_latlon_to_storage(write_offset, latlon)) {
                return nullptr;
            }
        }
        // read/write the count:
        const uint8_t count = fence_storage.read_uint8(inclusion_fence->storage_offset+1);
        fence_storage.write_uint8(inclusion_fence->storage_offset + 1 + 9, count);
        // read/write the type:
        const uint8_t t = fence_storage.read_uint8(inclusion_fence->storage_offset);
        fence_storage.write_uint8(inclusion_fence->storage_offset + 9, t);

        uint16_t write_offset = offset + 2 + 8*inclusion_fence->count + 9;
        if (!write_eos_to_storage(write_offset)) {
            return nullptr;
        }
    } else {
        if (fence_storage.read_uint8(eos_offset) != (uint8_t)AC_PolyFenceType::END_OF_STORAGE) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Expected end-of-storage marker at offset=%u", offset);
#endif
            return nullptr;
        }
        offset = eos_offset;
    }

    if (!write_type_to_storage(offset, AC_PolyFenceType::RETURN_POINT)) {
        return nullptr;
    }
    if (!write_latlon_to_storage(offset, Vector2l{0, 0})) {
        return nullptr;
    }
    if (inclusion_fence == nullptr) {
        if (!write_eos_to_storage(offset)) {
            return nullptr;
        }
    }

    if (!index_eeprom()) { // FIXME: should only void index if != SITL
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Failed to index eeprom after moving inclusion fence for return point");
#endif
        return nullptr;
    }

    return_point = find_first_fence(AC_PolyFenceType::RETURN_POINT);
    if (return_point == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Failed to get return point after indexing");
#endif
    }
    return return_point;
}

AC_PolyFence_loader::FenceIndex *AC_PolyFence_loader::get_or_create_include_fence()
{
    if (!check_indexed()) {
        return nullptr;
    }
    FenceIndex *inclusion = find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
    if (inclusion != nullptr) {
        return inclusion;
    }
    if (_total < 5) {
        return nullptr;
    }
    if (!write_type_to_storage(eos_offset, AC_PolyFenceType::POLYGON_INCLUSION)) {
        return nullptr;
    }
    fence_storage.write_uint8(eos_offset, 0);
    eos_offset++;
    if (!write_eos_to_storage(eos_offset)) {
        return nullptr;
    }

    if (!index_eeprom()) {
        return nullptr;
    }
    return find_first_fence(AC_PolyFenceType::POLYGON_INCLUSION);
}

void AC_PolyFence_loader::handle_msg_fence_point(GCS_MAVLINK &link, const mavlink_message_t& msg)
{
    if (!check_indexed()) {
        return;
    }

    mavlink_fence_point_t packet;
    mavlink_msg_fence_point_decode(&msg, &packet);

    if (_total != 0 && _total < 5) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid FENCE_TOTAL");
        return;
    }

    if (packet.count != _total) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, bad count (%u vs %u)", packet.count, _total.get());
        return;
    }

    if (packet.idx >= _total) {
        // this is a protocol failure
        link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, index past total (%u >= %u)", packet.idx, _total.get());
        return;
    }

    if (!check_latlng(packet.lat, packet.lng)) {
        link.send_text(MAV_SEVERITY_WARNING, "Invalid fence point, bad lat or lng");
        return;
    }

    if (!contains_compatible_fence()) {
        // the GCS has started to upload using the old protocol;
        // ensure we can accept it.  We must be able to index the
        // fence, so it must be valid (minimum number of points)
        if (!format()) {
            return;
        }
    }

    const Vector2l point{
        (int32_t)(packet.lat*1.0e7f),
        (int32_t)(packet.lng*1.0e7f)
    };

    if (packet.idx == 0) {
        // this is the return point; if we have a return point then
        // update it, otherwise create a return point fence thingy
        const FenceIndex *return_point = get_or_create_return_point();
        if (return_point == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("Didn't get return point");
#endif
            return;
        }
        uint16_t offset = return_point->storage_offset;
        offset++; // don't overwrite the type!
        if (!write_latlon_to_storage(offset, point)) {
            link.send_text(MAV_SEVERITY_WARNING, "PolyFence: storage write failed");
            return;
        }
    } else if (packet.idx == _total-1) {
        // this is the fence closing point; don't store it, and don't
        // check it against the first point in the fence as we may be
        // receiving the fence points out of order.  Note that ifthe
        // GCS attempts to read this back before sendng the first
        // point they will get 0s.
    } else {
        const FenceIndex *inclusion_fence = get_or_create_include_fence();
        if (inclusion_fence == nullptr) {
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("no inclusion fences found");
#endif
            return;
        }
        uint16_t offset = inclusion_fence->storage_offset;
        offset++; // skip type
        if (packet.idx > inclusion_fence->count) {
            // expand the storage space
            fence_storage.write_uint8(offset, packet.idx); // remembering that idx[0] is the return point....
        }
        offset++; // move past number of points
        offset += (packet.idx-1)*8;
        if (!write_latlon_to_storage(offset, point)) {
            link.send_text(MAV_SEVERITY_WARNING, "PolyFence: storage write failed");
            return;
        }
        if (eos_offset < offset) {
            if (!write_eos_to_storage(offset)) {
                return;
            }
        }
        void_index();
    }
}

/// handler for polygon fence messages with GCS
void AC_PolyFence_loader::handle_msg(GCS_MAVLINK &link, const mavlink_message_t& msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_FENCE_POINT:
        handle_msg_fence_point(link, msg);
        break;
    case MAVLINK_MSG_ID_FENCE_FETCH_POINT:
        handle_msg_fetch_fence_point(link, msg);
        break;
    default:
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            AP_HAL::panic("should not have been called");
#endif
        break;
    }
}
