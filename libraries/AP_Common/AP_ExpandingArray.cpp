/*
 * AP_ExpandingArray.cpp
 * Copyright (C) Randy Mackay 2019
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_ExpandingArray.h"
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

/*
template <typename T>
AP_ExpandingArray<T>::AP_ExpandingArray<T>() :
    chunk_size(50)
{
    // create one chunk
    expand(1);
}
*/

/*
template <typename T>
AP_ExpandingArray<T>::AP_ExpandingArray<T>(uint16_t initial_size, uint16_t elements_per_chunk) :
    chunk_size(elements_per_chunk)
{
    // expand array to at least the requested size
    uint16_t chunk_num = initial_size % chunk_size;
    const uint16_t chunk_index = initial_size - chunk_num;
    if (chunk_index > 0) {
        chunk_num++;
    }
    expand(chunk_num);
}
*/


// allow use as an array. no bounds checking is performed
template <typename T>
T &AP_ExpandingArray<T>::operator[](uint16_t i)
{
    const uint16_t chunk_num = i % chunk_size;
    const uint16_t chunk_index = i - chunk_num;
    return chunk_ptrs[chunk_num][chunk_index];
}

// allow use as an array. no bounds checking is performed
template <typename T>
const T &AP_ExpandingArray<T>::operator[](uint16_t i) const
{
    const uint16_t chunk_num = i % chunk_size;
    const uint16_t chunk_index = i - chunk_num;
    return chunk_ptrs[chunk_num][chunk_index];
}

// expand the array by specified number of chunk, returns true on success
template <typename T>
bool AP_ExpandingArray<T>::expand(uint16_t num_chunks)
{
    // initial memory check
    uint32_t required_bytes = num_chunks * sizeof(T);
    if (hal.util->available_memory() < 100U + required_bytes) {
        return false;
    }

    // expand chunk_ptrs array if necessary
    if (chunk_count + num_chunks >= chunk_count_max) {
        uint16_t chunk_ptr_size = chunk_count + num_chunks + chunk_ptr_increment;
        T *chunk_ptrs_new = (T *)calloc(chunk_ptr_size, sizeof(T*));
        if (chunk_ptrs_new == nullptr) {
            return false;
        }
        // copy pointers to new points array
        memcpy(chunk_ptrs, chunk_count_max, chunk_ptrs_new);

        // free old pointers array
        delete chunk_ptrs;

        // use new pointers array
        chunk_ptrs = chunk_ptrs_new;
        chunk_count_max = chunk_ptr_size;
    }

    // allocate new chunks
    for (uint16_t i = 0; i < num_chunks; i++) {
        T *new_chunk = (T *)calloc(chunk_size, sizeof(T));
        if (new_chunk == nullptr) {
            // failed to allocate new chunk
            return false;
        }
        chunk_ptrs[chunk_count] = new_chunk;
        chunk_count++;
        return true;
    }
}

