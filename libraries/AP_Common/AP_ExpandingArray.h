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

#pragma once

#include <AP_Common/AP_Common.h>

template <typename T>
class AP_ExpandingArray
{
public:

    AP_ExpandingArray<T>() :
        chunk_size(50)
    {   // create one chunk
        expand(1);
    }

    //AP_ExpandingArray<T>(uint16_t initial_size, uint16_t elements_per_chunk);

    /* Do not allow copies */
    //AP_ExpandingArray<T>(const AP_ExpandingArray<T> &other) = delete;
    //AP_ExpandingArray<T> &operator=(const AP_ExpandingArray<T>&) = delete;

    // current maximum number of items (using expand may increase this)
    uint16_t max_items() const { return chunk_size * chunk_count; }

    // allow use as an array. no bounds checking is performed
    T &operator[](uint16_t i);
    const T &operator[](uint16_t i) const;

    // expand the array by 1 chunk, returns true on success
    bool expand(uint16_t num_chunks = 1);

private:

    // chunk_ptrs array is grown by this many elements each time it fills
    const uint16_t chunk_ptr_increment = 50;

    uint16_t chunk_size;        // the number of T elements in each chunk
    T *chunk_ptrs;              // table of pointers to allocated chunks
    uint16_t chunk_count_max;   // number of elements in chunk_ptrs array
    uint16_t chunk_count;       // number of allocated chunks
};
