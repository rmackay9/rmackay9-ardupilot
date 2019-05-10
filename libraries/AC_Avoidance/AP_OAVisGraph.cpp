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

#include "AP_OAVisGraph.h"
#include <stdio.h>

extern const AP_HAL::HAL &hal;

// constructor with size argument
AP_OAVisGraph::AP_OAVisGraph(uint8_t size)
{
    init(size);
}

// initialise array to given size
bool AP_OAVisGraph::init(uint8_t size)
{
    // only allow init once
    if (_items != nullptr) {
        return false;
    }

    // allocate array
    _items = (VisGraphItem *)calloc(size, sizeof(VisGraphItem));
    if (_items != nullptr) {
        _num_items_max = size;
        return true;
    }

    return false;
}

// add item to visiblity graph, returns true on success, false if graph is full
bool AP_OAVisGraph::add_item(const OAItemID &id1, const OAItemID &id2, float distance_cm)
{
    // check there is space
    if (_num_items >= _num_items_max) {
        return false;
    }

    // add item
    _items[_num_items] = {id1, id2, distance_cm};
    _num_items++;
    return true;
}

// print all items to console for debug purposes
void AP_OAVisGraph::print_all_items()
{
    for (uint8_t i=0; i<_num_items; i++) {
        ::printf("%d: id1:%d-%d id2:%d-%d dist:%4.1f\n",
                (int)i,
                (int)_items[i].id1.id_type,
                (int)_items[i].id1.id_num,
                (int)_items[i].id2.id_type,
                (int)_items[i].id2.id_num,
                (double)_items[i].distance_cm);
    }
}
