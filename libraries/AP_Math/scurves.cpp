#define ALLOW_DOUBLE_MATH_FUNCTIONS

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

#include <AP_Math/AP_Math.h>
#include "scurves.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL &hal;

// constructor
scurves::scurves()
{
    segtype = SegmentType::EMPTY;
    otj = 0.0;
    jerk_max = 0.0;
    accel_max = 0.0;
    vel_max = 0.0;
    _t = 0.0;
    num_segs = 0;
}

scurves::scurves(float tj, float Jp, float Ap, float Vp) :
    otj(tj), jerk_max(Jp), accel_max(Ap), vel_max(Vp)
{
    _t = 0.0;
    num_segs = 0;
    segtype = SegmentType::EMPTY;
}

// initialise the S-curve track
bool scurves::init()
{
    segtype = SegmentType::EMPTY;
    _t = 0.0f;
    num_segs = 0;
    add_segment(0.0f, jtype_t::CONSTANT, 0.0f, 0.0f, 0.0f, 0.0f);
    _track.zero();
    _delta_unit_1.zero();
    _delta_unit_2.zero();
    _delta_unit_3.zero();

    return is_positive(otj) && is_positive(jerk_max) && is_positive(accel_max) && is_positive(vel_max);
}

// generate an optimal jerk limited curve in 3D space that moves over a straight line between two points
void scurves::calculate_straight_leg(const Vector3f &origin, const Vector3f &destination)
{
    if (!init()) {
        // Some parameters have been set to zero
        return;
    }

    _track = destination - origin;
    float track_length = _track.length();
    if (is_zero(track_length)) {
        // avoid possible divide by zero
        _delta_unit_1.zero();
    } else {
        _delta_unit_1 = _track.normalized();
        segtype = SegmentType::STRAIGHT;
        add_segments_straight(track_length);
    }
}

// generate jerk limited curve in 3D space approximating a spline between two points
void scurves::calculate_spline_leg(const Vector3f &origin, const Vector3f &destination, Vector3f origin_vector, Vector3f destination_vector)
{
    if (!init()) {
        // Some parameters have been set to zero
        return;
    }

    _track = destination - origin;
    if (_track.is_zero()) {
        return;
    } else if (origin_vector.is_zero() && destination_vector.is_zero()) {
        calculate_straight_leg(origin, destination);
        return;
    }

    float track_length = _track.length();
    // x is the length of the origin and destination vector assuming a unit length track
    // y is the length of the track assuming a unit length origin and destination vector
    float x, y;
    float cos_phi = 0.0f;
    float cos_alpha = 0.0f;

    // non-zero length : _track, origin_vector, destination_vector

    Vector3f track_unit = _track.normalized();
    if (destination_vector.is_zero()) {
        origin_vector.normalize();
        cos_phi = track_unit * origin_vector;
        y = cos_phi + safe_sqrt(9.0f - (1 - sq(cos_phi)));
        // y must always be positive because cos_phi -1 to 1
        x = 1.0 / y;
        destination_vector = track_unit - origin_vector * x;
        destination_vector.normalize();
        _delta_unit_1 = origin_vector;
        _delta_unit_3 = destination_vector;
    } else if (origin_vector.is_zero()) {
        destination_vector.normalize();
        cos_alpha = track_unit * destination_vector;
        y = cos_alpha + safe_sqrt(9.0f - (1 - sq(cos_alpha)));
        // y must always be positive because cos_phi -1 to 1
        x = 1.0 / y;
        origin_vector = destination_vector * x - track_unit;
        origin_vector.normalize();
        _delta_unit_1 = origin_vector;
        _delta_unit_3 = destination_vector;
    } else {
        origin_vector.normalize();
        destination_vector.normalize();
        cos_phi = track_unit * origin_vector;
        cos_alpha = track_unit * destination_vector;
        Vector3f vector_zero_gap = origin_vector + destination_vector - track_unit * (cos_phi + cos_alpha);
        float zero_gap_squared = vector_zero_gap.length_squared();
        y = MAX(2.0f, cos_phi + cos_alpha + safe_sqrt(4.0 - zero_gap_squared));
        x = 1.0 / y;
        _delta_unit_1 = origin_vector;
        _delta_unit_3 = destination_vector;
    }

    Vector3f track_2 = _track - (_delta_unit_1 + _delta_unit_3) * (track_length * x);
    _delta_unit_2 = track_2.normalized();
    segtype = SegmentType::SPLINE;
    add_segments_curved(track_length * x, track_2.length());
}

// increment time pointer and return the position, velocity and acceleration vectors relative to the origin
bool scurves::move_from_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    if (segtype == SegmentType::STRAIGHT) {
        return move_from_pva_straight(dt, time_scale, pos, vel, accel);
    } else if (segtype == SegmentType::SPLINE) {
        return move_from_pva_spline(dt, time_scale, pos, vel, accel);
    } else {
        return true;
    }
}

// return the position, velocity and acceleration vectors relative to the destination
bool scurves::move_to_pos_vel_accel(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    if (segtype == SegmentType::STRAIGHT) {
        return move_to_pva_straight(dt, time_scale, pos, vel, accel);
    } else if (segtype == SegmentType::SPLINE) {
        return move_to_pva_spline(dt, time_scale, pos, vel, accel);
    } else {
        return true;
    }
}

// return the position, velocity and acceleration vectors relative to the origin
bool scurves::move_from_time_pos_vel_accel(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    if (segtype == SegmentType::STRAIGHT) {
        return move_from_time_pva_straight(time, time_scale, pos, vel, accel);
    } else if (segtype == SegmentType::SPLINE) {
        return move_from_time_pva_spline(time, time_scale, pos, vel, accel);
    } else {
        return true;
    }
}

// return true if the current segment is a straight segment
bool scurves::is_straight() const
{
    return segtype == SegmentType::STRAIGHT;
}

// magnitude of the position vector at the end of the sequence
float scurves::pos_end() const
{
    if (segtype == SegmentType::STRAIGHT) {
        return pos_end_straight();
    } else if (segtype == SegmentType::SPLINE) {
        return pos_end_spline();
    } else {
        return 0.0f;
    }
}

// time at the end of the sequence
float scurves::time_end() const
{
    if (segtype == SegmentType::STRAIGHT) {
        return time_end_straight();
    } else if (segtype == SegmentType::SPLINE) {
        return time_end_spline();
    } else {
        return 0.0f;
    }
}

// time left before sequence will complete
float scurves::get_time_remaining() const
{
    if (segtype == SegmentType::STRAIGHT) {
        return get_time_remaining_straight();
    } else if (segtype == SegmentType::SPLINE) {
        return get_time_remaining_spline();
    } else {
        return 0.0f;
    }
}

// return true if the sequence is braking to a stop
bool scurves::braking() const
{
    if (segtype == SegmentType::STRAIGHT) {
        return braking_straight();
    } else if (segtype == SegmentType::SPLINE) {
        return braking_spline();
    } else {
        return true;
    }
}

// Straight implementations

// increment time pointer and return the position, velocity and acceleration vectors relative to the origin
bool scurves::move_from_pva_straight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(time_scale * dt);
    bool finish = move_from_time_pva_straight(_t, time_scale, pos, vel, accel);
    return finish;
}

// return the position, velocity and acceleration vectors relative to the origin
bool scurves::move_to_pva_straight(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(time_scale * dt);
    bool finish = move_from_time_pva_straight(_t, time_scale, pos, vel, accel);
    pos -= _track;
    return finish;
}

// return the position, velocity and acceleration vectors relative to the origin
bool scurves::move_from_time_pva_straight(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    float scurve_P1, scurve_V1, scurve_A1, scurve_J1;
    bool finish = update(time, scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += _delta_unit_1 * scurve_P1;
    vel += _delta_unit_1 * scurve_V1 * MIN(time_scale * 1.1f, 1.0f);
    accel += _delta_unit_1 * scurve_A1 * time_scale;
    return finish;
}

// Spline implementations

// increment time pointer and return the position, velocity and acceleration vectors relative to the origin
bool scurves::move_from_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(time_scale * dt);
    bool finish = move_from_time_pva_spline(_t, time_scale, pos, vel, accel);
    return finish;
}

// return the position, velocity and acceleration vectors relative to the destination
bool scurves::move_to_pva_spline(float dt, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    advance_time(time_scale * dt);
    bool finish = move_from_time_pva_spline(_t, time_scale, pos, vel, accel);
    pos -= _track;
    return finish;
}

// return the position, velocity and acceleration vectors relative to the origin
bool scurves::move_from_time_pva_spline(float time, float time_scale, Vector3f &pos, Vector3f &vel, Vector3f &accel)
{
    bool finish;
    float scurve_P1, scurve_V1, scurve_A1, scurve_J1;
    float time_start = segment[7].start_time;
    float time_mid = segment[num_segs - 1].start_time - segment[11].start_time;

    update(MIN(time, segment[11].start_time), scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += _delta_unit_1 * scurve_P1;
    vel += _delta_unit_1 * scurve_V1;
    accel += _delta_unit_1 * scurve_A1;

    update(MAX(time - time_start + segment[11].start_time, segment[11].start_time), scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += _delta_unit_2 * (scurve_P1 - segment[11].start_pos);
    vel += _delta_unit_2 * scurve_V1;
    accel += _delta_unit_2 * scurve_A1;

    update(MIN(segment[11].start_time - (time - time_start * 2 - time_mid + segment[11].start_time), segment[11].start_time), scurve_J1, scurve_A1, scurve_V1, scurve_P1);
    pos += _delta_unit_3 * (segment[11].start_pos - scurve_P1);
    vel += _delta_unit_3 * scurve_V1;
    accel += _delta_unit_3 * (-scurve_A1);

    finish = _t > (time_mid + 2.0 * time_start);
    finish = _t > (time_mid + 2.0 * time_start);

    vel *= MIN(time_scale * 1.1f, 1.0f);
    accel *= time_scale;
    return finish;
}

// debugging messages
void scurves::debug()
{
    hal.console->printf("\n");
    hal.console->printf("num_segs:%u, type:%4.2f, _t:%4.2f, otj:%4.2f, jerk_max:%4.2f, accel_max:%4.2f, vel_max:%4.2f\n",
                        (unsigned)num_segs, (double)segtype, (double)_t, (double)otj, (double)jerk_max, (double)accel_max, (double)vel_max);
    hal.console->printf("T, Jt, J, A, V, P \n");
    for (uint8_t i = 0; i < num_segs; i++) {
        hal.console->printf("i:%u, T:%4.2f, Jtype:%4.2f, J:%4.2f, A:%4.2f, V: %4.2f, P: %4.2f\n",
                            (unsigned)i, (double)segment[i].start_time, (double)segment[i].jtype, (double)segment[i].jerk_ref,
                            (double)segment[i].start_accel, (double)segment[i].start_vel, (double)segment[i].start_pos);
    }
    hal.console->printf("_track x:%4.2f, y:%4.2f, z:%4.2f\n", (double)_track.x, (double)_track.y, (double)_track.z);
    hal.console->printf("_delta_unit_1 x:%4.2f, y:%4.2f, z:%4.2f\n", (double)_delta_unit_1.x, (double)_delta_unit_1.y, (double)_delta_unit_1.z);
    hal.console->printf("_delta_unit_2 x:%4.2f, y:%4.2f, z:%4.2f\n", (double)_delta_unit_2.x, (double)_delta_unit_2.y, (double)_delta_unit_2.z);
    hal.console->printf("_delta_unit_2 x:%4.2f, y:%4.2f, z:%4.2f\n", (double)_delta_unit_3.x, (double)_delta_unit_3.y, (double)_delta_unit_3.z);
    hal.console->printf("\n");
}

// straight segment implementations of pos_end, time_end, time_to_end and braking
float scurves::pos_end_straight() const
{
    return segment[num_segs - 1].start_pos;
}

float scurves::time_end_straight() const
{
    return segment[num_segs - 1].start_time;
}

float scurves::get_time_remaining_straight() const
{
    return segment[num_segs - 1].start_time - _t;
}

bool scurves::braking_straight() const
{
    return _t >= segment[8].start_time;
}

// spline segment implementations of pos_end, time_end, time_to_end and braking
float scurves::pos_end_spline() const
{
    return segment[num_segs - 1].start_pos + segment[11].start_pos;
}

float scurves::time_end_spline() const
{
    return segment[num_segs - 1].start_time - segment[11].start_time + segment[7].start_time * 2.0;
}

float scurves::get_time_remaining_spline() const
{
    return time_end_spline() - _t;
}

bool scurves::braking_spline() const
{
    return _t > segment[num_segs - 1].start_time - segment[11].start_time + segment[7].start_time;
}

// generate constant jerk time segment
void scurves::add_segment_const_jerk(float tin, float J0)
{
    enum jtype_t Jtype = jtype_t::CONSTANT;
    float J = J0;
    float T = segment[num_segs - 1].start_time + tin;
    float A = segment[num_segs - 1].start_accel + J0 * tin;
    float V = segment[num_segs - 1].start_vel + segment[num_segs - 1].start_accel * tin + 0.5 * J0 * sq(tin);
    float P = segment[num_segs - 1].start_pos + segment[num_segs - 1].start_vel * tin + 0.5 * segment[num_segs - 1].start_accel * sq(tin) + (1 / 6.0) * J0 * powf(tin, 3.0);
    add_segment(T, Jtype, J, A, V, P);
}

// generate increasing jerk magnitude time segment based on a raised cosine profile
void scurves::add_segment_incr_jerk(float tj, float Jp)
{
    float Beta = M_PI / tj;
    float Alpha = Jp / 2.0;
    float AT = Alpha * tj;
    float VT = Alpha * (sq(tj) / 2.0 - 2.0 / sq(Beta));
    float PT = Alpha * ((-1.0 / sq(Beta)) * tj + (1 / 6.0) * powf(tj, 3.0));

    enum jtype_t Jtype = jtype_t::POSITIVE;
    float J = Jp;
    float T = segment[num_segs - 1].start_time + tj;
    float A = segment[num_segs - 1].start_accel + AT;
    float V = segment[num_segs - 1].start_vel + segment[num_segs - 1].start_accel * tj + VT;
    float P = segment[num_segs - 1].start_pos + segment[num_segs - 1].start_vel * tj + 0.5 * segment[num_segs - 1].start_accel * sq(tj) + PT;
    add_segment(T, Jtype, J, A, V, P);
}

// generate  decreasing jerk magnitude time segment based on a raised cosine profile
void scurves::add_segment_decr_jerk(float tj, float Jp)
{
    float Beta = M_PI / tj;
    float Alpha = Jp / 2.0;
    float AT = Alpha * tj;
    float VT = Alpha * (sq(tj) / 2.0 - 2.0 / sq(Beta));
    float PT = Alpha * ((-1.0 / sq(Beta)) * tj + (1 / 6.0) * powf(tj, 3.0));
    float A2T = Jp * tj;
    float V2T = Jp * sq(tj);
    float P2T = Alpha * ((-1.0 / sq(Beta)) * 2.0 * tj + (4.0 / 3.0) * powf(tj, 3.0));

    enum jtype_t Jtype = jtype_t::NEGATIVE;
    float J = Jp;
    float T = segment[num_segs - 1].start_time + tj;
    float A = (segment[num_segs - 1].start_accel - AT) + A2T;
    float V = (segment[num_segs - 1].start_vel - VT) + (segment[num_segs - 1].start_accel - AT) * tj + V2T;
    float P = (segment[num_segs - 1].start_pos - PT) + (segment[num_segs - 1].start_vel - VT) * tj + 0.5 * (segment[num_segs - 1].start_accel - AT) * sq(tj) + P2T;
    add_segment(T, Jtype, J, A, V, P);
}

// generate three time segment raised cosine jerk profile
void scurves::add_segments_incr_const_decr_jerk(float tj, float Jp, float Tcj)
{
    add_segment_incr_jerk(tj, Jp);
    add_segment_const_jerk(Tcj, Jp);
    add_segment_decr_jerk(tj, Jp);
}

// generate time segments for straight segment
void scurves::add_segments_straight(float Pp)
{
    if (is_zero(Pp)) {
        return;
    }

    float tj = otj;
    float Jp = jerk_max;
    float Ap = accel_max;
    float Vp = vel_max;

    float t2, t4, t6;
    cal_posfast(tj, Jp, Ap, Vp, Pp / 2.0, Jp, t2, t4, t6);

    add_segments_incr_const_decr_jerk(tj, Jp, t2);
    add_segment_const_jerk(t4, 0.0);
    add_segments_incr_const_decr_jerk(tj, -Jp, t6);

    float t8 = 2.0f * (Pp / 2.0 - segment[num_segs - 1].start_pos) / segment[num_segs - 1].start_vel;
    add_segment_const_jerk(t8, 0.0);

    add_segments_incr_const_decr_jerk(tj, -Jp, t6);
    add_segment_const_jerk(t4, 0.0);
    add_segments_incr_const_decr_jerk(tj, Jp, t2);
}

// generate time segments to generate large curved corners
void scurves::add_segments_curved(float Pp, float Pm)
{
    if (is_zero(Pp)) {
        return;
    }

    float tj = otj;
    float Jp = jerk_max;
    float Ap = accel_max;
    float Vp = vel_max;
    float Js;
    float Vs;
    float Ps;
    float tc = 0;
    float Jc = 0;
    float Ac = 0;
    float Vc = 0;
    float Pc = 0;

    Pc = Pp * 2.0 / 3.0;
    Ps = Pp - Pc;
    Vs = MIN(Vp, MIN(MIN(safe_sqrt(Ap * Pc), powf(0.5 * Jp * sq(Pc), 1.0 / 3.0)), Pc / (2 * tj)));

    float t2, t4, t6;
    cal_posfast(tj, Jp, Ap, Vs, Ps, Js, t2, t4, t6);
    add_segments_incr_const_decr_jerk(tj, Js, t2);
    add_segment_const_jerk(t4, 0.0);
    add_segments_incr_const_decr_jerk(tj, -Js, t6);

    Pc = Pp - segment[num_segs - 1].start_pos;
    Vc = segment[num_segs - 1].start_vel;

    Ac = MIN(MIN(Pc / (4 * tj * tj), Vc * Vc / Pc), powf((Jp * Jp) * Pc, 1.0 / 3.0) * 6.299605249474365E-1);

    Jc = powf(Ac, 3.0 / 2.0) * 1.0 / safe_sqrt(Pc) * 2.0;
    tc = Ac / Jc;

    add_segment_incr_jerk(tc, -Jc);
    add_segment_decr_jerk(tc, -Jc);
    add_segment_incr_jerk(tc, Jc);
    add_segment_decr_jerk(tc, Jc);

    add_segment_incr_jerk(tc, Jc);
    add_segment_decr_jerk(tc, Jc);
    add_segment_incr_jerk(tc, -Jc);
    add_segment_decr_jerk(tc, -Jc);

    float Tcv = (Pm - 2.0 * (segment[num_segs - 1].start_pos - Pp)) / segment[num_segs - 1].start_vel;
    add_segment_const_jerk(Tcv, 0.0);

    add_segment_incr_jerk(tc, -Jc);
    add_segment_decr_jerk(tc, -Jc);
    add_segment_incr_jerk(tc, Jc);
    add_segment_decr_jerk(tc, Jc);
}

// calculate duration of time segments for basic acceleration and deceleration curve from and to stationary.
void scurves::cal_posfast(float tj, float Jp, float Ap, float Vp, float Pp, float &Jp_out, float &t2_out, float &t4_out, float &t6_out) const
{
    if ((Vp < Jp * (tj * tj) * 2.0) || (Pp < Jp * (tj * tj * tj) * 4.0)) {
// solution = 0 - t6 t4 t2 = 0 0 0
        t4_out = MIN(Vp / (2.0 * Jp * tj * tj), Pp / (4.0 * Jp * tj * tj * tj));
        t2_out = 0;
        t4_out = 0;
        t6_out = 0;
    } else if (Ap < Jp * tj) {
// solution = 2 - t6 t4 t2 = 0 1 0
        Jp = Ap / tj;
        t2_out = 0;
        t4_out = MIN((Vp - Ap * tj * 2.0) / Ap, -3.0 * tj + safe_sqrt((Pp * 2.0) / Ap + tj * tj));
        t6_out = 0;
    } else {
        if ((Vp < Ap * tj + (Ap * Ap) / Jp) || (Pp < Ap * 1.0 / (Jp * Jp) * powf(Ap + Jp * tj, 2.0))) {
// solution = 5 - t6 t4 t2 = 1 0 1
            Ap = MIN(Ap, MIN(-Jp * tj * (1.0 / 2.0) + safe_sqrt(Jp * Vp * 4.0 + (Jp * Jp) * (tj * tj)) * (1.0 / 2.0), powf(Jp * tj - powf((Jp * Jp) * Pp * (1.0 / 2.0) + safe_sqrt((Jp * Jp * Jp * Jp) * (Pp * Pp) * (1.0 / 4.0) + (Jp * Jp * Jp * Jp * Jp) * Pp * (tj * tj * tj) * (1.0 / 2.7E1)) + (Jp * Jp * Jp) * (tj * tj * tj) * (1.0 / 2.7E1), 1.0 / 3.0) * 3.0, 2.0) * 1.0 / powf((Jp * Jp) * Pp * (1.0 / 2.0) + safe_sqrt((Jp * Jp * Jp * Jp) * (Pp * Pp) * (1.0 / 4.0) + (Jp * Jp * Jp * Jp * Jp) * Pp * (tj * tj * tj) * (1.0 / 2.7E1)) + (Jp * Jp * Jp) * (tj * tj * tj) * (1.0 / 2.7E1), 1.0 / 3.0) * (1.0 / 9.0)));
            t2_out = Ap / Jp - tj;
            t4_out = 0;
            t6_out = t2_out;
        } else {
// solution = 7 - t6 t4 t2 = 1 1 1
            t2_out = Ap / Jp - tj;
            t4_out = MIN(Vp / Ap - (Ap + Jp * tj) / Jp, (Ap * (-3.0 / 2.0) - Jp * tj * (3.0 / 2.0)) / Jp + (safe_sqrt(Ap * Ap * Ap * Ap + (Ap * Ap) * (Jp * Jp) * (tj * tj) + Ap * (Jp * Jp) * Pp * 8.0 + (Ap * Ap * Ap) * Jp * tj * 2.0) * (1.0 / 2.0)) / (Ap * Jp));
            t6_out = t2_out;
        }
    }
    Jp_out = Jp;
}

// calculate the jerk, acceleration, velocity and position at time t
bool scurves::update(float time, float &Jt_out, float &At_out, float &Vt_out, float &Pt_out)
{
    jtype_t Jtype;
    int8_t pnt = num_segs;
    float tj;
    float Jp, T0, A0, V0, P0;

    for (uint8_t i = 0; i < num_segs; i++) {
        if (time < segment[num_segs - 1 - i].start_time) {
            pnt = num_segs - 1 - i;
        }
    }
    if (pnt == 0) {
        Jtype = jtype_t::CONSTANT;
        Jp = 0.0f;
        T0 = segment[pnt].start_time;
        A0 = segment[pnt].start_accel;
        V0 = segment[pnt].start_vel;
        P0 = segment[pnt].start_pos;
    } else if (pnt == num_segs) {
        Jtype = jtype_t::CONSTANT;
        Jp = 0.0;
        T0 = segment[pnt - 1].start_time;
        A0 = segment[pnt - 1].start_accel;
        V0 = segment[pnt - 1].start_vel;
        P0 = segment[pnt - 1].start_pos;
    } else {
        Jtype = segment[pnt].jtype;
        Jp = segment[pnt].jerk_ref;
        tj = segment[pnt].start_time - segment[pnt - 1].start_time;
        T0 = segment[pnt - 1].start_time;
        A0 = segment[pnt - 1].start_accel;
        V0 = segment[pnt - 1].start_vel;
        P0 = segment[pnt - 1].start_pos;
    }

    switch (Jtype) {
    case jtype_t::CONSTANT:
        calc_javp_for_segment_const_jerk(time - T0, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case jtype_t::POSITIVE:
        calc_javp_for_segment_incr_jerk(time - T0, tj, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    case jtype_t::NEGATIVE:
        calc_javp_for_segment_decr_jerk(time - T0, tj, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
        break;
    }

    return pnt == num_segs;
}

// calculate the jerk, acceleration, velocity and position at time "time" when running the constant jerk time segment
void scurves::calc_javp_for_segment_const_jerk(float time, float J0, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    Jt = J0;
    At = A0 + J0 * time;
    Vt = V0 + A0 * time + 0.5 * J0 * (time * time);
    Pt = P0 + V0 * time + 0.5 * A0 * (time * time) + (1 / 6.0) * J0 * (time * time * time);
}

// Calculate the jerk, acceleration, velocity and position at time "time" when running the increasing jerk magnitude time segment based on a raised cosine profile
void scurves::calc_javp_for_segment_incr_jerk(float time, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    float Alpha = Jp / 2.0;
    float Beta = M_PI / tj;
    Jt = Alpha * (1.0 - cosf(Beta * time));
    At = A0 + Alpha * time - (Alpha / Beta) * sinf(Beta * time);
    Vt = V0 + A0 * time + (Alpha / 2.0) * (time * time) + (Alpha / (Beta * Beta)) * cosf(Beta * time) - Alpha / (Beta * Beta);
    Pt = P0 + V0 * time + 0.5 * A0 * (time * time) + (-Alpha / (Beta * Beta)) * time + Alpha * (time * time * time) / 6.0 + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * time);
}

// Calculate the jerk, acceleration, velocity and position at time "time" when running the  decreasing jerk magnitude time segment based on a raised cosine profile
void scurves::calc_javp_for_segment_decr_jerk(float time, float tj, float Jp, float A0, float V0, float P0, float &Jt, float &At, float &Vt, float &Pt) const
{
    float Alpha = Jp / 2.0;
    float Beta = M_PI / tj;
    float AT = Alpha * tj;
    float VT = Alpha * ((tj * tj) / 2.0 - 2.0 / (Beta * Beta));
    float PT = Alpha * ((-1.0 / (Beta * Beta)) * tj + (1 / 6.0) * (tj * tj * tj));
    Jt = Alpha * (1.0 - cosf(Beta * (time + tj)));
    At = (A0 - AT) + Alpha * (time + tj) - (Alpha / Beta) * sinf(Beta * (time + tj));
    Vt = (V0 - VT) + (A0 - AT) * time + 0.5 * Alpha * (time + tj) * (time + tj) + (Alpha / (Beta * Beta)) * cosf(Beta * (time + tj)) - Alpha / (Beta * Beta);
    Pt = (P0 - PT) + (V0 - VT) * time + 0.5 * (A0 - AT) * (time * time) + (-Alpha / (Beta * Beta)) * (time + tj) + (Alpha / 6.0) * (time + tj) * (time + tj) * (time + tj) + (Alpha / (Beta * Beta * Beta)) * sinf(Beta * (time + tj));
}

void scurves::add_segment(float start_time, enum jtype_t jtype, float jerk_ref, float start_accel, float start_vel, float start_pos)
{
    segment[num_segs].start_time = start_time;
    segment[num_segs].jtype = jtype;
    segment[num_segs].jerk_ref = jerk_ref;
    segment[num_segs].start_accel = start_accel;
    segment[num_segs].start_vel = start_vel;
    segment[num_segs].start_pos = start_pos;
    num_segs++;
}
