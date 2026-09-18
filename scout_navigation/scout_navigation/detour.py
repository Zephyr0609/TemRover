import numpy as np


def lateral_offset(nearest, farthest, half_width, margin, committed):
    """The smaller shift that clears the object; once a side is chosen it is kept for the whole run."""
    left = farthest + half_width + margin
    right = nearest - half_width - margin
    if committed:
        return left if committed > 0 else right
    return left if abs(left) <= abs(right) else right


def nearest_group(along, across, gap):
    """Returns of the first object ahead: sorted along the line and cut at the first gap wider than gap."""
    order = np.argsort(along)
    breaks = np.flatnonzero(np.diff(along[order]) > gap)
    keep = order[:breaks[0] + 1] if len(breaks) else order
    return along[keep], across[keep]


def hermite(fraction, start, start_slope, end):
    """Cubic from start with the given slope to end with zero slope, so the heading never jumps."""
    return ((2.0 * fraction ** 3 - 3.0 * fraction ** 2 + 1.0) * start
            + (fraction ** 3 - 2.0 * fraction ** 2 + fraction) * start_slope
            + (3.0 * fraction ** 2 - 2.0 * fraction ** 3) * end)


def ramp_length(shift, speed, lateral_acceleration):
    """A cubic ramp of length L to a shift D peaks at curvature 6 D / L^2, hence at speed^2 6 D / L^2."""
    return np.sqrt(6.0 * abs(shift) * speed ** 2 / lateral_acceleration)


def plan_detour(start, end, s_now, d_now, slope_now, s_range, offset, margin, speed,
                lateral_acceleration, segment_length):
    """Ramps sideways before the object, holds beside it, ramps back, then finishes the line."""
    direction = (end - start) / np.hypot(*(end - start))
    normal = np.array([-direction[1], direction[0]])
    length = float(np.hypot(*(end - start)))

    ramp_out_end = s_range[0] - margin
    hold_end = s_range[1] + margin
    ramp_in_end = min(hold_end + ramp_length(offset, speed, lateral_acceleration), length)
    stations = np.append(np.arange(s_now, ramp_in_end, segment_length), ramp_in_end)
    lateral = np.full_like(stations, offset)

    # Before the object: return to the line first when there is room, otherwise cross over directly
    return_end = s_now + ramp_length(d_now, speed, lateral_acceleration)
    ramp_out_start = ramp_out_end - ramp_length(offset, speed, lateral_acceleration)
    if return_end <= ramp_out_start:
        back_on_line = stations < return_end
        lateral[back_on_line] = hermite((stations[back_on_line] - s_now) / max(return_end - s_now, 1e-9),
                                        d_now, slope_now * (return_end - s_now), 0.0)
        lateral[(stations >= return_end) & (stations < ramp_out_start)] = 0.0
        out = (stations >= ramp_out_start) & (stations < ramp_out_end)
        lateral[out] = hermite((stations[out] - ramp_out_start) / (ramp_out_end - ramp_out_start),
                               0.0, 0.0, offset)
    else:
        ramp_out_start = max(s_now, ramp_out_end - ramp_length(offset - d_now, speed, lateral_acceleration))
        lateral[stations < ramp_out_start] = d_now
        out = (stations >= ramp_out_start) & (stations < ramp_out_end)
        start_slope = slope_now if ramp_out_start == s_now else 0.0
        lateral[out] = hermite((stations[out] - ramp_out_start) / (ramp_out_end - ramp_out_start),
                               d_now, start_slope * (ramp_out_end - ramp_out_start), offset)
    back = stations > hold_end
    lateral[back] = hermite((stations[back] - hold_end) / (ramp_in_end - hold_end), offset, 0.0, 0.0)

    points = start + np.outer(stations, direction) + np.outer(lateral, normal)
    steps = [('drive', first, second) for first, second in zip(points[:-1], points[1:])]
    if ramp_in_end < length:
        steps.append(('drive', points[-1], end))
    return steps
