"""Subject: what the camera can honestly claim, and how it says it on the wire.

One serialiser, so the real node and the Gazebo adapter cannot describe the same
observation differently -- they did, and the divergence was invisible because
nothing downstream reads the regions yet.

Wire schema for `regions` (short keys because this goes out at the frame rate):
  b  bbox as [x0, y0, x1, y1] in image pixels
  n  1 when the region reaches the near centre path, else 0
  k  'd' dark region, 'f' foreground region
  m  ground-plane distance in metres; ABSENT means unranged, never zero

Measured: the previous verbose form reached 11362 B at the 64-region cap, larger
than shipping the whole frame as JPEG (9813 B at q70), which defeated the point
of publishing evidence instead of pixels.
"""

# 48, not 64: the worst case must stay under 4 KB once regions carry a measured
# distance too. Measured at 320x240 with 3-digit boxes and a 3-decimal range --
# cap 64 ranged = 4135 B, cap 48 ranged = 3175 B. Regions arrive sorted by
# (near_path, area_px) so the cap drops the least significant, and region_count
# still reports the true total. Real frames produce 5 and 9 regions.
REGION_CAP = 48


def legacy_flags(result, previous_cliff=False):
    """Conservative flags when a camera observation cannot be evaluated."""
    if not result.get('quality', {}).get('valid', False):
        # Blindness requires a hold, but is not evidence of a new cliff.
        return bool(previous_cliff), True
    return bool(result['cliff']), bool(result['blocked'])


def encode_region(region):
    """One region, compactly. Distance is omitted rather than faked."""
    encoded = {
        'b': [int(v) for v in region['bbox_xyxy']],
        'n': int(bool(region.get('near_path'))),
        'k': 'd' if region.get('kind') == 'dark_region' else 'f',
    }
    distance = region.get('distance_m')
    if distance is not None:
        encoded['m'] = round(float(distance), 3)
    return encoded


def observation_payload(stamp, cliff, blocked, side, result, image_size, source,
                        detector='floor_foreground_v3'):
    """The evidence message. Verdicts come from the policy, facts from the frame.

    v3, not v2: the region schema is compact now, and `cliff` is constant False
    because the camera withdrew that verdict. The live capture in
    docs/validation/detector-main-8e9cb78/ was recorded against v2 and describes
    different behaviour, so the label has to separate them.
    """
    regions = result.get('regions', [])
    return {
        'stamp': float(stamp),
        'blocked': bool(blocked),
        'cliff': bool(cliff),
        'side': float(side),
        'source': source,
        'detector': detector,
        'quality': result['quality'],
        'regions': [encode_region(r) for r in regions[:REGION_CAP]],
        'region_count': int(result.get('region_count', len(regions))),
        'regions_truncated': bool(result.get('region_count', len(regions)) > REGION_CAP),
        'image_size': [int(image_size[0]), int(image_size[1])],
    }
