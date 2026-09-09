"""A translation lease cannot acknowledge the start of a rotation trial."""


def rotation_handoff_ready(now, started, acknowledgement, motion_limits, session, revision, geometry):
    if not acknowledgement or not motion_limits or not session or not revision or not geometry:
        return False
    ack_seen, ack = acknowledgement
    limits_seen, limits = motion_limits
    # Trial profiles are intentionally disabled for general driving, so
    # `applied` is false. The accepted identity plus live trial flags matter.
    return (started <= ack_seen <= now and 0 <= now-ack_seen <= .25 and
            ack.get('session') == session and ack.get('revision') == revision and
            ack.get('reason') == 'revoked' and
            started <= limits_seen <= now and 0 <= now-limits_seen <= .25 and
            limits.get('geometry_revision') == geometry and
            limits.get('rotation_trial') is True and limits.get('translation_trial') is False)
