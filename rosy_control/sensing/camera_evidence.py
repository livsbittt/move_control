"""Conservative legacy flags when a camera observation cannot be evaluated."""


def legacy_flags(result, previous_cliff=False):
    if not result.get('quality', {}).get('valid', False):
        # Blindness requires a hold, but is not evidence of a new cliff.
        return bool(previous_cliff), True
    return bool(result['cliff']), bool(result['blocked'])
