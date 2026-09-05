"""Control package: robot-frame geometry -> motion policy.

modes   — the one canonical /robot/mode label (hazard > action > bands).
recover — stuck/backup/escape policy (backup limits, turn space, signs).
route   — longest free straight line from the live scan.
"""
from .modes import MODES, Subject, WANDER_TO_MODE, nose_on_wall, pick_mode
from .recover import (ESCAPE_MIN_TURN, STUCK_CLEAR_M, backup_limit_m,
                      escape_may_abort, escape_may_desense,
                      have_turn_space, hazard_action, is_stuck_motion,
                      need_space_to_turn, ratio_sign, side_sign,
                      stuck_flip, stuck_kind, wall_first_move)
from .route import line_route

__all__ = ['ESCAPE_MIN_TURN', 'MODES', 'STUCK_CLEAR_M', 'Subject',
           'WANDER_TO_MODE', 'backup_limit_m', 'escape_may_abort',
           'escape_may_desense', 'have_turn_space', 'hazard_action',
           'is_stuck_motion', 'need_space_to_turn', 'nose_on_wall',
           'pick_mode', 'ratio_sign', 'side_sign', 'stuck_flip',
           'stuck_kind', 'wall_first_move', 'line_route']
