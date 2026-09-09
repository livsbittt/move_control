# Precision envelope and execution escape

The physical v18 trial escaped the off-center map start cell and moved 107.46 mm.
It later stopped because a requested steering component could not pass the
rotation clearance gate. Replanning selected another route, but that route also
needed rotation. Plan ETA was therefore not evidence of actual movement.

The saved v16 envelope is a configured 83 mm chassis radius, not a newly measured
104.06 mm chassis. Its full-turn requirement adds 2.39 mm for the estimated pivot
offset and 18.67 mm for twice the pivot uncertainty. The small approximately
10-degree endpoint excitation amplifies about 0.9 mm scan residual into about
5.2 mm pivot uncertainty; variation between directional estimates adds more.
Removing that uncertainty without improving the evidence would hide the problem.

## Refinement

The production rotation trial now visits +10, 0, -10, 0, +10, 0, -10, 0, +10, 0
degrees. Register consecutive nonzero settled endpoints directly, obtaining four
approximately 20-degree transitions, two in each direction. The first endpoint
only anchors the first transition. Adjacent transitions share an endpoint; they
are not treated as independent statistical samples and no square-root reduction
of uncertainty is used. Physical pose and translation guards are unchanged.

The cross-endpoint model has an explicit version, exact ordered endpoint-pair
identities, and ten response legs. Certificates and applied profiles validate the
sequence and envelope together. Existing eight-leg v1 certificates still load.
Operational angular compensation remains neutral; this refines the pivot
estimate, not the externally audited chassis footprint.

## Execution

The v20 isolated robot capture reproduced a permanent recovery wait after a
single missing proposal frame: 2.86 mm of actual reverse travel, a null proposal,
then fresh 2-8 mm candidates rejected by the previous used latch. The fixture is
`test/fixtures/execution_escape_dropout_20260910.json`.

Bounded missions now evaluate fresh measured proposals within the original
mission and stall deadlines. A missing proposal commands zero; fresh evidence
resumes the same escape episode. The controller does not exhaust a fixed retry
count in bounded sessions. Legacy unbounded recovery retains its retry limit.
An episode has an 80 mm travel cap, 20 second time cap, and 8 second no-progress
cap. These are limits, not commanded displacement. Each update selects a current
geometry-derived distance, preferring the shortest candidate that restores
rotation clearance. No-progress holds require real position or improved space
before rearming. A single clear frame cannot renew the episode clock.

The safety node projects observed points through candidate straight translations,
using the saved pivot envelope and actual directional travel limits. It never
changes the motor gate's authority. The dashboard exposes requested and final
commands, waiting reason, remaining episode time, and current proposed distance;
proposal distance and nominal ETA are not measured movement.

## Verification and current deployment boundary

Local executor and preflight changes passed 875 Python tests (one skipped) and 26 dashboard
tests. The actual five-frame dropout fixture reproduces reverse, reverse, zero,
reverse, reverse with an unchanged episode start time.

The v20 full calibration retrial passed all four linear legs but stopped before
rotation: partial scan, 51 missing bins, largest gap 11.015 degrees, nearest raw
return 123 mm, final gate rotation denied. A raw range is not chassis clearance;
mount offset and the current envelope must be included. New zero-only preflight
observation handling waits at most one second for fresh authoritative clearance,
counts against the existing 60 second rotation phase deadline, and does not
classify an observed obstacle inside the active envelope as a data gap.

The v21 executor archive was prepared but SSH transfer failed after the PC lost
its robot Wi-Fi connection. No v21 build, deployment, or physical acceptance has
occurred. The robot was explicitly stopped at the end of the v20 calibration
trial. The latest calibration remains failed; the previous verified v16
certificate backup must be restored before navigation, without replacing any
subsequent successfully measured certificate. Precision-envelope improvement and
v21 continuous physical navigation remain unverified.


## Obstacle invalidation of committed escape targets

A second planner deadlock was reproduced: an escape target invalidated by new
occupancy remained committed and returned no route on every tick. The planner
now releases that invalid target and evaluates alternatives in the same tick.
When an obstacle only blocks the direct segment, A* still detours to the valid
original target. When no footprint-clear route exists, it returns no route;
no obstacle or inflation is erased, and no arrival is fabricated. Regression
maps cover the detour, blocked target with another route, and blocked start.
