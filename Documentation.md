Path Planning — Documentation
Solution Overview: Virtual Track Completion

Instead of planning directly from the sparse detected cones, the planner firstcompletes a full virtual track around the car, then reads the path straightout of it. The core idea: if we had cones on both boundaries every meter, thepath would simply be the line joining the midpoints of corresponding left/rightcone pairs — so we synthesize exactly that.
Algorithm (src/path_planning.py — PathPlanning.generatePath)

    Estimate the driving direction from the cones, not blindly from yaw(in several scenarios the yaw points away from the track):
        Both sides visible: pair each blue cone with its nearest yellow cone("gates"); direction = first gate → last gate.
        One gate only: drive perpendicular to the gate line (from blue towardbeing centered), keeping yellow on the right / blue on the left.
        One side only: direction along the boundary line (first → last cone).
        No cones: straight along yaw.
        Yaw is used only as a tiebreaker/fallback sign check.
    Build a rough centerline through waypoints:
        Gate midpoints (both sides), ordered along the travel direction.
        If only one side is visible: each boundary cone offset inward by half atrack width, so the cone stays on its correct side of the path.
    Extend/truncate the centerline so it starts at the car, continues pastthe farthest cone (+1 m margin), and its total length is clamped to therequired 5–10 m.
    Place virtual cones: walk along the centerline in 1 m steps; at everystep put one virtual cone LEFT and one RIGHT of the centerline, snappedto integer grid coordinates. This continues until all detected cones arecovered — a complete virtual track around the car.
        Track width: measured from the real gates when both sides exist(clamped 1–3 m), otherwise a default of 2 m.
    Path = midpoints of corresponding virtual cone pairs, joined by straightsegments (no smoothing), densified to ≤ 0.5 m step and length clamped to5–10 m, as the task requires.

The virtual track is stored in planner.virtual_left / planner.virtual_rightand visualized in the tester as hollow circles alongside the real cones.
Part 2 — three cones on one side

No separate algorithm is needed: the "one side visible" branch already handlesany number of cones — each boundary cone becomes a waypoint offset inward byhalf a track width, so three cones simply produce a three-segment centerline(able to represent a bent/curved boundary). New scenarios 21–24 insrc/scenarios.py cover: three yellow in a line, three yellow curved, threeblue curved, and three yellow + one blue mixed.

Why this solution: it is the simplest approach that directly mirrors how areal track looks (paired boundary cones → centerline), it naturally covers allgiven configurations (0, 1, 2, 3+ cones per side), requires no externallibraries or heavy math, and makes the path trivially correct-by-construction:it is the midpoint line of a well-formed track.
Assumptions (allowed per README)

    Cone colors are correct; the track lies roughly ahead of the car.
    Default track width 2 m when a boundary is missing; measured width clampedto 1–3 m otherwise.
    A single cone behind the car is ignored; unpaired extra cones don't affectthe width (nearest-neighbor gate pairing, greedy).
    Cones farther than ~9 m are not covered (10 m path-length cap).

Limitations

    Straight-segment boundaries; sharp corners/S-curves are only approximated(on an integer grid a boundary cone may repeat or step one cell — cosmetic).
    Greedy nearest-neighbor pairing could mispair cones in dense/clustered maps.
    With zero cones the path is straight along yaw (no track information).
    Direction inference assumes the track does not double back within view.
