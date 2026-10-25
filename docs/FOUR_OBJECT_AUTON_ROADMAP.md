# Block-build roadmap

DO NOT work on Block N+1 until Block N is sufficiently repeatable under the team's
documented acceptance criteria. Choose those physical criteria on the field; this
task supplies no invented numeric threshold or pass result.

| Stage | Work | Gate before expansion |
|---|---|---|
| 1-object | B1_APPROACH, B1_FINAL_ALIGN, B1_SCORE, B1_EXIT | independently qualify each, then repeat the complete start-to-exit prefix |
| 2-object | append B2_PICKUP_APPROACH, PICKUP, RETURN, FINAL_ALIGN, SCORE, EXIT | B1 remains unchanged and repeatable; qualify B2 individually then 1+2 |
| 3-object | append the corresponding B3 phases | prove prefix 1+2, then 1+2+3 |
| 4-object | append the corresponding B4 phases | prove prefix 1+2+3, then full route |
| Optimization | change one speed/chain/overlap/correction choice at a time | repeated endpoint and timing evidence remains acceptable |
| Stress test | vary starts, battery, load and object placement deliberately | identify failure envelope and recovery/abort behavior |
| Skills reuse | compose the same qualified pickup/scoring/travel helpers | separately measured Skills start and geometry, same safety/logging |

Each new block is independently tested travel, pickup, alignment, score and exit.
Final heading uses trusted nav_turn_to after nav_go_to until another method is
qualified. No new block depends on nav_go_to_pose. Absolute corrections are optional
checkpoint operations, not a replacement for good tracking. Settling can be deliberate
at scoring/pickup; a transit checkpoint need not stop.

route_config.hpp stores stage configuration and qualified_prefix flags. Routes run
the same prior callback prefix rather than four duplicated autonomous functions.
left/right/safe/skills selector entries remain permanent; none contains fake motion.
Use mirror helpers only when a route explicitly opts in. Reflect across Y as
X -> -X, Y unchanged, heading -> wrap(360-heading). Field asymmetry can require
separate calibration. Skills must reuse mechanisms, logging, limits and localization
helpers; do not build a second mechanism/control stack.
