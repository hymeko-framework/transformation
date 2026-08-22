# robotics descriptions

79 `.hymeko` descriptions, grouped by what they are rather than left flat.

The **14 library descriptions stay at the root**. Every example imports at
least one of them — `meta_kinematics` alone is imported 39 times — so keeping
them one level up means every example reaches them the same way, as
`@"../meta_kinematics.hymeko"`.

Imports are resolved by `ModuleStore` as `base_dir.join(path)`, relative to the
importing file, with **no search fallback**. A path that is wrong simply fails
to load, so relative paths here have to be exact.

## Libraries (root)

The shared vocabulary. None of these import anything.

| File | Declares | Imported by |
|---|---|---|
| `meta_kinematics` | links, joints, geometry, axes, controllers, sensors | 39 |
| `meta_reward` | reward-term vocabulary | 15 |
| `meta_env` | environment / observation-space profile | 11 |
| `meta_scenario` | scenario composition profile | 5 |
| `meta_task` | task specification profile | 4 |
| `meta_controller` | controller profile | 3 |
| `meta_experiment` | experiment / run profile | 3 |
| `meta_strategy_graph` | strategy-as-graph profile | 3 |
| `meta_observation` | observation profile | 2 |
| `meta_topic` | ROS-topic profile | 2 |
| `meta_strategy` | strategy profile | 1 |
| `option_rl_meta` | option-level RL profile | 1 |
| `meta_context` | context profile | — |
| `metaworld_reward` | MetaWorld dense reward, as HyMeKo | — |

## Groups

| Folder | Holds | Count |
|---|---|---|
| `robots/` | kinematic models — arms, grippers, wheeled and legged platforms | 19 |
| `tasks/` | task, reward and training-graph specifications | 13 |
| `galambos/` | the Galambos planar-grasper family: env variants, planar models, task variants, scenario and strategy | 27 |
| `coin/` | the coin-delivery family: carry option, delivery, kinetic policy | 4 |
| `test_fixtures/` | descriptions that exist only to exercise the compiler — not examples | 2 |

Pre-existing folders, left as they were: `planning_examples/`, `sim/`,
`synthetic_fixtures/` (URDF *inputs*, not HyMeKo — hence the separate
`test_fixtures/` for HyMeKo test descriptions).

## Cross-group imports

Almost every dependency points at a root library. Only two examples reach
across groups, and both do it explicitly:

- `tasks/pick_place_scenario` → `../robots/arm_gripper_fanuc_import`
- `tasks/quadruped_stand` → `../robots/quadruped`

## Known problems

These predate the reorganisation and are unchanged by it:

- **`coin/coin_delivery_v1`** imports `meta_evaluation.hymeko`, which does not
  exist anywhere in the repository. The description cannot load.
- **`coin/coin_carry_option_v1`** and **`galambos/galambos_push`** use
  `on <event> to <phase>;` for FSM transitions. The grammar has no rule for it,
  so `on handoff` parses as a statement node and the parser fails on `to`.
  Their own header comments document that syntax as intended, so this reads as
  grammar work that was specified but never landed.
- Ten further descriptions — mostly the `galambos_task*` family, plus
  `pick_place_scenario` and `quadruped_stand` — parse but fail during lowering
  with `UnresolvedRef`. No test loads any of them, which is why it went
  unnoticed.

Compiling all 79: 66 succeed, 13 fail as above.
