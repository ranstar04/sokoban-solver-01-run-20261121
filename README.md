# Sokoban Solver — Push-Optimal, Room by Room

[![make.fun · sokoban-solver-01-run-20261121](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/ranstar04/sokoban-solver-01-run-20261121/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121/leaderboard?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/ranstar04/sokoban-solver-01-run-20261121/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121?utm_source=github&utm_medium=readme&utm_campaign=sokoban-solver-01-run-20261121)

Entry for the make.fun bounty [sokoban-solver-01-run-20261121](https://make.fun/bounty/sokoban-solver-01-run-20261121): write one program that answers every Sokoban level of a drawn set in as few pushes as possible.

Play make.fun bounties yourself with my referral: https://make.fun/r/dn9c31df

## How it works

A generated level is a one-wide hallway with one to four small rooms off it,
each room holding 3–4 boxes and as many goals behind a single door. A box
pushed out of its room can never come back, so every room is an independent
push-optimal subproblem and the level optimum is the sum of the room optima.

`main.ts` (Node 22, standard library only, no network, fully deterministic):

- splits a level into hallway plus rooms: the player's row is the hallway,
  each room is a floor component off that row, entered through its one door;
- solves each room with A* over pushes, not steps — a state is the sorted
  box cells plus the player's reachable region, and the heuristic is the
  cheapest box-to-goal matching on shortest-path distances;
- prunes with dead squares found by pull-reachability from the goals, plus
  corner and 2×2 deadlocks, so the search stays push-optimal;
- stitches the rooms together with breadth-first walks along the hallway,
  and falls back to the same A* engine over the whole board when a level
  does not decompose. Every answer is replay-verified before it is written.

## Results

- Practice set (kit): **1,000,000** — all five levels solved in exactly P\*
  pushes (13, 16, 29, 47, 60), in about 0.4 s for the whole set.
- Two freshly drawn sets from the kit generator (other drand rounds):
  **1,000,000** each, about 0.6 s per set — the room decomposition holds on
  unseen levels, not just the practice set.
- House baseline `plain-astar 1.0.0` on the practice set: 200,000 (it only
  solves the single-room level and leaves the rest empty).
- Two runs on the same input write byte-identical output.

## Run it

```sh
MAKEFUN_INPUT_DIR=practice/input MAKEFUN_OUTPUT_DIR=out node main.ts
```

`main.ts` reads `level.txt` for every instance under `MAKEFUN_INPUT_DIR`
(default `/input`) and writes `solution.txt` for each under
`MAKEFUN_OUTPUT_DIR` (default `/output`).
