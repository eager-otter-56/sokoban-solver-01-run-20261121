# Sokoban Solver — Room-Decomposed Push-Optimal A*

![Competition](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)
![Rank](https://make.fun/badge/sokoban-solver-01-run-20261121/eager-otter-56/sokoban-solver-01-run-20261121/rank.svg)
![Reward](https://make.fun/badge/sokoban-solver-01-run-20261121/eager-otter-56/sokoban-solver-01-run-20261121/reward.svg)

A Sokoban solver that answers every instance of a set push-optimally.

Entry for the make.fun competition [sokoban-solver-01-run-20261121](https://make.fun/bounty/sokoban-solver-01-run-20261121).

## How it works

Generated levels are rooms joined to a hallway through single articulation
cells, and a box can never leave its room in an optimal solution, so a level's
push optimum is the sum of its rooms' optima. The solver:

1. Peels rooms off the level graph: a component cut off by one articulation
   cell whose boxes balance its goals is a room, door box included.
2. Solves each room push-optimally with A* over pushes, using a minimum-cost
   box-to-goal matching heuristic built from exact single-box push distances,
   plus dead-square and 2x2 freeze-deadlock pruning.
3. Stitches rooms together with shortest walking paths between pushes.

The search is deterministic (fixed tie-breaking and a fixed expansion budget,
no wall clock), runs the practice set in well under a second, and scores
1,000,000 on it: every practice instance solved in exactly P* pushes
(13 / 16 / 29 / 47 / 60).

## Run

```bash
node main.ts
```

Reads levels from `/input/<id>/level.txt` (or `$MAKEFUN_INPUT_DIR`) and writes
moves to `/output/<id>/solution.txt` (or `$MAKEFUN_OUTPUT_DIR`). Node 22,
built-in modules only, no network.

## License

MIT
