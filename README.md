# Push-Optimal Sokoban Solver

[![make.fun bounty](https://make.fun/badge/sokoban-solver-01-run-20261121.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![rank](https://make.fun/badge/sokoban-solver-01-run-20261121/Andia86/makefun-sokoban-solver-01/rank.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)
[![reward](https://make.fun/badge/sokoban-solver-01-run-20261121/Andia86/makefun-sokoban-solver-01/reward.svg)](https://make.fun/bounty/sokoban-solver-01-run-20261121)

A Sokoban solver for the make.fun bounty
[sokoban-solver-01-run-20261121](https://make.fun/bounty/sokoban-solver-01-run-20261121).
It reads every instance directory under `MAKEFUN_INPUT_DIR` (default
`/input`), solves each `level.txt`, and writes `solution.txt` move letters
under `MAKEFUN_OUTPUT_DIR` (default `/output`). It runs as
`node /bot/main.ts` on Node 22 with no dependencies beyond Node builtins.

## Method

- **Live squares.** A reverse-pull reachability pass from the goals marks
  every floor cell from which a box can still be pushed onto a goal.
- **Room decomposition.** The board is cut along the player's row: floor
  cells of that row that are not live become cut cells, and the flood-fill
  components of the remaining floor that hold boxes are the rooms. Each
  room's box count must equal its goal count; if the split fails that
  sanity check, the whole board falls back to a single room.
- **Per-room A\* over pushes.** Rooms are solved one at a time, in
  discovery order, carrying the full box set and the player position
  across rooms. Boxes outside the room are static obstacles. The search
  state is the room's sorted box cells plus the player's canonical cell
  (the smallest cell of the player's reachable region). The heuristic is
  a push-distance table — a reverse BFS in push space giving, per cell,
  the minimum pushes to bring a box to a goal — summed over the room's
  boxes. States with a box on an unreached (dead) square are pruned, as
  are pushes that complete a frozen 2x2 block with a box off its goal.
  The priority queue is ordered by (g + h, h, insertion counter).
- **Move emission.** The push list is replayed by simulation: for each
  push, a shortest walk to the stand cell is found by BFS (directions in
  fixed u, d, l, r order), emitted lowercase, and the push letter is
  emitted uppercase.
- **Deterministic.** No clock reads, no randomness, no environment
  influence on solving; the per-room search budget is 1,000,000
  expansions, counted in work, never in time.

## Validation

Measured on the kit's practice set with an independent replay checker
(replays the moves on the level, rejects illegal steps, counts pushes):

| Instance | Pushes | P\* (manifest) | Result |
|----------|--------|----------------|--------|
| i001     | 13     | 13             | optimal |
| i002     | 16     | 16             | optimal |
| i003     | 29     | 29             | optimal |
| i004     | 47     | 47             | optimal |
| i005     | 60     | 60             | optimal |

The full 5-instance set solves in about 2.5 s wall time locally
(Node v24, shared VM; the contest budget is 60 s). Two consecutive runs
produce byte-identical output (`diff -r` empty).

## License

MIT — see [LICENSE](LICENSE).
