// Push-optimal Sokoban solver — a make.fun bounty entry.
//
// Method: decompose the level into rooms, then solve each room with an
// A* search over pushes.
//
//   * Live squares are found by reverse-pull reachability from the goals.
//   * The player's own row is cut at its non-live floor cells; flood-fill
//     components of the remaining floor that hold boxes are the rooms.
//   * Each room is solved independently with A* over pushes, using a
//     push-distance table as the heuristic, dead-square pruning and a
//     2x2 freeze prune. Boxes outside the room are static obstacles.
//   * The push list is turned into player moves by shortest-walk BFS
//     between pushes.
//
// The search is fully deterministic: no clock, no randomness, and the
// per-room budget is counted in expansions, never in time.

import { readdirSync, readFileSync, mkdirSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

// Directions, in the fixed order used everywhere: up, down, left, right.
const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

const ROOM_EXPANSION_BUDGET = 1000000;

// ---------------------------------------------------------------- parsing

// Parse an XSB level. Cells are linear indices i = y * w + x. Short rows
// are padded with walls and the outer frame is wall regardless of the
// characters found there.
function parseLevel(text) {
  const lines = text.split(/\r?\n/);
  while (lines.length > 0 && lines[lines.length - 1] === '') lines.pop();
  const h = lines.length;
  let w = 0;
  for (const ln of lines) w = Math.max(w, ln.length);
  const n = w * h;
  const wall = new Array(n).fill(false);
  const goal = new Array(n).fill(false);
  const boxes = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    const ln = lines[y];
    for (let x = 0; x < w; x++) {
      const c = x < ln.length ? ln[x] : '#';
      const i = y * w + x;
      if (c === '#') wall[i] = true;
      else if (c === '.') goal[i] = true;
      else if (c === '$') boxes.push(i);
      else if (c === '*') { goal[i] = true; boxes.push(i); }
      else if (c === '@') player = i;
      else if (c === '+') { goal[i] = true; player = i; }
    }
  }
  for (let x = 0; x < w; x++) { wall[x] = true; wall[(h - 1) * w + x] = true; }
  for (let y = 0; y < h; y++) { wall[y * w] = true; wall[y * w + w - 1] = true; }
  boxes.sort((a, b) => a - b);
  const goals = [];
  for (let i = 0; i < n; i++) if (goal[i]) goals.push(i);
  return { w, h, wall, goal, goals, boxes, player };
}

// ------------------------------------------------------- live squares

// A floor cell is live when a box on it can be pushed onto a goal:
// reverse BFS in push space starting from the goals. A cell y joins the
// set when, for some direction d, y + d is live and y - d (the cell the
// player stands on to push from y to y + d) is floor.
function computeLive(L) {
  const { w, h, wall, goal } = L;
  const live = new Array(w * h).fill(false);
  const q = [];
  for (let i = 0; i < w * h; i++) if (goal[i]) { live[i] = true; q.push(i); }
  let head = 0;
  while (head < q.length) {
    const z = q[head++];
    const zx = z % w, zy = (z / w) | 0;
    for (const d of DIRS) {
      const yx = zx - d.dx, yy = zy - d.dy;
      const sx = yx - d.dx, sy = yy - d.dy;
      if (yx < 0 || yy < 0 || yx >= w || yy >= h) continue;
      if (sx < 0 || sy < 0 || sx >= w || sy >= h) continue;
      const y = yy * w + yx, s = sy * w + sx;
      if (wall[y] || wall[s]) continue;
      if (!live[y]) { live[y] = true; q.push(y); }
    }
  }
  return live;
}

// ------------------------------------------------------- room splitting

// Cut the board along the player's row: floor cells of that row that are
// not live become cut cells. Flood-fill components of the remaining
// floor, in increasing cell order; components holding at least one box
// are rooms. Returns null when the split fails its sanity checks (no
// rooms, a room whose box count differs from its goal count, or a box
// left in no room), in which case the caller treats the whole board as
// a single room.
function segmentRooms(L, live) {
  const { w, h, wall, goal, boxes, player } = L;
  const prow = (player / w) | 0;
  const cut = new Array(w * h).fill(false);
  for (let x = 0; x < w; x++) {
    const i = prow * w + x;
    if (!wall[i] && !live[i]) cut[i] = true;
  }
  const comp = new Int32Array(w * h).fill(-1);
  const rooms = [];
  const boxSet = new Set(boxes);
  let compId = 0;
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || cut[i] || comp[i] >= 0) continue;
    const cells = [];
    const q = [i];
    comp[i] = compId;
    let head = 0;
    while (head < q.length) {
      const c = q[head++];
      cells.push(c);
      const cx = c % w, cy = (c / w) | 0;
      for (const d of DIRS) {
        const nx = cx + d.dx, ny = cy + d.dy;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const m = ny * w + nx;
        if (wall[m] || cut[m] || comp[m] >= 0) continue;
        comp[m] = compId;
        q.push(m);
      }
    }
    compId++;
    const rb = cells.filter((c) => boxSet.has(c)).sort((a, b) => a - b);
    if (rb.length === 0) continue;
    const rg = cells.filter((c) => goal[c]);
    rooms.push({ boxes: rb, goals: rg });
  }
  if (rooms.length === 0) return null;
  let total = 0;
  for (const r of rooms) {
    if (r.boxes.length !== r.goals.length) return null;
    total += r.boxes.length;
  }
  if (total !== boxes.length) return null;
  return rooms;
}

// ------------------------------------------------------- push distances

// Minimum pushes to bring a box from each cell to some goal of the room,
// ignoring the other movable boxes: reverse BFS in push space over cells
// that are floor and not static obstacles. Unreached cells stay -1 and
// act as Infinity (dead squares for movable boxes).
function pushDistances(L, staticSet, goals) {
  const { w, h, wall } = L;
  const dist = new Int32Array(w * h).fill(-1);
  const q = [];
  for (const g of goals) { dist[g] = 0; q.push(g); }
  let head = 0;
  while (head < q.length) {
    const x = q[head++];
    const k = dist[x];
    const xx = x % w, xy = (x / w) | 0;
    for (const d of DIRS) {
      const yx = xx - d.dx, yy = xy - d.dy;
      const sx = yx - d.dx, sy = yy - d.dy;
      if (yx < 0 || yy < 0 || yx >= w || yy >= h) continue;
      if (sx < 0 || sy < 0 || sx >= w || sy >= h) continue;
      const y = yy * w + yx, s = sy * w + sx;
      if (wall[y] || staticSet.has(y)) continue;
      if (wall[s] || staticSet.has(s)) continue;
      if (dist[y] < 0) { dist[y] = k + 1; q.push(y); }
    }
  }
  return dist;
}

// ------------------------------------------------------- player reach

// Flood fill of the cells the player can walk to from `start`, avoiding
// walls and the blocked set (static obstacles plus boxes). Returns the
// reachable mask and the canonical player cell: the smallest index in
// the region.
function playerRegion(L, blocked, start) {
  const { w, h, wall } = L;
  const seen = new Array(w * h).fill(false);
  const q = [start];
  seen[start] = true;
  let head = 0;
  let canon = start;
  while (head < q.length) {
    const c = q[head++];
    if (c < canon) canon = c;
    const cx = c % w, cy = (c / w) | 0;
    for (const d of DIRS) {
      const nx = cx + d.dx, ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const m = ny * w + nx;
      if (seen[m] || wall[m] || blocked.has(m)) continue;
      seen[m] = true;
      q.push(m);
    }
  }
  return { seen, canon };
}

// ------------------------------------------------------- 2x2 freeze

// True when the push that put a box on `target` completes a frozen 2x2:
// some 2x2 block containing target whose four cells are all wall, static
// obstacle or box, with at least one of its boxes not on a goal.
function freezePrune(L, staticSet, boxSet, target) {
  const { w, h, wall, goal } = L;
  const tx = target % w, ty = (target / w) | 0;
  const corners = [
    [tx - 1, ty - 1], [tx, ty - 1], [tx - 1, ty], [tx, ty],
  ];
  for (const [ox, oy] of corners) {
    if (ox < 0 || oy < 0 || ox + 1 >= w || oy + 1 >= h) continue;
    const cells = [oy * w + ox, oy * w + ox + 1, (oy + 1) * w + ox, (oy + 1) * w + ox + 1];
    let allBlocked = true;
    let anyBoxOffGoal = false;
    for (const c of cells) {
      const isBox = boxSet.has(c) || staticSet.has(c);
      if (!wall[c] && !isBox) { allBlocked = false; break; }
      if (isBox && !goal[c]) anyBoxOffGoal = true;
    }
    if (allBlocked && anyBoxOffGoal) return true;
  }
  return false;
}

// ------------------------------------------------------- binary heap

// Min-heap of entries ordered by (f, then h, then insertion counter).
function heapLess(a, b) {
  if (a.f !== b.f) return a.f < b.f;
  if (a.h !== b.h) return a.h < b.h;
  return a.c < b.c;
}
function heapPush(heap, e) {
  heap.push(e);
  let i = heap.length - 1;
  while (i > 0) {
    const p = (i - 1) >> 1;
    if (!heapLess(heap[i], heap[p])) break;
    const t = heap[i]; heap[i] = heap[p]; heap[p] = t;
    i = p;
  }
}
function heapPop(heap) {
  const top = heap[0];
  const last = heap.pop();
  if (heap.length > 0) {
    heap[0] = last;
    let i = 0;
    for (;;) {
      const l = 2 * i + 1, r = 2 * i + 2;
      let m = i;
      if (l < heap.length && heapLess(heap[l], heap[m])) m = l;
      if (r < heap.length && heapLess(heap[r], heap[m])) m = r;
      if (m === i) break;
      const t = heap[i]; heap[i] = heap[m]; heap[m] = t;
      i = m;
    }
  }
  return top;
}

// ------------------------------------------------------- room solver

// A* over pushes for one room. Movable boxes start at room.boxes; static
// obstacles are walls plus staticSet. Returns the push list
// [{ from, dir }] in order, or null when the room cannot be solved
// within the expansion budget.
function solveRoom(L, room, staticSet, startPlayer) {
  const { w, wall, goal } = L;
  const dist = pushDistances(L, staticSet, room.goals);
  const goalSet = new Set(room.goals);
  const startBoxes = room.boxes.slice();

  const heuristic = (bx) => {
    let s = 0;
    for (const b of bx) {
      const d = dist[b];
      if (d < 0) return Infinity;
      s += d;
    }
    return s;
  };
  const solved = (bx) => bx.every((b) => goalSet.has(b));
  const keyOf = (bx, canon) => bx.join(',') + '|' + canon;

  const startBlocked = new Set(staticSet);
  for (const b of startBoxes) startBlocked.add(b);
  const startRegion = playerRegion(L, startBlocked, startPlayer);
  const startH = heuristic(startBoxes);
  if (startH === Infinity) return null;

  // State store; ids are indices into these parallel arrays.
  const sBoxes = [startBoxes];
  const sCanon = [startRegion.canon];
  const sG = [0];
  const sParent = [-1];
  const sFrom = [-1];
  const sDir = [-1];
  const best = new Map();
  best.set(keyOf(startBoxes, startRegion.canon), 0);
  const heap = [];
  let counter = 0;
  heapPush(heap, { f: startH, h: startH, c: counter++, id: 0 });

  let expansions = 0;
  while (heap.length > 0) {
    const cur = heapPop(heap);
    const id = cur.id;
    const bx = sBoxes[id];
    if (sG[id] > (best.get(keyOf(bx, sCanon[id])) ?? Infinity)) continue;
    if (solved(bx)) {
      const pushes = [];
      for (let s = id; sParent[s] >= 0; s = sParent[s]) {
        pushes.push({ from: sFrom[s], dir: sDir[s] });
      }
      pushes.reverse();
      return pushes;
    }
    if (expansions >= ROOM_EXPANSION_BUDGET) return null;
    expansions++;

    const boxSet = new Set(bx);
    const blocked = new Set(staticSet);
    for (const b of bx) blocked.add(b);
    const region = playerRegion(L, blocked, sCanon[id]);

    for (let bi = 0; bi < bx.length; bi++) {
      const b = bx[bi];
      const bxX = b % w, bxY = (b / w) | 0;
      for (let di = 0; di < 4; di++) {
        const d = DIRS[di];
        const sx = bxX - d.dx, sy = bxY - d.dy;
        const tx = bxX + d.dx, ty = bxY + d.dy;
        if (sx < 0 || sy < 0 || sx >= w || sy >= L.h) continue;
        if (tx < 0 || ty < 0 || tx >= w || ty >= L.h) continue;
        const stand = sy * w + sx;
        const target = ty * w + tx;
        if (!region.seen[stand]) continue;
        if (wall[target] || staticSet.has(target) || boxSet.has(target)) continue;
        if (dist[target] < 0) continue;
        const nb = bx.slice();
        nb[bi] = target;
        nb.sort((a, b2) => a - b2);
        const nbSet = new Set(nb);
        if (freezePrune(L, staticSet, nbSet, target)) continue;
        const nh = heuristic(nb);
        if (nh === Infinity) continue;
        const ng = sG[id] + 1;
        const nBlocked = new Set(staticSet);
        for (const b2 of nb) nBlocked.add(b2);
        // After the push the player stands on the box's old cell.
        const nRegion = playerRegion(L, nBlocked, b);
        const k = keyOf(nb, nRegion.canon);
        if ((best.get(k) ?? Infinity) <= ng) continue;
        best.set(k, ng);
        const nid = sBoxes.length;
        sBoxes.push(nb);
        sCanon.push(nRegion.canon);
        sG.push(ng);
        sParent.push(id);
        sFrom.push(b);
        sDir.push(di);
        heapPush(heap, { f: ng + nh, h: nh, c: counter++, id: nid });
      }
    }
  }
  return null;
}

// ------------------------------------------------------- move emission

// Shortest walk from `from` to `to` avoiding walls and every box in
// boxSet, exploring directions in the fixed u, d, l, r order. Returns
// the walk letters (lowercase).
function walkPath(L, boxSet, from, to) {
  if (from === to) return '';
  const { w, h, wall } = L;
  const prev = new Int32Array(w * h).fill(-2);
  const prevDir = new Int32Array(w * h).fill(-1);
  const q = [from];
  prev[from] = -1;
  let head = 0;
  while (head < q.length) {
    const c = q[head++];
    if (c === to) break;
    const cx = c % w, cy = (c / w) | 0;
    for (let di = 0; di < 4; di++) {
      const d = DIRS[di];
      const nx = cx + d.dx, ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const m = ny * w + nx;
      if (prev[m] !== -2 || wall[m] || boxSet.has(m)) continue;
      prev[m] = c;
      prevDir[m] = di;
      q.push(m);
    }
  }
  if (prev[to] === -2) return '';
  const letters = [];
  for (let c = to; c !== from; c = prev[c]) letters.push(DIRS[prevDir[c]].ch);
  letters.reverse();
  return letters.join('');
}

// Turn a room's push list into move letters by simulation against the
// full box set (all rooms). Walks are lowercase, pushes UPPERCASE.
// Returns the letters plus the end player cell and box set.
function emitMoves(L, pushes, startPlayer, startBoxes) {
  const { w } = L;
  const boxes = new Set(startBoxes);
  let player = startPlayer;
  let out = '';
  for (const p of pushes) {
    const d = DIRS[p.dir];
    const stand = p.from - (d.dy * w + d.dx);
    out += walkPath(L, boxes, player, stand);
    out += d.ch.toUpperCase();
    boxes.delete(p.from);
    boxes.add(p.from + d.dy * w + d.dx);
    player = p.from;
  }
  return { moves: out, player, boxes };
}

// ------------------------------------------------------- whole instance

function solveInstance(text) {
  const L = parseLevel(text);
  if (L.player < 0 || L.boxes.length === 0) return '';
  const live = computeLive(L);
  let rooms = segmentRooms(L, live);
  if (rooms === null) {
    rooms = [{ boxes: L.boxes.slice(), goals: L.goals.slice() }];
  }
  let allBoxes = new Set(L.boxes);
  let player = L.player;
  let solution = '';
  for (const room of rooms) {
    const roomBoxSet = new Set(room.boxes);
    const staticSet = new Set();
    for (const b of allBoxes) if (!roomBoxSet.has(b)) staticSet.add(b);
    const pushes = solveRoom(L, room, staticSet, player);
    if (pushes === null) break; // room failed: keep what we have
    const emitted = emitMoves(L, pushes, player, allBoxes);
    solution += emitted.moves;
    player = emitted.player;
    allBoxes = emitted.boxes;
  }
  return solution;
}

// ------------------------------------------------------- entry point

function main() {
  const inDir = process.env.MAKEFUN_INPUT_DIR ?? '/input';
  const outDir = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
  let ids = [];
  try {
    ids = readdirSync(inDir, { withFileTypes: true })
      .filter((e) => e.isDirectory())
      .map((e) => e.name)
      .sort();
  } catch {
    ids = [];
  }
  for (const id of ids) {
    let solution = '';
    try {
      const text = readFileSync(join(inDir, id, 'level.txt'), 'utf8');
      solution = solveInstance(text);
    } catch {
      solution = '';
    }
    try {
      const dir = join(outDir, id);
      mkdirSync(dir, { recursive: true });
      writeFileSync(join(dir, 'solution.txt'), solution);
    } catch {
      // Nothing more can be done for this instance.
    }
  }
}

main();
