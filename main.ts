// Sokoban solver: room-decomposed push-optimal A*.
// Levels from this generator are built as rooms joined to a hallway through
// single articulation cells, and a box can never leave its room in an optimal
// solution, so the level's push optimum is the sum of its rooms' optima.
// This program finds those rooms by articulation peeling (a component cut off
// by one cell whose boxes balance its goals), solves each room push-optimally
// with A* over pushes (matching heuristic on exact push distances, dead-square
// and 2x2 freeze pruning), and stitches the rooms together with walking paths.
// Deterministic: fixed tie-breaking and a fixed expansion budget, no clock.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const I = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const O = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';
const DIRS = [
  { dx: 0, dy: -1, ch: 'u' },
  { dx: 0, dy: 1, ch: 'd' },
  { dx: -1, dy: 0, ch: 'l' },
  { dx: 1, dy: 0, ch: 'r' },
];

function parseLevel(text) {
  const rows = text.replace(/\r/g, '').split('\n');
  while (rows.length && rows[rows.length - 1] === '') rows.pop();
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals = [];
  const boxes = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const ch = x < rows[y].length ? rows[y][x] : '#';
      if (ch === '#') continue;
      const id = y * w + x;
      wall[id] = 0;
      if (ch === '.' || ch === '*' || ch === '+') goals.push(id);
      if (ch === '$' || ch === '*') boxes.push(id);
      if (ch === '@' || ch === '+') player = id;
    }
  }
  return { w, h, wall, goals, boxes, player };
}

// BFS walking paths on cells that are not walls and not blocked by a box.
// Returns a Map from cell -> {from, ch} covering the reachable region.
function walkReach(level, blocked, start) {
  const { w, h, wall } = level;
  const prev = new Map([[start, null]]);
  const queue = [start];
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi];
    const cx = c % w, cy = (c / w) | 0;
    for (const d of DIRS) {
      const nx = cx + d.dx, ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (wall[n] || blocked.has(n) || prev.has(n)) continue;
      prev.set(n, { from: c, ch: d.ch });
      queue.push(n);
    }
  }
  return prev;
}
function pathString(prev, target) {
  if (!prev.has(target)) return null;
  let s = '';
  let c = target;
  while (prev.get(c)) {
    s = prev.get(c).ch + s;
    c = prev.get(c).from;
  }
  return s;
}

// Split the level into rooms by articulation peeling on the non-wall graph.
function peelRooms(level) {
  const { w, h, wall } = level;
  const all = [];
  for (let i = 0; i < w * h; i++) if (!wall[i]) all.push(i);
  const goalSet = new Set(level.goals);
  let remaining = new Set(all);
  let remainingBoxes = new Set(level.boxes);
  const rooms = [];
  const neighbors = (c, scope) => {
    const cx = c % w, cy = (c / w) | 0;
    const out = [];
    for (const d of DIRS) {
      const nx = cx + d.dx, ny = cy + d.dy;
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (scope.has(n)) out.push(n);
    }
    return out;
  };
  for (let iter = 0; iter < 24; iter++) {
    if (remainingBoxes.size === 0) break;
    let best = null;
    for (const a of remaining) {
      // Components of remaining \ {a}.
      const seen = new Set([a]);
      for (const start of remaining) {
        if (seen.has(start)) continue;
        const comp = [];
        const stack = [start];
        seen.add(start);
        let hasPlayer = false;
        while (stack.length) {
          const c = stack.pop();
          comp.push(c);
          if (c === level.player) hasPlayer = true;
          for (const n of neighbors(c, remaining)) {
            if (n !== a && !seen.has(n)) { seen.add(n); stack.push(n); }
          }
        }
        if (hasPlayer) continue;
        let nb = 0, ng = 0;
        for (const c of comp) {
          if (remainingBoxes.has(c)) nb++;
          if (goalSet.has(c)) ng++;
        }
        if (remainingBoxes.has(a)) nb++;
        if (goalSet.has(a)) ng++;
        if (nb === 0 || nb !== ng) continue;
        const cand = { size: comp.length, door: a, cells: comp, nb };
        if (!best || cand.size < best.size ||
            (cand.size === best.size && (cand.door < best.door ||
             (cand.door === best.door && Math.min(...cand.cells) < Math.min(...best.cells))))) {
          best = cand;
        }
      }
    }
    if (!best) break;
    const cellSet = new Set(best.cells);
    const rboxes = level.boxes.filter((b) => cellSet.has(b) || b === best.door);
    const rgoals = level.goals.filter((g) => cellSet.has(g) || g === best.door);
    rooms.push({ cells: cellSet, door: best.door, boxes: rboxes, goals: rgoals });
    for (const c of best.cells) remaining.delete(c);
    for (const b of rboxes) remainingBoxes.delete(b);
  }
  if (remainingBoxes.size > 0) {
    const cellSet = new Set(remaining);
    rooms.push({
      cells: cellSet,
      door: level.player,
      boxes: level.boxes.filter((b) => remainingBoxes.has(b)),
      goals: level.goals.filter((g) => cellSet.has(g)),
    });
  }
  return rooms;
}

// Exact push distances from every cell to each goal, ignoring other boxes:
// reverse BFS over pulls inside the allowed region.
function pushDistances(level, allowed, goal) {
  const { w, h, wall } = level;
  const dist = new Map([[goal, 0]]);
  const queue = [goal];
  const free = (c) => allowed.has(c) && !wall[c];
  for (let qi = 0; qi < queue.length; qi++) {
    const c = queue[qi];
    const cx = c % w, cy = (c / w) | 0;
    for (const d of DIRS) {
      // Forward push moved a box from p to c along d, player standing at p - d.
      const px = cx - d.dx, py = cy - d.dy;
      const sx = px - d.dx, sy = py - d.dy;
      if (px < 0 || py < 0 || sx < 0 || sy < 0 || px >= w || py >= h || sx >= w || sy >= h) continue;
      const p = py * w + px, s = sy * w + sx;
      if (!free(p) || !free(s) || dist.has(p)) continue;
      dist.set(p, dist.get(c) + 1);
      queue.push(p);
    }
  }
  return dist;
}

// Minimum-cost box->goal assignment (DP over subsets), cost from pushDistances.
function matchingHeuristic(boxes, dists) {
  const n = boxes.length;
  const cost = boxes.map((b) => dists.map((dm) => (dm.has(b) ? dm.get(b) : Infinity)));
  for (let i = 0; i < n; i++) {
    let any = false;
    for (let j = 0; j < n; j++) if (cost[i][j] !== Infinity) { any = true; break; }
    if (!any) return Infinity;
  }
  const dp = new Array(1 << n).fill(Infinity);
  dp[0] = 0;
  for (let mask = 0; mask < (1 << n); mask++) {
    const i = mask.toString(2).split('1').length - 1; // boxes assigned so far
    if (i >= n || dp[mask] === Infinity) continue;
    for (let j = 0; j < n; j++) {
      if (mask & (1 << j)) continue;
      const v = dp[mask] + cost[i][j];
      if (v < dp[mask | (1 << j)]) dp[mask | (1 << j)] = v;
    }
  }
  return dp[(1 << n) - 1];
}

// Push-optimal A* for one room. Returns {moves, endPlayer, endBoxes} or null.
function solveRoom(level, allowed, boxes0, goals, startPlayer, extraWalls) {
  const { w, h, wall } = level;
  const goalSet = new Set(goals);
  const dists = goals.map((g) => pushDistances(level, allowed, g));
  const dead = new Set();
  for (const c of allowed) {
    if (goalSet.has(c)) continue;
    if (dists.every((dm) => !dm.has(c))) dead.add(c);
  }
  for (const b of boxes0) if (dead.has(b)) return null;
  const h0 = matchingHeuristic(boxes0, dists);
  if (h0 === Infinity) return null;
  const blockedStatic = extraWalls ?? new Set();
  const isFree = (c) => allowed.has(c) && !wall[c] && !blockedStatic.has(c);

  const keyOf = (boxes, playerRep) => boxes.join(',') + '|' + playerRep;
  const heap = [];
  let counter = 0;
  const pushHeap = (node) => {
    heap.push(node);
    let i = heap.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (heap[p].f < heap[i].f || (heap[p].f === heap[i].f && heap[p].h <= heap[i].h)) break;
      [heap[p], heap[i]] = [heap[i], heap[p]];
      i = p;
    }
  };
  const popHeap = () => {
    const top = heap[0];
    const last = heap.pop();
    if (heap.length) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let m = i;
        if (l < heap.length && (heap[l].f < heap[m].f || (heap[l].f === heap[m].f && heap[l].h < heap[m].h))) m = l;
        if (r < heap.length && (heap[r].f < heap[m].f || (heap[r].f === heap[m].f && heap[r].h < heap[m].h))) m = r;
        if (m === i) break;
        [heap[m], heap[i]] = [heap[i], heap[m]];
        i = m;
      }
    }
    return top;
  };

  const startBoxes = [...boxes0].sort((a, b) => a - b);
  const startReach = walkReach(level, new Set(startBoxes.concat([...blockedStatic])), startPlayer);
  let startRep = Infinity;
  for (const c of startReach.keys()) if (c < startRep) startRep = c;
  const seen = new Map([[keyOf(startBoxes, startRep), 0]]);
  pushHeap({ boxes: startBoxes, player: startPlayer, g: 0, h: h0, f: h0, parent: null, moves: '', n: counter++ });
  const MAX_EXPAND = 1500000;
  let expanded = 0;
  while (heap.length) {
    const node = popHeap();
    if (node.boxes.every((b) => goalSet.has(b))) {
      // Reconstruct moves.
      const parts = [];
      for (let nd = node; nd.parent; nd = nd.parent) parts.push(nd.moves);
      parts.reverse();
      return { moves: parts.join(''), endPlayer: node.player, endBoxes: node.boxes };
    }
    if (++expanded > MAX_EXPAND) return null;
    const boxSet = new Set(node.boxes);
    const reach = walkReach(level, new Set([...boxSet, ...blockedStatic]), node.player);
    for (let bi = 0; bi < node.boxes.length; bi++) {
      const b = node.boxes[bi];
      const bx = b % w, by = (b / w) | 0;
      for (const d of DIRS) {
        const sx = bx - d.dx, sy = by - d.dy;
        const tx = bx + d.dx, ty = by + d.dy;
        if (sx < 0 || sy < 0 || tx < 0 || ty < 0 || sx >= w || sy >= h || tx >= w || ty >= h) continue;
        const stand = sy * w + sx, target = ty * w + tx;
        if (!reach.has(stand) || !isFree(target) || boxSet.has(target)) continue;
        if (dead.has(target)) continue;
        const nb = [...node.boxes];
        nb[bi] = target;
        nb.sort((a, b2) => a - b2);
        // 2x2 freeze deadlock: a 2x2 block of walls/boxes holding a box off goal.
        let frozen = false;
        const nbSet = new Set(nb);
        for (const c of nb) {
          if (goalSet.has(c)) continue;
          const cx = c % w, cy = (c / w) | 0;
          for (const [ox, oy] of [[0, 0], [-1, 0], [0, -1], [-1, -1]]) {
            const cells = [];
            let solid = true;
            for (const [dx2, dy2] of [[0, 0], [1, 0], [0, 1], [1, 1]]) {
              const x2 = cx + ox + dx2, y2 = cy + oy + dy2;
              if (x2 < 0 || y2 < 0 || x2 >= w || y2 >= h) { solid = false; break; }
              const cc = y2 * w + x2;
              cells.push(cc);
              if (!wall[cc] && !nbSet.has(cc)) { solid = false; break; }
            }
            if (solid && cells.some((cc) => nbSet.has(cc) && !goalSet.has(cc))) { frozen = true; break; }
          }
          if (frozen) break;
        }
        if (frozen) continue;
        const hh = matchingHeuristic(nb, dists);
        if (hh === Infinity) continue;
        const walk = pathString(reach, stand);
        if (walk === null) continue;
        // Canonical player region after the push: player stands on the old box cell.
        const nreach = walkReach(level, new Set([...nbSet, ...blockedStatic]), b);
        let rep = Infinity;
        for (const c of nreach.keys()) if (c < rep) rep = c;
        const key = keyOf(nb, rep);
        const g = node.g + 1;
        if (seen.has(key) && seen.get(key) <= g) continue;
        seen.set(key, g);
        pushHeap({ boxes: nb, player: b, g, h: hh, f: g + hh, parent: node, moves: walk + d.ch, n: counter++ });
      }
    }
  }
  return null;
}

function solveLevel(text) {
  const level = parseLevel(text);
  if (level.player < 0 || level.boxes.length === 0) return '';
  const rooms = peelRooms(level);
  const curBoxes = new Set(level.boxes);
  let player = level.player;
  let out = '';
  for (const room of rooms) {
    if (room.boxes.length === 0) continue;
    // Travel to the room's entry: the door cell itself, or a stand cell next
    // to a box that starts on the door.
    let startPlayer;
    if (!curBoxes.has(room.door)) {
      const reach = walkReach(level, curBoxes, player);
      const p = pathString(reach, room.door);
      if (p === null) return out;
      out += p;
      player = room.door;
      startPlayer = player;
    } else {
      let ok = false;
      for (const d of DIRS) {
        const dw = level.w;
        const tx = (room.door % dw) + d.dx, ty = ((room.door / dw) | 0) + d.dy;
        const sx = (room.door % dw) - d.dx, sy = ((room.door / dw) | 0) - d.dy;
        if (tx < 0 || ty < 0 || sx < 0 || sy < 0 || tx >= dw || ty >= level.h || sx >= dw || sy >= level.h) continue;
        const target = ty * dw + tx, stand = sy * dw + sx;
        if (!room.cells.has(target) || curBoxes.has(target) || curBoxes.has(stand) || level.wall[stand]) continue;
        const reach = walkReach(level, curBoxes, player);
        const p = pathString(reach, stand);
        if (p === null) continue;
        out += p;
        player = stand;
        startPlayer = stand;
        ok = true;
        break;
      }
      if (!ok) return out;
    }
    const allowed = new Set([...room.cells, room.door, startPlayer]);
    const roomBoxes = room.boxes.filter((b) => curBoxes.has(b));
    let res = solveRoom(level, allowed, roomBoxes, room.goals, startPlayer, null);
    if (!res) {
      // Fallback: search the whole floor, other rooms' boxes as fixed walls.
      const allCells = new Set();
      for (let i = 0; i < level.w * level.h; i++) if (!level.wall[i]) allCells.add(i);
      const others = new Set([...curBoxes].filter((b) => !roomBoxes.includes(b)));
      res = solveRoom(level, allCells, roomBoxes, room.goals, startPlayer, others);
    }
    if (!res) return out;
    out += res.moves;
    player = res.endPlayer;
    for (const b of roomBoxes) curBoxes.delete(b);
    for (const b of res.endBoxes) curBoxes.add(b);
  }
  return out;
}

for (const id of readdirSync(I).sort()) {
  const moves = solveLevel(readFileSync(join(I, id, 'level.txt'), 'utf8'));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, 'solution.txt'), `${moves}\n`);
}
