// Push-optimal Sokoban solver.
//
// Idea: a generated level is a one-wide hallway with one to four small rooms
// off it, each room holding 3-4 boxes and as many goals behind a single door.
// A box pushed out of its room can never come back, so every room is an
// independent push-optimal subproblem and the level optimum is the sum of
// the room optima. Each room is solved with A* over pushes (not steps):
// a state is the sorted box cells plus the player's reachable region, the
// heuristic is the cheapest box-to-goal matching on shortest-path distances,
// and dead squares (found by pull-reachability from the goals), corner and
// 2x2 deadlocks prune the search. Rooms are stitched together with plain
// breadth-first walks along the hallway. A whole-board A* with the same
// engine is the fallback when a level does not decompose. Everything is
// deterministic: fixed neighbour order, count-based expansion caps, no clock,
// no randomness, so two runs write the same bytes.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from 'node:fs';
import { join } from 'node:path';

const INPUT = process.env.MAKEFUN_INPUT_DIR ?? '/input';
const OUTPUT = process.env.MAKEFUN_OUTPUT_DIR ?? '/output';

// Directions in fixed order: up, down, left, right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ['u', 'd', 'l', 'r'];

interface Parsed {
  w: number;
  h: number;
  wall: Uint8Array;
  goals: number[];
  boxes: number[];
  player: number;
}

function parseLevel(text: string): Parsed {
  const rows = text.split('\n').filter((r) => r.length > 0);
  const h = rows.length;
  const w = Math.max(...rows.map((r) => r.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goals: number[] = [];
  const boxes: number[] = [];
  let player = -1;
  rows.forEach((row, y) => {
    for (let x = 0; x < row.length; x++) {
      const i = y * w + x;
      const c = row[x];
      if (c === '#') continue;
      wall[i] = 0;
      if (c === '.' || c === '*' || c === '+') goals.push(i);
      if (c === '$' || c === '*') boxes.push(i);
      if (c === '@' || c === '+') player = i;
    }
  });
  boxes.sort((a, b) => a - b);
  goals.sort((a, b) => a - b);
  return { w, h, wall, goals, boxes, player };
}

// Breadth-first distances from `from` over passable cells (wall[i] === 0).
function bfsDist(from: number, pass: Uint8Array, w: number, h: number): Int32Array {
  const dist = new Int32Array(w * h).fill(-1);
  const q = [from];
  dist[from] = 0;
  for (let head = 0; head < q.length; head++) {
    const c = q[head];
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (pass[n] === 0 || dist[n] !== -1) continue;
      dist[n] = dist[c] + 1;
      q.push(n);
    }
  }
  return dist;
}

// Shortest walk from `from` to `to` over passable cells avoiding `blocked`
// (a Uint8Array mask), as a move string, or null when unreachable.
function walkPath(
  from: number, to: number, pass: Uint8Array, blocked: Uint8Array, w: number, h: number,
): string | null {
  if (from === to) return '';
  const prev = new Int32Array(w * h).fill(-1);
  const prevCh = new Array<string>(w * h).fill('');
  const q = [from];
  prev[from] = from;
  for (let head = 0; head < q.length; head++) {
    const c = q[head];
    if (c === to) break;
    const x = c % w;
    const y = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = x + DX[d];
      const ny = y + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (pass[n] === 0 || blocked[n] === 1 || prev[n] !== -1) continue;
      prev[n] = c;
      prevCh[n] = CH[d];
      q.push(n);
    }
  }
  if (prev[to] === -1) return null;
  let out = '';
  let c = to;
  while (c !== from) {
    out = prevCh[c] + out;
    c = prev[c];
  }
  return out;
}

// Cells a box can ever sit on while still reaching a goal: pull-reachability
// from the goals, ignoring other boxes (an over-approximation, so pruning a
// cell outside it is safe). A box on a cell outside this set is dead.
function liveCells(pass: Uint8Array, goals: number[], w: number, h: number): Uint8Array {
  const live = new Uint8Array(w * h);
  const q: number[] = [];
  for (const g of goals) {
    if (!live[g]) {
      live[g] = 1;
      q.push(g);
    }
  }
  for (let head = 0; head < q.length; head++) {
    const b = q[head];
    const x = b % w;
    const y = (b / w) | 0;
    for (let d = 0; d < 4; d++) {
      // Pull the box from b to n: the player stands on n, then steps away
      // to n + dir, and the box follows into n. So n + dir must be floor.
      const nx = x + DX[d];
      const ny = y + DY[d];
      const px = x + 2 * DX[d];
      const py = y + 2 * DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      if (px < 0 || py < 0 || px >= w || py >= h) continue;
      const n = ny * w + nx;
      const p = py * w + px;
      if (pass[n] === 0 || pass[p] === 0) continue;
      if (!live[n]) {
        live[n] = 1;
        q.push(n);
      }
    }
  }
  return live;
}

interface SearchNode {
  boxes: number[];
  player: number;
  g: number;
  h: number;
  parent: number;
  seg: string;
}

// Push-space A*. `pass` marks the floor the player and boxes may use;
// boxes may only ever stand on cells of `pass`. Returns the full move
// string from `startPlayer` that puts every box on a goal in the fewest
// pushes, or null when unsolved within the expansion cap.
function pushAStar(
  pass: Uint8Array, boxes0: number[], goals: number[], startPlayer: number,
  w: number, h: number, maxExpand: number,
): string | null {
  const n = boxes0.length;
  if (n === 0) return '';
  if (n !== goals.length) return null;
  const goalSet = new Uint8Array(w * h);
  for (const g of goals) goalSet[g] = 1;
  const live = liveCells(pass, goals, w, h);
  for (const b of boxes0) if (!live[b]) return null;
  // Shortest-path distances from each goal, ignoring boxes.
  const distFromGoal = goals.map((g) => bfsDist(g, pass, w, h));
  // Cheapest box-to-goal matching (brute-force permutations, n <= 4 here,
  // and small in general); Infinity when some box cannot reach a goal.
  const heuristic = (boxes: number[]): number => {
    let best = Infinity;
    const used = new Array<boolean>(n).fill(false);
    const rec = (i: number, cost: number): void => {
      if (cost >= best) return;
      if (i === n) {
        best = cost;
        return;
      }
      for (let j = 0; j < n; j++) {
        if (used[j]) continue;
        const d = distFromGoal[j][boxes[i]];
        if (d < 0) continue;
        used[j] = true;
        rec(i + 1, cost + d);
        used[j] = false;
      }
    };
    rec(0, 0);
    return best;
  };

  const isSolved = (boxes: number[]): boolean => boxes.every((b) => goalSet[b] === 1);
  // A 2x2 block of walls and boxes holding no goal freezes every box in
  // it: none of them can ever move again. Only blocks holding the box just
  // pushed can newly become dead, so only those are checked.
  const is2x2Dead = (boxes: number[], moved: number): boolean => {
    const set = new Set(boxes);
    const x = moved % w;
    const y = (moved / w) | 0;
    for (let oy = -1; oy <= 0; oy++) {
      for (let ox = -1; ox <= 0; ox++) {
        const cx = x + ox;
        const cy = y + oy;
        if (cx < 0 || cy < 0 || cx + 1 >= w || cy + 1 >= h) continue;
        const cells = [cy * w + cx, cy * w + cx + 1, (cy + 1) * w + cx, (cy + 1) * w + cx + 1];
        if (!cells.includes(moved)) continue;
        // Every cell wall (pass 0) or box, and none a goal.
        if (cells.every((c) => goalSet[c] === 0 && (pass[c] === 0 || set.has(c)))) return true;
      }
    }
    return false;
  };

  // Canonical player: the smallest cell the player can reach from its cell
  // on floor minus boxes. Pushes depend only on boxes plus this region.
  const canonical = (boxes: number[], player: number): number => {
    const blocked = new Uint8Array(w * h);
    for (const b of boxes) blocked[b] = 1;
    const seen = new Uint8Array(w * h);
    const q = [player];
    seen[player] = 1;
    let min = player;
    for (let head = 0; head < q.length; head++) {
      const c = q[head];
      if (c < min) min = c;
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = y + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const nn = ny * w + nx;
        if (pass[nn] === 0 || blocked[nn] === 1 || seen[nn] === 1) continue;
        seen[nn] = 1;
        q.push(nn);
      }
    }
    return min;
  };

  const nodes: SearchNode[] = [];
  const best = new Map<string, number>();
  // Binary heap of node indices ordered by (g+h, h, insertion order).
  const heap: number[] = [];
  const seq: number[] = [];
  let counter = 0;
  const heapLess = (a: number, b: number): boolean => {
    const fa = nodes[a].g + nodes[a].h;
    const fb = nodes[b].g + nodes[b].h;
    if (fa !== fb) return fa < fb;
    if (nodes[a].h !== nodes[b].h) return nodes[a].h < nodes[b].h;
    return seq[a] < seq[b];
  };
  const heapPush = (idx: number): void => {
    heap.push(idx);
    let i = heap.length - 1;
    while (i > 0) {
      const p = ((i - 1) / 2) | 0;
      if (!heapLess(heap[i], heap[p])) break;
      const t = heap[i];
      heap[i] = heap[p];
      heap[p] = t;
      i = p;
    }
  };
  const heapPop = (): number => {
    const top = heap[0];
    const last = heap.pop() as number;
    if (heap.length > 0) {
      heap[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < heap.length && heapLess(heap[l], heap[m])) m = l;
        if (r < heap.length && heapLess(heap[r], heap[m])) m = r;
        if (m === i) break;
        const t = heap[i];
        heap[i] = heap[m];
        heap[m] = t;
        i = m;
      }
    }
    return top;
  };

  const startBoxes = [...boxes0].sort((a, b) => a - b);
  const h0 = heuristic(startBoxes);
  if (!isFinite(h0)) return null;
  nodes.push({ boxes: startBoxes, player: startPlayer, g: 0, h: h0, parent: -1, seg: '' });
  seq.push(counter++);
  best.set(startBoxes.join(',') + '|' + canonical(startBoxes, startPlayer), 0);
  heapPush(0);

  let expanded = 0;
  while (heap.length > 0) {
    const idx = heapPop();
    const node = nodes[idx];
    const key = node.boxes.join(',') + '|' + canonical(node.boxes, node.player);
    if ((best.get(key) ?? Infinity) < node.g) continue;
    if (isSolved(node.boxes)) {
      let out = '';
      let cur = idx;
      const parts: string[] = [];
      while (cur !== -1) {
        parts.push(nodes[cur].seg);
        cur = nodes[cur].parent;
      }
      parts.reverse();
      out = parts.join('');
      return out;
    }
    if (++expanded > maxExpand) return null;
    // Player reachability from this node's player cell, boxes blocked.
    const blocked = new Uint8Array(w * h);
    const boxSet = new Set(node.boxes);
    for (const b of node.boxes) blocked[b] = 1;
    const reach = new Uint8Array(w * h);
    {
      const q = [node.player];
      reach[node.player] = 1;
      for (let head = 0; head < q.length; head++) {
        const c = q[head];
        const x = c % w;
        const y = (c / w) | 0;
        for (let d = 0; d < 4; d++) {
          const nx = x + DX[d];
          const ny = y + DY[d];
          if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
          const nn = ny * w + nx;
          if (pass[nn] === 0 || blocked[nn] === 1 || reach[nn] === 1) continue;
          reach[nn] = 1;
          q.push(nn);
        }
      }
    }
    for (let bi = 0; bi < n; bi++) {
      const b = node.boxes[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const dx = bx + DX[d];
        const dy = by + DY[d];
        const px = bx - DX[d];
        const py = by - DY[d];
        if (dx < 0 || dy < 0 || dx >= w || dy >= h) continue;
        if (px < 0 || py < 0 || px >= w || py >= h) continue;
        const dest = dy * w + dx;
        const behind = py * w + px;
        if (pass[dest] === 0 || boxSet.has(dest)) continue;
        if (pass[behind] === 0 || reach[behind] === 0) continue;
        if (!live[dest] && goalSet[dest] === 0) continue;
        const nextBoxes = [...node.boxes];
        nextBoxes[bi] = dest;
        nextBoxes.sort((a, b2) => a - b2);
        if (is2x2Dead(nextBoxes, dest)) continue;
        const hh = heuristic(nextBoxes);
        if (!isFinite(hh)) continue;
        const g2 = node.g + 1;
        const key2 = nextBoxes.join(',') + '|' + canonical(nextBoxes, b);
        if ((best.get(key2) ?? Infinity) <= g2) continue;
        best.set(key2, g2);
        const walk = walkPath(node.player, behind, pass, blocked, w, h);
        if (walk === null) continue;
        const ni = nodes.length;
        nodes.push({ boxes: nextBoxes, player: b, g: g2, h: hh, parent: idx, seg: walk + CH[d] });
        seq.push(counter++);
        heapPush(ni);
      }
    }
  }
  return null;
}

interface Room {
  cells: number[];
  boxes: number[];
  goals: number[];
  door: number;
  entry: number;
}

// Split a level into hallway plus rooms. The hallway is the player's row;
// each room is a floor component off that row, entered through one door
// cell adjacent to one hallway cell. Returns null when the level does not
// fit that shape.
function decompose(p: Parsed): Room[] | null {
  const { w, h, wall } = p;
  const hallRow = (p.player / w) | 0;
  const seen = new Int32Array(w * h).fill(-1);
  const rooms: Room[] = [];
  const boxRoom = new Int32Array(w * h).fill(-1);
  const goalRoom = new Int32Array(w * h).fill(-1);
  let comp = 0;
  for (let i = 0; i < w * h; i++) {
    if (wall[i] === 1) continue;
    const y = (i / w) | 0;
    if (y === hallRow || seen[i] !== -1) continue;
    const cells: number[] = [];
    const q = [i];
    seen[i] = comp;
    for (let head = 0; head < q.length; head++) {
      const c = q[head];
      cells.push(c);
      const x = c % w;
      const cy = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        if (ny === hallRow) continue;
        const nn = ny * w + nx;
        if (wall[nn] === 1 || seen[nn] !== -1) continue;
        seen[nn] = comp;
        q.push(nn);
      }
    }
    const boxes = p.boxes.filter((b) => seen[b] === comp);
    const goals = p.goals.filter((g) => seen[g] === comp);
    if (boxes.length === 0 && goals.length === 0) {
      comp++;
      continue;
    }
    if (boxes.length === 0 || boxes.length !== goals.length) return null;
    // The door: the one component cell next to a hallway floor cell.
    let door = -1;
    let entry = -1;
    let doors = 0;
    for (const c of cells) {
      const x = c % w;
      const y = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = x + DX[d];
        const ny = y + DY[d];
        if (ny !== hallRow || nx < 0 || nx >= w) continue;
        const n = ny * w + nx;
        if (wall[n] === 1) continue;
        doors++;
        door = c;
        entry = n;
      }
    }
    if (doors !== 1) return null;
    for (const b of boxes) boxRoom[b] = comp;
    for (const g of goals) goalRoom[g] = comp;
    rooms.push({ cells, boxes, goals, door, entry });
    comp++;
  }
  if (rooms.length === 0) return null;
  if (p.boxes.some((b) => boxRoom[b] === -1)) return null;
  if (p.goals.some((g) => goalRoom[g] === -1)) return null;
  rooms.sort((a, b) => a.door - b.door);
  return rooms;
}

// Simulate moves on the parsed level; returns the final boxes and player,
// or null on an illegal step.
function simulate(
  p: Parsed, moves: string,
): { boxes: number[]; player: number } | null {
  const boxSet = new Set(p.boxes);
  let player = p.player;
  for (const ch of moves) {
    const d = CH.indexOf(ch);
    if (d === -1) return null;
    const x = player % p.w;
    const y = (player / p.w) | 0;
    const nx = x + DX[d];
    const ny = y + DY[d];
    if (nx < 0 || ny < 0 || nx >= p.w || ny >= p.h) return null;
    const next = ny * p.w + nx;
    if (p.wall[next] === 1) return null;
    if (boxSet.has(next)) {
      const bx = nx + DX[d];
      const by = ny + DY[d];
      if (bx < 0 || by < 0 || bx >= p.w || by >= p.h) return null;
      const beyond = by * p.w + bx;
      if (p.wall[beyond] === 1 || boxSet.has(beyond)) return null;
      boxSet.delete(next);
      boxSet.add(beyond);
    }
    player = next;
  }
  return { boxes: [...boxSet].sort((a, b) => a - b), player };
}

function solveLevel(text: string): string {
  const p = parseLevel(text);
  const goalSet = new Set(p.goals);
  const solved = (boxes: number[]): boolean => boxes.every((b) => goalSet.has(b));
  if (solved(p.boxes)) return '';
  const rooms = decompose(p);
  if (rooms !== null) {
    let moves = '';
    let curBoxes = [...p.boxes];
    let curPlayer = p.player;
    let ok = true;
    for (const room of rooms) {
      // Walk to the hallway cell outside this room's door.
      const passAll = new Uint8Array(p.w * p.h);
      for (let i = 0; i < p.w * p.h; i++) passAll[i] = p.wall[i] === 0 ? 1 : 0;
      const blocked = new Uint8Array(p.w * p.h);
      for (const b of curBoxes) blocked[b] = 1;
      const walk = walkPath(curPlayer, room.entry, passAll, blocked, p.w, p.h);
      if (walk === null) {
        ok = false;
        break;
      }
      const sim1 = simulate({ ...p, boxes: curBoxes, player: curPlayer }, walk);
      if (sim1 === null) {
        ok = false;
        break;
      }
      moves += walk;
      curBoxes = sim1.boxes;
      curPlayer = sim1.player;
      // Solve this room alone: its floor plus the entry cell to stand on.
      const passRoom = new Uint8Array(p.w * p.h);
      for (const c of room.cells) passRoom[c] = 1;
      passRoom[room.entry] = 1;
      const seg = pushAStar(passRoom, room.boxes, room.goals, room.entry, p.w, p.h, 1000000);
      if (seg === null) {
        ok = false;
        break;
      }
      const sim2 = simulate({ ...p, boxes: curBoxes, player: curPlayer }, seg);
      if (sim2 === null) {
        ok = false;
        break;
      }
      moves += seg;
      curBoxes = sim2.boxes;
      curPlayer = sim2.player;
    }
    if (ok) {
      const fin = simulate(p, moves);
      if (fin !== null && solved(fin.boxes)) return moves;
    }
  }
  // Fallback: the same engine over the whole board.
  const passAll = new Uint8Array(p.w * p.h);
  for (let i = 0; i < p.w * p.h; i++) passAll[i] = p.wall[i] === 0 ? 1 : 0;
  const whole = pushAStar(passAll, p.boxes, p.goals, p.player, p.w, p.h, 2000000);
  if (whole !== null) {
    const fin = simulate(p, whole);
    if (fin !== null && solved(fin.boxes)) return whole;
  }
  return '';
}

for (const id of readdirSync(INPUT).sort()) {
  const moves = solveLevel(readFileSync(join(INPUT, id, 'level.txt'), 'utf8'));
  mkdirSync(join(OUTPUT, id), { recursive: true });
  writeFileSync(join(OUTPUT, id, 'solution.txt'), `${moves}\n`);
}
