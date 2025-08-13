# graph.py

from vertex import Vertex
from edge import Edge
from periphery import Periphery
from utils_geom import (
    v_add, v_sub, v_scale, v_len, v_norm, v_dot, v_rot90_ccw, v_rot90_cw
)
from PyQt5.QtCore import QPointF
from typing import Optional
import math
import json
import secrets
import random
from statistics import median
from bisect import bisect_right
from dataclasses import dataclass, field

INITIAL_RADIUS = 200.0
PI = math.pi
    
@dataclass
class RedrawConfig:
    # Force-directed parameters
    spring_k: float = 0.18
    max_step_frac: float = 0.12
    repulsion_cutoff_frac: float = 2.8
    repulsion_strength_frac: float = 0.25
    
    # Congestion parameters
    congestion_cutoff_frac: float = 1.9
    congestion_beta_frac: float = 0.30
    congestion_thresh: int = 10
    
    # Tutte relaxation parameters
    tutte_rounds: int = 12
    adaptive_tutte_gamma: float = 1.30
    
    # Heuristic parameters
    long_edge_reduce_rounds: int = 2
    short_edge_boost_rounds: int = 2
    fan_spread_iters: int = 2
    
    # Final separation
    separation_iters: int = 2

@dataclass
class GraphConfig:
    # Define different configs for different graph sizes
    small_graph_v_thresh: int = 1200
    medium_graph_v_thresh: int = 6000

    # Default configs
    default: RedrawConfig = field(default_factory=RedrawConfig)
    
    # A lighter config for faster, less precise redraws
    light: RedrawConfig = field(default_factory=lambda: RedrawConfig(
        spring_k=0.16, max_step_frac=0.11, repulsion_cutoff_frac=2.4,
        repulsion_strength_frac=0.20, congestion_thresh=8,
        tutte_rounds=5, adaptive_tutte_gamma=1.20, long_edge_reduce_rounds=1,
        short_edge_boost_rounds=2, fan_spread_iters=1, separation_iters=1
    ))

class Graph:
    def __init__(self):
        self.vertices = []
        self.edges = []
        self.periphery = Periphery()
        self.labelMode = 2  # 0=color-only, 1=index-only, 2=color+index
        self._last_add_info = None
        self._adjacency = {}
        self.config = GraphConfig()

        # Robust randomness
        self._rng = random.Random(secrets.randbits(64))

        # Target edge length (EMA) to keep edge lengths homogeneous at scale
        self._target_len = None

        # Redraw & scale management
        self._adds_since_redraw = 0
        self._MAX_WORKING_RADIUS: Optional[float] = 150000.0

        # Auto-expand/view integration flags (UI reads these)
        self.auto_expand = True
        self.auto_expand_mode = "fit"  # "fit" | "infinite"
        self.view_padding = 40.0
        self.target_edge_px = 18.0

        # UI callback hook
        self.on_layout_changed = None

        # Debug info for random choice
        self.last_random_choice = None

        # Strict homogenization defaults (slightly tightened)
        self.strict_homog_enabled = True
        self.homog_tol = 9.0e-4            # relative tolerance on edge length (tighter)
        self.homog_max_rounds_small = 80   # V <= 1200
        self.homog_max_rounds_medium = 45  # V <= 5000
        self.homog_max_rounds_large = 20   # larger
        self.homog_clamp_frac = 0.10       # fraction of target length per move clamp

        # Ultra-homogeneity controls
        self.lock_target_len = True         # keep target length stable (based on periphery)
        self.ultra_homog = True             # enable ultra polishing
        self.ultra_small_threshold = 4000
        self.ultra_tol_small = 4.0e-4       # tighter tolerance for small graphs
        self.ultra_tol_large = 7.0e-4       # slightly looser for big graphs
        self.ultra_rounds_small = 150
        self.ultra_rounds_large = 90
        self.ultra_edge_proj_frac_small = 0.12  # project top 12% worst edges (small graphs)
        self.ultra_edge_proj_frac_large = 0.06  # project top 6% (large graphs)
        self.ultra_clamp_frac = 0.12            # per-edge clamp during projection

        # Auto-polish after each add
        self.auto_polish_after_add = True
        self.auto_polish_p95 = 1.05         # stop when 95% of edges within ±5%
        self.auto_polish_max_passes = 2     # at most 2 extra passes per insertion

    # --------------------------
    # Small helpers
    # --------------------------
    def _hash_u32(self, *args):
        h = 0x811C9DC5
        for a in args:
            x = int(a) & 0xffffffff
            h ^= x
            h = (h * 0x01000193) & 0xffffffff
        return h

    def _rotate_vec(self, v: QPointF, ang_rad: float) -> QPointF:
        c = math.cos(ang_rad)
        s = math.sin(ang_rad)
        return QPointF(v.x() * c - v.y() * s, v.x() * s + v.y() * c)

    def _approx_diameter(self, idx: int) -> float:
        if idx >= 1000:
            return 40.0
        elif idx >= 100:
            return 36.0
        elif idx >= 10:
            return 32.0
        else:
            return 30.0

    def _min_sep_length(self):
        vis = [v.getDiameter() for v in self.vertices if v and v.isVisible()]
        avg_d = (sum(vis) / len(vis)) if vis else 30.0
        tl = self._quick_target_length()
        return max(0.24 * tl, 0.55 * avg_d) + 3.0

    # --------------------------
    # Segment intersection helpers
    # --------------------------
    def _orient(self, a: QPointF, b: QPointF, c: QPointF):
        return (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x())

    def _on_segment(self, a: QPointF, b: QPointF, c: QPointF, eps=1e-9):
        return (min(a.x(), b.x()) - eps <= c.x() <= max(a.x(), b.x()) + eps and
                min(a.y(), b.y()) - eps <= c.y() <= max(a.y(), b.y()) + eps and
                abs(self._orient(a, b, c)) <= eps)

    def _segments_properly_intersect(self, a: QPointF, b: QPointF, c: QPointF, d: QPointF, eps=1e-9):
        o1 = self._orient(a, b, c)
        o2 = self._orient(a, b, d)
        o3 = self._orient(c, d, a)
        o4 = self._orient(c, d, b)
        if (o1 * o2 < -eps) and (o3 * o4 < -eps):
            return True
        # Treat touching/collinear as non-crossing (fine for planarity)
        if abs(o1) <= eps and self._on_segment(a, b, c, eps): return False
        if abs(o2) <= eps and self._on_segment(a, b, d, eps): return False
        if abs(o3) <= eps and self._on_segment(c, d, a, eps): return False
        if abs(o4) <= eps and self._on_segment(c, d, b, eps): return False
        return False

    def _bbox_disjoint(self, a, b, c, d, eps=1e-9):
        min_ax = min(a.x(), b.x()) - eps
        max_ax = max(a.x(), b.x()) + eps
        min_ay = min(a.y(), b.y()) - eps
        max_ay = max(a.y(), b.y()) + eps
        min_cx = min(c.x(), d.x()) - eps
        max_cx = max(c.x(), d.x()) + eps
        min_cy = min(c.y(), d.y()) - eps
        max_cy = max(c.y(), d.y()) + eps
        return (max_ax < min_cx) or (max_cx < min_ax) or (max_ay < min_cy) or (max_cy < min_ay)

    def _point_segment_distance(self, p: QPointF, a: QPointF, b: QPointF) -> float:
        ax, ay = a.x(), a.y()
        bx, by = b.x(), b.y()
        px, py = p.x(), p.y()
        abx, aby = (bx - ax), (by - ay)
        denom = abx * abx + aby * aby
        if denom <= 1e-18:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * abx + (py - ay) * aby) / denom
        t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
        qx = ax + t * abx
        qy = ay + t * aby
        return math.hypot(px - qx, py - qy)

    def _new_edges_too_close_to_vertices(self, new_pos: QPointF, arc_indices) -> bool:
        base = max(6.0, 0.14 * self._quick_target_length())
        per_set = set(arc_indices)
        for vi in arc_indices:
            p2 = self.vertices[vi].getPosition()
            minx = min(new_pos.x(), p2.x()) - base
            maxx = max(new_pos.x(), p2.x()) + base
            miny = min(new_pos.y(), p2.y()) - base
            maxy = max(new_pos.y(), p2.y()) + base
            for vtx in self.vertices:
                if not vtx:
                    continue
                j = vtx.getIndex()
                if j in per_set:
                    continue
                pj = vtx.getPosition()
                if pj.x() < minx or pj.x() > maxx or pj.y() < miny or pj.y() > maxy:
                    continue
                d_need = 0.5 * (self._approx_diameter(len(self.vertices)) + vtx.getDiameter()) + base
                if self._point_segment_distance(pj, new_pos, p2) < d_need:
                    return True
        return False

    def _new_edges_cross_any(self, new_pos: QPointF, arc_indices):
        for vi in arc_indices:
            p2 = self.vertices[vi].getPosition()
            for e in self.edges:
                u = e.getStartVertex().getIndex()
                v = e.getEndVertex().getIndex()
                if u == vi or v == vi:
                    continue
                p3 = self.vertices[u].getPosition()
                p4 = self.vertices[v].getPosition()
                if self._bbox_disjoint(new_pos, p2, p3, p4):
                    continue
                if self._segments_properly_intersect(new_pos, p2, p3, p4):
                    return True
        return False

    # --------------------------
    # Base graph ops
    # --------------------------
    def clear(self):
        self.vertices.clear()
        self.edges.clear()
        self.periphery.clear()
        self._adjacency.clear()
        self._target_len = None
        self._adds_since_redraw = 0
        self.last_random_choice = None
        self._last_add_info = None

    def _add_edge_internal(self, edge):
        v1_idx = edge.getStartVertex().getIndex()
        v2_idx = edge.getEndVertex().getIndex()
        if v2_idx in self._adjacency.get(v1_idx, set()):
            return
        self.edges.append(edge)
        self._adjacency.setdefault(v1_idx, set()).add(v2_idx)
        self._adjacency.setdefault(v2_idx, set()).add(v1_idx)

    def getDegree(self, vertex_index):
        return len(self._adjacency.get(vertex_index, set()))

    def startBasicGraph(self, n: int = 3):
        """
        Initialize a new graph with n seed vertices.
        Spec requires: first graph is a triangle.
        - n>=3: convex n-gon with triangulation by fanning from V0.
        """
        self.clear()
        try:
            n = int(n)
        except Exception:
            n = 3
        # Clamp to 3..10 to satisfy "first graph is a triangle"
        n = max(3, min(10, n))

        # n >= 3: place on a circle
        for i in range(n):
            angle = 2 * PI * i / n - PI / 2
            pos = QPointF(INITIAL_RADIUS * math.cos(angle), INITIAL_RADIUS * math.sin(angle))
            v = Vertex(i, pos, (i % 4) + 1, origin="seed")
            self.vertices.append(v)
            self._adjacency[i] = set()

        # Outer cycle edges
        for i in range(n):
            self._add_edge_internal(Edge(self.vertices[i], self.vertices[(i + 1) % n]))

        # Fan triangulation from V0: connect 0->i for i=2..n-2
        if n > 3:
            for i in range(2, n - 1):
                self._add_edge_internal(Edge(self.vertices[0], self.vertices[i]))

        # Set periphery and visibility
        self.periphery.initialize(list(range(n)))
        self.goToVertex(n)

        # Target length stabilized from periphery edges
        self._target_len = self._compute_target_edge_length()

    # --------------------------
    # Periphery geometry helpers
    # --------------------------
    def _periphery_pts(self):
        return [self.vertices[i].getPosition() for i in self.periphery.getIndices()]

    def _periphery_center(self):
        pts = self._periphery_pts()
        if not pts:
            return QPointF(0, 0)
        cx = sum(p.x() for p in pts) / len(pts)
        cy = sum(p.y() for p in pts) / len(pts)
        return QPointF(cx, cy)

    def _polygon_signed_area_pts(self, pts):
        if len(pts) < 3:
            return 0.0
        A = 0.0
        n = len(pts)
        for i in range(n):
            j = (i + 1) % n
            A += pts[i].x() * pts[j].y() - pts[j].x() * pts[i].y()
        return 0.5 * A

    def _periphery_orientation_ccw(self):
        area = self._polygon_signed_area_pts(self._periphery_pts())
        return 1 if area > 0 else (-1 if area < 0 else 0)

    def _point_in_polygon(self, pt, poly_pts):
        x, y = pt.x(), pt.y()
        inside = False
        n = len(poly_pts)
        if n < 3:
            return False
        for i in range(n):
            j = (i + 1) % n
            xi, yi = poly_pts[i].x(), poly_pts[i].y()
            xj, yj = poly_pts[j].x(), poly_pts[j].y()
            intersect = ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi)
            if intersect:
                inside = not inside
        return inside

    # --------------------------
    # Target edge length control (periphery-first)
    # --------------------------
    def _compute_target_edge_length(self):
        # Prefer periphery edges (stable target)
        per = self.periphery.getIndices()
        if len(per) >= 3:
            pts = [self.vertices[i].getPosition() for i in per]
            lens = []
            for i in range(len(pts)):
                j = (i + 1) % len(pts)
                lens.append(v_len(v_sub(pts[j], pts[i])))
            if lens:
                return float(median(lens))

        # Fallback: median over visible edges
        lens2 = []
        for e in self.edges:
            if e.isVisible():
                p1 = e.getStartVertex().getPosition()
                p2 = e.getEndVertex().getPosition()
                lens2.append(v_len(v_sub(p2, p1)))
        return float(median(lens2)) if lens2 else 60.0

    def _quick_target_length(self):
        return self._target_len if self._target_len is not None else 60.0

    # --------------------------
    # Core add operations
    # --------------------------
    def _redraw_frequency(self):
        V = len(self.vertices)
        if V < 300: return 1
        if V < 1000: return 6
        if V < 3000: return 12
        return 25

    def addVertex(self, periphery_segment, origin="manual"):
        # Must be a proper CW arc: contiguous and not the full cycle
        if not self.periphery.isContiguous(periphery_segment) or len(periphery_segment) < 2:
            return False, -1
        if len(periphery_segment) >= self.periphery.size():
            # Disallow full-cycle arc; pick a proper arc
            return False, -1

        newIdx = len(self.vertices)
        pos, outward_dir = self.computeNewVertexPosition(periphery_segment, return_outward=True)
        colorIdx = (newIdx % 4) + 1
        newVertex = Vertex(newIdx, pos, colorIdx, origin=origin)
        self.vertices.append(newVertex)
        self._adjacency[newIdx] = set()

        m = len(periphery_segment)
        center = (m - 1) * 0.5
        order = sorted(range(m), key=lambda i: abs(i - center))
        for k in order:
            idx = periphery_segment[k]
            self._add_edge_internal(Edge(newVertex, self.vertices[idx]))

        self._greedy_color(newIdx)
        self.periphery.updateAfterAddition(periphery_segment, newIdx)

        # Tighten edge lengths locally around the new vertex
        per_set = set(self.periphery.getIndices())
        pinned_local = per_set - {newIdx}  # let the new vertex move a bit

        self.enforce_unit_lengths_local(
            center_idx=newIdx,
            pinned=pinned_local,
            target_length=self._quick_target_length(),
            rounds=28 if len(self.vertices) < 2000 else 18,
            clamp_frac=0.10,
            tol=9e-4
        )

        # Quick long-edge cutback pass
        tl_now = self._quick_target_length()
        self._long_edge_reduce(pinned=pinned_local, target_length=tl_now,
                            hi1=1.30, hi2=1.85, gain1=0.55, gain2=0.75,
                            max_step=0.16 * tl_now, rounds=1)

        self._incremental_relax(newIdx, local_rounds=3)

        # Local neighborhood (1–2 hops)
        per_set = set(self.periphery.getIndices())
        pinned_local = per_set - {newIdx}
        local = set([newIdx])
        local.update(self._adjacency.get(newIdx, set()))
        for u in list(local):
            local.update(self._adjacency.get(u, set()))

        # Local min-distance spacing
        self._separate_min_dist_grid(
            pinned=pinned_local,
            dmin=self._min_sep_length(),
            iters=2,
            only_indices=local
        )

        # Local vertex–edge clearance
        edge_clear = max(0.5 * self.vertices[newIdx].getDiameter() + 0.20 * self._quick_target_length(),
                        0.55 * self._quick_target_length())
        self._ve_clearance_grid(
            pinned=pinned_local,
            clear_dist=edge_clear,
            iters=2,
            clamp_step=0.28 * self._quick_target_length(),
            only_indices=local
        )

        # Light global redraw
        self.redraw_planar(iterations=6, light=True)


        # Skip heavy extra polishing if floating is on
        if self.auto_polish_after_add :
            self._auto_polish_after_add(set(self.periphery.getIndices()))
            self._adds_since_redraw += 1

            if self._adds_since_redraw >= self._redraw_frequency():
                self.redraw_planar(iterations=None, light=False)
                self._adds_since_redraw = 0

        spawn_dist = max(1.5 * self._quick_target_length(), 40.0)
        mult = 2.6 if len(periphery_segment) <= 3 else 2.0
        if outward_dir is None or v_len(outward_dir) < 1e-6:
            spawn_pos = v_add(pos, QPointF(spawn_dist, 0.0))
        else:
            spawn_pos = v_add(pos, v_scale(outward_dir, spawn_dist * mult))
        self._last_add_info = {"index": newIdx, "spawn_pos": spawn_pos, "final_pos": pos}

        if self.on_layout_changed:
            self.on_layout_changed(self.get_bounding_box(), self._periphery_center(), self.target_edge_px)

        return True, newIdx

    # -------- Random helpers --------
    def _sample_uniform_valid_segment(self, peri, deg_min=5):
        n = len(peri)
        if n < 2:
            return None

        good = [1 if self.getDegree(peri[i]) >= deg_min else 0 for i in range(n)]

        if n == 2:
            s = self._rng.randint(0, 1)
            return s, 2

        if sum(good) == n:
            s = self._rng.randint(0, n - 1)
            L = self._rng.randint(2, n)
            return s, L

        good2 = good + good
        size2 = 2 * n
        next_bad = [0] * (size2 + 1)
        next_bad[size2] = size2
        for i in range(size2 - 1, -1, -1):
            next_bad[i] = i if good2[i] == 0 else next_bad[i + 1]

        pref = [0] * (n + 1)
        for s in range(n):
            start = s + 1
            nb = next_bad[start]
            span = nb - start
            if span > n - 2:
                span = n - 2
            if span < 0:
                span = 0
            pref[s + 1] = pref[s] + (span + 1)

        total = pref[n]
        if total <= 0:
            return None

        r = self._rng.randrange(total)
        s = bisect_right(pref, r) - 1
        r_local = r - pref[s]
        L = 2 + r_local
        return s, L

    def _sample_uniform_segment_all(self, peri, length_bias="uniform", gamma=1.35):
        n = len(peri)
        if n < 2:
            return None
        if n == 2:
            return self._rng.randint(0, 1), 2

        if length_bias == "uniform":
            L = 2 + self._rng.randrange(n - 1)
        else:
            if length_bias == "favor_long":
                weights = [float(L ** gamma) for L in range(2, n + 1)]
            elif length_bias == "favor_short":
                weights = [float((n + 2 - L) ** gamma) for L in range(2, n + 1)]
            else:
                weights = [1.0 for _ in range(2, n + 1)]
            total = sum(weights)
            r = self._rng.random() * total
            acc = 0.0
            L = 2
            for k, w in enumerate(weights, start=2):
                acc += w
                if r <= acc:
                    L = k
                    break

        s = self._rng.randrange(n)
        return s, L

    def _segment_respects_degree_safeguard(self, segment, deg_min_pre=4):
        """
        True if all interior vertices of segment have pre-add degree >= deg_min_pre.
        That ensures they will be >= deg_min_pre + 1 after the insertion.
        """
        if len(segment) < 2:
            return False
        for vi in segment[1:-1]:
            if self.getDegree(vi) < deg_min_pre:
                return False
        return True

    def addRandomVertex(self):
        """
        Real random insertion:
        - allows arcs of any length L in [2 .. n-1] (touches ≥2 periphery vertices)
        - enforces the client safeguard: any vertex hidden by the arc must end up deg ≥ 5
        (i.e., all interior vertices have pre-add degree ≥ 4)
        - does NOT require the new vertex itself to have deg ≥ 5
        """
        peri = self.periphery.getIndices()
        n = len(peri)
        if n < 2:
            return False, -1

        # Fast, uniform sampler over valid arcs (no full-cycle)
        pick = self._sample_uniform_valid_segment_fast(
            peri,
            require_new_deg_ge5=False,  # client requirement: relax new vertex degree
            deg_min_pre=4               # interior pre-degree >= 4 => post-add >= 5
        )
        if pick is None:
            return False, -1

        s, L = pick
        chosen_segment = [peri[(s + k) % n] for k in range(L)]
        ok, new_idx = self.addVertex(chosen_segment, origin="random")
        if ok:
            self.last_random_choice = {"L": L}
            if self.on_layout_changed:
                self.on_layout_changed(self.get_bounding_box(), self._periphery_center(), self.target_edge_px)
        return ok, new_idx

    def addVertexBySelection(self, start_v_idx, end_v_idx):
        segment = self.periphery.getSegment(start_v_idx, end_v_idx)
        if not segment:
            return False, -1
        return self.addVertex(segment, origin="manual")

    def goToVertex(self, m):
        for v in self.vertices:
            v.setVisible(v.getIndex() < m)
        for e in self.edges:
            is_vis = e.getStartVertex().isVisible() and e.getEndVertex().isVisible()
            e.setVisible(is_vis)

    # --------------------------
    # Placement and relaxation
    # --------------------------
    def _circle_two_points_outward(self, a: QPointF, b: QPointF, r: float, outward_dir: QPointF) -> Optional[QPointF]:
        dx = b.x() - a.x()
        dy = b.y() - a.y()
        d = math.hypot(dx, dy)
        if d < 1e-9 or d > 2.0 * r + 1e-9:
            return None

        mx = 0.5 * (a.x() + b.x())
        my = 0.5 * (a.y() + b.y())
        h2 = r * r - 0.25 * d * d
        if h2 < 0:
            h2 = 0.0
        h = math.sqrt(h2)

        ux = -dy / max(d, 1e-9)
        uy = dx / max(d, 1e-9)

        q1 = QPointF(mx + ux * h, my + uy * h)
        q2 = QPointF(mx - ux * h, my - uy * h)

        v1x, v1y = q1.x() - mx, q1.y() - my
        v2x, v2y = q2.x() - mx, q2.y() - my
        s1 = v1x * outward_dir.x() + v1y * outward_dir.y()
        s2 = v2x * outward_dir.x() + v2y * outward_dir.y()
        return q1 if s1 >= s2 else q2
    def _ve_clearance_grid(self, pinned, clear_dist, iters=2, clamp_step=None,
                        only_indices=None, skip_incident=True):
        """
        Push non-pinned vertices away from nearby edges (vertex–edge clearance).
        Grid-accelerated: indexes edge AABBs expanded by clear_dist.
        """
        if clear_dist <= 0:
            return
        if clamp_step is None:
            clamp_step = 0.28 * max(clear_dist, 1.0)

        # Helper: projection on segment (ax,ay)-(bx,by)
        def _proj(px, py, ax, ay, bx, by):
            abx = bx - ax; aby = by - ay
            denom = abx*abx + aby*aby
            if denom <= 1e-18:
                return ax, ay, 0.0
            t = ((px - ax)*abx + (py - ay)*aby) / denom
            t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
            qx = ax + t * abx; qy = ay + t * aby
            return qx, qy, t

        # Build set of vertices to process
        if only_indices is None:
            idxs = [i for i in range(len(self.vertices)) if self.vertices[i].isVisible()]
        else:
            idxs = [i for i in only_indices if self.vertices[i].isVisible()]

        # Precompute edge list and edge grid
        edges = []
        for e in self.edges:
            if not e.isVisible():
                continue
            u = e.getStartVertex().getIndex()
            v = e.getEndVertex().getIndex()
            edges.append((u, v))

        if not edges or not idxs:
            return

        # Grid: cell size = clear_dist
        cell = clear_dist
        inv_cell = 1.0 / max(1e-9, cell)

        for _ in range(max(1, iters)):
            # Build grid of expanded edge AABBs
            grid = {}
            pts = [v.getPosition() for v in self.vertices]
            for ei, (u, v) in enumerate(edges):
                p1 = pts[u]; p2 = pts[v]
                minx = min(p1.x(), p2.x()) - clear_dist
                maxx = max(p1.x(), p2.x()) + clear_dist
                miny = min(p1.y(), p2.y()) - clear_dist
                maxy = max(p1.y(), p2.y()) + clear_dist
                cx0 = int(math.floor(minx * inv_cell)); cx1 = int(math.floor(maxx * inv_cell))
                cy0 = int(math.floor(miny * inv_cell)); cy1 = int(math.floor(maxy * inv_cell))
                for cx in range(cx0, cx1 + 1):
                    for cy in range(cy0, cy1 + 1):
                        grid.setdefault((cx, cy), []).append(ei)

            # Accumulate pushes per vertex
            moves = {}
            for i in idxs:
                if i in pinned:
                    continue
                pi = pts[i]; px, py = pi.x(), pi.y()
                cx = int(math.floor(px * inv_cell)); cy = int(math.floor(py * inv_cell))

                accx = accy = 0.0
                # Neighborhood cells
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        eids = grid.get((cx + dx, cy + dy), [])
                        for ei in eids:
                            u, v = edges[ei]
                            if skip_incident and (i == u or i == v):
                                continue
                            pA = pts[u]; pB = pts[v]
                            qx, qy, _ = _proj(px, py, pA.x(), pA.y(), pB.x(), pB.y())
                            dx0 = px - qx; dy0 = py - qy
                            dist = math.hypot(dx0, dy0)
                            if dist >= clear_dist or dist < 1e-9:
                                continue
                            deficit = (clear_dist - dist)
                            nx = dx0 / (dist if dist > 1e-9 else 1.0)
                            ny = dy0 / (dist if dist > 1e-9 else 1.0)
                            # Quadratic falloff near the edge
                            w = min(1.0, (deficit / clear_dist))
                            fmag = deficit * (0.75 * w)  # gentle but effective
                            accx += nx * fmag; accy += ny * fmag

                if accx != 0.0 or accy != 0.0:
                    mv_len = math.hypot(accx, accy)
                    if mv_len > clamp_step:
                        s = clamp_step / mv_len
                        accx *= s; accy *= s
                    moves[i] = QPointF(accx, accy)

            # Apply moves (with a minimal planarity guard)
            for i, mv in moves.items():
                pi = self.vertices[i].getPosition()
                cand = QPointF(pi.x() + mv.x(), pi.y() + mv.y())
                # Tiny guard: if moving creates crossings with incident edges, reduce step
                # (cheap approximation: try half step)
                if self._has_crossings_quick(max_reports=1):
                    cand = QPointF(pi.x() + 0.5 * mv.x(), pi.y() + 0.5 * mv.y())
                self.vertices[i].setPosition(cand)
    # graph.py (add this helper)

    def _sample_uniform_valid_segment_fast(self, peri, require_new_deg_ge5=True, deg_min_pre=4):
        """
        O(n) preprocessing + O(1) sampling of one valid segment.
        Valid = contiguous CW arc (length L) such that:
        - 2 <= L <= n-1  (no full-cycle arc)
        - all interior vertices have deg >= deg_min_pre (so they become >= deg_min_pre+1)
        - if require_new_deg_ge5: L >= 5 (so new vertex has deg >= 5)
        Returns (start_index_in_peri, L) or None.
        """
        n = len(peri)
        if n < 2:
            return None

        # Pre-degrees of periphery vertices
        deg_ok = [1 if self.getDegree(peri[i]) >= deg_min_pre else 0 for i in range(n)]
        # Quick edge case: n == 2 → only arc length 2 possible, but L must be <= n-1 (disallowed)
        if n == 2:
            return None

        # Build "next bad" over doubled array to get max span of consecutive good after s+1
        good2 = deg_ok + deg_ok
        size2 = 2 * n
        next_bad = [0] * (size2 + 1)
        next_bad[size2] = size2
        for i in range(size2 - 1, -1, -1):
            next_bad[i] = i if good2[i] == 0 else next_bad[i + 1]

        Lmin = 5 if require_new_deg_ge5 else 2  # min arc length (new vertex degree constraint)
        total = 0
        weights = [0] * n

        # Compute how many valid lengths per start s
        for s in range(n):
            start = s + 1                   # first interior position
            nb = next_bad[start]            # first bad index after start
            span = nb - start               # number of consecutive good interiors
            # Cap span so that L <= n-1 (no full-cycle)
            span = min(span, n - 3)         # because L = 2 + span, and we want L <= n-1
            low = max(Lmin, 2)
            high = min(n - 1, 2 + span)
            w = max(0, high - low + 1)
            weights[s] = w
            total += w

        if total <= 0:
            return None

        r = self._rng.randrange(total)
        s = 0
        while r >= weights[s]:
            r -= weights[s]
            s += 1

        # Weights[s] = number of allowable L values starting at s
        start = s + 1
        nb = next_bad[start]
        span = min(nb - start, n - 3)
        low = max(Lmin, 2)
        # r is an offset in [0, weights[s)-1]; map to L = low + r
        L = low + r
        return s, L

    def computeNewVertexPosition(self, peripheryIndices, return_outward=False):
        assert len(peripheryIndices) >= 2
        L_arc = len(peripheryIndices)

        arc_pts = [self.vertices[i].getPosition() for i in peripheryIndices]
        arc_centroid = QPointF(
            sum(p.x() for p in arc_pts) / len(arc_pts),
            sum(p.y() for p in arc_pts) / len(arc_pts)
        )

        per_center = self._periphery_center()

        radial_out = v_norm(v_sub(arc_centroid, per_center))
        if v_len(radial_out) < 1e-6:
            radial_out = QPointF(1.0, 0.0)

        # Estimate outward normal direction based on periphery orientation
        normals = []
        orient_ccw = (self._periphery_orientation_ccw() > 0)
        for i in range(len(peripheryIndices) - 1):
            a = self.vertices[peripheryIndices[i]].getPosition()
            b = self.vertices[peripheryIndices[i + 1]].getPosition()
            e = v_sub(b, a)
            L = v_len(e)
            if L < 1e-6:
                continue
            n = v_rot90_cw(e) if orient_ccw else v_rot90_ccw(e)
            nL = v_len(n)
            if nL > 1e-6:
                normals.append(v_scale(n, 1.0 / nL))

        if normals:
            nx = sum(n.x() for n in normals) / len(normals)
            ny = sum(n.y() for n in normals) / len(normals)
            outward_dir = QPointF(nx, ny)
            if v_len(outward_dir) < 1e-6:
                outward_dir = radial_out
            else:
                outward_dir = v_scale(outward_dir, 1.0 / v_len(outward_dir))
                if v_dot(outward_dir, radial_out) < 0:
                    outward_dir = v_scale(outward_dir, -1.0)
        else:
            outward_dir = radial_out

        # Small jitter by arc size (extend to larger arcs to avoid identical rays)
        if L_arc <= 8:
            if L_arc <= 2:
                jitter_deg = 10.0
            elif L_arc == 3:
                jitter_deg = 7.0
            elif L_arc == 4:
                jitter_deg = 5.0
            else:
                jitter_deg = 3.0  # arcs 5..8
            key = self._hash_u32(peripheryIndices[0], peripheryIndices[-1], len(self.vertices))
            t = ((key % 10007) / 10007.0) * 2.0 - 1.0
            outward_dir = self._rotate_vec(outward_dir, math.radians(jitter_deg) * t)
            if v_dot(outward_dir, radial_out) < 0:
                outward_dir = v_scale(outward_dir, -1.0)

        target = self._quick_target_length()

        # Prefer exact two-circle intersection for small arcs
        new_pos = None
        if L_arc <= 3:
            a = arc_pts[0]
            b = arc_pts[-1]
            cand = self._circle_two_points_outward(a, b, target, outward_dir)
            if cand is not None:
                new_pos = cand

        # Fallback: centroid + outward_dir quadratic solve
        if new_pos is None:
            A_vals, B_vals = [], []
            for p in arc_pts:
                di0 = v_sub(arc_centroid, p)
                A_vals.append(di0.x() * di0.x() + di0.y() * di0.y())
                B_vals.append(v_dot(outward_dir, di0))
            A = sum(A_vals) / len(A_vals) if A_vals else 0.0
            B = sum(B_vals) / len(B_vals) if B_vals else 0.0
            T2 = target * target
            disc = B * B - (A - T2)
            r0 = 0.0
            if disc >= 0:
                r0 = max(0.0, -B + math.sqrt(disc))
            new_pos = v_add(arc_centroid, v_scale(outward_dir, r0))
        # Keep away from the straight chord between arc endpoints
        a = arc_pts[0]
        b = arc_pts[-1]
        min_chord_clear = max(0.45 * target, 0.5 * self._min_sep_length())
        tries_c = 0
        while tries_c < 24 and self._point_segment_distance(new_pos, a, b) < min_chord_clear:
            new_pos = v_add(new_pos, v_scale(outward_dir, 0.5 * min_chord_clear))
            tries_c += 1
        # Safety pushes: outside polygon and outside periphery radius margin
        per_pts_all = self._periphery_pts()
        per_center_all = self._periphery_center()
        per_radii = [v_len(v_sub(p, per_center_all)) for p in per_pts_all] if per_pts_all else [INITIAL_RADIUS]
        max_per_r = max(per_radii) if per_radii else INITIAL_RADIUS
        base_margin = max(20.0, 0.15 * target)
        if L_arc <= 2:
            margin = base_margin * 2.4; step_out = max(12.0, target * 0.40)
        elif L_arc == 3:
            margin = base_margin * 2.0; step_out = max(12.0, target * 0.34)
        elif L_arc == 4:
            margin = base_margin * 1.6; step_out = max(12.0, target * 0.30)
        elif L_arc <= 6:
            margin = base_margin * 1.3; step_out = max(12.0, target * 0.26)
        else:
            margin = base_margin; step_out = max(10.0, target * 0.22)

        tries = 0
        def radial_dist(q): return v_len(v_sub(q, per_center_all))
        while tries < 48 and (self._point_in_polygon(new_pos, per_pts_all) or radial_dist(new_pos) <= max_per_r + margin):
            new_pos = v_add(new_pos, v_scale(outward_dir, step_out)); tries += 1

        tries2 = 0
        while tries2 < 64 and self._new_edges_cross_any(new_pos, peripheryIndices):
            new_pos = v_add(new_pos, v_scale(outward_dir, step_out * 1.15)); tries2 += 1

        tries2b = 0
        while tries2b < 32 and self._new_edges_too_close_to_vertices(new_pos, peripheryIndices):
            new_pos = v_add(new_pos, v_scale(outward_dir, step_out * 0.9)); tries2b += 1

        new_idx = len(self.vertices)
        d_new = self._approx_diameter(new_idx)
        pad = max(6.0, 0.18 * target)
        tries3 = 0
        while tries3 < 32:
            too_close = False
            for vtx in self.vertices:
                if not vtx: continue
                d = v_len(v_sub(new_pos, vtx.getPosition()))
                threshold = 0.5 * (d_new + vtx.getDiameter()) + pad
                if d < threshold:
                    new_pos = v_add(new_pos, v_scale(outward_dir, max(step_out * 0.4, threshold - d)))
                    too_close = True
                    break
            if not too_close: break
            tries3 += 1
                # Final anti-stacking: ensure safe clearance from existing edges (float off lines)
        # This prevents the new vertex from sitting on top of long chords like (1-9).
        def _proj_point_on_seg(px, py, ax, ay, bx, by):
            abx = bx - ax
            aby = by - ay
            denom = abx * abx + aby * aby
            if denom <= 1e-18:
                return ax, ay, 0.0
            t = ((px - ax) * abx + (py - ay) * aby) / denom
            if t < 0.0: t = 0.0
            elif t > 1.0: t = 1.0
            qx = ax + t * abx
            qy = ay + t * aby
            return qx, qy, t

        # Desired clearance from any existing edge

        edge_clear = max(0.5 * d_new + 0.28 * target, 0.70 * target)
        max_iter_edge = 30
        step_cap = 0.45 * target

        it_e = 0
        while it_e < max_iter_edge:
            nearest = None  # (deficit, nx, ny)
            px, py = new_pos.x(), new_pos.y()
            for e in self.edges:
                # All existing edges (this includes long boundary chords)
                u = e.getStartVertex().getIndex()
                v = e.getEndVertex().getIndex()
                pA = self.vertices[u].getPosition()
                pB = self.vertices[v].getPosition()
                ax, ay = pA.x(), pA.y()
                bx, by = pB.x(), pB.y()

                qx, qy, _ = _proj_point_on_seg(px, py, ax, ay, bx, by)
                dx = px - qx
                dy = py - qy
                dist = math.hypot(dx, dy)
                if dist < edge_clear:
                    deficit = (edge_clear - dist)
                    # Normalized push direction away from the edge
                    if dist < 1e-9:
                        nx, ny = outward_dir.x(), outward_dir.y()
                    else:
                        nx, ny = dx / dist, dy / dist
                    # Keep the worst (largest deficit)
                    if (nearest is None) or (deficit > nearest[0]):
                        nearest = (deficit, nx, ny)

            if nearest is None:
                break  # already clear of all edges

            deficit, nx, ny = nearest
            # Limit the step, try directly away from the closest edge
            step = min(step_cap, deficit)
            cand = QPointF(px + nx * step, py + ny * step)

            # Planarity guard: if the move would cause new edges to cross, push outward instead
            if self._new_edges_cross_any(cand, peripheryIndices):
                cand = v_add(new_pos, v_scale(outward_dir, min(step_cap, deficit * 1.10)))

            new_pos = cand
            it_e += 1

        # FINAL SAFETY CHECK: If after all attempts, the position is still bad,
        # fall back to a "dumb but safe" placement far away. The layout
        # engine will pull it into a better position later.
        is_safe = not self._new_edges_cross_any(new_pos, peripheryIndices) and \
                  not self._new_edges_too_close_to_vertices(new_pos, peripheryIndices)

        if not is_safe:
            # Fallback placement: push it far out along the outward direction.
            fallback_distance = self._quick_target_length() * 5.0
            new_pos = v_add(arc_centroid, v_scale(outward_dir, fallback_distance))
            print("Warning: computeNewVertexPosition heuristics failed. Using fallback placement.")

        if return_outward:
            return new_pos, outward_dir
        return new_pos

    # --------------------------
    # Incremental local relax
    # --------------------------
    def _incremental_relax(self, seed_idx, local_rounds=3):
        per_set = set(self.periphery.getIndices())
        local = set([seed_idx])
        local.update(self._adjacency.get(seed_idx, set()))
        for u in list(local):
            local.update(self._adjacency.get(u, set()))

        for _ in range(max(1, local_rounds)):
            for i in list(local):
                if i in per_set or not self.vertices[i].isVisible():
                    continue
                neigh = self._adjacency.get(i, set())
                if not neigh:
                    continue
                sx = sy = 0.0
                cnt = 0
                for nb in neigh:
                    p = self.vertices[nb].getPosition()
                    sx += p.x(); sy += p.y(); cnt += 1
                if cnt > 0:
                    self.vertices[i].setPosition(QPointF(sx / cnt, sy / cnt))

    # --------------------------
    # Hard min-distance separation (grid accelerated)
    # --------------------------
    def _separate_min_dist_grid(self, pinned, dmin, iters=2, clamp_step=None, only_indices=None):
        if dmin <= 0: return
        if clamp_step is None: clamp_step = dmin * 0.6

        def build_index_set():
            if only_indices is None:
                return [i for i in range(len(self.vertices)) if self.vertices[i].isVisible()]
            return [i for i in only_indices if self.vertices[i].isVisible()]

        for _ in range(max(1, iters)):
            idxs = build_index_set()
            cell = dmin; inv_cell = 1.0 / max(1e-9, cell)
            grid = {}
            pos = {i: self.vertices[i].getPosition() for i in idxs}
            for i in idxs:
                p = pos[i]
                cx = int(math.floor(p.x() * inv_cell))
                cy = int(math.floor(p.y() * inv_cell))
                grid.setdefault((cx, cy), []).append(i)

            moves = {i: QPointF(0, 0) for i in idxs if i not in pinned}
            for i in idxs:
                pi = pos[i]
                cxi = int(math.floor(pi.x() * inv_cell))
                cyi = int(math.floor(pi.y() * inv_cell))
                for dx in (-1, 0, 1):
                    for dy in (-1, 0, 1):
                        cell_list = grid.get((cxi + dx, cyi + dy), [])
                        for j in cell_list:
                            if j <= i: continue
                            pj = pos[j]
                            dxij = pj.x() - pi.x(); dyij = pj.y() - pi.y()
                            dist = math.hypot(dxij, dyij)
                            if dist < 1e-9 or dist >= dmin: continue
                            ov = (dmin - dist)
                            if dist < 1e-9:
                                ux, uy = 1.0, 0.0
                            else:
                                ux, uy = dxij / dist, dyij / dist
                            mi = ov * 0.5; mj = ov * 0.5
                            if i in pinned and j in pinned:
                                continue
                            elif i in pinned:
                                mi = 0.0; mj = ov
                            elif j in pinned:
                                mi = ov; mj = 0.0

                            if i not in pinned:
                                mv = v_scale(QPointF(-ux, -uy), min(mi, clamp_step))
                                moves[i] = v_add(moves.get(i, QPointF(0, 0)), mv)
                            if j not in pinned:
                                mv = v_scale(QPointF(ux, uy), min(mj, clamp_step))
                                moves[j] = v_add(moves.get(j, QPointF(0, 0)), mv)

            for i, mv in moves.items():
                if mv.x() == 0 and mv.y() == 0: continue
                p = self.vertices[i].getPosition()
                self.vertices[i].setPosition(v_add(p, mv))

    # --------------------------
    # Grid-accelerated repulsion (for redraw cycles)
    # --------------------------
    def _repulsion_grid(self, pinned, cutoff, strength):
        if cutoff <= 0 or strength <= 0: return {}

        cell = cutoff; inv_cell = 1.0 / max(1e-9, cell)
        grid = {}; pos_cache = {}
        N = len(self.vertices)
        for i in range(N):
            v = self.vertices[i]
            if not v.isVisible(): continue
            p = v.getPosition()
            pos_cache[i] = p
            cx = int(math.floor(p.x() * inv_cell))
            cy = int(math.floor(p.y() * inv_cell))
            grid.setdefault((cx, cy), []).append(i)

        forces = {}
        for i in range(N):
            if i in pinned: continue
            vi = self.vertices[i]
            if not vi.isVisible(): continue
            pi = pos_cache[i]
            cx = int(math.floor(pi.x() * inv_cell))
            cy = int(math.floor(pi.y() * inv_cell))
            neigh_cells = [(cx + dx, cy + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]
            accx = accy = 0.0
            for cc in neigh_cells:
                if cc not in grid: continue
                for j in grid[cc]:
                    if j == i: continue
                    if j in self._adjacency.get(i, set()): continue
                    pj = pos_cache[j]
                    dx = pi.x() - pj.x(); dy = pi.y() - pj.y()
                    dist = math.hypot(dx, dy)
                    if dist < 1e-6 or dist > cutoff: continue
                    t = 1.0 - dist / cutoff
                    fmag = strength * (t * t)
                    accx += (dx / dist) * fmag
                    accy += (dy / dist) * fmag
            if accx != 0.0 or accy != 0.0:
                forces[i] = QPointF(accx, accy)
        return forces

    # --------------------------
    # Congestion-aware forces (declump dense patches)
    # --------------------------
    def _congestion_forces(self, pinned, cutoff, beta, thresh, target_len):
        if cutoff <= 0 or beta <= 0: return {}

        cell = cutoff; inv_cell = 1.0 / max(1e-9, cell)
        grid = {}; pos_cache = {}
        N = len(self.vertices)
        for i in range(N):
            v = self.vertices[i]
            if not v.isVisible(): continue
            p = v.getPosition()
            pos_cache[i] = p
            cx = int(math.floor(p.x() * inv_cell))
            cy = int(math.floor(p.y() * inv_cell))
            grid.setdefault((cx, cy), []).append(i)

        forces = {}
        for i in range(N):
            if i in pinned: continue
            if not self.vertices[i].isVisible(): continue
            pi = pos_cache[i]
            cx = int(math.floor(pi.x() * inv_cell))
            cy = int(math.floor(pi.y() * inv_cell))
            neigh_cells = [(cx + dx, cy + dy) for dx in (-1, 0, 1) for dy in (-1, 0, 1)]

            cnt = 0; sx = sy = 0.0
            for cc in neigh_cells:
                if cc not in grid: continue
                for j in grid[cc]:
                    if j == i: continue
                    if j in self._adjacency.get(i, set()): continue
                    pj = pos_cache[j]
                    dx = pi.x() - pj.x(); dy = pi.y() - pj.y()
                    dist = math.hypot(dx, dy)
                    if dist <= cutoff:
                        cnt += 1; sx += pj.x(); sy += pj.y()

            if cnt >= thresh:
                cxn = sx / cnt; cyn = sy / cnt
                dx = pi.x() - cxn; dy = pi.y() - cyn
                d = math.hypot(dx, dy)
                if d > 1e-9:
                    scale = min(0.6, (cnt - thresh + 1) / (thresh))
                    fmag = beta * scale
                    fx = dx / d * fmag; fy = dy / d * fmag
                    forces[i] = v_add(forces.get(i, QPointF(0, 0)), QPointF(fx, fy))
        return forces

    # --------------------------
    # Long-edge reducer (aggressively contracts very long edges)
    # --------------------------
    def _long_edge_reduce(self, pinned, target_length, hi1=1.30, hi2=1.85,
                          gain1=0.55, gain2=0.75, max_step=None, rounds=1):
        if target_length <= 0:
            return
        if max_step is None:
            max_step = 0.16 * target_length

        for _ in range(max(1, rounds)):
            for e in self.edges:
                if not e.isVisible(): continue
                u = e.getStartVertex().getIndex()
                v = e.getEndVertex().getIndex()
                pu = self.vertices[u].getPosition()
                pv = self.vertices[v].getPosition()
                dx = pv.x() - pu.x(); dy = pv.y() - pu.y()
                dist = math.hypot(dx, dy)
                if dist < 1e-9:
                    continue

                ratio = dist / target_length
                if ratio <= hi1:
                    continue

                # stronger as ratio approaches hi2
                t = (ratio - hi1) / max(1e-9, (hi2 - hi1))
                t = max(0.0, min(1.0, t))
                gain = (1 - t) * gain1 + t * gain2

                corr = (dist - target_length) * gain
                ux, uy = dx / dist, dy / dist

                # Move endpoints toward each other to shorten
                if (u not in pinned) and (v not in pinned):
                    mv = min(max_step, 0.5 * corr)
                    self.vertices[u].setPosition(QPointF(pu.x() + ux * mv, pu.y() + uy * mv))
                    self.vertices[v].setPosition(QPointF(pv.x() - ux * mv, pv.y() - uy * mv))
                elif (u in pinned) and (v not in pinned):
                    mv = min(max_step, corr)
                    self.vertices[v].setPosition(QPointF(pv.x() - ux * mv, pv.y() - uy * mv))
                elif (v in pinned) and (u not in pinned):
                    mv = min(max_step, corr)
                    self.vertices[u].setPosition(QPointF(pu.x() + ux * mv, pu.y() + uy * mv))
                # both pinned: skip

    # --------------------------
    # Short-edge boost (opens tiny triangles)
    # --------------------------
    def _short_edge_boost(self, pinned, target_length, factor=0.34, thresh=0.66, max_step=None, rounds=2):
        if target_length <= 0: return
        if max_step is None: max_step = target_length * 0.12

        for _ in range(max(1, rounds)):
            forces = {}
            for e in self.edges:
                if not e.isVisible(): continue
                u = e.getStartVertex().getIndex()
                v = e.getEndVertex().getIndex()
                if u in pinned and v in pinned: continue
                pu = self.vertices[u].getPosition()
                pv = self.vertices[v].getPosition()
                diff = v_sub(pv, pu)
                dist = v_len(diff)
                if dist < 1e-9: continue
                if dist < thresh * target_length:
                    deficit = (thresh * target_length - dist)
                    fmag = factor * deficit
                    dir_uv = v_scale(diff, 1.0 / dist)
                    fvec = v_scale(dir_uv, fmag)
                    if u not in pinned:
                        forces[u] = v_sub(forces.get(u, QPointF(0, 0)), fvec)
                    if v not in pinned:
                        forces[v] = v_add(forces.get(v, QPointF(0, 0)), fvec)
            for i, f in forces.items():
                flen = v_len(f)
                if flen > max_step:
                    f = v_scale(f, max_step / max(1e-9, flen))
                pos = self.vertices[i].getPosition()
                self.vertices[i].setPosition(v_add(pos, f))

    # --------------------------
    # Edge-length homogenizer (soft)
    # --------------------------
    def _edge_length_homogenize(self, pinned, target_length,
                                lo=0.94, hi=1.06, gain=0.50,
                                max_step=None, rounds=2):
        if target_length <= 1e-9: return
        if max_step is None: max_step = target_length * 0.10

        for _ in range(max(1, rounds)):
            for e in self.edges:
                if not e.isVisible(): continue
                u = e.getStartVertex().getIndex()
                v = e.getEndVertex().getIndex()
                if u in pinned and v in pinned: continue

                pu = self.vertices[u].getPosition()
                pv = self.vertices[v].getPosition()
                dx = pv.x() - pu.x(); dy = pv.y() - pu.y()
                dist = math.hypot(dx, dy)
                if dist < 1e-9: continue

                ratio = dist / target_length
                if lo <= ratio <= hi: continue

                ux, uy = dx / dist, dy / dist
                corr = (target_length - dist) * gain

                if u in pinned and v not in pinned:
                    mvx, mvy = ux * corr, uy * corr
                    mv_len = math.hypot(mvx, mvy)
                    if mv_len > max_step:
                        s = max_step / mv_len
                        mvx, mvy = mvx * s, mvy * s
                    self.vertices[v].setPosition(QPointF(pv.x() + mvx, pv.y() + mvy))

                elif v in pinned and u not in pinned:
                    mvx, mvy = -ux * corr, -uy * corr
                    mv_len = math.hypot(mvx, mvy)
                    if mv_len > max_step:
                        s = max_step / mv_len
                        mvx, mvy = mvx * s, mvy * s
                    self.vertices[u].setPosition(QPointF(pu.x() + mvx, pu.y() + mvy))

                else:
                    mvux, mvuy = -ux * (0.5 * corr), -uy * (0.5 * corr)
                    mvvx, mvvy =  ux * (0.5 * corr),  uy * (0.5 * corr)
                    len_u = math.hypot(mvux, mvuy)
                    if len_u > max_step:
                        s = max_step / len_u
                        mvux, mvuy = mvux * s, mvuy * s
                    len_v = math.hypot(mvvx, mvvy)
                    if len_v > max_step:
                        s = max_step / len_v
                        mvvx, mvvy = mvvx * s, mvvy * s
                    self.vertices[u].setPosition(QPointF(pu.x() + mvux, pu.y() + mvuy))
                    self.vertices[v].setPosition(QPointF(pv.x() + mvvx, pv.y() + mvvy))

    # --------------------------
    # Strict unit-length projector (near-exact, with crossing guard)
    # --------------------------
    def _snapshot_positions(self):
        return [self.vertices[i].getPosition() for i in range(len(self.vertices))]

    def _restore_positions(self, snap):
        for i, p in enumerate(snap):
            self.vertices[i].setPosition(p)

    def _has_crossings_quick(self, cell_size=None, max_reports=1):
        edges = [(e.getStartVertex().getIndex(), e.getEndVertex().getIndex())
                 for e in self.edges if e.isVisible()]
        if len(edges) <= 1:
            return False

        if cell_size is None:
            cell_size = self._quick_target_length()

        pts = [v.getPosition() for v in self.vertices]
        grid = {}
        aabbs = []

        def put(aabb, idx):
            (minx, miny, maxx, maxy) = aabb
            cx0 = int(math.floor(minx / cell_size)); cx1 = int(math.floor(maxx / cell_size))
            cy0 = int(math.floor(miny / cell_size)); cy1 = int(math.floor(maxy / cell_size))
            for cx in range(cx0, cx1 + 1):
                for cy in range(cy0, cy1 + 1):
                    grid.setdefault((cx, cy), []).append(idx)

        for idx, (u, v) in enumerate(edges):
            p1, p2 = pts[u], pts[v]
            aabb = (min(p1.x(), p2.x()), min(p1.y(), p2.y()),
                    max(p1.x(), p2.x()), max(p1.y(), p2.y()))
            aabbs.append(aabb)
            put(aabb, idx)

        seen = set()
        hits = 0
        for cell, eidxs in grid.items():
            L = len(eidxs)
            if L < 2: continue
            for ii in range(L):
                i = eidxs[ii]
                u1, v1 = edges[i]
                a, b = pts[u1], pts[v1]
                for jj in range(ii + 1, L):
                    j = eidxs[jj]
                    key = (min(i, j), max(i, j))
                    if key in seen: continue
                    seen.add(key)
                    u2, v2 = edges[j]
                    if u1 in (u2, v2) or v1 in (u2, v2): continue
                    c, d = pts[u2], pts[v2]
                    A = aabbs[i]; B = aabbs[j]
                    if (A[2] < B[0]) or (B[2] < A[0]) or (A[3] < B[1]) or (B[3] < A[1]):
                        continue
                    if self._segments_properly_intersect(a, b, c, d):
                        hits += 1
                        if hits >= max_reports:
                            return True
        return False

    def enforce_unit_lengths_strict(self, pinned, target_length, tol=1.2e-3,
                                    max_rounds=60, clamp_frac=0.08, separation_iters=1):
        if target_length <= 1e-9 or not self.edges:
            return

        step = max(1e-6, clamp_frac * target_length)
        best_snap = self._snapshot_positions()
        best_err = float('inf')

        for it in range(max(1, max_rounds)):
            max_abs = 0.0

            for e in self.edges:
                if not e.isVisible(): continue
                u = e.getStartVertex().getIndex()
                v = e.getEndVertex().getIndex()
                pu = self.vertices[u].getPosition()
                pv = self.vertices[v].getPosition()
                dx = pv.x() - pu.x(); dy = pv.y() - pu.y()
                dist = math.hypot(dx, dy)
                if dist < 1e-9: continue

                err = (target_length - dist)  # positive => lengthen
                max_abs = max(max_abs, abs(err))
                if abs(err) <= tol * target_length:
                    continue

                ux, uy = dx / dist, dy / dist

                if (u not in pinned) and (v not in pinned):
                    mu = max(-step, min(step, -0.5 * err))
                    mv = max(-step, min(step,  0.5 * err))
                    self.vertices[u].setPosition(QPointF(pu.x() + ux * mu, pu.y() + uy * mu))
                    self.vertices[v].setPosition(QPointF(pv.x() + ux * mv, pv.y() + uy * mv))
                elif (u in pinned) and (v not in pinned):
                    mv = max(-step, min(step, err))
                    self.vertices[v].setPosition(QPointF(pv.x() + ux * mv, pv.y() + uy * mv))
                elif (v in pinned) and (u not in pinned):
                    mu = max(-step, min(step, -err))
                    self.vertices[u].setPosition(QPointF(pu.x() + ux * mu, pu.y() + uy * mu))
                # both pinned: skip

            self._separate_min_dist_grid(pinned=pinned, dmin=self._min_sep_length(),
                                         iters=separation_iters, clamp_step=step * 1.0)

            if self._has_crossings_quick(cell_size=target_length, max_reports=1):
                self._restore_positions(best_snap)
                step *= 0.6
                if step < 1e-6:
                    break
                continue

            if max_abs < best_err:
                best_err = max_abs
                best_snap = self._snapshot_positions()

            if max_abs <= tol * target_length:
                break

            step = min(step * 1.05, clamp_frac * target_length)

        self._restore_positions(best_snap)

    # --------------------------
    # Local star unit-length projector
    # --------------------------
    def enforce_unit_lengths_local(self, center_idx: int, pinned, target_length: float,
                                   rounds: int = 28, clamp_frac: float = 0.10, tol: float = 9e-4) -> bool:
        neigh = list(self._adjacency.get(center_idx, set()))
        if not neigh or target_length <= 1e-9:
            return True

        step = max(1e-6, clamp_frac * target_length)
        snap = self.vertices[center_idx].getPosition()
        x = snap
        deg = len(neigh)

        for _ in range(max(1, rounds)):
            mvx = mvy = 0.0
            max_abs = 0.0
            for nb in neigh:
                p = self.vertices[nb].getPosition()
                dx = x.x() - p.x()
                dy = x.y() - p.y()
                dist = math.hypot(dx, dy)
                if dist < 1e-9:
                    continue
                err = target_length - dist
                max_abs = max(max_abs, abs(err))
                ux, uy = dx / dist, dy / dist
                mvx += ux * err
                mvy += uy * err

            if deg > 0:
                mvx /= deg
                mvy /= deg

            mvlen = math.hypot(mvx, mvy)
            if mvlen <= tol * target_length * 0.15:
                break

            gamma = 0.85 if deg <= 3 else 0.65
            if mvlen > step:
                s = step / mvlen
                mvx *= s
                mvy *= s

            x = QPointF(x.x() + gamma * mvx, x.y() + gamma * mvy)
            self.vertices[center_idx].setPosition(x)

            if max_abs <= tol * target_length:
                break

        local = set(neigh) | {center_idx}
        self._separate_min_dist_grid(pinned=set(pinned), dmin=self._min_sep_length(),
                                     iters=1, clamp_step=step, only_indices=local)

        if self._has_crossings_quick(max_reports=1):
            self.vertices[center_idx].setPosition(snap)
            return False
        return True

    # --------------------------
    # Tutte relaxation (positive weights, baseline)
    # --------------------------
    def _tutte_relax(self, pinned, rounds=8, jitter_eps=0.03, gauss_seidel=False):
        eps = max(0.0, min(0.2, float(jitter_eps)))

        def hash01(i, j):
            h = (i * 73856093) ^ (j * 19349663)
            h &= 0xffffffff
            return (h % 10007) / 10007.0

        for _ in range(max(1, rounds)):
            if gauss_seidel:
                for i, v in enumerate(self.vertices):
                    if i in pinned or not v.isVisible(): continue
                    neigh = self._adjacency.get(i, set())
                    if not neigh: continue
                    sx = sy = sw = 0.0
                    for nb in neigh:
                        w = 1.0 + eps * (2.0 * hash01(i, nb) - 1.0)
                        p = self.vertices[nb].getPosition()
                        sx += w * p.x(); sy += w * p.y(); sw += w
                    if sw > 1e-12:
                        self.vertices[i].setPosition(QPointF(sx / sw, sy / sw))
            else:
                newpos = {}
                for i, v in enumerate(self.vertices):
                    if i in pinned or not v.isVisible(): continue
                    neigh = self._adjacency.get(i, set())
                    if not neigh: continue
                    sx = sy = sw = 0.0
                    for nb in neigh:
                        w = 1.0 + eps * (2.0 * hash01(i, nb) - 1.0)
                        p = self.vertices[nb].getPosition()
                        sx += w * p.x(); sy += w * p.y(); sw += w
                    if sw > 1e-12:
                        newpos[i] = QPointF(sx / sw, sy / sw)
                for i, p in newpos.items():
                    self.vertices[i].setPosition(p)

    # --------------------------
    # Adaptive positive-weight Tutte (planarity-safe IRLS)
    # --------------------------
    def _tutte_relax_adaptive(self, pinned, target_len, rounds=8, gamma=1.25,
                              wmin=0.35, wmax=3.2, gauss_seidel=True):
        """
        Planarity-safe: positive weights + convex boundary.
        Weights w_ij = clamp( (L_ij / target_len)^gamma, [wmin, wmax] ).
        Long edges (L > target) get larger weights (shrink), short edges get smaller weights (grow).
        """
        if target_len <= 1e-9:
            return

        for _ in range(max(1, rounds)):
            if gauss_seidel:
                # In-place updates (faster convergence)
                for i, v in enumerate(self.vertices):
                    if (i in pinned) or (not v.isVisible()):
                        continue
                    neigh = self._adjacency.get(i, set())
                    if not neigh:
                        continue
                    sx = sy = sw = 0.0
                    pi = v.getPosition()
                    for j in neigh:
                        pj = self.vertices[j].getPosition()
                        dx = pj.x() - pi.x()
                        dy = pj.y() - pi.y()
                        L = math.hypot(dx, dy)
                        r = L / target_len if target_len > 1e-9 else 1.0
                        w = max(wmin, min(wmax, pow(max(1e-9, r), gamma)))
                        sx += w * pj.x()
                        sy += w * pj.y()
                        sw += w
                    if sw > 1e-12:
                        self.vertices[i].setPosition(QPointF(sx / sw, sy / sw))
            else:
                # Jacobi
                newpos = {}
                for i, v in enumerate(self.vertices):
                    if (i in pinned) or (not v.isVisible()):
                        continue
                    neigh = self._adjacency.get(i, set())
                    if not neigh:
                        continue
                    sx = sy = sw = 0.0
                    pi = v.getPosition()
                    for j in neigh:
                        pj = self.vertices[j].getPosition()
                        dx = pj.x() - pi.x()
                        dy = pj.y() - pi.y()
                        L = math.hypot(dx, dy)
                        r = L / target_len if target_len > 1e-9 else 1.0
                        w = max(wmin, min(wmax, pow(max(1e-9, r), gamma)))
                        sx += w * pj.x()
                        sy += w * pj.y()
                        sw += w
                    if sw > 1e-12:
                        newpos[i] = QPointF(sx / sw, sy / sw)
                for i, p in newpos.items():
                    self.vertices[i].setPosition(p)

    # --------------------------
    # Fan spreading
    # --------------------------
    def _spread_fans(self, pinned, angle_min_deg=7.0, iters=3, k_frac=0.06):
        import math
        from PyQt5.QtCore import QPointF as _QPF

        angle_min = math.radians(max(0.1, angle_min_deg))
        target = self._quick_target_length()
        max_step = target * max(0.01, min(0.2, k_frac))

        def v_len_local(a): return math.hypot(a.x(), a.y())

        for _ in range(max(1, iters)):
            forces = {}
            for u in range(len(self.vertices)):
                if not self.vertices[u].isVisible(): continue
                pu = self.vertices[u].getPosition()
                neigh = list(self._adjacency.get(u, set()))
                if len(neigh) < 2: continue

                dirs = []
                for nb in neigh:
                    pv = self.vertices[nb].getPosition()
                    dx = pv.x() - pu.x(); dy = pv.y() - pu.y()
                    L = math.hypot(dx, dy)
                    if L < 1e-6: continue
                    dirs.append((math.atan2(dy, dx), nb, dx / L, dy / L))
                if len(dirs) < 2: continue
                dirs.sort(key=lambda t: t[0])

                m = len(dirs)
                for k in range(m):
                    ang_i, nb_i, dix, diy = dirs[k]
                    ang_j, nb_j, djx, djy = dirs[(k + 1) % m]
                    diff = ang_j - ang_i
                    if diff <= 0: diff += 2 * math.pi
                    if diff >= angle_min: continue

                    cross = dix * djy - diy * djx
                    sign = 1.0 if cross > 0 else -1.0
                    deficit = (angle_min - diff) / angle_min
                    mag = min(max_step, target * 0.06 * deficit)

                    if sign > 0:
                        ti = _QPF(diy * mag, -dix * mag)
                        tj = _QPF(-djy * mag, djx * mag)
                    else:
                        ti = _QPF(-diy * mag, dix * mag)
                        tj = _QPF(djy * mag, -djx * mag)

                    if nb_i not in pinned and self.vertices[nb_i].isVisible():
                        forces[nb_i] = v_add(forces.get(nb_i, _QPF(0, 0)), ti)
                    elif u not in pinned:
                        forces[u] = v_sub(forces.get(u, _QPF(0, 0)), ti)

                    if nb_j not in pinned and self.vertices[nb_j].isVisible():
                        forces[nb_j] = v_add(forces.get(nb_j, _QPF(0, 0)), tj)
                    elif u not in pinned:
                        forces[u] = v_sub(forces.get(u, _QPF(0, 0)), tj)

            for i, f in forces.items():
                fl = v_len_local(f)
                if fl > max_step:
                    f = v_scale(f, max_step / max(1e-9, fl))
                pos = self.vertices[i].getPosition()
                self.vertices[i].setPosition(v_add(pos, f))

    # --------------------------
    # Near-rim spreading
    # --------------------------
    def _spread_near_rim(self, pinned, center, R, angle_min_deg=12.0, iters=2, k_frac=0.07, band=(0.88, 0.98)):
        import math
        from PyQt5.QtCore import QPointF as _QPF

        angle_min = math.radians(max(0.1, angle_min_deg))
        target = self._quick_target_length()
        max_step = target * max(0.01, min(0.25, k_frac))

        def v_len_local(a): return math.hypot(a.x(), a.y())

        for _ in range(max(1, iters)):
            forces = {}
            for u in range(len(self.vertices)):
                if not self.vertices[u].isVisible(): continue
                pu = self.vertices[u].getPosition()
                dx0 = pu.x() - center.x(); dy0 = pu.y() - center.y()
                ru = math.hypot(dx0, dy0)
                if u in pinned or (ru <= band[0] * R or ru >= band[1] * R): continue

                neigh = list(self._adjacency.get(u, set()))
                if len(neigh) < 2: continue

                dirs = []
                for nb in neigh:
                    pv = self.vertices[nb].getPosition()
                    dx = pv.x() - pu.x(); dy = pv.y() - pu.y()
                    L = math.hypot(dx, dy)
                    if L < 1e-6: continue
                    dirs.append((math.atan2(dy, dx), nb, dx / L, dy / L))
                if len(dirs) < 2: continue
                dirs.sort(key=lambda t: t[0])

                tx, ty = dy0 / max(ru, 1e-9), -dx0 / max(ru, 1e-9)

                m = len(dirs)
                for k in range(m):
                    ang_i, nb_i, dix, diy = dirs[k]
                    ang_j, nb_j, djx, djy = dirs[(k + 1) % m]
                    rim_min = math.radians(16.0)
                    nb_i_on_rim = (nb_i in pinned)
                    nb_j_on_rim = (nb_j in pinned)
                    local_min = rim_min if (nb_i_on_rim and nb_j_on_rim) else angle_min

                    diff = ang_j - ang_i
                    if diff <= 0: diff += 2 * math.pi
                    if diff >= local_min: continue

                    cross = dix * djy - diy * djx
                    sgn = 1.0 if cross > 0 else -1.0

                    deficit = (local_min - diff) / local_min
                    mag = min(max_step, target * 0.085 * deficit)

                    tu = _QPF(tx * mag * sgn, ty * mag * sgn)
                    rin = target * 0.05 * deficit
                    ru_safe = max(ru, 1e-9)
                    ru_ix = -dx0 / ru_safe * rin; ru_iy = -dy0 / ru_safe * rin

                    if u not in pinned:
                        forces[u] = v_add(forces.get(u, _QPF(0, 0)), _QPF(tu.x() + ru_ix, tu.y() + ru_iy))
                    else:
                        if nb_i not in pinned and self.vertices[nb_i].isVisible():
                            forces[nb_i] = v_add(forces.get(nb_i, _QPF(0, 0)), _QPF(tx * mag, ty * mag))
                        if nb_j not in pinned and self.vertices[nb_j].isVisible():
                            forces[nb_j] = v_add(forces.get(nb_j, _QPF(0, 0)), _QPF(-tx * mag, -ty * mag))

            for i, f in forces.items():
                fl = v_len_local(f)
                if fl > max_step:
                    f = v_scale(f, max_step / max(1e-9, fl))
                pos = self.vertices[i].getPosition()
                self.vertices[i].setPosition(v_add(pos, f))

    # --------------------------
    # Ultra homogeneity helpers
    # --------------------------
    def _edge_exact_project_axis(self, u: int, v: int, target: float,
                                 clamp_step: float, pinned) -> bool:
        if u in pinned and v in pinned:
            return False
        pu = self.vertices[u].getPosition()
        pv = self.vertices[v].getPosition()
        dx = pv.x() - pu.x(); dy = pv.y() - pu.y()
        dist = math.hypot(dx, dy)  # FIXED (was mistakenly assigning to math.rphypot)
        if dist < 1e-9:
            return False
        ux, uy = dx / dist, dy / dist
        delta = target - dist  # >0 => lengthen, <0 => shorten

        pu_new, pv_new = pu, pv
        if (u not in pinned) and (v not in pinned):
            alpha = 0.5 * delta
            alpha = max(-clamp_step, min(clamp_step, alpha))
            pu_new = QPointF(pu.x() - ux * alpha, pu.y() - uy * alpha)
            pv_new = QPointF(pv.x() + ux * alpha, pv.y() + uy * alpha)
        elif (u in pinned) and (v not in pinned):
            beta = max(-clamp_step, min(clamp_step, delta))
            pv_new = QPointF(pv.x() + ux * beta, pv.y() + uy * beta)
        elif (v in pinned) and (u not in pinned):
            beta = max(-clamp_step, min(clamp_step, -delta))
            pu_new = QPointF(pu.x() - ux * beta, pu.y() - uy * beta)

        snap_u = self.vertices[u].getPosition()
        snap_v = self.vertices[v].getPosition()
        self.vertices[u].setPosition(pu_new)
        self.vertices[v].setPosition(pv_new)

        # Guard: no crossings
        if self._has_crossings_quick(cell_size=target, max_reports=1):
            self.vertices[u].setPosition(snap_u)
            self.vertices[v].setPosition(snap_v)
            return False
        return True

    def polish_homogeneity_ultra(self, pinned, light=False):
        if not self.ultra_homog:
            return
        target = self._quick_target_length()
        if target <= 1e-9 or not self.edges:
            return

        # Gather visible edges with absolute relative error
        edges = []
        for e in self.edges:
            if not e.isVisible():
                continue
            u = e.getStartVertex().getIndex()
            v = e.getEndVertex().getIndex()
            pu = self.vertices[u].getPosition()
            pv = self.vertices[v].getPosition()
            L = math.hypot(pv.x() - pu.x(), pv.y() - pu.y())
            if L > 1e-9:
                err = abs(L / target - 1.0)
                edges.append((err, u, v))

        E = len(edges)
        if E < 2:
            return  # nothing meaningful to adjust

        # For tiny graphs, do a light strict pass and skip the top-K projection
        if E < 6:
            self.enforce_unit_lengths_strict(
                pinned=pinned,
                target_length=target,
                tol=max(self.ultra_tol_small, 8e-4),
                max_rounds=30,
                clamp_frac=min(self.ultra_clamp_frac, 0.10),
                separation_iters=1 if light else 2
            )
            return

        # Sort worst-first
        edges.sort(reverse=True, key=lambda t: t[0])

        # How many edges to project (clamped)
        frac = (self.ultra_edge_proj_frac_small if len(self.vertices) <= self.ultra_small_threshold
                else self.ultra_edge_proj_frac_large)
        if light:
            frac *= 0.6
        K = max(1, int(frac * E))
        K = min(K, E)  # CLAMP to avoid IndexError

        clamp_step = self.ultra_clamp_frac * target
        changed = False
        for k in range(K):
            _, u, v = edges[k]
            if self._edge_exact_project_axis(u, v, target, clamp_step, pinned):
                changed = True

        if changed:
            # Light separation to keep nodes from colliding
            self._separate_min_dist_grid(pinned=pinned, dmin=self._min_sep_length(),
                                        iters=1 if light else 2, clamp_step=target * 0.08)

        # Finish with a final strict pass (tighter) to polish
        tol = self.ultra_tol_small if len(self.vertices) <= self.ultra_small_threshold else self.ultra_tol_large
        rounds = self.ultra_rounds_small if len(self.vertices) <= self.ultra_small_threshold else self.ultra_rounds_large
        clamp_frac = min(self.ultra_clamp_frac, 0.10 if light else self.ultra_clamp_frac)
        self.enforce_unit_lengths_strict(
            pinned=pinned,
            target_length=target,
            tol=tol,
            max_rounds=rounds,
            clamp_frac=clamp_frac,
            separation_iters=1 if light else 2
        )

    # --------------------------
    # Homogeneity quick metric (p95)
    # --------------------------
    def _homog_p95_ratio(self) -> float:
        T = self._quick_target_length()
        if T <= 1e-9 or not self.edges:
            return 1.0
        ratios = []
        for e in self.edges:
            if not e.isVisible():
                continue
            u = e.getStartVertex().getIndex()
            v = e.getEndVertex().getIndex()
            p1 = self.vertices[u].getPosition()
            p2 = self.vertices[v].getPosition()
            L = math.hypot(p2.x() - p1.x(), p2.y() - p1.y())
            ratios.append(L / T)
        if not ratios:
            return 1.0
        ratios.sort()
        i = min(len(ratios) - 1, int(0.95 * len(ratios)))
        return ratios[i]

    def _auto_polish_after_add(self, pinned):
        # Up to N focused polish passes until p95 is tight enough
        for _ in range(max(0, int(self.auto_polish_max_passes))):
            if self._homog_p95_ratio() <= float(self.auto_polish_p95):
                break
            T = self._quick_target_length()
            # Strong planarity-safe push toward unit lengths
            self._tutte_relax_adaptive(
                pinned=pinned,
                target_len=T,
                rounds=7,
                gamma=1.35,
                wmin=0.30, wmax=4.0,
                gauss_seidel=True
            )
            # Clamp very long edges and gently expand short ones
            self._long_edge_reduce(pinned, T, hi1=1.18, hi2=1.70,
                                   gain1=0.60, gain2=0.85,
                                   max_step=0.18 * T, rounds=1)
            self._edge_length_homogenize(pinned, T, lo=0.95, hi=1.05,
                                         gain=0.60, max_step=0.12 * T, rounds=1)
            # Tight strict pass with guard
            self.enforce_unit_lengths_strict(
                pinned=pinned,
                target_length=T,
                tol=4e-4,
                max_rounds=90,
                clamp_frac=0.10,
                separation_iters=2
            )
            # Keep a little spacing
            self._separate_min_dist_grid(pinned=pinned, dmin=self._min_sep_length(), iters=1)

    # --------------------------
    # Redraw (global)
    # --------------------------
# graph.py

    def redraw_planar(self, iterations=None, radius=None, light=False):
        per = self.periphery.getIndices()
        if len(per) < 3:
            return

        V = len(self.vertices)
        if iterations is None:
            if V < 500: iters = 30
            elif V < 2000: iters = 22
            else: iters = 14
        else:
            iters = max(1, iterations)

        cx = sum(self.vertices[i].getPosition().x() for i in per) / len(per)
        cy = sum(self.vertices[i].getPosition().y() for i in per) / len(per)
        center = QPointF(cx, cy)

        tl_exact = self._compute_target_edge_length()
        if tl_exact > 0:
            if self.lock_target_len:
                if self._target_len is None:
                    self._target_len = tl_exact
            else:
                self._target_len = (self._target_len + tl_exact) * 0.5 if self._target_len else tl_exact
        target_length = self._quick_target_length()

        if radius is None:
            m = len(per)
            if m >= 3 and math.sin(math.pi / m) > 1e-12:
                radius = max(200.0, target_length / (2.0 * math.sin(math.pi / m)))
            else:
                radius = 200.0

        m = len(per)
        for k, vi in enumerate(per):
            theta = 2 * math.pi * k / m - math.pi / 2.0
            x = center.x() + radius * math.cos(theta)
            y = center.y() + radius * math.sin(theta)
            self.vertices[vi].setPosition(QPointF(x, y))
        pinned = set(per)

        # Tutte-like averaging (warm-up)
        for _ in range(iters if not light else max(1, min(4, iters))):
            for v in self.vertices:
                if v.getIndex() in pinned or not v.isVisible():
                    continue
                neigh = self._adjacency.get(v.getIndex(), set())
                if not neigh: continue
                sx = sy = 0.0; cnt = 0
                for nb in neigh:
                    p = self.vertices[nb].getPosition()
                    sx += p.x(); sy += p.y(); cnt += 1
                if cnt > 0:
                    self.vertices[v.getIndex()].setPosition(QPointF(sx / cnt, sy / cnt))
        
        # ---- START OF THE CORRECTED SECTION ----
        
        # Setup parameters from the configuration object
        cfg = self.config.light if light else self.config.default

        if V < self.config.small_graph_v_thresh:
            cycles = 6 if not light else 2
        elif V < self.config.medium_graph_v_thresh:
            cycles = 8 if not light else 3
        else:
            cycles = 10 if not light else 4

        spring_k = cfg.spring_k
        max_step = target_length * cfg.max_step_frac
        rep_cutoff = target_length * cfg.repulsion_cutoff_frac
        rep_strength = target_length * cfg.repulsion_strength_frac
        cong_cutoff = target_length * cfg.congestion_cutoff_frac
        cong_beta = target_length * cfg.congestion_beta_frac
        cong_thresh = cfg.congestion_thresh
        final_tutte_rounds = cfg.tutte_rounds
        sep_iters = cfg.separation_iters
        seboost_rounds = cfg.short_edge_boost_rounds

        # Main force-directed loop
        for _ in range(cycles):
            forces = {i: QPointF(0, 0) for i in range(V)
                    if i not in pinned and self.vertices[i].isVisible()}
            
            # Edge spring forces
            for e in self.edges:
                if not e.isVisible(): continue
                u, v = e.getStartVertex().getIndex(), e.getEndVertex().getIndex()
                if u in pinned and v in pinned: continue
                pu, pv = self.vertices[u].getPosition(), self.vertices[v].getPosition()
                diff = v_sub(pv, pu)
                dist = v_len(diff)
                if dist < 1e-9: continue
                dir_uv = v_scale(diff, 1.0 / dist)
                fmag = (dist - target_length) * spring_k
                fvec = v_scale(dir_uv, fmag)
                if u not in pinned: forces[u] = v_add(forces[u], fvec)
                if v not in pinned: forces[v] = v_sub(forces[v], fvec)
            
            # Repulsion forces
            rep_f = self._repulsion_grid(pinned, cutoff=rep_cutoff, strength=rep_strength)
            for i, f in rep_f.items(): forces[i] = v_add(forces.get(i, QPointF(0, 0)), f)

            # Congestion forces
            cong_f = self._congestion_forces(pinned, cutoff=cong_cutoff, beta=cong_beta,
                                            thresh=cong_thresh, target_len=target_length)
            for i, f in cong_f.items(): forces[i] = v_add(forces.get(i, QPointF(0, 0)), f)

            # Inward push to contain the graph
            R = radius; t0 = 0.90; inward_k = target_length * (0.24 if light else 0.28)
            for i in list(forces.keys()):
                p = self.vertices[i].getPosition()
                dx, dy = p.x() - center.x(), p.y() - center.y()
                r = math.hypot(dx, dy)
                if r > 1e-9 and r > t0 * R:
                    over = (r / R) - t0
                    push = inward_k * (over * over)
                    forces[i] = v_add(forces[i], QPointF(-dx / r * push, -dy / r * push))

            # Apply all forces
            for i, f in forces.items():
                flen = v_len(f)
                if flen > max_step:
                    f = v_scale(f, max_step / max(1e-9, flen))
                pos = self.vertices[i].getPosition()
                self.vertices[i].setPosition(v_add(pos, f))

            self._tutte_relax(pinned, rounds=1, jitter_eps=0.02, gauss_seidel=True)


        # Planarity-safe IRLS: adaptive positive-weight Tutte to push lengths toward target
        small = (len(self.vertices) <= 1200)
        for _ in range(3 if light else (4 if small else 3)):
            self._tutte_relax_adaptive(
                pinned=pinned,
                target_len=target_length,
                rounds=5 if light else (7 if small else 6),
                gamma=1.20 if light else (1.30 if small else 1.25),
                wmin=0.32, wmax=3.6 if small else 3.2,
                gauss_seidel=True
            )

        # Aggressively shrink only the very long edges
        self._long_edge_reduce(pinned, target_length,
                               hi1=1.30, hi2=1.85,
                               gain1=0.55, gain2=0.75,
                               max_step=max_step, rounds=1 if light else 2)

        self._short_edge_boost(pinned, target_length, factor=0.34, thresh=0.66, max_step=max_step, rounds=seboost_rounds)
        self._spread_fans(pinned, angle_min_deg=9.0, iters=1 if light else 2, k_frac=0.05 if light else 0.06)
        self._spread_near_rim(pinned, center, radius,
                              angle_min_deg=12.0, iters=1 if light else 2, k_frac=0.06 if light else 0.07,
                              band=(0.88, 0.98))

        if light:
            self._edge_length_homogenize(pinned, target_length, lo=0.94, hi=1.06, gain=0.55,
                                         max_step=max_step, rounds=2)
        else:
            self._edge_length_homogenize(pinned, target_length, lo=0.95, hi=1.05, gain=0.60,
                                         max_step=max_step, rounds=3)

        self._separate_min_dist_grid(pinned=pinned, dmin=self._min_sep_length(), iters=sep_iters)
        
                # Global vertex–edge clearance (one pass is enough)
        edge_clear_glob = max(0.5 * self._min_sep_length(), 0.50 * target_length)
        self._ve_clearance_grid(
            pinned=pinned,
            clear_dist=edge_clear_glob,
            iters=1 if light else 2,
            clamp_step=0.22 * target_length
        )

        self._tutte_relax(pinned, rounds=final_tutte_rounds, jitter_eps=0.015, gauss_seidel=True)

# graph.py — replace the END of redraw_planar with this block (keep everything above unchanged)

        # Strict unit-length pass (near-exact) with crossing guard
        if self.strict_homog_enabled:
            if V <= 1200:
                rounds = self.homog_max_rounds_small
            elif V <= 5000:
                rounds = self.homog_max_rounds_medium
            else:
                rounds = self.homog_max_rounds_large
            self.enforce_unit_lengths_strict(
                pinned=pinned,
                target_length=target_length,
                tol=self.homog_tol if not light else max(self.homog_tol, 2.5e-3),
                max_rounds=rounds if not light else max(8, rounds // 3),
                clamp_frac=self.homog_clamp_frac,
                separation_iters=1 if light else 2
            )

        # Final planarity audit and rescue
        if self._has_crossings_quick(max_reports=1):
            # Pure Tutte (positive weights) with pinned convex boundary
            for _ in range(45 if not light else 25):
                self._tutte_relax(pinned, rounds=1, jitter_eps=0.0, gauss_seidel=True)
            # Light spacing to remove near-overlaps
            self._separate_min_dist_grid(
                pinned=pinned,
                dmin=self._min_sep_length(),
                iters=1,
                clamp_step=self._quick_target_length() * 0.08
            )
            # Short strict unit-length polish with crossing guard
            self.enforce_unit_lengths_strict(
                pinned=pinned,
                target_length=target_length,
                tol=max(self.homog_tol, 1.5e-3 if light else 1.0e-3),
                max_rounds=40 if not light else 22,
                clamp_frac=min(self.homog_clamp_frac, 0.10),
                separation_iters=1
            )

        # Finish: normalize and notify UI
        self._normalize_and_recenter(max_radius=self._MAX_WORKING_RADIUS)
        if self.on_layout_changed:
            self.on_layout_changed(self.get_bounding_box(), self._periphery_center(), self.target_edge_px)
    # --------------------------
    # Normalize & recenter
    # --------------------------
    def _normalize_and_recenter(self, max_radius=None):
        per = self.periphery.getIndices()
        if not per: return
        center = self._periphery_center()

        tx = -center.x(); ty = -center.y()
        if abs(tx) > 1e-6 or abs(ty) > 1e-6:
            for v in self.vertices:
                p = v.getPosition()
                self.vertices[v.getIndex()].setPosition(QPointF(p.x() + tx, p.y() + ty))

        if max_radius is None:
            return

        max_r = 0.0
        for v in self.vertices:
            p = v.getPosition()
            r = math.hypot(p.x(), p.y())
            if r > max_r: max_r = r

        if max_r > max_radius:
            s = max_radius / max(1e-9, max_r)
            for v in self.vertices:
                p = v.getPosition()
                self.vertices[v.getIndex()].setPosition(QPointF(p.x() * s, p.y() * s))
            if self._target_len:
                self._target_len *= s

    # --------------------------
    # Validation and stats
    # --------------------------
    def validate_coloring(self):
        bad = []
        for e in self.edges:
            u = e.getStartVertex().getIndex()
            v = e.getEndVertex().getIndex()
            if self.vertices[u].getColorIndex() == self.vertices[v].getColorIndex():
                bad.append((u, v))
        return bad

    def get_stats(self):
        seed = sum(1 for v in self.vertices if v and v.getOrigin() == "seed")
        random_cnt = sum(1 for v in self.vertices if v and v.getOrigin() == "random")
        manual = sum(1 for v in self.vertices if v and v.getOrigin() == "manual")
        return {
            "total_vertices": len(self.vertices),
            "seed": seed,
            "random": random_cnt,
            "manual": manual,
            "edges": len(self.edges),
            "periphery_size": len(self.periphery.getIndices()),
        }

    def homogeneity_report(self, visible_only=True):
        lens = []
        for e in self.edges:
            if visible_only and not e.isVisible():
                continue
            p1 = e.getStartVertex().getPosition()
            p2 = e.getEndVertex().getPosition()
            lens.append(v_len(v_sub(p2, p1)))
        if not lens:
            return {}

        T = self._quick_target_length()
        if T <= 1e-9:
            return {"edges": len(lens), "target_length": T}

        ratios = [L / T for L in lens]
        ratios_sorted = sorted(ratios)
        p95 = ratios_sorted[int(max(0, min(len(ratios_sorted) - 1, 0.95 * len(ratios_sorted))))]
        from statistics import mean, pstdev
        return {
            "edges": len(lens),
            "target_length": T,
            "mean_ratio": mean(ratios),
            "std_ratio": pstdev(ratios) if len(ratios) > 1 else 0.0,
            "p95_ratio": p95,
            "min_ratio": ratios_sorted[0],
            "max_ratio": ratios_sorted[-1],
        }

    # --------------------------
    # Persistence
    # --------------------------
    def save_to_json(self, filepath):
        try:
            verts = [v for v in self.vertices if v is not None]
            data = {
                "vertices": [{
                    "index": v.getIndex(),
                    "pos": [v.getPosition().x(), v.getPosition().y()],
                    "color": v.getColorIndex(),
                    "origin": v.getOrigin(),
                } for v in verts],
                "edges": [{"start": e.getStartVertex().getIndex(), "end": e.getEndVertex().getIndex()} for e in self.edges],
                "periphery": self.periphery.getIndices(),
                "labelMode": int(self.labelMode),
                "target_length": self._target_len
            }
            with open(filepath, 'w') as f:
                json.dump(data, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving graph: {e}")
            return False

    def load_from_json(self, filepath):
        self.clear()
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)

            self.vertices = [None] * len(data["vertices"])
            for v_data in data["vertices"]:
                pos = QPointF(v_data["pos"][0], v_data["pos"][1])
                v_idx = v_data["index"]
                origin = v_data.get("origin", "manual")
                self.vertices[v_idx] = Vertex(v_idx, pos, v_data.get("color", (v_idx % 4) + 1), origin=origin)
                self._adjacency[v_idx] = set()

            for e_data in data["edges"]:
                start_v = self.vertices[e_data["start"]]
                end_v = self.vertices[e_data["end"]]
                self._add_edge_internal(Edge(start_v, end_v))

            self.periphery.initialize(data["periphery"])

            if "labelMode" in data:
                self.labelMode = int(data["labelMode"])
            elif "showVertexIndices" in data:  # legacy files
                self.labelMode = 1 if data.get("showVertexIndices", False) else 0
            else:
                self.labelMode = 2  # default: colored + numbered

            self._target_len = data.get("target_length", None)
            if self._target_len is None:
                self._target_len = self._compute_target_edge_length()

            self.goToVertex(len(self.vertices))
            self._normalize_and_recenter(max_radius=self._MAX_WORKING_RADIUS)
            return True
        except Exception as e:
            print(f"Error loading graph: {e}")
            self.startBasicGraph()
            return False

    # --------------------------
    # Getters (used by UI)
    # --------------------------
    def getVertices(self):
        return self.vertices

    def getEdges(self):
        return self.edges

    def getPeriphery(self):
        return self.periphery.getIndices()

    # --- Label mode helpers ---
    def get_label_mode(self) -> int:
        return self.labelMode

    def cycle_label_mode(self) -> int:
        self.labelMode = (self.labelMode + 1) % 3
        return self.labelMode

    # --- Animation info for last addition ---
    def get_last_add_info(self):
        return self._last_add_info

    # --------------------------
    # World/UI helpers and flags
    # --------------------------

    def set_max_working_radius(self, limit: Optional[float]):
        self._MAX_WORKING_RADIUS = float(limit) if (limit is not None) else None

    def get_max_working_radius(self) -> Optional[float]:
        return self._MAX_WORKING_RADIUS

    def get_bounding_box(self, visible_only=True):
        xs = []; ys = []
        for v in self.vertices:
            if visible_only and not v.isVisible(): continue
            p = v.getPosition()
            xs.append(p.x()); ys.append(p.y())
        if not xs:
            return (0.0, 0.0, 0.0, 0.0)
        return (min(xs), min(ys), max(xs), max(ys))

    def get_center(self) -> QPointF:
        return self._periphery_center()

    def set_auto_expand(self, enabled: bool):
        self.auto_expand = bool(enabled)

    def get_auto_expand(self) -> bool:
        return self.auto_expand

    def set_auto_expand_mode(self, mode: str):
        mode_l = (mode or "").strip().lower()
        if mode_l in ("fit", "infinite"):
            self.auto_expand_mode = mode_l

    def get_auto_expand_mode(self) -> str:
        return self.auto_expand_mode

    def set_view_padding(self, px: float):
        self.view_padding = float(px)

    def get_view_padding(self) -> float:
        return self.view_padding

    def set_target_edge_px(self, px: float):
        self.target_edge_px = float(px)

    def get_target_edge_px(self) -> float:
        return self.target_edge_px

    # --------------------------
    # Coloring helpers (restored)
    # --------------------------
    def _greedy_color(self, vid: int):
        # Choose the smallest color in {1..4} not used by neighbors
        used = {self.vertices[n].getColorIndex() for n in self._adjacency.get(vid, set())}
        c = 1
        while c in used and c < 4:
            c += 1
        self.vertices[vid].setColorIndex(c)

    def setVertexColor(self, vertex_index, color_id):
        try:
            cid = int(color_id)
        except Exception:
            return False
        if not (1 <= cid <= 4):
            return False
        if 0 <= vertex_index < len(self.vertices):
            self.vertices[vertex_index].setColorIndex(cid)
            return True
        return False
    
    def validate_invariants(self, verbose=False) -> bool:
        ok = True
        per = list(self.periphery.getIndices())
        h = len(per)
        V = len(self.vertices)
        E = sum(1 for e in self.edges if e.isVisible())

        if not self.periphery.validate():
            ok = False
            if verbose: print("Periphery map inconsistent or duplicates.")

        for i in range(h):
            u = per[i]; v = per[(i + 1) % h]
            if v not in self._adjacency.get(u, set()):
                ok = False
                if verbose: print(f"Missing periphery edge ({u}, {v}).")

        for u, nbrs in self._adjacency.items():
            if u < 0 or u >= V or self.vertices[u] is None:
                ok = False
                if verbose: print(f"Bad vertex in adjacency: {u}")
                continue
            if u in nbrs:
                ok = False
                if verbose: print(f"Self-loop at {u}")
            for v in nbrs:
                if v < 0 or v >= V or self.vertices[v] is None:
                    ok = False
                    if verbose: print(f"Invalid neighbor {v} for {u}")
                    continue
                if u not in self._adjacency.get(v, set()):
                    ok = False
                    if verbose: print(f"Asymmetry: {u} has {v}, but {v} missing {u}")

        if V >= 3 and h >= 3:
            expected_E = 3 * V - 3 - h
            if E != expected_E:
                ok = False
                if verbose: print(f"E={E}, expected={expected_E} (V={V}, h={h})")

        return ok
