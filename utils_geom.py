# utils_geom.py

from PyQt5.QtCore import QPointF
import math

EPS = 1e-9

def v_add(a: QPointF, b: QPointF) -> QPointF:
    return QPointF(a.x() + b.x(), a.y() + b.y())

def v_sub(a: QPointF, b: QPointF) -> QPointF:
    return QPointF(a.x() - b.x(), a.y() - b.y())

def v_scale(a: QPointF, s: float) -> QPointF:
    return QPointF(a.x() * s, a.y() * s)

def v_mid(a: QPointF, b: QPointF) -> QPointF:
    return QPointF((a.x() + b.x()) * 0.5, (a.y() + b.y()) * 0.5)

def v_len(a: QPointF) -> float:
    return math.hypot(a.x(), a.y())

def v_len2(a: QPointF) -> float:
    return a.x() * a.x() + a.y() * a.y()

def v_norm(a: QPointF) -> QPointF:
    L = v_len(a)
    return QPointF(0.0, 0.0) if L == 0.0 else v_scale(a, 1.0 / L)

def v_norm_safe(a: QPointF, fallback: QPointF = QPointF(1.0, 0.0)) -> QPointF:
    L = v_len(a)
    return fallback if L < EPS else v_scale(a, 1.0 / L)

def v_dot(a: QPointF, b: QPointF) -> float:
    return a.x() * b.x() + a.y() * b.y()

def v_cross(a: QPointF, b: QPointF) -> float:
    # 2D "z-component" cross product (scalar)
    return a.x() * b.y() - a.y() * b.x()

def v_rot90_ccw(a: QPointF) -> QPointF:
    return QPointF(-a.y(), a.x())

def v_rot90_cw(a: QPointF) -> QPointF:
    return QPointF(a.y(), -a.x())

def v_rotate(a: QPointF, theta_rad: float) -> QPointF:
    c = math.cos(theta_rad)
    s = math.sin(theta_rad)
    return QPointF(a.x() * c - a.y() * s, a.x() * s + a.y() * c)

def v_lerp(a: QPointF, b: QPointF, t: float) -> QPointF:
    # t in [0,1]
    return QPointF(a.x() + (b.x() - a.x()) * t, a.y() + (b.y() - a.y()) * t)

def v_dist(a: QPointF, b: QPointF) -> float:
    return math.hypot(a.x() - b.x(), a.y() - b.y())

def project_point_on_segment(p: QPointF, a: QPointF, b: QPointF):
    """
    Return (q, t) where q is the closest point to p on segment ab,
    and t is the clamped parameter in [0,1].
    """
    ax, ay = a.x(), a.y()
    bx, by = b.x(), b.y()
    px, py = p.x(), p.y()
    abx, aby = (bx - ax), (by - ay)
    denom = abx * abx + aby * aby
    if denom <= EPS:
        return QPointF(ax, ay), 0.0
    t = ((px - ax) * abx + (py - ay) * aby) / denom
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    qx = ax + t * abx
    qy = ay + t * aby
    return QPointF(qx, qy), t

def point_segment_distance(p: QPointF, a: QPointF, b: QPointF) -> float:
    q, _ = project_point_on_segment(p, a, b)
    return v_dist(p, q)

def bbox_of_segment(a: QPointF, b: QPointF):
    return (
        min(a.x(), b.x()), min(a.y(), b.y()),
        max(a.x(), b.x()), max(a.y(), b.y())
    )

def bbox_disjoint(a1: QPointF, a2: QPointF, b1: QPointF, b2: QPointF, pad: float = 0.0) -> bool:
    min_ax = min(a1.x(), a2.x()) - pad; max_ax = max(a1.x(), a2.x()) + pad
    min_ay = min(a1.y(), a2.y()) - pad; max_ay = max(a1.y(), a2.y()) + pad
    min_bx = min(b1.x(), b2.x()) - pad; max_bx = max(b1.x(), b2.x()) + pad
    min_by = min(b1.y(), b2.y()) - pad; max_by = max(b1.y(), b2.y()) + pad
    return (max_ax < min_bx) or (max_bx < min_ax) or (max_ay < min_by) or (max_by < min_ay)

# utils_geom.py (additions)

def orient(a: QPointF, b: QPointF, c: QPointF) -> float:
    # Positive if a->b->c is CCW
    return v_cross(v_sub(b, a), v_sub(c, a))

def on_segment_strict(a: QPointF, b: QPointF, p: QPointF) -> bool:
    # p strictly inside segment ab (not endpoints)
    if abs(orient(a, b, p)) > EPS:
        return False
    minx, maxx = min(a.x(), b.x()), max(a.x(), b.x())
    miny, maxy = min(a.y(), b.y()), max(a.y(), b.y())
    if p.x() <= minx + EPS or p.x() >= maxx - EPS: return False
    if p.y() <= miny + EPS or p.y() >= maxy - EPS: return False
    return True

def segments_intersect_strict(a1: QPointF, a2: QPointF, b1: QPointF, b2: QPointF) -> bool:
    # True if the open segments (a1,a2) and (b1,b2) intersect at a point not shared by endpoints
    if bbox_disjoint(a1, a2, b1, b2, pad=0.0):
        return False
    o1 = orient(a1, a2, b1)
    o2 = orient(a1, a2, b2)
    o3 = orient(b1, b2, a1)
    o4 = orient(b1, b2, a2)

    if o1 == 0.0 and on_segment_strict(a1, a2, b1): return True
    if o2 == 0.0 and on_segment_strict(a1, a2, b2): return True
    if o3 == 0.0 and on_segment_strict(b1, b2, a1): return True
    if o4 == 0.0 and on_segment_strict(b1, b2, a2): return True

    return (o1 * o2 < 0.0) and (o3 * o4 < 0.0)