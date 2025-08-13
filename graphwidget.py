# graphwidget.py

from PyQt5.QtWidgets import (
    QGraphicsView, QGraphicsScene, QGraphicsEllipseItem,
    QGraphicsSimpleTextItem, QMessageBox, QFileDialog, QMenu
)

from PyQt5.QtCore import Qt, QPointF, QTimeLine, QRectF, QTimer
from PyQt5.QtGui import QPen, QColor, QPainter, QImage, QPainterPath, QTransform
from PyQt5.QtWidgets import QGraphicsScene as QGS
from typing import Optional
from graph import Graph
from utils_geom import v_add, v_sub, v_scale, v_len, v_lerp, v_mid
import math

# Zoom behavior constants (stabilized)
ZOOM_FACTOR = 1.15
ZOOM_MAX = 5000.0
MIN_ABS_SCALE = 1e-3
MIN_REL_TO_FIT = 0.25

# Animation tuning (60 FPS + slower center)
ANIM_FPS = 60
ANIM_DT_MS = 16                  # ~60 FPS (1000/60 ≈ 16.67; Qt uses ints, so 16 is closest)
ADD_ANIM_MS = 420                # vertex position animation duration
CENTER_ANIM_MS = 600             # center-graph camera animation duration (slower, smoother)
REFRAME_ANIM_MS = 480            # if you ever call smart reframe

# Strict planar render: draw all edges as straight lines (no curves)
STRICT_PLANAR_RENDER = False

# Fixed curve angles (degrees) - used only if STRICT_PLANAR_RENDER = False
PERIPHERY_CURVE_DEG = 30.0
INTERIOR_CURVE_DEG = 15.0


class GraphWidget(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.graph = Graph()
        scene = QGraphicsScene(self)
        scene.setItemIndexMethod(QGS.NoIndex)
        self.setScene(scene)

        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.NoDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setViewportUpdateMode(QGraphicsView.SmartViewportUpdate)
        self.setCacheMode(QGraphicsView.CacheBackground)

        self.currentZoom = 1.0
        self._base_fit_scale = None
        self.selectingPeriphery = False
        self.peripheryStartIndex = -1
        self.panning = False
        self.lastPanPoint = None

        self._cameraTimeline = None
        self._highlightTimeline = None
        self._highlightedVertex = -1
        self._highlightProgress = 0.0
        self._animTimeline = None  # vertex position animation

        self.SAFE_MODE = True
        self._in_update_scene = False

        self.palette_outline = [
            QColor("#e74c3c"),
            QColor("#3498db"),
            QColor("#2ecc71"),
            QColor("#f1c40f"),
        ]
        self.palette_fill = [
            QColor("#ff6b6b"),
            QColor("#48dbfb"),
            QColor("#1dd1a1"),
            QColor("#feca57"),
        ]

        # Clean background (no grid)
        self.gridEnabled = False
        self.gridBackground = QColor(255, 255, 255)

        # Aggregation thresholds (10k+ friendly)
        self.AGGREGATE_EDGES_FROM = 2500
        self.AGGREGATE_VERTICES_FROM = 2500

        # Hook: ignore auto camera moves during animations
        self.graph.on_layout_changed = self._onLayoutChanged

    # --------------------------
    # Animation helpers (60 FPS)
    # --------------------------

    def _makeTimeline(self, duration_ms: int) -> QTimeLine:
        tl = QTimeLine(int(duration_ms), self)
        tl.setUpdateInterval(ANIM_DT_MS)  # ~60 FPS
        tl.setCurveShape(QTimeLine.EaseInOutCurve)
        return tl

    def _snapshot_positions(self):
        snap = {}
        for v in self.graph.getVertices():
            if not v:
                continue
            p = v.getPosition()
            snap[v.getIndex()] = QPointF(p.x(), p.y())
        return snap

    def _apply_positions_snapshot(self, snap):
        for idx, p in snap.items():
            if 0 <= idx < len(self.graph.vertices) and self.graph.vertices[idx]:
                self.graph.vertices[idx].setPosition(p)

    def _apply_interpolated_positions(self, pre, post, t):
        keys = set(pre.keys()) | set(post.keys())
        for idx in keys:
            a = pre.get(idx, post.get(idx))
            b = post.get(idx, pre.get(idx))
            if a is None or b is None:
                continue
            pos = v_lerp(a, b, t)
            self.graph.vertices[idx].setPosition(pos)
        self.updateGraphScene()

    def _animate_transition(self, pre, post, duration_ms=ADD_ANIM_MS):
        # Stop camera anim if running to avoid blending
        if self._cameraTimeline and self._cameraTimeline.state() == QTimeLine.Running:
            self._cameraTimeline.stop()
        if self._highlightTimeline and self._highlightTimeline.state() == QTimeLine.Running:
            self._highlightTimeline.stop()

        # Start from 'pre'
        self._apply_positions_snapshot(pre)
        self.updateGraphScene()

        tl = self._makeTimeline(duration_ms)
        tl.valueChanged.connect(lambda t: self._apply_interpolated_positions(pre, post, t))

        def finish():
            # Snap to final layout
            self._apply_positions_snapshot(post)
            self.updateGraphScene()
            # Smoothly center to the exact view that centerGraph() would produce
            self.animateCenterGraph(duration_ms=CENTER_ANIM_MS)

        tl.finished.connect(finish)
        tl.start()
        self._animTimeline = tl

    def animateCenterGraph(self, duration_ms=CENTER_ANIM_MS):
        # Equivalent final state to centerGraph(), but animated (no jerk)
        if not self.scene().items():
            return
        rect = self.scene().itemsBoundingRect()
        adjust = max(50, len(self.graph.getVertices()) / 100)
        target_rect = rect.adjusted(-adjust, -adjust, adjust, adjust)
        self._animateToSceneRect(target_rect, duration_ms)

    def _animateToSceneRect(self, target_rect: QRectF, duration_ms=REFRAME_ANIM_MS):
        if target_rect.isEmpty():
            return
        start_transform = self.transform()
        self.fitInView(target_rect, Qt.KeepAspectRatio)
        end_transform = self.transform()
        self.setTransform(start_transform)

        if self._cameraTimeline and self._cameraTimeline.state() == QTimeLine.Running:
            self._cameraTimeline.stop()

        timeline = self._makeTimeline(duration_ms)
        timeline.valueChanged.connect(
            lambda t: self.setTransform(
                self._interpolateTransform(start_transform, end_transform, t)
            )
        )
        timeline.finished.connect(lambda: setattr(self, "_base_fit_scale", self.transform().m11()))
        timeline.start()
        self._cameraTimeline = timeline

    def _interpolateTransform(self, a: QTransform, b: QTransform, t: float) -> QTransform:
        t = max(0.0, min(1.0, t))
        m11 = a.m11() + (b.m11() - a.m11()) * t
        m22 = a.m22() + (b.m22() - a.m22()) * t
        m12 = a.m12() + (b.m12() - a.m12()) * t
        m21 = a.m21() + (b.m21() - a.m21()) * t
        dx = a.m31() + (b.m31() - a.m31()) * t
        dy = a.m32() + (b.m32() - a.m32()) * t
        return QTransform(m11, m12, m21, m22, dx, dy)

    # Optional: runtime control from your UI
    def setAnimationFps(self, fps: int):
        global ANIM_FPS, ANIM_DT_MS
        ANIM_FPS = int(max(1, fps))
        ANIM_DT_MS = max(1, int(round(1000.0 / ANIM_FPS)))

    def setAnimationDurations(self, add_ms: Optional[int] = None,
                              center_ms: Optional[int] = None,
                              reframe_ms: Optional[int] = None):
        global ADD_ANIM_MS, CENTER_ANIM_MS, REFRAME_ANIM_MS
        if add_ms is not None: ADD_ANIM_MS = int(max(1, add_ms))
        if center_ms is not None: CENTER_ANIM_MS = int(max(1, center_ms))
        if reframe_ms is not None: REFRAME_ANIM_MS = int(max(1, reframe_ms))

    # --------------------------
    # Auto-layout hook (disable during animations)
    # --------------------------
    def _onLayoutChanged(self, bbox_tuple, center_point: QPointF, target_edge_px: float):
        # Avoid camera changes mid-vertex animation
        if self._animTimeline and self._animTimeline.state() == QTimeLine.Running:
            return
        # We intentionally do nothing here; camera is centered smoothly after each add.

    # --------------------------
    # Graph lifecycle
    # --------------------------
    def setGridEnabled(self, flag: bool):
        self.gridEnabled = bool(flag)
        self.viewport().update()

    def confirmStartGraph(self):
        # Ask for confirmation only if we already have a graph with > 3 vertices
        if len(self.graph.getVertices()) > 3:
            reply = QMessageBox.question(
                self, "Confirm Reset",
                "This will reset the current graph. Are you sure?",
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            if reply != QMessageBox.Yes:
                return

        # Prompt for number of seed vertices (3..10 per client spec)
        from PyQt5.QtWidgets import QInputDialog
        n, ok = QInputDialog.getInt(
            self, "New Graph",
            "Number of seed vertices (3..10):",
            value=3, min=3, max=10
        )
        if not ok:
            return
        self.startBasicGraph(n)

    def startBasicGraph(self, n: int = 3):
        self.resetTransform()
        self.currentZoom = 1.0

        self.graph.startBasicGraph(n)
        try:
            self.parent().spinGoTo.setMaximum(100000)
            self.parent().spinGoTo.setValue(len(self.graph.getVertices()))
        except Exception:
            pass
        self.updateGraphScene()
        self.centerGraph()

    def addRandomVertex(self):
        pre = self._snapshot_positions()
        success, new_v_idx = self.graph.addRandomVertex()

        if success:
            try:
                m = len(self.graph.getVertices())
                self.parent().spinGoTo.setValue(m)
                self.goToVertex(m)
            except Exception:
                pass

            # Large graphs: AA off for speed
            if len(self.graph.getVertices()) > 8000:
                self.setRenderHint(QPainter.Antialiasing, False)

            post = self._snapshot_positions()
            info = self.graph.get_last_add_info()
            if info and info.get("index") not in pre:
                pre[info["index"]] = info["spawn_pos"]
            self._animate_transition(pre, post, duration_ms=ADD_ANIM_MS)

            stats = self.graph.get_stats()
            try:
                main_window = self.parent()
                if main_window and hasattr(main_window, 'statusBar'):
                    Linfo = ""
                    if self.graph.last_random_choice:
                        Linfo = f", touches={self.graph.last_random_choice.get('L')}"
                    main_window.statusBar().showMessage(
                        f"Random vertex {new_v_idx + 1} added. Total={stats['total_vertices']} "
                        f"(seed={stats['seed']}, random={stats['random']}, manual={stats['manual']}){Linfo}",
                        3500
                    )
            except Exception:
                pass
        else:
            try:
                self.parent().statusBar().showMessage(
                    "Could not add random vertex. (Rule violated or no valid moves found)", 4000
                )
            except Exception:
                pass

    def addVertexBySelection(self):
        self.selectingPeriphery = True
        self.peripheryStartIndex = -1
        self.setCursor(Qt.CrossCursor)
        try:
            self.parent().statusBar().showMessage("Select the first periphery vertex (Vp).")
        except Exception:
            pass

    def redrawPlanar(self, iterations=200):
        self.graph.redraw_planar(iterations=iterations)
        self.updateGraphScene()
        # If you want auto center after redraws (not adds), uncomment:
        # self.animateCenterGraph(duration_ms=CENTER_ANIM_MS)
        try:
            stats = self.graph.get_stats()
            self.parent().statusBar().showMessage(
                f"Redraw complete. Total={stats['total_vertices']} "
                f"(seed={stats['seed']}, random={stats['random']}, manual={stats['manual']}), "
                f"periphery={stats['periphery_size']}",
                4000
            )
        except Exception:
            pass

    # ---------- Export ----------
    def exportAsImage(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export as Image", "", "PNG Files (*.png);;SVG Files (*.svg)")
        if not path:
            return

        if self.scene().sceneRect().isEmpty():
            self.scene().setSceneRect(self.scene().itemsBoundingRect())

        if path.lower().endswith(".svg"):
            try:
                from PyQt5.QtSvg import QSvgGenerator
            except Exception:
                QMessageBox.warning(self, "SVG Export", "QtSvg module not available. Please export PNG.")
                return
            generator = QSvgGenerator()
            generator.setFileName(path)
            rect = self.scene().itemsBoundingRect().adjusted(-10, -10, 10, 10)
            size = rect.size().toSize()
            if size.width() <= 0 or size.height() <= 0:
                QMessageBox.warning(self, "Export", "Scene rect is empty; cannot export.")
                return
            generator.setSize(size)
            generator.setViewBox(rect)
            painter = QPainter()
            if painter.begin(generator):
                self.scene().render(painter, target=QRectF(0, 0, size.width(), size.height()), source=rect)
                painter.end()
        else:
            rect = self.scene().itemsBoundingRect().adjusted(-10, -10, 10, 10)
            size = rect.size().toSize()
            w = max(1, size.width())
            h = max(1, size.height())

            MAX_DIM = 12000
            scale = min(1.0, MAX_DIM / max(w, h))
            w2 = max(1, int(w * scale))
            h2 = max(1, int(h * scale))

            image = QImage(w2, h2, QImage.Format_ARGB32_Premultiplied)
            image.fill(Qt.transparent)

            painter = QPainter(image)
            if painter.isActive():
                self.scene().render(painter, target=QRectF(0, 0, w2, h2), source=rect)
                painter.end()
                if not image.save(path):
                    QMessageBox.warning(self, "Export", "Failed to save the PNG image.")
            else:
                from PyQt5.QtGui import QPixmap
                pix = QPixmap(w2, h2)
                pix.fill(Qt.transparent)
                p2 = QPainter(pix)
                if p2.isActive():
                    self.scene().render(p2, target=QRectF(0, 0, w2, h2), source=rect)
                    p2.end()
                    if not pix.save(path):
                        QMessageBox.warning(self, "Export", "Failed to save the PNG image (pixmap fallback).")
                else:
                    QMessageBox.warning(self, "Export", "Failed to start painter for image export. Try smaller image or SVG.")

    # ---------- Edge width ----------
    def _edgeWidthPx(self):
        zoom = max(0.0001, self.transform().m11())
        V = len(self.graph.vertices)
        return max(0.25, 2.0 / (1.0 + 0.9 * zoom + 0.00012 * V))

    def _scaleNow(self):
        return max(1e-9, self.transform().m11())

    def _minAllowedScale(self):
        base = self._base_fit_scale if self._base_fit_scale is not None else self._scaleNow()
        return max(MIN_ABS_SCALE, base * MIN_REL_TO_FIT)

    def drawBackground(self, painter, rect):
        painter.fillRect(rect, self.gridBackground)

    # ---------- Helpers (curved code retained but not used in strict mode) ----------
    def _periphery_is_ccw(self) -> bool:
        peri = self.graph.getPeriphery()
        if len(peri) < 3:
            return True
        pts = [self.graph.vertices[i].getPosition() for i in peri]
        area2 = 0.0
        n = len(pts)
        for i in range(n):
            j = (i + 1) % n
            area2 += pts[i].x() * pts[j].y() - pts[j].x() * pts[i].y()
        return area2 > 0.0

    def _outward_normal_for_edge(self, p1: QPointF, p2: QPointF, orient_ccw: bool) -> QPointF:
        dx = p2.x() - p1.x()
        dy = p2.y() - p1.y()
        L = math.hypot(dx, dy)
        if L < 1e-9:
            return QPointF(0.0, 0.0)
        if orient_ccw:
            nx, ny = dy / L, -dx / L
        else:
            nx, ny = -dy / L, dx / L
        return QPointF(nx, ny)

    def _sagitta_for_angle(self, chord_len: float, degrees: float) -> float:
        if degrees <= 0.0 or chord_len <= 0.0:
            return 0.0
        theta = math.radians(degrees)
        s_denom = 2.0 * math.sin(theta / 2.0)
        if abs(s_denom) < 1e-9:
            return 0.0
        return chord_len * (1.0 - math.cos(theta / 2.0)) / s_denom

    def _fixed_quadratic_control_in_face(self, u: int, v: int, p1: QPointF, p2: QPointF, degrees: float) -> Optional[QPointF]:
        au = self.graph._adjacency.get(u, set())
        av = self.graph._adjacency.get(v, set())
        thirds = list(au.intersection(av))
        if not thirds:
            return None

        dx = p2.x() - p1.x()
        dy = p2.y() - p1.y()
        L = math.hypot(dx, dy)
        if L < 1e-9:
            return None
        n = QPointF(-dy / L, dx / L)
        mx, my = (p1.x() + p2.x()) * 0.5, (p1.y() + p2.y()) * 0.5

        best_pw = None
        best_h = 0.0
        best_sgn = 1.0
        for w in thirds:
            pw = self.graph.vertices[w].getPosition()
            signed = (dx * (pw.y() - p1.y()) - dy * (pw.x() - p1.x())) / L
            h = abs(signed)
            sgn = 1.0 if signed > 0 else -1.0
            if h > best_h:
                best_h = h
                best_sgn = sgn
                best_pw = pw
        if best_pw is None:
            return None

        s_target = self._sagitta_for_angle(L, degrees)
        s = min(s_target, 0.45 * best_h, 0.20 * L)
        ctrl = QPointF(mx + best_sgn * n.x() * (2.0 * s), my + best_sgn * n.y() * (2.0 * s))

        tries = 0
        while tries < 10 and not self._point_in_triangle(ctrl, p1, p2, best_pw):
            s *= 0.7
            ctrl = QPointF(mx + best_sgn * n.x() * (2.0 * s), my + best_sgn * n.y() * (2.0 * s))
            tries += 1

        if not self._point_in_triangle(ctrl, p1, p2, best_pw):
            ctrl = QPointF((p1.x() + p2.x() + best_pw.x()) / 3.0, (p1.y() + p2.y() + best_pw.y()) / 3.0)

        return ctrl

    def _point_in_triangle(self, p: QPointF, a: QPointF, b: QPointF, c: QPointF) -> bool:
        def sign(p1, p2, p3):
            return (p1.x() - p3.x()) * (p2.y() - p3.y()) - (p2.x() - p3.x()) * (p1.y() - p3.y())
        b1 = sign(p, a, b) < 0.0
        b2 = sign(p, b, c) < 0.0
        b3 = sign(p, c, a) < 0.0
        return (b1 == b2) and (b2 == b3)

    # ---------- Build star order (kept for potential future needs) ----------
    def _compute_star_geometry(self, visible_only=True):
        V = len(self.graph.vertices)
        star_order = {}
        ang_map = {}
        for u in range(V):
            vtx = self.graph.vertices[u]
            if not vtx or (visible_only and not vtx.isVisible()):
                continue
            pu = vtx.getPosition()
            neigh = list(self.graph._adjacency.get(u, set()))
            pairs = []
            for nb in neigh:
                nb_v = self.graph.vertices[nb]
                if not nb_v or (visible_only and not nb_v.isVisible()):
                    continue
                pv = nb_v.getPosition()
                ang = math.atan2(pv.y() - pu.y(), pv.x() - pu.x())
                pairs.append((ang, nb))
            pairs.sort(key=lambda t: t[0])
            order = [nb for ang, nb in pairs]
            star_order[u] = order
            for ang, nb in pairs:
                ang_map[(u, nb)] = ang
        return star_order, ang_map

    def updateGraphScene(self):
        if self._in_update_scene:
            return
        self._in_update_scene = True
        try:
            self.scene().clear()

            edge_width = self._edgeWidthPx()
            edge_pen = QPen(QColor(60, 60, 60))
            edge_pen.setWidthF(edge_width)
            edge_pen.setCosmetic(True)
            edge_pen.setCapStyle(Qt.RoundCap)
            edge_pen.setJoinStyle(Qt.RoundJoin)

            vertex_pen = QPen(Qt.black, 2)
            vertex_pen.setCosmetic(True)

            all_visible_vertices = [v for v in self.graph.getVertices() if v and v.isVisible()]
            if not all_visible_vertices:
                self.scene().setSceneRect(self.scene().itemsBoundingRect())
                self.viewport().update()
                return

            V = len(self.graph.vertices)
            aggregate_edges = (V >= self.AGGREGATE_EDGES_FROM)

            # --- Edges ---
            use_curved_edges = not STRICT_PLANAR_RENDER
            V = len(self.graph.vertices)
            aggregate_edges = (V >= self.AGGREGATE_EDGES_FROM)

            if aggregate_edges:
                path_all = QPainterPath()
                for edge in self.graph.getEdges():
                    if not edge.isVisible(): 
                        continue
                    p1 = edge.getStartVertex().getPosition()
                    p2 = edge.getEndVertex().getPosition()
                    path_all.moveTo(p1)
                    path_all.lineTo(p2)
                item = self.scene().addPath(path_all, edge_pen)
                item.setZValue(-10)
            else:
                # Precompute periphery info once
                P = self.graph.periphery
                peri = P.getIndices()
                n_peri = len(peri)
                orient_ccw = self._periphery_is_ccw()

                for edge in self.graph.getEdges():
                    if not edge.isVisible():
                        continue

                    start_v = edge.getStartVertex()
                    end_v = edge.getEndVertex()
                    u, v = start_v.getIndex(), end_v.getIndex()
                    p1, p2 = start_v.getPosition(), end_v.getPosition()

                    path = QPainterPath(p1)
                    control_point = None

                    if (not STRICT_PLANAR_RENDER) and n_peri >= 2:
                        # Robust periphery-adjacency check (O(1) with indexOf)
                        iu = P.indexOf(u)
                        is_periphery_edge = False
                        if iu >= 0:
                            is_periphery_edge = (peri[(iu + 1) % n_peri] == v) or (peri[(iu - 1) % n_peri] == v)

                        if is_periphery_edge:
                            # Curve outward along the outer face
                            is_uv_order = True
                            if iu >= 0 and n_peri > 0:
                                is_uv_order = (peri[(iu + 1) % n_peri] == v)
                            p1d = p1 if is_uv_order else p2
                            p2d = p2 if is_uv_order else p1

                            chord_len = v_len(v_sub(p2d, p1d))
                            degrees = PERIPHERY_CURVE_DEG
                            sagitta = self._sagitta_for_angle(chord_len, degrees)
                            midpoint = v_mid(p1d, p2d)
                            outward_normal = self._outward_normal_for_edge(p1d, p2d, orient_ccw)
                            control_point = v_add(midpoint, v_scale(outward_normal, 2.0 * sagitta))

                    if control_point:
                        path.quadTo(control_point, p2)
                    else:
                        path.lineTo(p2)

                    item = self.scene().addPath(path, edge_pen)
                    item.setZValue(-10)

            # Nodes
            label_mode = self.graph.get_label_mode()
            aggregate_vertices = (V >= self.AGGREGATE_VERTICES_FROM) and (label_mode == 0)
            if aggregate_vertices:
                circles_path = QPainterPath()
                for v in self.graph.getVertices():
                    if v and v.isVisible():
                        d, pos = v.getDiameter(), v.getPosition()
                        circles_path.addEllipse(pos.x() - d / 2, pos.y() - d / 2, d, d)
                item = self.scene().addPath(circles_path, vertex_pen, QColor("#ffffff"))
                item.setZValue(10)
            else:
                scale_now = max(1e-6, self.transform().m11())
                for v in self.graph.getVertices():
                    if not v or not v.isVisible():
                        continue
                    d, pos = v.getDiameter(), v.getPosition()

                    if label_mode == 1:
                        pen = QPen(QColor("#000000"), 2)
                        pen.setCosmetic(True)
                        fill_color = QColor("#ffffff")
                    else:
                        color_idx = (v.getColorIndex() - 1) % 4
                        pen = QPen(self.palette_outline[color_idx], 2)
                        pen.setCosmetic(True)
                        fill_color = self.palette_fill[color_idx]

                    ellipse_item = self.scene().addEllipse(
                        pos.x() - d / 2, pos.y() - d / 2, d, d, pen, fill_color
                    )
                    ellipse_item.setZValue(10)

                    if label_mode in (1, 2) and 0.05 <= scale_now <= 40.0:
                        text = QGraphicsSimpleTextItem(str(v.getIndex() + 1))
                        text_rect = text.boundingRect()
                        text.setPos(pos.x() - text_rect.width() / 2, pos.y() - text_rect.height() / 2)
                        text.setBrush(Qt.black)
                        text.setZValue(20)
                        self.scene().addItem(text)

            # Fit scene rect (no view change)
            br = self.scene().itemsBoundingRect()
            if not br.isEmpty():
                self.scene().setSceneRect(br.adjusted(-50, -50, 50, 50))

            self.viewport().update()

        finally:
            self._in_update_scene = False

    def goToVertex(self, m):
        if not self.graph.getVertices() or m > len(self.graph.getVertices()):
            try:
                self.parent().spinGoTo.setValue(len(self.graph.getVertices()))
            except Exception:
                pass
            return
        self.graph.goToVertex(m)
        self.updateGraphScene()

    def zoomIn(self):
        scale_now = self._scaleNow()
        target = min(ZOOM_MAX, scale_now * ZOOM_FACTOR)
        if target <= scale_now + 1e-12:
            return
        factor = target / scale_now
        self.scale(factor, factor)
        self.currentZoom = self.transform().m11()
        self.viewport().update()

    def zoomOut(self):
        scale_now = self._scaleNow()
        min_scale = self._minAllowedScale()
        target = scale_now / ZOOM_FACTOR
        if target <= min_scale + 1e-12:
            return
        factor = target / scale_now
        self.scale(factor, factor)
        self.currentZoom = self.transform().m11()
        self.viewport().update()

    def centerGraph(self):
        if self.scene().items():
            rect = self.scene().itemsBoundingRect()
            adjust = max(50, len(self.graph.vertices) / 100)
            safe_rect = rect.adjusted(-adjust, -adjust, adjust, adjust)
            if safe_rect.width() < 1e-6 or safe_rect.height() < 1e-6:
                return
            self.fitInView(safe_rect, Qt.KeepAspectRatio)
            self.currentZoom = self.transform().m11()
            self._base_fit_scale = self.currentZoom
            self.viewport().update()

    def smoothCenterGraph(self, duration_ms=CENTER_ANIM_MS):
        if not self.scene().items():
            return
        rect = self.scene().itemsBoundingRect()
        adjust = max(50, len(self.graph.vertices) / 100)
        target_rect = rect.adjusted(-adjust, -adjust, adjust, adjust)
        self._animateToSceneRect(target_rect, duration_ms)

    def toggleVertexDisplay(self):
        mode = self.graph.cycle_label_mode()
        try:
            name = ["Color", "Index", "Color + Index"][mode]
            self.parent().statusBar().showMessage(f"Label mode: {name}", 2000)
        except Exception:
            pass
        self.updateGraphScene()

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.zoomIn()
        else:
            self.zoomOut()
        event.accept()

    def mousePressEvent(self, event):
        if self.selectingPeriphery:
            scenePos = self.mapToScene(event.pos())
            vIdx = self.findVertexAtPosition(scenePos)
            if vIdx == -1:
                self.selectingPeriphery = False
                self.setCursor(Qt.ArrowCursor)
                try:
                    self.parent().statusBar().showMessage("Selection cancelled.", 2000)
                except Exception:
                    pass
                return
            if vIdx not in self.graph.getPeriphery():
                try:
                    self.parent().statusBar().showMessage(f"Vertex {vIdx + 1} is not on the periphery.", 3000)
                except Exception:
                    pass
                return
            if self.peripheryStartIndex == -1:
                self.peripheryStartIndex = vIdx
                try:
                    self.parent().statusBar().showMessage(f"Vp={vIdx + 1} selected. Now select the second vertex (Vq).")
                except Exception:
                    pass
            elif vIdx != self.peripheryStartIndex:
                peripheryEndIndex = vIdx
                pre = self._snapshot_positions()
                success, new_v_idx = self.graph.addVertexBySelection(self.peripheryStartIndex, peripheryEndIndex)

                if success:
                    try:
                        self.parent().spinGoTo.setValue(len(self.graph.getVertices()))
                    except Exception:
                        pass
                    post = self._snapshot_positions()
                    info = self.graph.get_last_add_info()
                    if info and info.get("index") not in pre:
                        pre[info["index"]] = info["spawn_pos"]
                    self._animate_transition(pre, post, duration_ms=ADD_ANIM_MS)

                    try:
                        self.parent().statusBar().showMessage(f"New vertex {new_v_idx + 1} added.", 3000)
                    except Exception:
                        pass
                else:
                    QMessageBox.warning(self, "Error", "Could not add vertex.\nReason: Invalid periphery segment.")
                self.selectingPeriphery = False
                self.setCursor(Qt.ArrowCursor)
            return

        if event.button() == Qt.LeftButton:
            self.panning = True
            self.lastPanPoint = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.panning:
            delta = self.mapToScene(self.lastPanPoint) - self.mapToScene(event.pos())
            self.lastPanPoint = event.pos()
            self.translate(delta.x(), delta.y())
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton and self.panning:
            self.panning = False
            self.setCursor(Qt.ArrowCursor)
        super().mouseReleaseEvent(event)

    def contextMenuEvent(self, event):
        menu = QMenu(self)
        menu.addAction("Add Random Vertex (R)", self.addRandomVertex)
        menu.addAction("Add Vertex by Selection (A)", self.addVertexBySelection)
        menu.addSeparator()
        menu.addAction("Redraw (Planar) (D)", self.redrawPlanar)
        menu.addAction("Toggle Labels (T)", self.toggleVertexDisplay)
        menu.addAction("Center Graph (C)", self.centerGraph)
        menu.exec_(event.globalPos())

    def findVertexAtPosition(self, scenePos):
        for v in reversed(self.graph.getVertices()):
            if not v or not v.isVisible():
                continue
            dx = v.getPosition().x() - scenePos.x()
            dy = v.getPosition().y() - scenePos.y()
            dist = math.hypot(dx, dy)
            if dist <= v.getDiameter() / 2:
                return v.getIndex()
        return -1