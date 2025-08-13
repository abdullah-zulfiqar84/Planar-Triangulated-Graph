# app.py
import os, sys, types, math
import streamlit as st
import plotly.graph_objects as go

# ----- PyQt5 stub so your engine imports work -----
if 'PyQt5' not in sys.modules:
    pkg = types.ModuleType('PyQt5')
    sys.modules['PyQt5'] = pkg
if 'PyQt5.QtCore' not in sys.modules:
    qtcore = types.ModuleType('PyQt5.QtCore')
    class QPointF:
        __slots__ = ('_x','_y')
        def __init__(self, x=0.0, y=0.0): self._x=float(x); self._y=float(y)
        def x(self): return self._x
        def y(self): return self._y
    qtcore.QPointF = QPointF
    sys.modules['PyQt5.QtCore'] = qtcore

# ----- Add py/ to sys.path and import your engine -----
ROOT = os.path.dirname(__file__)
PYDIR = os.path.join(ROOT, 'py')
if PYDIR not in sys.path:
    sys.path.append(PYDIR)

from graph import Graph  # your file
# -----------------------------------------------------

st.set_page_config(page_title="Planar Triangulated Graph (Web)", layout="wide")

# Session state
if 'graph' not in st.session_state:
    g = Graph()
    g.startBasicGraph(3)
    st.session_state.graph = g

g: Graph = st.session_state.graph

# UI
col_btns, col_plot = st.columns([1, 4], gap="large")

with col_btns:
    st.markdown("### Controls")
    if st.button("Start Triangle"):
        g.startBasicGraph(3)
    if st.button("Add Random Vertex"):
        ok, _ = g.addRandomVertex()
        if not ok:
            st.warning("No valid random move (rule would be violated).")
    st.divider()

    # Manual add
    per = list(g.getPeriphery())
    if len(per) >= 2:
        # Show 1-based labels in UI but store zero-based indices
        labels = [f"{i+1}" for i in per]
        idx_map = {f"{i+1}": i for i in per}
        vp_label = st.selectbox("Pick Vp (on periphery)", labels, key="vp")
        vq_label = st.selectbox("Pick Vq (on periphery)", labels, key="vq")
        if st.button("Add Vertex by Selection"):
            vp = int(vp_label) - 1
            vq = int(vq_label) - 1
            ok, _ = g.addVertexBySelection(vp, vq)
            if not ok:
                st.error("Invalid periphery segment or rule would be violated.")
    st.divider()
    # Info
    stats = g.get_stats()
    st.markdown(
        f"Vertices: {stats['total_vertices']}  \n"
        f"Periphery size: {stats['periphery_size']}  \n"
        f"Edges: {stats['edges']}  \n"
        f"Seed: {stats['seed']}, Random: {stats['random']}, Manual: {stats['manual']}"
    )

# --------- Plotting helpers (Plotly) ----------
def periphery_is_ccw(per, vertices):
    if len(per) < 3:
        return True
    area2 = 0.0
    for i in range(len(per)):
        a = vertices[per[i]].getPosition()
        b = vertices[per[(i+1)%len(per)]].getPosition()
        area2 += a.x()*b.y() - b.x()*a.y()
    return area2 > 0.0

def is_peri_edge(per, u, v):
    if not per: return False
    n = len(per)
    try:
        iu = per.index(u)
    except ValueError:
        return False
    return per[(iu+1)%n] == v or per[(iu-1)%n] == v

def periphery_control_point(p1, p2, ccw, degrees=30.0):
    # p1,p2 are QPointF
    dx = p2.x() - p1.x(); dy = p2.y() - p1.y()
    L = math.hypot(dx, dy) or 1.0
    nx = (dy/L) if ccw else (-dy/L)
    ny = (-dx/L) if ccw else (dx/L)
    mx = (p1.x()+p2.x())*0.5; my = (p1.y()+p2.y())*0.5
    theta = math.radians(degrees)
    denom = 2.0*math.sin(theta/2.0) or 1.0
    sag = L*(1.0 - math.cos(theta/2.0)) / denom
    cx = mx + nx*2.0*sag
    cy = my + ny*2.0*sag
    return cx, cy

def quad_bezier_points(x1, y1, cx, cy, x2, y2, steps=20):
    ptsx = []; ptsy = []
    for i in range(steps+1):
        t = i/steps
        # Quadratic Bezier: B(t)= (1-t)^2 P0 + 2(1-t)t C + t^2 P1
        x = (1-t)*(1-t)*x1 + 2*(1-t)*t*cx + t*t*x2
        y = (1-t)*(1-t)*y1 + 2*(1-t)*t*cy + t*t*y2
        ptsx.append(x); ptsy.append(y)
    return ptsx, ptsy

def colors_for_idx(idx):
    # same palette as your desktop UI
    outline = ['#e74c3c','#3498db','#2ecc71','#f1c40f']
    fill    = ['#ff6b6b','#48dbfb','#1dd1a1','#feca57']
    return outline[(idx-1)%4], fill[(idx-1)%4]

# --------- Build Plotly figure ----------
with col_plot:
    fig = go.Figure()

    per = list(g.getPeriphery())
    verts = g.getVertices()
    edges = g.getEdges()
    ccw = periphery_is_ccw(per, verts)

    # Draw edges
    for e in edges:
        if not e.isVisible():
            continue
        u = e.getStartVertex().getIndex()
        v = e.getEndVertex().getIndex()
        p1 = e.getStartVertex().getPosition()
        p2 = e.getEndVertex().getPosition()
        if is_peri_edge(per, u, v):
            cx, cy = periphery_control_point(p1, p2, ccw, degrees=30.0)
            xs, ys = quad_bezier_points(p1.x(), p1.y(), cx, cy, p2.x(), p2.y(), steps=24)
        else:
            xs, ys = [p1.x(), p2.x()], [p1.y(), p2.y()]
        fig.add_trace(go.Scatter(
            x=xs, y=ys,
            mode='lines',
            line=dict(color='rgba(60,60,60,1)', width=1),
            hoverinfo='skip',
            showlegend=False
        ))

    # Draw vertices
    vx = []; vy = []; txt = []; marker_color=[]; marker_line=[]
    for v in verts:
        if not v or not v.isVisible(): continue
        p = v.getPosition()
        vx.append(p.x()); vy.append(p.y())
        txt.append(str(v.getIndex()+1))
        oc, fc = colors_for_idx(v.getColorIndex())
        marker_color.append(fc); marker_line.append(oc)

    fig.add_trace(go.Scatter(
        x=vx, y=vy, mode='markers+text',
        text=txt, textposition='middle center',
        marker=dict(
            size=18, color=marker_color,
            line=dict(width=2, color=marker_line),
        ),
        hoverinfo='skip',
        showlegend=False
    ))

    fig.update_yaxes(scaleanchor="x", scaleratio=1)
    fig.update_layout(
        margin=dict(l=20, r=20, t=10, b=10),
        xaxis=dict(visible=False), yaxis=dict(visible=False),
        dragmode='pan', height=700
    )
    st.plotly_chart(fig, use_container_width=True)