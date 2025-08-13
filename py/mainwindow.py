# mainwindow.py
from PyQt5.QtWidgets import (
    QMainWindow, QStatusBar, QAction, QFileDialog,
    QMessageBox, QDockWidget, QWidget, QVBoxLayout,
    QPushButton, QSpinBox, QLabel, QGroupBox, QShortcut, QInputDialog, QCheckBox
)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QKeySequence
import traceback
from graphwidget import GraphWidget

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Planar Triangulated Graph Editor")

        self.graphWidget = GraphWidget(self)
        self.setCentralWidget(self.graphWidget)

        self.setStatusBar(QStatusBar(self))

        self.createActions()
        self.createMenuBar()
        self.createControlsDock()
        self.createShortcuts()

        # Start empty. Click New Graph (N/S) to create the seed triangle.

    def createActions(self):
        self.finalizeAction = QAction("&Finalize → G4", self, triggered=self.finalizeAndG4)
        self.newAction = QAction("&New Graph", self, triggered=self.graphWidget.confirmStartGraph)
        self.saveAction = QAction("&Save Graph", self, triggered=self.saveGraph)
        self.loadAction = QAction("&Load Graph", self, triggered=self.loadGraph)
        self.exportAction = QAction("E&xport as Image...", self, triggered=self.graphWidget.exportAsImage)

        self.addRandomAction = QAction("Add &Random Vertex", self, triggered=self.graphWidget.addRandomVertex)
        self.addSelectionAction = QAction("Add Vertex by &Selection", self, triggered=self.graphWidget.addVertexBySelection)
        self.redrawAction = QAction("&Redraw (Planar)", self, triggered=lambda: self.graphWidget.redrawPlanar(iterations=200))
        self.validateColoringAction = QAction("&Validate Coloring", self, triggered=self.validateColoring)

        self.graphInfoAction = QAction("&Graph Info", self, triggered=self.showGraphInfo)

        self.zoomInAction = QAction("Zoom &In", self, triggered=self.graphWidget.zoomIn)
        self.zoomOutAction = QAction("Zoom &Out", self, triggered=self.graphWidget.zoomOut)
        self.centerAction = QAction("&Center Graph", self, triggered=self.graphWidget.centerGraph)
        self.toggleLabelsAction = QAction("&Toggle Vertex Labels", self, triggered=self.graphWidget.toggleVertexDisplay)

        # PyQt5-safe creation for a checkable action

        self.aboutAction = QAction("&About", self, triggered=self.showAbout)

    def createMenuBar(self):
        menuBar = self.menuBar()

        fileMenu = menuBar.addMenu("&File")
        fileMenu.addAction(self.newAction)
        fileMenu.addAction(self.saveAction)
        fileMenu.addAction(self.loadAction)
        fileMenu.addAction(self.exportAction)

        editMenu = menuBar.addMenu("&Edit")
        editMenu.addAction(self.addRandomAction)
        editMenu.addAction(self.addSelectionAction)
        editMenu.addSeparator()
        editMenu.addAction(self.redrawAction)
        editMenu.addSeparator()
        editMenu.addAction(self.validateColoringAction)
        editMenu.addAction(self.graphInfoAction)
        editMenu.addAction(self.finalizeAction)

        viewMenu = menuBar.addMenu("&View")
        viewMenu.addAction(self.zoomInAction)
        viewMenu.addAction(self.zoomOutAction)
        viewMenu.addAction(self.centerAction)
        viewMenu.addAction(self.toggleLabelsAction)
        aboutMenu = menuBar.addMenu("&About")
        aboutMenu.addAction(self.aboutAction)

    def createControlsDock(self):
        dock = QDockWidget("Controls", self)
        dock.setAllowedAreas(Qt.RightDockWidgetArea)

        mainControlsWidget = QWidget()
        mainLayout = QVBoxLayout(mainControlsWidget)
        mainLayout.setAlignment(Qt.AlignTop)

        fileGroup = QGroupBox("File")
        fileLayout = QVBoxLayout()
        btn_new = QPushButton("New Graph (S or N)")
        btn_save = QPushButton("Save Graph (Ctrl+S)")
        btn_load = QPushButton("Load Graph (Ctrl+O)")
        btn_export = QPushButton("Export as Image (Ctrl+E)")
        fileLayout.addWidget(btn_new)
        fileLayout.addWidget(btn_save)
        fileLayout.addWidget(btn_load)
        fileLayout.addWidget(btn_export)
        fileGroup.setLayout(fileLayout)

        editGroup = QGroupBox("Edit")
        editLayout = QVBoxLayout()
        btn_add_random = QPushButton("Add Random Vertex (R)")
        btn_add_selection = QPushButton("Add Vertex by Selection (A)")
        btn_redraw = QPushButton("Redraw (Planar) (D)")
        btn_validate = QPushButton("Validate Coloring")
        btn_info = QPushButton("Graph Info (I)")
        btn_finalize = QPushButton("Finalize → G4 (F)")
        editLayout.addWidget(btn_finalize)
        editLayout.addWidget(btn_add_random)
        editLayout.addWidget(btn_add_selection)
        editLayout.addWidget(btn_redraw)
        editLayout.addWidget(btn_validate)
        editLayout.addWidget(btn_info)
        editGroup.setLayout(editLayout)

        viewGroup = QGroupBox("View")
        viewLayout = QVBoxLayout()
        btn_center = QPushButton("Center Graph (C)")
        btn_zoom_in = QPushButton("Zoom In (+)")
        btn_zoom_out = QPushButton("Zoom Out (-)")
        btn_toggle_labels = QPushButton("Toggle Labels (T)")
        viewLayout.addWidget(btn_center)
        viewLayout.addWidget(btn_zoom_in)
        viewLayout.addWidget(btn_zoom_out)
        viewLayout.addWidget(btn_toggle_labels)
        viewGroup.setLayout(viewLayout)

        self.spinGoTo = QSpinBox()
        self.spinGoTo.setMinimum(1)
        self.spinGoTo.setMaximum(100000)
        self.spinGoTo.setToolTip("Go to vertex m (1-based). Shows vertices 1..m.")
        self.spinGoTo.editingFinished.connect(lambda: self.graphWidget.goToVertex(self.spinGoTo.value()))

        mainLayout.addWidget(fileGroup)
        mainLayout.addWidget(editGroup)
        mainLayout.addWidget(viewGroup)
        mainLayout.addSpacing(15)
        mainLayout.addWidget(QLabel("Go to Vertex m:"))
        mainLayout.addWidget(self.spinGoTo)

        dock.setWidget(mainControlsWidget)
        self.addDockWidget(Qt.RightDockWidgetArea, dock)

        btn_new.clicked.connect(self.newAction.trigger)
        btn_save.clicked.connect(self.saveAction.trigger)
        btn_load.clicked.connect(self.loadAction.trigger)
        btn_export.clicked.connect(self.exportAction.trigger)
        btn_add_random.clicked.connect(self.addRandomAction.trigger)
        btn_add_selection.clicked.connect(self.addSelectionAction.trigger)
        btn_redraw.clicked.connect(self.redrawAction.trigger)
        btn_validate.clicked.connect(self.validateColoringAction.trigger)
        btn_info.clicked.connect(self.graphInfoAction.trigger)
        btn_center.clicked.connect(self.centerAction.trigger)
        btn_zoom_in.clicked.connect(self.zoomInAction.trigger)
        btn_zoom_out.clicked.connect(self.zoomOutAction.trigger)
        btn_toggle_labels.clicked.connect(self.toggleLabelsAction.trigger)
        # Make the dock button flip the QAction's checked state
        btn_finalize.clicked.connect(self.finalizeAction.trigger)

    def createShortcuts(self):
        QShortcut(QKeySequence("N"), self, self.newAction.trigger)
        QShortcut(QKeySequence("S"), self, self.newAction.trigger)
        QShortcut(QKeySequence("Ctrl+S"), self, self.saveAction.trigger)
        QShortcut(QKeySequence("Ctrl+O"), self, self.loadAction.trigger)
        QShortcut(QKeySequence("Ctrl+E"), self, self.exportAction.trigger)
        QShortcut(QKeySequence("R"), self, self.addRandomAction.trigger)
        QShortcut(QKeySequence("A"), self, self.addSelectionAction.trigger)
        QShortcut(QKeySequence("D"), self, self.redrawAction.trigger)
        QShortcut(QKeySequence("G"), self, self.promptGoTo)
        QShortcut(QKeySequence("C"), self, self.centerAction.trigger)
        QShortcut(QKeySequence(Qt.Key_Plus), self, self.zoomInAction.trigger)
        QShortcut(QKeySequence(Qt.Key_Equal), self, self.zoomInAction.trigger)
        QShortcut(QKeySequence(Qt.Key_Minus), self, self.zoomOutAction.trigger)
        QShortcut(QKeySequence("T"), self, self.toggleLabelsAction.trigger)
        QShortcut(QKeySequence("I"), self, self.graphInfoAction.trigger)
        # Use toggle so the QAction's checked state updates correctly
        QShortcut(QKeySequence("F"), self, self.finalizeAction.trigger)
        QShortcut(QKeySequence("F1"), self, self.aboutAction.trigger)
        
    def saveGraph(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save Graph", "", "JSON Files (*.json)")
        if path:
            if self.graphWidget.graph.save_to_json(path):
                self.statusBar().showMessage(f"Graph saved to {path}", 5000)
            else:
                QMessageBox.warning(self, "Error", "Could not save the graph.")

    def loadGraph(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load Graph", "", "JSON Files (*.json)")
        if path:
            if self.graphWidget.graph.load_from_json(path):
                max_verts = len(self.graphWidget.graph.getVertices())
                self.spinGoTo.setMaximum(max(max_verts, 100000))
                self.spinGoTo.setValue(max_verts)
                self.graphWidget.updateGraphScene()
                self.graphWidget.centerGraph()
                self.statusBar().showMessage(f"Graph loaded from {path}", 5000)
            else:
                QMessageBox.warning(self, "Error", "Could not load the graph.")

    def promptGoTo(self):
        max_verts = len(self.graphWidget.graph.getVertices())
        if max_verts == 0:
            return
        m, ok = QInputDialog.getInt(
            self, "Go to vertex m", f"Enter m (1..{max_verts}):",
            value=min(max_verts, self.spinGoTo.value()), min=1, max=max_verts
        )
        if ok:
            self.spinGoTo.setValue(m)
            self.graphWidget.goToVertex(m)

    def validateColoring(self):
        bad = self.graphWidget.graph.validate_coloring()
        if not bad:
            self.statusBar().showMessage("Coloring is valid.", 4000)
        else:
            QMessageBox.warning(self, "Coloring Conflicts", f"{len(bad)} edges have same-colored endpoints.")

    def showGraphInfo(self):
        stats = self.graphWidget.graph.get_stats()
        self.statusBar().showMessage(
            f"Vertices: {stats['total_vertices']} (seed={stats['seed']}, random={stats['random']}, manual={stats['manual']}), "
            f"Edges: {stats['edges']}, Periphery: {stats['periphery_size']}",
            6000
        )

    
    def finalizeAndG4(self):
        # Strong redraw (auto iteration count inside graph)
        try:
            self.graphWidget.graph.redraw_planar(iterations=None)
        except (ValueError, RuntimeError) as e: # Catch specific, expected errors
            print(f"Redraw failed, falling back. Error: {e}")
            traceback.print_exc() # Log the full error for debugging
            self.graphWidget.graph.redraw_planar(iterations=30)
        # Update scene and center
        self.graphWidget.updateGraphScene()
        self.graphWidget.centerGraph()
        # Go to vertex 4 (clamped to max)
        max_verts = len(self.graphWidget.graph.getVertices())
        target = 4 if max_verts >= 4 else max_verts
        self.spinGoTo.setValue(target)
        self.graphWidget.goToVertex(target)
        self.statusBar().showMessage("Finalize complete → showing up to vertex 4.", 4000)
        
    def showAbout(self):
        text = """
        <div style='min-width:380px'>
        <h3 style='margin:0 0 6px 0'>Planar Triangulated Graph Editor</h3>
        <div style='margin-top:4px; line-height:1.55; color:#333'>
            An interactive editor for building and coloring planar triangulated graphs that grow along the outer face.<br>
            Random or assisted insertions with real-time planarity/triangulation safety, smooth pan/zoom, and 10k+ performance.
        </div>
        <div style='margin-top:12px; line-height:1.45'>
            <b>Developed by:</b> Abdullah Zulfiqar<br>
            email: <a href='mailto:redspectorlabs@gmail.com'>redspectorlabs@gmail.com</a><br>
            <span>Computer Engineer • Designer • Researcher</span>
        </div>
        </div>
        """
        dlg = QMessageBox(self)
        dlg.setWindowTitle("About")
        dlg.setTextFormat(Qt.RichText)
        dlg.setText(text)
        dlg.setStandardButtons(QMessageBox.Ok)
        dlg.setTextInteractionFlags(Qt.TextBrowserInteraction | Qt.LinksAccessibleByMouse)
        dlg.exec_()
