# edge.py
from __future__ import annotations
from typing import Tuple

class Edge:
    __slots__ = ("_start", "_end", "_colorIndex", "_visible")

    def __init__(self, startVertex, endVertex, colorIndex: int = 1):
        # Prevent loops
        if startVertex.getIndex() == endVertex.getIndex():
            raise ValueError("Edge endpoints must be distinct (no loops).")
        # Canonical orientation (small index first)
        if startVertex.getIndex() > endVertex.getIndex():
            startVertex, endVertex = endVertex, startVertex

        self._start = startVertex
        self._end = endVertex
        # Clamp color to 1..4
        c = int(colorIndex)
        self._colorIndex = c if 1 <= c <= 4 else 1
        self._visible = True

    # --- Getters and Setters ---
    def getStartVertex(self): return self._start
    def getEndVertex(self): return self._end
    def getColorIndex(self): return self._colorIndex
    def setColorIndex(self, colorIdx: int):
        c = int(colorIdx)
        self._colorIndex = c if 1 <= c <= 4 else self._colorIndex
    def isVisible(self): return self._visible
    def setVisible(self, v: bool): self._visible = bool(v)

    # Convenience: tuple key by endpoint indices
    def key(self) -> Tuple[int, int]:
        return (self._start.getIndex(), self._end.getIndex())

    def __eq__(self, other) -> bool:
        return isinstance(other, Edge) and self.key() == other.key()

    def __hash__(self) -> int:
        return hash(self.key())

    def __repr__(self):
        return f"E({self._start.getIndex()} - {self._end.getIndex()})"