# vertex.py

from PyQt5.QtCore import QPointF
from typing import Tuple, Union

class Vertex:
    __slots__ = ("_index", "_position", "_colorIndex", "_visible", "_diameter", "_origin")

    def __init__(self, index: int, position: Union[QPointF, Tuple[float, float]],
                 colorIndex: int = 1, origin: str = "manual"):
        self._index = int(index)
        if isinstance(position, QPointF):
            self._position = position
        else:
            x, y = position  # type: ignore[assignment]
            self._position = QPointF(float(x), float(y))
        c = int(colorIndex)
        self._colorIndex = c if 1 <= c <= 4 else 1
        self._visible = True
        self._diameter = self.calculateDiameter(self._index)
        self._origin = origin  # "seed" | "random" | "manual"

    @staticmethod
    def calculateDiameter(idx: int) -> float:
        # Increase diameter for larger indices to fit the number nicely
        if idx >= 1000:
            return 40.0
        elif idx >= 100:
            return 36.0
        elif idx >= 10:
            return 32.0
        else:
            return 30.0

    # --- Getters and Setters (kept for compatibility) ---
    def getIndex(self) -> int:
        return self._index

    def getPosition(self) -> QPointF:
        return self._position

    def setPosition(self, pos: Union[QPointF, Tuple[float, float]]) -> None:
        if isinstance(pos, QPointF):
            self._position = pos
        else:
            x, y = pos  # type: ignore[assignment]
            self._position = QPointF(float(x), float(y))

    def moveBy(self, dx: float, dy: float) -> None:
        self._position = QPointF(self._position.x() + dx, self._position.y() + dy)

    def pos_tuple(self) -> Tuple[float, float]:
        return (self._position.x(), self._position.y())

    def getColorIndex(self) -> int:
        return self._colorIndex

    def setColorIndex(self, colorIdx: int) -> None:
        c = int(colorIdx)
        if 1 <= c <= 4:
            self._colorIndex = c
        # else ignore to keep a valid color

    def getDiameter(self) -> float:
        return self._diameter

    def setDiameter(self, d: float) -> None:
        self._diameter = float(d)

    def isVisible(self) -> bool:
        return self._visible

    def setVisible(self, v: bool) -> None:
        self._visible = bool(v)

    def getOrigin(self) -> str:
        return self._origin

    def setOrigin(self, s: str) -> None:
        self._origin = s

    def __repr__(self) -> str:
        return f"V({self._index})"
