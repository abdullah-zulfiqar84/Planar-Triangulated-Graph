# periphery.py
from typing import Iterable, List, Dict, Tuple, Optional

class Periphery:
    """
    Maintains the periphery (outer face) as a CW cycle of vertex indices.
    - indices: internal mutable list storing the periphery cycle in CW order
    - _pos: maps vertex -> position in indices for O(1) lookups
    """

    def __init__(self):
        self.indices: List[int] = []
        self._pos: Dict[int, int] = {}  # vertex -> position in indices (for O(1) lookups)

    # -------- internal helpers --------
    def _rebuild_index_map(self):
        self._pos = {v: i for i, v in enumerate(self.indices)}

    # Exposed in case a caller mutates indices internally (shouldn't happen with getIndices tuple)
    def refreshIndexMap(self):
        self._rebuild_index_map()

    # -------- basic ops --------
    def initialize(self, initialIndices: Iterable[int]):
        self.indices = list(initialIndices)
        self._rebuild_index_map()

    def getIndices(self) -> Tuple[int, ...]:
        """
        Return the periphery as an immutable tuple to prevent external mutation,
        which would otherwise desync the index map.
        """
        return tuple(self.indices)

    def clear(self):
        self.indices.clear()
        self._pos.clear()

    def indexOf(self, v: int) -> int:
        return self._pos.get(v, -1)

    def contains(self, v: int) -> bool:
        return v in self._pos

    def size(self) -> int:
        return len(self.indices)

    # -------- queries --------
    def isContiguous(self, testIndices: Iterable[int]) -> bool:
        """
        Checks if testIndices form a contiguous segment of the periphery (clockwise).
        O(k) with O(1) start lookup.
        """
        seq = list(testIndices)
        if not seq or len(seq) > len(self.indices):
            return False
        start_pos = self._pos.get(seq[0], -1)
        if start_pos < 0:
            return False
        n = len(self.indices)
        for i in range(len(seq)):
            if self.indices[(start_pos + i) % n] != seq[i]:
                return False
        return True
    
    def cw_distance(self, u: int, v: int, inclusive: bool = False) -> int:
        """Number of steps moving CW from u to v. If inclusive, counts u as well."""
        if not self.indices:
            return -1
        iu = self._pos.get(u, -1)
        iv = self._pos.get(v, -1)
        if iu < 0 or iv < 0:
            return -1
        n = len(self.indices)
        dist = (iv - iu) % n
        return dist + (1 if inclusive else 0)
        
    def isProperArc(self, seq: Iterable[int]) -> bool:
        """True if seq is a CW-contiguous segment, not the full cycle, and length >= 2."""
        lst = list(seq)
        if not self.isContiguous(lst):
            return False
        n = len(self.indices)
        return 2 <= len(lst) < n
    
    def getSegment(self, start_node: int, end_node: int) -> Optional[List[int]]:
        """
        Returns the clockwise segment from start_node to end_node (inclusive).
        O(length). Returns None if either endpoint is not on the periphery.
        If start_node == end_node, returns [start_node] (caller can reject len < 2).
        """
        if not self.indices:
            return None
        ip = self._pos.get(start_node, -1)
        iq = self._pos.get(end_node, -1)
        if ip < 0 or iq < 0:
            return None
        segment: List[int] = []
        n = len(self.indices)
        curr = ip
        while True:
            segment.append(self.indices[curr])
            if curr == iq:
                break
            curr = (curr + 1) % n
        return segment

    # -------- update after outward insertion --------
    def updateAfterAddition(self, touchedIndices: Iterable[int], newVertexIndex: int):
        """
        Replace the CW segment (Vp ... Vq) with [Vp, New, Vq].
        All interior vertices in that segment are removed from the periphery.
        """
        seq = list(touchedIndices)
        if not self.isProperArc(seq):
            raise ValueError("updateAfterAddition: touchedIndices must be a proper CW arc (len in [2, n-1]).")

        vp = seq[0]
        vq = seq[-1]
        # Ensure caller passed the exact CW segment
        expected = self.getSegment(vp, vq)
        if expected is None or list(expected) != seq:
            raise ValueError("updateAfterAddition: touchedIndices is not the exact CW Vp..Vq segment.")

        ip = self._pos.get(vp, -1)
        iq = self._pos.get(vq, -1)
        if ip < 0 or iq < 0:
            raise ValueError(f"Could not update periphery: Vp({vp}) or Vq({vq}) not found.")

        n = len(self.indices)
        res = []
        # CW complement: (vq -> vp) exclusive (i.e., from iq+1 up to ip-1)
        k = (iq + 1) % n
        while k != ip:
            res.append(self.indices[k])
            k = (k + 1) % n

        # Append [Vp, New, Vq] as the new CW link across the cut
        res.extend([vp, newVertexIndex, vq])

        self.indices = res
        self._rebuild_index_map()
        return tuple(self.indices)

    # -------- integrity check --------
    def validate(self) -> bool:
        """
        Basic invariants:
        - All periphery vertices are unique
        - _pos is consistent with indices
        """
        if len(set(self.indices)) != len(self.indices):
            return False
        if any(self._pos.get(v, None) != i for i, v in enumerate(self.indices)):
            return False
        return True
    
    def neighborsOnPeriphery(self, u: int) -> Tuple[Optional[int], Optional[int]]:
        if not self.indices or u not in self._pos:
            return None, None
        n = len(self.indices)
        i = self._pos[u]
        return self.indices[(i - 1) % n], self.indices[(i + 1) % n]
    
    def isEdgeOnPeriphery(self, u: int, v: int) -> bool:

        """True if u and v are adjacent along the periphery cycle. O(1)."""
        if not self.indices:
            return False
        n = len(self.indices)
        iu = self._pos.get(u, -1)
        if iu < 0:
            return False
        return (
            v == self.indices[(iu - 1) % n] or
            v == self.indices[(iu + 1) % n]
        )