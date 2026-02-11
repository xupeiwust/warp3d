#!/usr/bin/env python3
"""
RVE periodic boundary condition (PBC) generator for WARP3D.

Enhancements in this version:
1) Origin-agnostic: does NOT assume Xmin=Ymin=Zmin=0.
   - Detects xmin/xmax, ymin/ymax, zmin/zmax from node coordinates.
   - Uses detected bounds for boundary classification and Δ-vectors.

2) DUMMY_EPS_MAP format extended (NO backward compatibility per user request):
     i  j  dummy_node  driver_node  dof
   where:
     - dummy_node,dof appear in the MPC equations (with coefficient = -Δx_j)
     - driver_node,dof receives an absolute displacement constraint = eps_ij
       (the user links driver_node to dummy_node in the WARP3D deck with stiff links)

3) Sequential node numbering check:
   - Requires node lines to be in order AND node ids to be exactly 1..n_nodes.
   - Errors out otherwise.

4) Correct ordering for WARP3D:
   - Emits keyword "constraints" before absolute constraints.
   - Emits keyword "multipoint" before MPC equations.
   - Prints ALL absolute constraints (including those implied by MPC collapse) before MPCs.

5) Two-pass closure:
   - If an MPC collapses to a single remaining physical term, it becomes an implied absolute constraint.
   - The closure iteration continues until no new implied constraints are discovered.

Input file format (commas optional, spaces OK; '#' comments allowed anywhere):
  1) n_nodes n_elems                  (n_elems read but ignored)
  2) Lx Ly Lz                         (DECLARED; cross-checked vs detected)
  3) vertex nodes A B C D E F G H     (IDs only)
  4) eps11 eps12 eps13
  5) eps21 eps22 eps23
  6) eps31 eps32 eps33
  Optional:
     ABS_CONSTRAINTS n_fix
        node_id dof1 dof2 ...
  Required for dummy-node method:
     DUMMY_EPS_MAP n_map
        i j dummy_node driver_node dof
        (example) 1 1 113132 113200 u
  Then:
     n_nodes lines: node_id x y z     (node_id must be 1..n_nodes sequentially)
  Remaining lines (elements) are ignored.
"""

__version__ = "2026.01.29"


from dataclasses import dataclass, field
from typing import Dict, List, Tuple, Set, Optional, TextIO
import sys
import traceback


# ----------------------------
# Data structures
# ----------------------------

@dataclass
class Node:
    nid: int
    x: float
    y: float
    z: float
    faces: Set[str] = field(default_factory=set)  # {"X-","Y+","Z-"} etc.
    kind: str = ""  # "interior", "face", "edge", "vertex"


@dataclass
class Pairing:
    minus_node: int
    plus_node: int
    delta: Tuple[float, float, float]  # (dx,dy,dz)


# ----------------------------
# Utilities
# ----------------------------

def is_close(a: float, b: float = 0.0, tol: float = 1.0e-12) -> bool:
    return abs(a - b) <= tol


def float_to_str(val: float, tol: float = 1.0e-12) -> str:
    """Human-friendly float format: avoid scientific notation for ordinary magnitudes."""
    if abs(val) < tol:
        return "0.0"
    v = round(val, 10)
    s = f"{v:.10f}".rstrip("0").rstrip(".")
    if "." not in s:
        s += ".0"
    return s


def round_coord(val: float, ndigits: int = 6) -> float:
    return round(val, ndigits)


def data_tokens(line: str) -> List[str]:
    """
    Return tokens from the non-comment part of a line.
    Splits off '#' inline comments, then splits on commas/whitespace.
    """
    core = line.split("#", 1)[0].strip()
    if not core:
        return []
    return [p for p in core.replace(",", " ").split() if p]


# ----------------------------
# Parsing
# ----------------------------

def parse_model_def(filename: str):
    with open(filename, "r") as f:
        raw_lines = f.readlines()

    # Drop blank lines and full-line '#' comments; keep inline '#' for data_tokens()
    lines: List[str] = []
    for line in raw_lines:
        s = line.strip()
        if not s:
            continue
        if s.startswith("#"):
            continue
        lines.append(line.rstrip("\n"))

    if len(lines) < 6:
        raise ValueError(
            f"File {filename} has too few data lines after stripping comments/blanks: {len(lines)}"
        )

    # 1) n_nodes n_elems
    parts = data_tokens(lines[0])
    if len(parts) != 2:
        raise ValueError(f"Bad line 1. Need 'n_nodes n_elems'. Got: {lines[0]!r} tokens={parts}")
    n_nodes = int(parts[0])
    _n_elems = int(parts[1])  # read but ignored

    # 2) Lx Ly Lz (declared)
    parts = data_tokens(lines[1])
    if len(parts) != 3:
        raise ValueError(f"Bad line 2. Need 'Lx Ly Lz'. Got: {lines[1]!r} tokens={parts}")
    Lx_decl, Ly_decl, Lz_decl = map(float, parts)

    # 3) vertex IDs A..H
    v = [int(p) for p in data_tokens(lines[2])]
    if len(v) != 8:
        raise ValueError(f"Bad line 3. Need 8 vertex ids A..H. Got: {lines[2]!r} tokens={v}")
    vertex_ids = {"A": v[0], "B": v[1], "C": v[2], "D": v[3], "E": v[4], "F": v[5], "G": v[6], "H": v[7]}

    # 4-6) eps rows
    eps: List[List[float]] = []
    for r in range(3):
        parts = data_tokens(lines[3 + r])
        if len(parts) != 3:
            raise ValueError(f"Bad strain row line {4+r}. Got: {lines[3+r]!r} tokens={parts}")
        eps.append([float(parts[0]), float(parts[1]), float(parts[2])])

    fixed_dofs: Set[Tuple[int, str]] = set()

    # NEW DUMMY_EPS_MAP structure (i,j)->(dummy_node, driver_node, dof)
    dummy_eps_map: Dict[Tuple[int, int], Tuple[int, int, str]] = {}

    idx = 6

    # Optional ABS_CONSTRAINTS
    if idx < len(lines):
        tokens = data_tokens(lines[idx])
        if tokens and tokens[0].upper() == "ABS_CONSTRAINTS":
            if len(tokens) != 2:
                raise ValueError(f"ABS_CONSTRAINTS header must be 'ABS_CONSTRAINTS n'. Got: {lines[idx]!r}")
            n_fix = int(tokens[1])
            idx += 1
            for _ in range(n_fix):
                if idx >= len(lines):
                    raise ValueError("ABS_CONSTRAINTS block truncated.")
                parts = data_tokens(lines[idx])
                if len(parts) < 2:
                    raise ValueError(f"Bad ABS_CONSTRAINTS entry: {lines[idx]!r}")
                nid = int(parts[0])
                for dch in parts[1:]:
                    d = dch.lower()
                    if d not in ("u", "v", "w"):
                        raise ValueError(f"Unknown DOF {dch!r} in ABS_CONSTRAINTS.")
                    fixed_dofs.add((nid, d))
                idx += 1

    # Required DUMMY_EPS_MAP (for dummy-node method)
    if idx >= len(lines):
        raise ValueError("Expected DUMMY_EPS_MAP block, but reached end of file.")

    tokens = data_tokens(lines[idx])
    if not tokens or tokens[0].upper() != "DUMMY_EPS_MAP":
        raise ValueError(f"Expected 'DUMMY_EPS_MAP n' line at: {lines[idx]!r}")

    if len(tokens) != 2:
        raise ValueError(f"DUMMY_EPS_MAP header must be 'DUMMY_EPS_MAP n'. Got: {lines[idx]!r}")

    n_map = int(tokens[1])
    idx += 1

    for _ in range(n_map):
        if idx >= len(lines):
            raise ValueError("DUMMY_EPS_MAP block truncated.")
        parts = data_tokens(lines[idx])
        if len(parts) != 5:
            raise ValueError(f"Bad DUMMY_EPS_MAP entry: {lines[idx]!r} tokens={parts}")
        i = int(parts[0]); j = int(parts[1])
        dn = int(parts[2]); drv = int(parts[3])
        dof = parts[4].lower()
        if i not in (1, 2, 3) or j not in (1, 2, 3):
            raise ValueError(f"Invalid (i,j)=({i},{j}) in DUMMY_EPS_MAP.")
        if dof not in ("u", "v", "w"):
            raise ValueError(f"Invalid dof {dof!r} in DUMMY_EPS_MAP.")
        dummy_eps_map[(i, j)] = (dn, drv, dof)
        idx += 1

    # Nodes (must be sequential 1..n_nodes in file order)
    node_end = idx + n_nodes
    if node_end > len(lines):
        raise ValueError(f"Expected {n_nodes} node lines starting at index {idx}, but insufficient lines.")

    nodes: Dict[int, Node] = {}
    for k in range(idx, node_end):
        parts = data_tokens(lines[k])
        if len(parts) != 4:
            raise ValueError(f"Bad node line: {lines[k]!r} tokens={parts}")
        expected_id = (k - idx) + 1
        nid = int(parts[0])
        if nid != expected_id:
            raise ValueError(
                f"Node numbering not sequential. Expected node {expected_id} on this line, got {nid}. "
                f"Line: {lines[k]!r}"
            )
        x, y, z = map(float, parts[1:])
        nodes[nid] = Node(nid=nid, x=x, y=y, z=z)

    return (Lx_decl, Ly_decl, Lz_decl), vertex_ids, eps, nodes, fixed_dofs, dummy_eps_map


# ----------------------------
# Bounds + Classification (origin-agnostic)
# ----------------------------

def detect_bounds(nodes: Dict[int, Node]) -> Tuple[float, float, float, float, float, float]:
    xs = [nd.x for nd in nodes.values()]
    ys = [nd.y for nd in nodes.values()]
    zs = [nd.z for nd in nodes.values()]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    zmin, zmax = min(zs), max(zs)
    return xmin, xmax, ymin, ymax, zmin, zmax


def classify_boundary_nodes(bounds: Tuple[float, float, float, float, float, float],
                            nodes: Dict[int, Node],
                            tol_rel: float = 1.0e-6) -> None:
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    Lx = xmax - xmin
    Ly = ymax - ymin
    Lz = zmax - zmin
    Lmax = max(abs(Lx), abs(Ly), abs(Lz))
    tol = tol_rel * Lmax if Lmax > 0.0 else tol_rel

    for nd in nodes.values():
        faces: Set[str] = set()
        if abs(nd.x - xmin) <= tol: faces.add("X-")
        if abs(nd.x - xmax) <= tol: faces.add("X+")
        if abs(nd.y - ymin) <= tol: faces.add("Y-")
        if abs(nd.y - ymax) <= tol: faces.add("Y+")
        if abs(nd.z - zmin) <= tol: faces.add("Z-")
        if abs(nd.z - zmax) <= tol: faces.add("Z+")
        nd.faces = faces

        nf = len(faces)
        if nf == 0: nd.kind = "interior"
        elif nf == 1: nd.kind = "face"
        elif nf == 2: nd.kind = "edge"
        elif nf == 3: nd.kind = "vertex"
        else: nd.kind = "degenerate"


# ----------------------------
# Pairings (faces/edges/vertices)
# ----------------------------

def _pair_face_nodes(nodes: Dict[int, Node], minus_face: str, plus_face: str) -> List[Pairing]:
    if minus_face[0] == "X":
        key = lambda nd: (round_coord(nd.y), round_coord(nd.z))
    elif minus_face[0] == "Y":
        key = lambda nd: (round_coord(nd.x), round_coord(nd.z))
    elif minus_face[0] == "Z":
        key = lambda nd: (round_coord(nd.x), round_coord(nd.y))
    else:
        raise ValueError(f"Bad face id {minus_face!r}")

    minus_nodes = [nd for nd in nodes.values() if minus_face in nd.faces]
    plus_nodes  = [nd for nd in nodes.values() if plus_face  in nd.faces]

    m = {key(nd): nd.nid for nd in minus_nodes}
    p = {key(nd): nd.nid for nd in plus_nodes}

    out: List[Pairing] = []
    for k, mid in m.items():
        if k not in p:
            continue
        pid = p[k]
        a = nodes[mid]; b = nodes[pid]
        out.append(Pairing(minus_node=mid, plus_node=pid, delta=(b.x-a.x, b.y-a.y, b.z-a.z)))
    return out


def build_face_pairings(nodes: Dict[int, Node]) -> Dict[str, List[Pairing]]:
    return {
        "X": _pair_face_nodes(nodes, "X-", "X+"),
        "Y": _pair_face_nodes(nodes, "Y-", "Y+"),
        "Z": _pair_face_nodes(nodes, "Z-", "Z+"),
    }


def _axis_coord(nd: Node, axis: str) -> float:
    return nd.x if axis == "X" else (nd.y if axis == "Y" else nd.z)


def build_edge_pairings(nodes: Dict[int, Node]) -> Dict[str, List[Pairing]]:
    axes = {"X", "Y", "Z"}
    specs = [
        ("X-Y (--)/(++)", ("X-", "Y-"), ("X+", "Y+")),
        ("X-Y (-+)/(+-)", ("X-", "Y+"), ("X+", "Y-")),
        ("X-Z (--)/(++)", ("X-", "Z-"), ("X+", "Z+")),
        ("X-Z (-+)/(+-)", ("X-", "Z+"), ("X+", "Z-")),
        ("Y-Z (--)/(++)", ("Y-", "Z-"), ("Y+", "Z+")),
        ("Y-Z (-+)/(+-)", ("Y-", "Z+"), ("Y+", "Z-")),
    ]

    edge_pairings: Dict[str, List[Pairing]] = {}

    for label, minus_faces, plus_faces in specs:
        minus_set = set(minus_faces)
        plus_set  = set(plus_faces)

        minus_nodes = [nd for nd in nodes.values() if nd.kind == "edge" and nd.faces == minus_set]
        plus_nodes  = [nd for nd in nodes.values() if nd.kind == "edge" and nd.faces == plus_set]

        if not minus_nodes or not plus_nodes:
            edge_pairings[label] = []
            continue

        used_axes = {minus_faces[0][0], minus_faces[1][0]}
        param_axis = (axes - used_axes).pop()

        km = {round_coord(_axis_coord(nd, param_axis)): nd.nid for nd in minus_nodes}
        kp = {round_coord(_axis_coord(nd, param_axis)): nd.nid for nd in plus_nodes}

        plist: List[Pairing] = []
        for k, mid in km.items():
            if k not in kp:
                continue
            pid = kp[k]
            a = nodes[mid]; b = nodes[pid]
            plist.append(Pairing(minus_node=mid, plus_node=pid, delta=(b.x-a.x, b.y-a.y, b.z-a.z)))
        edge_pairings[label] = plist

    return edge_pairings


def build_vertex_pairings_from_vertices(
    nodes: Dict[int, Node],
    vertex_ids: Dict[str, int],
) -> List[Pairing]:
    defs = [("G", "A"), ("F", "D"), ("H", "B"), ("C", "E")]
    out: List[Pairing] = []
    for plus_lbl, minus_lbl in defs:
        pid = vertex_ids[plus_lbl]
        mid = vertex_ids[minus_lbl]
        if pid not in nodes or mid not in nodes:
            raise ValueError(f"Vertex id missing in node list: {plus_lbl}->{minus_lbl} ({pid}->{mid})")
        a = nodes[mid]; b = nodes[pid]
        out.append(Pairing(minus_node=mid, plus_node=pid, delta=(b.x-a.x, b.y-a.y, b.z-a.z)))
    return out


# ----------------------------
# Emission helpers
# ----------------------------

def w(out: TextIO, s: str) -> None:
    out.write(s + "\n")


def emit_symbolic_mpcs_for_pairs(out: TextIO, label: str, pair_list: List[Pairing],
                                 nodes: Dict[int, Node],
                                 required_kind: Optional[str] = None) -> None:
    w(out, "!")
    w(out, f"! --- {label} (symbolic eps_ij) ---")
    comp = {1: "u", 2: "v", 3: "w"}

    for p in pair_list:
        a = nodes.get(p.minus_node); b = nodes.get(p.plus_node)
        if a is None or b is None:
            continue
        if required_kind is not None and (a.kind != required_kind or b.kind != required_kind):
            continue

        dx, dy, dz = p.delta
        deltas = [dx, dy, dz]

        for i in (1, 2, 3):
            dof = comp[i]
            line = f"{p.plus_node} 1.0 {dof}  - {p.minus_node} 1.0 {dof}"
            for j, dv in enumerate(deltas, start=1):
                if dv == 0.0:
                    continue
                coeff = -dv
                sign = "+" if coeff >= 0.0 else "-"
                line += f" {sign} {float_to_str(abs(coeff))} eps_{{{i}{j}}} 1.0 {dof}"
            line += " = 0."
            w(out, "!   " + line)


def generate_dummy_mpcs_for_pairs(
    pair_list: List[Pairing],
    nodes: Dict[int, Node],
    eps: List[List[float]],
    dummy_eps_map: Dict[Tuple[int, int], Tuple[int, int, str]],
    fixed_dofs: Set[Tuple[int, str]],
    required_kind: Optional[str] = None,
    eps_tol: float = 1.0e-14,
    one_term_tol: float = 1.0e-12,
) -> Tuple[List[str], Set[Tuple[int, str]], List[str]]:
    """
    Generate MPC lines for a given pairing list under current fixed_dofs.
    Returns:
      mpc_lines: list[str]
      implied_abs: set[(nid, dof)] derived from single-term residuals
      warnings: list[str]
    """
    comp = {1: "u", 2: "v", 3: "w"}
    mpc_lines: List[str] = []
    implied_abs: Set[Tuple[int, str]] = set()
    warnings: List[str] = []

    for p in pair_list:
        a = nodes.get(p.minus_node); b = nodes.get(p.plus_node)
        if a is None or b is None:
            continue
        if required_kind is not None and (a.kind != required_kind or b.kind != required_kind):
            continue

        dx, dy, dz = p.delta
        deltas = [dx, dy, dz]

        plus_id  = p.plus_node
        minus_id = p.minus_node

        for i in (1, 2, 3):
            dof = comp[i]
            terms: List[Tuple[float, int, str]] = []

            # physical terms
            if (plus_id, dof) not in fixed_dofs:
                terms.append((+1.0, plus_id, dof))
            if (minus_id, dof) not in fixed_dofs:
                terms.append((-1.0, minus_id, dof))

            # dummy terms for nonzero strain components
            eps_row = eps[i-1]
            for j, dv in enumerate(deltas, start=1):
                eij = eps_row[j-1]
                if abs(eij) < eps_tol or dv == 0.0:
                    continue
                key = (i, j)
                if key not in dummy_eps_map:
                    warnings.append(f"WARNING: eps_{i}{j} nonzero but missing from DUMMY_EPS_MAP.")
                    continue
                dn, _drv, ddof = dummy_eps_map[key]
                if (dn, ddof) in fixed_dofs:
                    continue
                terms.append((-dv, dn, ddof))

            if not terms:
                continue

            if len(terms) == 1:
                coeff, nid, dch = terms[0]
                is_physical = nid in nodes  # dummy nodes normally not in nodes dict
                if is_physical and is_close(abs(coeff), 1.0, one_term_tol):
                    implied_abs.add((nid, dch))
                else:
                    warnings.append(
                        f"WARNING: single-term residual not auto-converted: "
                        f"(node {nid}, dof {dch}, coeff {float_to_str(coeff)})"
                    )
                continue

            # assemble line
            parts: List[str] = []
            for k, (coeff, nid, dch) in enumerate(terms):
                if k == 0:
                    parts.append(f"{nid} {float_to_str(coeff)} {dch}")
                else:
                    sign = "+" if coeff >= 0.0 else "-"
                    parts.append(f"{sign} {nid} {float_to_str(abs(coeff))} {dch}")
            mpc_lines.append("  " + "  ".join(parts) + " = 0.")

    return mpc_lines, implied_abs, warnings


def closure_generate_all_mpcs(
    face_pairings: Dict[str, List[Pairing]],
    edge_pairings: Dict[str, List[Pairing]],
    vertex_pairings: List[Pairing],
    nodes: Dict[int, Node],
    eps: List[List[float]],
    dummy_eps_map: Dict[Tuple[int, int], Tuple[int, int, str]],
    fixed_initial: Set[Tuple[int, str]],
    eps_tol: float = 1.0e-14,
) -> Tuple[Set[Tuple[int, str]], List[str], List[str]]:
    """
    Iteratively generate MPCs while collecting implied absolute constraints until closure.
    Returns:
      fixed_final (closed),
      all_mpc_lines (final under fixed_final),
      warnings (deduped)
    """
    fixed = set(fixed_initial)
    warnings_all: List[str] = []

    for _iter in range(50):
        implied_total: Set[Tuple[int, str]] = set()

        for d in ("X", "Y", "Z"):
            _, implied, warns = generate_dummy_mpcs_for_pairs(
                face_pairings[d], nodes, eps, dummy_eps_map, fixed,
                required_kind="face", eps_tol=eps_tol
            )
            implied_total |= implied
            warnings_all.extend(warns)

        for _, plist in edge_pairings.items():
            _, implied, warns = generate_dummy_mpcs_for_pairs(
                plist, nodes, eps, dummy_eps_map, fixed,
                required_kind="edge", eps_tol=eps_tol
            )
            implied_total |= implied
            warnings_all.extend(warns)

        _, implied, warns = generate_dummy_mpcs_for_pairs(
            vertex_pairings, nodes, eps, dummy_eps_map, fixed,
            required_kind=None, eps_tol=eps_tol
        )
        implied_total |= implied
        warnings_all.extend(warns)

        new_implied = implied_total - fixed
        if not new_implied:
            break
        fixed |= new_implied
    else:
        warnings_all.append("WARNING: implied-constraint closure hit iteration limit (50).")

    # Final MPC lines under closed fixed set
    all_mpc_lines: List[str] = []
    for d in ("X", "Y", "Z"):
        mpcs, _, warns = generate_dummy_mpcs_for_pairs(
            face_pairings[d], nodes, eps, dummy_eps_map, fixed,
            required_kind="face", eps_tol=eps_tol
        )
        all_mpc_lines.extend(mpcs)
        warnings_all.extend(warns)

    for _, plist in edge_pairings.items():
        mpcs, _, warns = generate_dummy_mpcs_for_pairs(
            plist, nodes, eps, dummy_eps_map, fixed,
            required_kind="edge", eps_tol=eps_tol
        )
        all_mpc_lines.extend(mpcs)
        warnings_all.extend(warns)

    mpcs, _, warns = generate_dummy_mpcs_for_pairs(
        vertex_pairings, nodes, eps, dummy_eps_map, fixed,
        required_kind=None, eps_tol=eps_tol
    )
    all_mpc_lines.extend(mpcs)
    warnings_all.extend(warns)

    warnings_all = sorted(set(warnings_all))
    return fixed, all_mpc_lines, warnings_all


# ----------------------------
# Driver constraints from strain tensor
# ----------------------------

def build_driver_constraints(
    eps: List[List[float]],
    dummy_eps_map: Dict[Tuple[int, int], Tuple[int, int, str]],
    eps_tol: float = 1.0e-14,
) -> Dict[Tuple[int, str], float]:
    """
    Returns mapping (driver_node, dof) -> value (eps_ij),
    omitting eps_ij ~ 0.

    If multiple (i,j) map to the same (driver_node,dof), values must match (within tolerance).
    """
    drv: Dict[Tuple[int, str], float] = {}
    for (i, j), (_dn, drv_node, dof) in dummy_eps_map.items():
        val = eps[i-1][j-1]
        if abs(val) < eps_tol:
            continue
        key = (drv_node, dof)
        if key in drv:
            if not is_close(drv[key], val, tol=1.0e-12):
                raise ValueError(
                    f"Conflicting driver constraint for node {drv_node} dof {dof}: "
                    f"{drv[key]} vs {val} from eps_{i}{j}"
                )
        else:
            drv[key] = val
    return drv


def emit_constraints_block(
    out: TextIO,
    zero_constraints: Set[Tuple[int, str]],
    driver_constraints: Dict[Tuple[int, str], float],
) -> None:
    """
    Emit WARP3D "constraints" block:
      - zero constraints as nid dof 0.0
      - driver constraints as nid dof value
    """
    w(out, "!")
    w(out, "! --- ABSOLUTE CONSTRAINTS (must appear before 'multipoint') ---")

    if not zero_constraints and not driver_constraints:
        w(out, "!   (none)")
        return

    w(out, "constraints")

    # Driver constraints first (nonzero), then zeros
    for (nid, dch), val in sorted(driver_constraints.items()):
        w(out, f"  {nid} {dch} {float_to_str(val)}")

    for (nid, dch) in sorted(zero_constraints):
        # Avoid duplicating if also present as driver constraint (should not happen)
        if (nid, dch) in driver_constraints:
            raise ValueError(
                f"Conflict: node {nid} dof {dch} has both a driver constraint and a zero constraint."
            )
        w(out, f"  {nid} {dch} 0.0")




# ----------------------------
# Abaqus export (optional)
# ----------------------------

ABQ_DOF_MAP: Dict[str, int] = {"u": 1, "v": 2, "w": 3}


def parse_warp3d_mpc_line(line: str) -> List[Tuple[int, str, float]]:
    """Parse one WARP3D MPC line of the form:
         '  nid c dof  +/- nid c dof  ... = 0.'
       Returns list of (nid, dof, coeff) with signed coefficients.
    """
    s = line.strip()
    if not s:
        return []
    if "=" not in s:
        raise ValueError(f"Cannot parse MPC line (no '='): {line!r}")
    s = s.split("=", 1)[0].strip()
    toks = s.split()
    if len(toks) < 3:
        raise ValueError(f"Cannot parse MPC line (too few tokens): {line!r}")

    terms: List[Tuple[int, str, float]] = []

    # First term: nid coeff dof
    nid = int(toks[0])
    coeff = float(toks[1])
    dof = toks[2].lower()
    if dof not in ABQ_DOF_MAP:
        raise ValueError(f"Unknown DOF in MPC line: {dof!r} line={line!r}")
    terms.append((nid, dof, coeff))

    i = 3
    while i < len(toks):
        sign_tok = toks[i]
        if sign_tok not in ("+", "-"):
            raise ValueError(f"Expected '+' or '-' in MPC line, got {sign_tok!r}: {line!r}")
        if i + 3 >= len(toks):
            raise ValueError(f"Truncated MPC line near token {i}: {line!r}")
        nid = int(toks[i + 1])
        coeff = float(toks[i + 2])
        dof = toks[i + 3].lower()
        if dof not in ABQ_DOF_MAP:
            raise ValueError(f"Unknown DOF in MPC line: {dof!r} line={line!r}")
        if sign_tok == "-":
            coeff = -coeff
        terms.append((nid, dof, coeff))
        i += 4

    return terms


def emit_abaqus_boundary(out: TextIO,
                         zero_constraints: Set[Tuple[int, str]],
                         driver_constraints: Dict[Tuple[int, str], float]) -> None:
    """Emit Abaqus *BOUNDARY constraints.
    Uses:
      - zero_constraints: (nid,dof) fixed to 0
      - driver_constraints: (nid,dof)->value (typically eps_ij)
    """
    if not zero_constraints and not driver_constraints:
        return

    out.write("** --- BOUNDARY (absolute constraints) ---\n")
    out.write("*BOUNDARY\n")

    # Driver constraints first (nonzero), then zeros (consistent with WARP3D block)
    for (nid, dch), val in sorted(driver_constraints.items()):
        dofnum = ABQ_DOF_MAP[dch]
        out.write(f"{nid}, {dofnum}, {dofnum}, {float_to_str(val)}\n")

    for (nid, dch) in sorted(zero_constraints):
        if (nid, dch) in driver_constraints:
            raise ValueError(
                f"Conflict: node {nid} dof {dch} has both a driver constraint and a zero constraint."
            )
        dofnum = ABQ_DOF_MAP[dch]
        out.write(f"{nid}, {dofnum}, {dofnum}, 0.0\n")


def emit_abaqus_dummy_pair_ties(out: TextIO,
                               dummy_eps_map: Dict[Tuple[int, int], Tuple[int, int, str]]) -> None:
    """Emit Abaqus *EQUATION ties between each (proxy,dummy) node and its (driver) node.

    In WARP3D, these are enforced via link2 elements tying u,v,w between the proxy and driver.
    Abaqus has no implicit equivalent here, so we emit 3 homogeneous equations per pair:

        u_proxy - u_driver = 0
        v_proxy - v_driver = 0
        w_proxy - w_driver = 0

    Parameters
    ----------
    dummy_eps_map:
        Maps (i,j) -> (proxy_node, driver_node, dof_char). We tie *all three* translational
        DOFs for each unique (proxy_node, driver_node) pair, independent of dof_char.
    """
    if not dummy_eps_map:
        return

    # Unique proxy/driver pairs (deterministic order)
    pairs = sorted({(dn, drv) for (_, _), (dn, drv, _) in dummy_eps_map.items()})
    if not pairs:
        return

    out.write("** --- DUMMY LINK TIES (proxy-driver, u/v/w) ---\n")
    for proxy, driver in pairs:
        for dofnum in (1, 2, 3):  # 1=u, 2=v, 3=w
            out.write("*EQUATION\n")
            out.write("2\n")
            out.write(f"{proxy}, {dofnum}, 1.0\n")
            out.write(f"{driver}, {dofnum}, -1.0\n")

def emit_abaqus_equations(out: TextIO, mpc_lines: List[str], header_comment: Optional[str] = None) -> None:
    """Emit Abaqus *EQUATION blocks corresponding to the provided WARP3D MPC lines."""
    if header_comment:
        out.write(f"** --- {header_comment} ---\n")
    for line in mpc_lines:
        terms = parse_warp3d_mpc_line(line)
        if not terms:
            continue
        out.write("*EQUATION\n")
        out.write(f"{len(terms)}\n")
        for nid, dch, coeff in terms:
            out.write(f"{nid}, {ABQ_DOF_MAP[dch]}, {float_to_str(coeff)}\n")


def build_mpc_groups_under_fixed(face_pairings: Dict[str, List[Pairing]],
                                edge_pairings: Dict[str, List[Pairing]],
                                vertex_pairings: List[Pairing],
                                nodes: Dict[int, Node],
                                eps: List[List[float]],
                                dummy_eps_map: Dict[Tuple[int, int], Tuple[int, int, str]],
                                fixed: Set[Tuple[int, str]],
                                eps_tol: float = 1.0e-14) -> List[Tuple[str, List[str]]]:
    """Re-generate MPC lines under a *given* fixed set, organized into labeled groups.
    This is cosmetic (for Abaqus comments) and does not affect the WARP3D output.
    Returns list of (label, mpc_lines) in a deterministic order.
    """
    groups: List[Tuple[str, List[str]]] = []

    # Faces: X, Y, Z directions
    for d in ("X", "Y", "Z"):
        mpcs, _, _ = generate_dummy_mpcs_for_pairs(
            face_pairings[d], nodes, eps, dummy_eps_map, fixed,
            required_kind="face", eps_tol=eps_tol
        )
        groups.append((f"FACE MPCs (direction {d})", mpcs))

    # Edges: stable ordering by key
    for key in sorted(edge_pairings.keys()):
        plist = edge_pairings[key]
        mpcs, _, _ = generate_dummy_mpcs_for_pairs(
            plist, nodes, eps, dummy_eps_map, fixed,
            required_kind="edge", eps_tol=eps_tol
        )
        groups.append((f"EDGE MPCs (type {key})", mpcs))

    # Vertices
    mpcs, _, _ = generate_dummy_mpcs_for_pairs(
        vertex_pairings, nodes, eps, dummy_eps_map, fixed,
        required_kind=None, eps_tol=eps_tol
    )
    groups.append(("VERTEX MPCs (body diagonals)", mpcs))

    return groups


def emit_abaqus_file(filename: str,
                     in_name: str,
                     bounds: Tuple[float, float, float, float, float, float],
                     L_decl: Tuple[float, float, float],
                     vertex_ids: Dict[str, int],
                     eps: List[List[float]],
                     dummy_eps_map: Dict[Tuple[int, int], Tuple[int, int, str]],
                     zero_constraints: Set[Tuple[int, str]],
                     driver_constraints: Dict[Tuple[int, str], float],
                     mpc_groups: List[Tuple[str, List[str]]]) -> None:
    """Write an Abaqus-format constraint file with clear comment headers."""
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    Lx_decl, Ly_decl, Lz_decl = L_decl
    Lx_det = xmax - xmin
    Ly_det = ymax - ymin
    Lz_det = zmax - zmin

    with open(filename, "w") as out:
        out.write("** ------------------------------------------------------------\n")
        out.write("** RVE PBC generator output (Abaqus format)\n")
        out.write(f"** Input file : {in_name}\n")
        out.write("**\n")
        out.write("** Declared dimensions from input file:\n")
        out.write(f"**   Lx_decl={float_to_str(Lx_decl)}  Ly_decl={float_to_str(Ly_decl)}  Lz_decl={float_to_str(Lz_decl)}\n")
        out.write("**\n")
        out.write("** Detected bounds from node coordinates:\n")
        out.write(f"**   xmin={float_to_str(xmin)}  xmax={float_to_str(xmax)}   => Lx_det={float_to_str(Lx_det)}\n")
        out.write(f"**   ymin={float_to_str(ymin)}  ymax={float_to_str(ymax)}   => Ly_det={float_to_str(Ly_det)}\n")
        out.write(f"**   zmin={float_to_str(zmin)}  zmax={float_to_str(zmax)}   => Lz_det={float_to_str(Lz_det)}\n")
        out.write("**\n")
        out.write(f"** Vertex nodes: {vertex_ids}\n")
        out.write("**\n")
        out.write("** Enforced strain tensor eps_ij (row-wise):\n")
        for i, row in enumerate(eps, start=1):
            out.write("**   row {}: {}\n".format(i, "  ".join(float_to_str(x) for x in row)))
        out.write("**\n")
        out.write("** Dummy strain mapping (eps_ij -> dummy_node, driver_node, dof):\n")
        for (i, j), (dn, drv, dch) in sorted(dummy_eps_map.items()):
            out.write(f"**   eps_{i}{j} -> dummy {dn} {dch},  driver {drv} {dch}\n")
        out.write("**\n")

        emit_abaqus_dummy_pair_ties(out, dummy_eps_map=dummy_eps_map)


        out.write("**\n")
        out.write("** --- MULTI-POINT CONSTRAINTS (*EQUATION) ---\n")
        out.write("**\n")
        for label, lines in mpc_groups:
            if not lines:
                out.write(f"** --- {label}: (none) ---\n")
                continue
            emit_abaqus_equations(out, lines, header_comment=label)

        emit_abaqus_boundary(out, zero_constraints=zero_constraints, driver_constraints=driver_constraints)

        out.write("**\n")
        out.write(f"** Total zero absolute constraints (final, incl. implied): {len(zero_constraints)}\n")
        out.write(f"** Total driver constraints (from eps_ij): {len(driver_constraints)}\n")
        out.write(f"** Total MPC equations emitted: {sum(len(v) for _, v in mpc_groups)}\n")
        out.write("** ------------------------------------------------------------\n")
# ----------------------------
# Main
# ----------------------------

def main() -> None:
    print(f"WARP3D/Abaqus RVE PBC generator (version {__version__})")
    print("------------------------------------------------------------")
    in_name = input("Enter input control file name [rve_code_input.txt]: ").strip() or "rve_code_input.txt"

    fmt = input("Select output format (warp3d / abaqus / both) [warp3d]: ").strip().lower() or "warp3d"
    if fmt not in ("warp3d", "abaqus", "both"):
        raise ValueError(f"Unknown output format: {fmt!r}. Use warp3d, abaqus, or both.")

    out_name = None
    if fmt in ("warp3d", "both"):
        out_name = input("Enter WARP3D constraints output file [rve_pbc_output.txt]: ").strip() or "rve_pbc_output.txt"

    abaqus_out_name = None
    if fmt in ("abaqus", "both"):
        if out_name:
            base = out_name.rsplit(".", 1)[0] if "." in out_name else out_name
            abaqus_name_default = base + "_abaqus.inp"
        else:
            base = in_name.rsplit(".", 1)[0] if "." in in_name else in_name
            abaqus_name_default = base + "_abaqus.inp"
        abaqus_out_name = input(f"Enter Abaqus constraints output file [{abaqus_name_default}]: ").strip() or abaqus_name_default

    sym_ans = input("Print full symbolic equations (can be very large)? (y/n) [n]: ").strip().lower() or "n"
    emit_symbolic = sym_ans.startswith("y")

    # Parse
    (Lx_decl, Ly_decl, Lz_decl), vertex_ids, eps, nodes, fixed_dofs, dummy_eps_map = parse_model_def(in_name)

    # Detect bounds and classify (origin-agnostic)
    bounds = detect_bounds(nodes)
    xmin, xmax, ymin, ymax, zmin, zmax = bounds
    Lx_det = xmax - xmin
    Ly_det = ymax - ymin
    Lz_det = zmax - zmin
    classify_boundary_nodes(bounds, nodes, tol_rel=1.0e-6)

    # Pairings
    face_pairings = build_face_pairings(nodes)
    edge_pairings = build_edge_pairings(nodes)
    vertex_pairings = build_vertex_pairings_from_vertices(nodes, vertex_ids)

    # Closure pass: implied zero constraints + final MPC lines
    fixed_final, mpc_lines, warnings = closure_generate_all_mpcs(
        face_pairings, edge_pairings, vertex_pairings,
        nodes, eps, dummy_eps_map, fixed_dofs,
        eps_tol=1.0e-14
    )

    # Driver constraints from eps_ij (nonzero only)
    driver_constraints = build_driver_constraints(eps, dummy_eps_map, eps_tol=1.0e-14)

    # Conflict check: a dof cannot be both zero-fixed and nonzero-driven
    for key, val in driver_constraints.items():
        if key in fixed_final:
            raise ValueError(
                f"Conflict: driver constraint {key[0]} {key[1]} = {val} "
                f"but same DOF appears in ABS_CONSTRAINTS/implied-zero set."
            )

    # Cross-check declared vs detected dims (for reporting only)
    dim_tol = 1.0e-6 * max(abs(Lx_det), abs(Ly_det), abs(Lz_det), 1.0)

    # Organize MPCs into labeled groups for Abaqus sections (cosmetic only)
    mpc_groups = build_mpc_groups_under_fixed(
        face_pairings, edge_pairings, vertex_pairings,
        nodes, eps, dummy_eps_map, fixed_final,
        eps_tol=1.0e-14
    )

    # Write output(s)
    if fmt in ("warp3d", "both"):
        with open(out_name, "w") as out:
            w(out, "! ------------------------------------------------------------")
            w(out, "! RVE PBC generator output (origin-agnostic bounds)")
            w(out, f"! Input file : {in_name}")

            w(out, "!")
            w(out, "! Declared dimensions from input file:")
            w(out, f"!   Lx_decl={float_to_str(Lx_decl)}  Ly_decl={float_to_str(Ly_decl)}  Lz_decl={float_to_str(Lz_decl)}")

            w(out, "!")
            w(out, "! Detected bounds from node coordinates:")
            w(out, f"!   xmin={float_to_str(xmin)}  xmax={float_to_str(xmax)}   => Lx_det={float_to_str(Lx_det)}")
            w(out, f"!   ymin={float_to_str(ymin)}  ymax={float_to_str(ymax)}   => Ly_det={float_to_str(Ly_det)}")
            w(out, f"!   zmin={float_to_str(zmin)}  zmax={float_to_str(zmax)}   => Lz_det={float_to_str(Lz_det)}")

            if (abs(Lx_det - Lx_decl) > dim_tol or
                abs(Ly_det - Ly_decl) > dim_tol or
                abs(Lz_det - Lz_decl) > dim_tol):
                w(out, "!")
                w(out, "! WARNING: Declared (Lx,Ly,Lz) differ from detected (xmax-xmin, etc.).")
                w(out, "!          PBC generation uses DETECTED bounds for face classification and Δ-vectors.")
            else:
                w(out, "!")
                w(out, "! OK: Declared (Lx,Ly,Lz) consistent with detected bounds (within tolerance).")

            w(out, "!")
            w(out, f"! Vertex nodes: {vertex_ids}")

            w(out, "!")
            w(out, "! Enforced strain tensor eps_ij (row-wise):")
            for i, row in enumerate(eps, start=1):
                w(out, f"!   row {i}: " + "  ".join(float_to_str(x) for x in row))

            w(out, "!")
            w(out, "! Dummy strain mapping (eps_ij -> dummy_node, driver_node, dof):")
            for (i, j), (dn, drv, dch) in sorted(dummy_eps_map.items()):
                w(out, f"!   eps_{i}{j} -> dummy {dn} {dch},  driver {drv} {dch}")

            if warnings:
                w(out, "!")
                w(out, "! --- WARNINGS ---")
                for msg in warnings:
                    w(out, "! " + msg)

            # Optional symbolic section
            if emit_symbolic:
                w(out, "!")
                w(out, "!")
                w(out, "!\t\t.... Full symbolic periodic RVE boundary conditions .....")
                w(out, "!\t\t          Assumes all terms are non-zero in strain")
                w(out, "!\t\t          tensor loading. No =0 absolute constraints")
                w(out, "!")

                for d in ("X", "Y", "Z"):
                    emit_symbolic_mpcs_for_pairs(out, f"Face-node MPCs, direction {d}",
                                                 face_pairings[d], nodes, required_kind="face")
                for label, plist in edge_pairings.items():
                    emit_symbolic_mpcs_for_pairs(out, f"Edge-node MPCs, edge type {label}",
                                                 plist, nodes, required_kind="edge")
                emit_symbolic_mpcs_for_pairs(out, "Vertex-node MPCs, body diagonals",
                                             vertex_pairings, nodes, required_kind=None)

            # Constraints MUST come before MPCs
            emit_constraints_block(out, zero_constraints=fixed_final, driver_constraints=driver_constraints)

            # MPC section
            w(out, "!")
            w(out, "!")
            w(out, "!\t\t.... Full ready-to-use periodic RVE boundary conditions .....")
            w(out, "!\t\t          in WARP3D required format")
            w(out, "!")
            w(out, "!")
            w(out, "! --- MULTI-POINT CONSTRAINTS (MPCs) ---")
            w(out, "multipoint")
            for line in mpc_lines:
                w(out, line)

            w(out, "!")
            w(out, f"! Total zero absolute constraints (final, incl. implied): {len(fixed_final)}")
            w(out, f"! Total driver constraints (from eps_ij): {len(driver_constraints)}")
            w(out, f"! Total MPC equations emitted: {len(mpc_lines)}")
            w(out, "! ------------------------------------------------------------")

    if fmt in ("abaqus", "both"):
        emit_abaqus_file(
            filename=abaqus_out_name,
            in_name=in_name,
            bounds=bounds,
            L_decl=(Lx_decl, Ly_decl, Lz_decl),
            vertex_ids=vertex_ids,
            eps=eps,
            dummy_eps_map=dummy_eps_map,
            zero_constraints=fixed_final,
            driver_constraints=driver_constraints,
            mpc_groups=mpc_groups,
        )
    if fmt in ("warp3d", "both"):
        print(f"Wrote WARP3D constraints to: {out_name}")
    if fmt in ("abaqus", "both"):
        print(f"Wrote Abaqus constraints to: {abaqus_out_name}")
    print(f"Sequential node check: PASSED (node ids 1..n)")
    print(f"Detected bounds: xmin={xmin}, xmax={xmax}, ymin={ymin}, ymax={ymax}, zmin={zmin}, zmax={zmax}")
    print(f"Detected dims  : Lx={Lx_det}, Ly={Ly_det}, Lz={Lz_det}")
    print(f"Zero absolute constraints emitted (final): {len(fixed_final)}")
    print(f"Driver constraints emitted: {len(driver_constraints)}")
    print(f"MPC equations emitted: {len(mpc_lines)}")
    if warnings:
        print(f"Warnings: {len(warnings)} (see output file(s))")



if __name__ == "__main__":
    try:
        main()
    except FileNotFoundError as e:
        # Most common: input control file not found
        fname = getattr(e, "filename", None) or str(e)
        print(f"ERROR: File not found: {fname}", file=sys.stderr)
        print("       Check the input control file name/path and try again.", file=sys.stderr)
        raise SystemExit(2)
    except ValueError as e:
        # Most common: malformed control file line or invalid option
        print(f"ERROR: {e}", file=sys.stderr)
        raise SystemExit(2)
    except Exception:
        print("ERROR: Unexpected failure. Traceback follows.", file=sys.stderr)
        traceback.print_exc()
        raise SystemExit(1)
