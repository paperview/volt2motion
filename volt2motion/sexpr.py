"""Minimal S-expression reader for KiCad files (.kicad_pcb, .kicad_sch).

KiCad files are nested lists like (footprint "R_0603" (at 10 20) ...).
This returns them as nested Python lists of strings, which is all the
importer needs. It does not depend on a KiCad install or version.
"""
from __future__ import annotations

import re

_TOKEN = re.compile(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()"]+')


def parse(text: str) -> list:
    stack: list[list] = [[]]
    for m in _TOKEN.finditer(text):
        tok = m.group(0)
        if tok == "(":
            stack.append([])
        elif tok == ")":
            if len(stack) < 2:
                raise ValueError("unbalanced ')' in S-expression")
            node = stack.pop()
            stack[-1].append(node)
        elif tok.startswith('"'):
            stack[-1].append(_unescape(tok[1:-1]))
        else:
            stack[-1].append(tok)
    if len(stack) != 1:
        raise ValueError("unbalanced '(' in S-expression")
    roots = stack[0]
    if len(roots) != 1:
        raise ValueError("expected one top-level expression")
    return roots[0]


def _unescape(s: str) -> str:
    return re.sub(r"\\(.)", lambda m: {"n": "\n", "t": "\t"}.get(m.group(1), m.group(1)), s)


def quote(s: str) -> str:
    return '"' + s.replace("\\", "\\\\").replace('"', '\\"') + '"'


def children(node: list, head: str) -> list[list]:
    return [c for c in node if isinstance(c, list) and c and c[0] == head]


def child(node: list, head: str) -> list | None:
    for c in node:
        if isinstance(c, list) and c and c[0] == head:
            return c
    return None


def atom(node: list | None, index: int = 1, default=None):
    if node is None or len(node) <= index or isinstance(node[index], list):
        return default
    return node[index]


def floats(node: list | None, n: int, default: float = 0.0) -> list[float]:
    """First n numeric atoms after the head, padded with default."""
    out = []
    if node is not None:
        for v in node[1:]:
            if isinstance(v, list):
                break
            try:
                out.append(float(v))
            except ValueError:
                break
            if len(out) == n:
                break
    return out + [default] * (n - len(out))
