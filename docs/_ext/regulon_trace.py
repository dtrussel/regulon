# =============================================================================
# regulon_trace.py — cross-link requirement and test IDs into the specs.
#
# Every public declaration in the C headers carries a traceability annotation
# naming the requirements it satisfies and the tests that cover it.  The
# Doxygen input filter (regulon-c/scripts/doxygen_filter.py) turns those into
# "Satisfies"/"Tests" sections, so the IDs reach the rendered API reference as
# plain text.  This extension makes them navigable.
#
# It works in two stages:
#
#   1. While reading the specification documents it records where each ID is
#      defined - a section whose title begins with the ID (how the test plan
#      is written) or a table row whose first cell is the ID (how the
#      requirement tables are written) - and attaches an anchor there.
#
#   2. After resolution it rewrites bare IDs on the API reference pages into
#      links pointing at those anchors.
#
# IDs with no definition are left as plain text rather than reported, so a
# partially-specified module never fails a `-W` build.
#
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================
"""Sphinx extension linking Regulon requirement/test IDs to their specs."""

from __future__ import annotations

import re
from typing import Any

from docutils import nodes
from sphinx.application import Sphinx
from sphinx.environment import BuildEnvironment
from sphinx.transforms.post_transforms import SphinxPostTransform

# Requirement, design-decision and test-case identifiers, e.g. RON-FR-730,
# RON-SR-013, RON-PR-003, RON-DD-19, RON-TC-LQR-010-FV.
_ID_RE = re.compile(r"\bRON-(?:FR|SR|PR|NF|IF|DD|TC)-[A-Z0-9]+(?:-[A-Z0-9]+)*\b")

# A section title that opens with an ID, e.g.
# "RON-TC-LQR-001 - Init and Basic Control Law".
_TITLE_ID_RE = re.compile(r"^\s*(RON-(?:FR|SR|PR|NF|IF|DD|TC)-[A-Z0-9]+(?:-[A-Z0-9]+)*)\b")

# Documents whose IDs get rewritten into links.
_LINK_PREFIXES = ("api/",)

_ENV_KEY = "regulon_trace_targets"


def _anchor_for(identifier: str) -> str:
    """Return the HTML anchor used for *identifier*."""
    return "trace-" + identifier.lower()


def _targets(env: BuildEnvironment) -> dict[str, tuple[str, str, bool]]:
    """Return the ID -> (docname, anchor) map, creating it on first use."""
    if not hasattr(env, _ENV_KEY):
        setattr(env, _ENV_KEY, {})
    return getattr(env, _ENV_KEY)


def _register(
    env: BuildEnvironment,
    identifier: str,
    node: nodes.Element,
    *,
    authoritative: bool,
) -> None:
    """Attach an anchor for *identifier* to *node* and record its location.

    A section definition is *authoritative* and always wins; table rows only
    fill in IDs that no section defines, so an ID documented in both places
    links to its narrative section rather than to a summary row.
    """
    targets = _targets(env)
    existing = targets.get(identifier)
    if existing is not None and not authoritative:
        return
    if existing is not None and existing[2] and authoritative:
        return

    anchor = _anchor_for(identifier)
    if anchor not in node["ids"]:
        node["ids"].append(anchor)
    targets[identifier] = (env.docname, anchor, authoritative)


def _first_cell_text(row: nodes.Element) -> str:
    """Return the plain text of a table row's first cell."""
    for entry in row.findall(nodes.entry):
        return entry.astext().strip()
    return ""


def _collect(app: Sphinx, doctree: nodes.document) -> None:
    """Record every ID definition found in a specification document."""
    env = app.env
    if env.docname.startswith(_LINK_PREFIXES):
        return

    # Sections first: their titles are the richest definition of an ID.
    for section in doctree.findall(nodes.section):
        title = next(iter(section.findall(nodes.title)), None)
        if title is None:
            continue
        match = _TITLE_ID_RE.match(title.astext())
        if match:
            _register(env, match.group(1), section, authoritative=True)

    # Then table rows whose first cell is exactly an ID.
    for row in doctree.findall(nodes.row):
        text = _first_cell_text(row)
        if _ID_RE.fullmatch(text):
            _register(env, text, row, authoritative=False)


class RegulonTraceLinks(SphinxPostTransform):
    """Rewrite bare requirement/test IDs on API pages into links."""

    default_priority = 900

    def run(self, **kwargs: Any) -> None:
        if not self.env.docname.startswith(_LINK_PREFIXES):
            return
        targets = _targets(self.env)
        if not targets:
            return

        for text_node in list(self.document.findall(nodes.Text)):
            parent = text_node.parent
            if isinstance(parent, (nodes.literal_block, nodes.comment, nodes.reference)):
                continue
            replacement = self._linkify(str(text_node), targets)
            if replacement is not None:
                parent.replace(text_node, replacement)

    def _linkify(
        self, text: str, targets: dict[str, tuple[str, str, bool]]
    ) -> list[nodes.Node] | None:
        """Split *text* into alternating plain and reference nodes."""
        pieces: list[nodes.Node] = []
        cursor = 0
        found = False

        for match in _ID_RE.finditer(text):
            target = targets.get(match.group(0))
            if target is None:
                continue
            docname, anchor = target[0], target[1]
            if match.start() > cursor:
                pieces.append(nodes.Text(text[cursor : match.start()]))
            uri = self.app.builder.get_relative_uri(self.env.docname, docname)
            reference = nodes.reference("", "", internal=True, refuri=f"{uri}#{anchor}")
            reference += nodes.literal(match.group(0), match.group(0))
            pieces.append(reference)
            cursor = match.end()
            found = True

        if not found:
            return None
        if cursor < len(text):
            pieces.append(nodes.Text(text[cursor:]))
        return pieces


def _purge(app: Sphinx, env: BuildEnvironment, docname: str) -> None:
    """Drop targets defined by a document that is being re-read."""
    targets = _targets(env)
    for identifier in [k for k, v in targets.items() if v[0] == docname]:
        del targets[identifier]


def _merge(
    app: Sphinx,
    env: BuildEnvironment,
    docnames: list[str],
    other: BuildEnvironment,
) -> None:
    """Combine targets collected by parallel read workers."""
    _targets(env).update(_targets(other))


def setup(app: Sphinx) -> dict[str, Any]:
    app.connect("doctree-read", _collect)
    app.connect("env-purge-doc", _purge)
    app.connect("env-merge-info", _merge)
    app.add_post_transform(RegulonTraceLinks)
    return {
        "version": "1.0.0",
        "parallel_read_safe": True,
        "parallel_write_safe": True,
    }
