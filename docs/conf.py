# =============================================================================
# Sphinx configuration for the Regulon documentation site.
#
# The site unifies three bodies of material that previously lived apart:
#
#   * the C11 API reference, generated from the public headers by Doxygen and
#     rendered through Breathe;
#   * the specification set (SRS/SADS/IS/TP) and the MISRA deviation records,
#     which were already written as reStructuredText;
#   * hand-written narrative guides for installing, using and verifying the
#     library.
#
# Doxygen runs automatically as part of the build (see _run_doxygen below), so
# a plain `sphinx-build` is enough and no separate step has to be remembered.
#
# RON-IS-001 §8.1
# SPDX-License-Identifier: MIT
# =============================================================================
"""Sphinx build configuration for the Regulon documentation."""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

DOCS_DIR = Path(__file__).parent.resolve()
REPO_ROOT = DOCS_DIR.parent
REGULON_C = REPO_ROOT / "regulon-c"
DOXYGEN_XML = REGULON_C / "doxygen_output" / "xml"

# Local extensions (see docs/_ext/).
sys.path.insert(0, str(DOCS_DIR / "_ext"))


# -----------------------------------------------------------------------------
# Project metadata
# -----------------------------------------------------------------------------
def _read_version() -> str:
    """Return the library version, parsed from the C project's CMakeLists.

    regulon-c/CMakeLists.txt is the single source of truth for the version;
    duplicating it here would be one more thing to forget at release time.
    """
    text = (REGULON_C / "CMakeLists.txt").read_text(encoding="utf-8")
    match = re.search(r"project\s*\(\s*regulon\s+VERSION\s+([0-9]+\.[0-9]+\.[0-9]+)", text)
    return match.group(1) if match else "0.0.0"


project = "Regulon"
author = "Regulon contributors"
copyright = "2026, Regulon contributors"
release = _read_version()
version = ".".join(release.split(".")[:2])


# -----------------------------------------------------------------------------
# Doxygen
# -----------------------------------------------------------------------------
def _run_doxygen() -> None:
    """Regenerate the Doxygen XML that Breathe reads.

    Set REGULON_SKIP_DOXYGEN=1 to reuse existing XML, which makes repeated
    prose-only rebuilds noticeably faster.
    """
    if os.environ.get("REGULON_SKIP_DOXYGEN"):
        return
    if shutil.which("doxygen") is None:
        raise RuntimeError(
            "doxygen was not found on PATH but is required to build the API "
            "reference. Install it (apt install doxygen) or set "
            "REGULON_SKIP_DOXYGEN=1 to build the prose pages only."
        )
    subprocess.run(["doxygen", "Doxyfile"], cwd=REGULON_C, check=True)


_run_doxygen()


# -----------------------------------------------------------------------------
# Extensions
# -----------------------------------------------------------------------------
extensions = [
    "breathe",
    "myst_parser",
    "sphinx.ext.graphviz",
    "sphinx.ext.mathjax",
    "sphinx.ext.todo",
    "sphinx_copybutton",
    "sphinx_design",
    "regulon_trace",
]

breathe_projects = {"regulon": str(DOXYGEN_XML)}
breathe_default_project = "regulon"
# The headers are C, not C++; without this Breathe files them under the C++
# domain and renders C declarations with C++ semantics.
breathe_domain_by_extension = {"h": "c"}
breathe_default_members = ("members",)
breathe_show_include = False

myst_enable_extensions = ["colon_fence", "deflist", "linkify", "substitution"]
myst_heading_anchors = 3

# The specs carry hand-written "Contents" directives and manual section
# numbering, which is what they need as standalone documents.
suppress_warnings: list[str] = []

templates_path = ["_templates"]
exclude_patterns = [
    "_build",
    "Thumbs.db",
    ".DS_Store",
    "requirements.txt",
    # Per-phase implementation plans and closure evidence are internal
    # development records rather than user-facing documentation.
    "plans/**",
]

# Resolve bare `single backtick` markup to code rather than leaving it
# ambiguous; the specs use it heavily for identifiers.
default_role = "literal"

nitpicky = False


# -----------------------------------------------------------------------------
# HTML output
# -----------------------------------------------------------------------------
html_theme = "furo"
html_title = f"Regulon {release}"
html_static_path = ["_static"]
html_css_files = ["custom.css"]
html_show_sourcelink = False
html_copy_source = False

html_theme_options = {
    "sidebar_hide_name": False,
    "navigation_with_keys": True,
    "top_of_page_buttons": [],
    "source_repository": "https://github.com/dtrussel/regulon/",
    "source_branch": "main",
    "source_directory": "docs/",
    "light_css_variables": {
        "color-brand-primary": "#1a5fb4",
        "color-brand-content": "#1a5fb4",
        "font-stack--monospace": "'JetBrains Mono', 'Fira Code', ui-monospace, monospace",
    },
    "dark_css_variables": {
        "color-brand-primary": "#78aeed",
        "color-brand-content": "#78aeed",
    },
    "footer_icons": [
        {
            "name": "GitHub",
            "url": "https://github.com/dtrussel/regulon",
            "class": "fa-brands fa-github",
            "html": (
                '<svg stroke="currentColor" fill="currentColor" viewBox="0 0 16 16">'
                '<path fill-rule="evenodd" d="M8 0C3.58 0 0 3.58 0 8c0 3.54 2.29 6.53 '
                "5.47 7.59.4.07.55-.17.55-.38 0-.19-.01-.82-.01-1.49-2.01.37-2.53-.49-2.69"
                "-.94-.09-.23-.48-.94-.82-1.13-.28-.15-.68-.52-.01-.53.63-.01 1.08.58 1.23"
                ".82.72 1.21 1.87.87 2.33.66.07-.52.28-.87.51-1.07-1.78-.2-3.64-.89-3.64"
                "-3.95 0-.87.31-1.59.82-2.15-.08-.2-.36-1.02.08-2.12 0 0 .67-.21 2.2.82.64"
                "-.18 1.32-.27 2-.27.68 0 1.36.09 2 .27 1.53-1.04 2.2-.82 2.2-.82.44 1.1.16"
                " 1.92.08 2.12.51.56.82 1.27.82 2.15 0 3.07-1.87 3.75-3.65 3.95.29.25.54.73"
                '.54 1.48 0 1.07-.01 1.93-.01 2.2 0 .21.15.46.55.38A8.012 8.012 0 0 0 16 8c0'
                '-4.42-3.58-8-8-8z"></path></svg>'
            ),
        },
    ],
}

pygments_style = "friendly"
pygments_dark_style = "material"
