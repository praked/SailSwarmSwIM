# docs/source/conf.py

import os
import sys
from datetime import datetime

# -----------------------------------------------------------------------------
# Path setup
# -----------------------------------------------------------------------------
# Add project root to sys.path so Sphinx can import SwarmSwIM as a module
# Repo layout assumed:
#   <repo_root>/
#       SwarmSwIM/
#       docs/
#           source/
#               conf.py
sys.path.insert(0, os.path.abspath(os.path.join("..", "..")))

# -----------------------------------------------------------------------------
# Project information
# -----------------------------------------------------------------------------
project = "SwarmSwIM: Sailing Extension"
author = "SwarmSwIM contributors"
current_year = datetime.now().year
copyright = f"{current_year}, {author}"

# Try to get the package version, fall back if not importable
try:
    import SwarmSwIM  # noqa: F401

    # If SwarmSwIM exposes __version__, use it; otherwise keep a dummy string
    release = getattr(SwarmSwIM, "__version__", "0.1.0")
except Exception:
    release = "0.1.0"

# -----------------------------------------------------------------------------
# General configuration
# -----------------------------------------------------------------------------
extensions = [
    "myst_parser",          # Markdown (README, etc.)
    "sphinx.ext.autodoc",   # API docs from docstrings
    "sphinx.ext.autosummary",
    "sphinx.ext.napoleon",  # Google/Numpy docstring support
    "sphinx.ext.viewcode",  # Link to source
]

# Recognize both .rst and .md
source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

# Use index.md as the root document.
# Typical pattern: docs/source/index.md either *is* the README (copy/symlink)
# or includes it via MyST:
#   ```{include} ../../README.md
#   ```
root_doc = "index"

# MyST configuration (Markdown → Sphinx)
myst_enable_extensions = [
    "colon_fence",
    "deflist",
    "linkify",
    "substitution",
    "tasklist",
]

# Make autosummary generate stub pages automatically
autosummary_generate = True

# Autodoc defaults
autodoc_default_options = {
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}

# -----------------------------------------------------------------------------
# HTML output
# -----------------------------------------------------------------------------
html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]

# Optional: nice sidebar title/logo hooks if you add them later
# html_logo = "_static/logo.png"
# html_favicon = "_static/favicon.ico"