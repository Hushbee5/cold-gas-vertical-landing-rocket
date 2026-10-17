# -*- coding: utf-8 -*-
"""Build the evidence-corrected GNC diagram from its SVG source."""
import subprocess
from pathlib import Path

node = Path.home() / '.cache/codex-runtimes/codex-primary-runtime/dependencies/node/bin/node.exe'
subprocess.run([str(node), str(Path(__file__).with_name('blok_diyagram_codex.cjs'))], check=True)
