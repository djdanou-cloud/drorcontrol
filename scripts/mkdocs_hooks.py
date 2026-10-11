"""MkDocs hooks for the DrorControl docs site (mkdocs.yml → hooks).

* Before each build: generate the MAVLink reference (scripts/gen_mavlink_docs.py) into
  docs/reference/mavlink/ and copy the design-token CSS (ui/tokens/generated/tokens.css) into
  docs/stylesheets/. Both are build outputs (gitignored); files are rewritten only when their
  content changes, so `mkdocs serve` does not loop.
* On each page: links that leave docs/ (e.g. ../../crates/dc-core/src/lib.rs) point to the file
  on GitHub, so the site has no dead links and `mkdocs build --strict` still catches real ones.
"""
import posixpath
import re
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
REPO = "https://github.com/djdanou-cloud/drorcontrol"
sys.path.insert(0, str(ROOT / "scripts"))

import gen_mavlink_docs  # noqa: E402

LINK = re.compile(r"(\]\()([^)\s]+)(\))")


def on_pre_build(config, **kwargs):
    docs = Path(config["docs_dir"])
    gen_mavlink_docs.generate(docs / "reference/mavlink")
    src = ROOT / "ui/tokens/generated/tokens.css"
    dst = docs / "stylesheets/tokens.css"
    if not dst.exists() or dst.read_bytes() != src.read_bytes():
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(src, dst)


def on_page_markdown(markdown, page, config, files, **kwargs):
    here = posixpath.dirname(page.file.src_uri)
    docs_rel = posixpath.relpath(config["docs_dir"], str(ROOT)) if Path(config["docs_dir"]).is_absolute() else "docs"

    def fix(m):
        target = m.group(2)
        if re.match(r"^[a-z][a-z0-9+.-]*:|^#|^/", target, re.I):
            return m.group(0)
        path, _, frag = target.partition("#")
        resolved = posixpath.normpath(posixpath.join(here, path))
        if not resolved.startswith("../"):
            return m.group(0)
        repo_path = posixpath.normpath(posixpath.join(docs_rel, resolved))
        kind = "tree" if path.endswith("/") or (ROOT / repo_path).is_dir() else "blob"
        url = f"{REPO}/{kind}/main/{repo_path}" + (f"#{frag}" if frag else "")
        return f"{m.group(1)}{url}{m.group(3)}"

    out, fence = [], False
    for line in markdown.split("\n"):
        if line.lstrip().startswith(("```", "~~~")):
            fence = not fence
        out.append(line if fence else LINK.sub(fix, line))
    return "\n".join(out)
