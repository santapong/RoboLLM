"""Fetch only the pinned Menagerie G1 files, checking Git blob identities."""
from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from urllib.request import Request, urlopen

REVISION = "8161bba264d7fa7c99ca301e91e7fb44737676ad"
REPOSITORY = "google-deepmind/mujoco_menagerie"
DEFAULT_ASSETS = Path(__file__).resolve().parent / "assets" / "unitree_g1"


def _get(url: str) -> bytes:
    with urlopen(Request(url, headers={"User-Agent": "RoboLLM-humanoid-demo"}), timeout=45) as response:
        return response.read()


def _blob_sha(data: bytes) -> str:
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()


def fetch(destination: Path = DEFAULT_ASSETS) -> Path:
    tree = json.loads(_get(f"https://api.github.com/repos/{REPOSITORY}/git/trees/{REVISION}?recursive=1"))
    if tree.get("truncated"):
        raise RuntimeError("Incomplete upstream tree; refusing partial provenance")
    entries = {item["path"]: item for item in tree["tree"] if item["type"] == "blob"}
    prefix = "unitree_g1/"
    base = f"https://raw.githubusercontent.com/{REPOSITORY}/{REVISION}/"
    model = _get(base + prefix + "g1.xml")
    if _blob_sha(model) != entries[prefix + "g1.xml"]["sha"]:
        raise RuntimeError("Upstream XML identity mismatch")
    meshes = ["assets/" + item.attrib["file"] for item in ET.fromstring(model).findall("./asset/mesh")]
    paths = ["g1.xml", "README.md", "LICENSE", *meshes]
    destination.mkdir(parents=True, exist_ok=True)

    def retrieve(relative: str) -> dict:
        expected = entries[prefix + relative]["sha"]
        path = destination / relative
        data = path.read_bytes() if path.exists() else b""
        if _blob_sha(data) != expected:
            data = model if relative == "g1.xml" else _get(base + prefix + relative)
        if _blob_sha(data) != expected:
            raise RuntimeError(f"Git blob checksum mismatch: {relative}")
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".partial")
        temporary.write_bytes(data)
        temporary.replace(path)
        return {"path": relative, "git_blob_sha1": expected, "sha256": hashlib.sha256(data).hexdigest(), "bytes": len(data)}

    with ThreadPoolExecutor(max_workers=4) as executor:
        files = list(executor.map(retrieve, paths))
    manifest = {"repository": REPOSITORY, "revision": REVISION, "files": files}
    (destination / "provenance.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Verified {len(files)} files ({sum(item['bytes'] for item in files):,} bytes) at {destination}")
    return destination


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_ASSETS)
    fetch(parser.parse_args().destination)
