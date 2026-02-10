#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Download a model asset archive (e.g. from Google Drive) and extract into MarsSim.

Default target:
  <repo_root>/rover_gazebo/models

Supports:
  - Google Drive file id (recommended)
  - Direct URL (best-effort; for non-Drive links)

Usage examples:
  python -m world_plugins.scripts.download_models --gdrive-file-id <FILE_ID>
  python -m world_plugins.scripts.download_models --gdrive-url "https://drive.google.com/file/d/<FILE_ID>/view"
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import sys
import tempfile
import urllib.parse
import urllib.request
from pathlib import Path
from typing import Optional
import zipfile
import tarfile
import time


def _repo_root() -> Path:
    # .../MarsSim/world_plugins/scripts/download_models.py -> parents[2] == .../MarsSim
    return Path(__file__).resolve().parents[2]


def _parse_gdrive_file_id(url_or_id: str) -> str:
    s = (url_or_id or "").strip()
    if not s:
        raise ValueError("Empty Google Drive id/url")

    # If it's already an id (common: 20-80 chars, includes - _)
    if re.fullmatch(r"[A-Za-z0-9_-]{10,}", s):
        return s

    # Try to extract from common URL patterns:
    # - https://drive.google.com/file/d/<id>/view
    m = re.search(r"/file/d/([A-Za-z0-9_-]+)", s)
    if m:
        return m.group(1)

    # - https://drive.google.com/open?id=<id>
    q = urllib.parse.urlparse(s).query
    qs = urllib.parse.parse_qs(q)
    if "id" in qs and qs["id"]:
        return qs["id"][0]

    raise ValueError(f"Could not parse Google Drive file id from: {s!r}")


def _fmt_bytes(n: float) -> str:
    units = ["B", "KB", "MB", "GB", "TB"]
    u = 0
    while n >= 1024 and u < len(units) - 1:
        n /= 1024.0
        u += 1
    if u == 0:
        return f"{int(n)}{units[u]}"
    return f"{n:.2f}{units[u]}"


def _fmt_eta(seconds: float) -> str:
    if seconds <= 0 or seconds == float("inf"):
        return "--:--"
    m, s = divmod(int(seconds + 0.5), 60)
    h, m = divmod(m, 60)
    if h:
        return f"{h:d}:{m:02d}:{s:02d}"
    return f"{m:02d}:{s:02d}"


def _stream_download(req: urllib.request.Request, dst_path: Path, desc: str = "Downloading") -> None:
    dst_path.parent.mkdir(parents=True, exist_ok=True)

    t0 = time.monotonic()
    last_print = 0.0
    downloaded = 0

    with urllib.request.urlopen(req) as resp, open(dst_path, "wb") as f:
        total = resp.headers.get("Content-Length")
        total = int(total) if total and total.isdigit() else None

        # one-line progress (stderr)
        def _print(force: bool = False) -> None:
            nonlocal last_print
            now = time.monotonic()
            if not force and (now - last_print) < 0.2:
                return
            last_print = now

            elapsed = max(now - t0, 1e-9)
            speed = downloaded / elapsed  # B/s
            eta = ((total - downloaded) / speed) if (total is not None and speed > 1e-9) else float("inf")

            if total is None:
                line = f"\r{desc}: { _fmt_bytes(downloaded) }  { _fmt_bytes(speed) }/s  ETA {_fmt_eta(eta)}"
            else:
                pct = (downloaded / total) * 100.0 if total else 0.0
                line = (
                    f"\r{desc}: {pct:6.2f}%  "
                    f"{_fmt_bytes(downloaded)}/{_fmt_bytes(total)}  "
                    f"{_fmt_bytes(speed)}/s  ETA {_fmt_eta(eta)}"
                )
            sys.stderr.write(line)
            sys.stderr.flush()

        chunk_size = 1024 * 1024  # 1MB
        while True:
            chunk = resp.read(chunk_size)
            if not chunk:
                break
            f.write(chunk)
            downloaded += len(chunk)
            _print()

        _print(force=True)
        sys.stderr.write("\n")
        sys.stderr.flush()


def _download_gdrive_file(file_id: str, dst_path: Path) -> None:
    dst_path.parent.mkdir(parents=True, exist_ok=True)

    # 1) First request (may return small file directly, or HTML "download-form")
    base = "https://drive.google.com/uc?export=download"
    url = f"{base}&id={urllib.parse.quote(file_id)}"

    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp:
        data = resp.read()
        ct = resp.headers.get("Content-Type", "")

        # Small files may be returned directly
        if "text/html" not in ct and b"confirm=" not in data:
            with open(dst_path, "wb") as f:
                f.write(data)
            sys.stderr.write(f"Downloaded: {_fmt_bytes(len(data))}\n")
            return

        html = data.decode("utf-8", errors="ignore")

    # 2) Parse form action
    m_action = re.search(r'<form[^>]+id="download-form"[^>]+action="([^"]+)"', html)
    if not m_action:
        raise RuntimeError("Cannot find download-form action in Google Drive HTML page.")
    action = m_action.group(1)
    action_url = urllib.parse.urljoin("https://drive.google.com", action)

    # 3) Parse hidden inputs
    inputs = dict(
        re.findall(
            r'<input[^>]+type="hidden"[^>]+name="([^"]+)"[^>]+value="([^"]*)"',
            html,
        )
    )
    inputs["id"] = file_id

    query = urllib.parse.urlencode(inputs)
    final_url = f"{action_url}?{query}"

    # 4) Stream-download real file with progress
    req2 = urllib.request.Request(final_url, headers={"User-Agent": "Mozilla/5.0"})
    _stream_download(req2, dst_path, desc="Downloading (Google Drive)")


def _is_within_directory(base: Path, target: Path) -> bool:
    try:
        target.resolve().relative_to(base.resolve())
        return True
    except Exception:
        return False


def _safe_extract_zip(zip_path: Path, out_dir: Path) -> None:
    with zipfile.ZipFile(zip_path) as zf:
        for member in zf.infolist():
            member_path = out_dir / member.filename
            if not _is_within_directory(out_dir, member_path):
                raise RuntimeError(f"Unsafe zip member path: {member.filename}")
        zf.extractall(out_dir)


def _safe_extract_tar(tar_path: Path, out_dir: Path) -> None:
    with tarfile.open(tar_path) as tf:
        for member in tf.getmembers():
            member_path = out_dir / member.name
            if not _is_within_directory(out_dir, member_path):
                raise RuntimeError(f"Unsafe tar member path: {member.name}")
        tf.extractall(out_dir)


def _pick_extracted_root(extract_dir: Path) -> Path:
    entries = [p for p in extract_dir.iterdir() if p.name not in [".DS_Store"]]
    if len(entries) == 1 and entries[0].is_dir():
        return entries[0]
    return extract_dir


def _merge_into_models(src_root: Path, models_dir: Path) -> None:
    """
    Merge extracted content into models_dir.
    If archive contains a top-level 'models' directory, merge its contents instead.
    """
    src = src_root
    if (src_root / "models").is_dir():
        src = src_root / "models"

    models_dir.mkdir(parents=True, exist_ok=True)
    for item in src.iterdir():
        dst = models_dir / item.name
        if item.is_dir():
            shutil.copytree(item, dst, dirs_exist_ok=True)
        else:
            dst.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(item, dst)


def main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gdrive-file-id", default=os.getenv("MARS_SIM_MODELS_GDRIVE_FILE_ID"))
    parser.add_argument("--gdrive-url", default=os.getenv("MARS_SIM_MODELS_GDRIVE_URL"))
    parser.add_argument("--direct-url", default=os.getenv("MARS_SIM_MODELS_DIRECT_URL"))
    parser.add_argument("--out", default=None, help="Target models dir (default: <repo>/rover_gazebo/models)")
    parser.add_argument("--filename", default="models_archive", help="Base name for downloaded archive")
    args = parser.parse_args(argv)

    repo = _repo_root()
    models_dir = Path(args.out).expanduser().resolve() if args.out else (repo / "rover_gazebo" / "models").resolve()

    # Decide download source
    archive_path: Path
    with tempfile.TemporaryDirectory() as td:
        td_path = Path(td)

        if args.gdrive_file_id or args.gdrive_url:
            fid = _parse_gdrive_file_id(args.gdrive_file_id or args.gdrive_url)
            archive_path = td_path / args.filename
            _download_gdrive_file(fid, archive_path)

        elif args.direct_url:
            archive_path = td_path / args.filename
            req = urllib.request.Request(args.direct_url, headers={"User-Agent": "Mozilla/5.0"})
            _stream_download(req, archive_path, desc="Downloading (direct url)")
        else:
            raise SystemExit("Provide --gdrive-file-id / --gdrive-url / --direct-url (or env vars).")

        # Detect archive type (by signature / suffix best-effort)
        extract_dir = td_path / "extract"
        extract_dir.mkdir(parents=True, exist_ok=True)

        # If user didn't provide a suffix, try to guess by magic bytes
        head = archive_path.read_bytes()[:4]
        is_zip = head.startswith(b"PK\x03\x04") or str(archive_path).lower().endswith(".zip")
        is_tar = str(archive_path).lower().endswith((".tar", ".tar.gz", ".tgz", ".tar.xz", ".tar.bz2"))

        if is_zip:
            _safe_extract_zip(archive_path, extract_dir)
        elif is_tar:
            _safe_extract_tar(archive_path, extract_dir)
        else:
            # last chance: tarfile can sometimes open without suffix
            try:
                _safe_extract_tar(archive_path, extract_dir)
            except Exception as e:
                raise RuntimeError(f"Unknown archive type: {archive_path}") from e

        root = _pick_extracted_root(extract_dir)
        _merge_into_models(root, models_dir)

    print(f"Models installed into: {models_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
