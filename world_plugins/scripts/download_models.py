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


def _download_gdrive_file(file_id: str, dst_path: Path) -> None:
    """
    Minimal Google Drive downloader (no external deps).
    Handles the common 'confirm download' token flow.
    """
    dst_path.parent.mkdir(parents=True, exist_ok=True)

    base = "https://drive.google.com/uc?export=download"
    url = f"{base}&id={urllib.parse.quote(file_id)}"

    # First request: may return file or an HTML page requiring confirmation
    req = urllib.request.Request(url, headers={"User-Agent": "Mozilla/5.0"})
    with urllib.request.urlopen(req) as resp:
        data = resp.read()

        # Heuristic: HTML => need confirm token
        ct = resp.headers.get("Content-Type", "")
        if "text/html" in ct or b"confirm=" in data:
            html = data.decode("utf-8", errors="ignore")
            m = re.search(r"confirm=([0-9A-Za-z_]+)", html)
            if not m:
                raise RuntimeError(
                    "Google Drive download requires confirmation token, but token not found. "
                    "Try a smaller file, change sharing settings to 'Anyone with the link', "
                    "or use a direct non-Drive URL."
                )
            confirm = m.group(1)
            url2 = f"{base}&confirm={confirm}&id={urllib.parse.quote(file_id)}"
            req2 = urllib.request.Request(url2, headers={"User-Agent": "Mozilla/5.0"})
            with urllib.request.urlopen(req2) as resp2, open(dst_path, "wb") as f:
                shutil.copyfileobj(resp2, f)
        else:
            with open(dst_path, "wb") as f:
                f.write(data)


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
        # download to temp
        if args.gdrive_file_id or args.gdrive_url:
            fid = _parse_gdrive_file_id(args.gdrive_file_id or args.gdrive_url)
            archive_path = td_path / args.filename
            _download_gdrive_file(fid, archive_path)
        elif args.direct_url:
            archive_path = td_path / args.filename
            req = urllib.request.Request(args.direct_url, headers={"User-Agent": "Mozilla/5.0"})
            with urllib.request.urlopen(req) as resp, open(archive_path, "wb") as f:
                shutil.copyfileobj(resp, f)
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
