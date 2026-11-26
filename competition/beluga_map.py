"""Prepare cached world-coordinate VDB observations and a conservative free-space sampler."""
from pathlib import Path
import json
import shutil
import subprocess
import tempfile

import numpy as np
from scipy.ndimage import distance_transform_edt
import yaml

from . import pcd
from .config import fingerprint, write_json
from .localization import sha256


def pgm(path):
    """Read binary/ascii 8-bit PGM including comments; never interpret unknown as free."""
    with Path(path).open("rb") as f:
        def token():
            value = bytearray()
            while True:
                ch = f.read(1)
                if not ch:
                    if value:
                        return bytes(value)
                    raise ValueError("Truncated PGM")
                if ch == b"#":
                    f.readline()
                elif ch.isspace():
                    if value:
                        return bytes(value)
                else:
                    value.extend(ch)
        magic, width, height, maximum = token(), int(token()), int(token()), int(token())
        if magic not in (b"P5", b"P2") or maximum != 255 or not 0 < width*height <= 16_000_000:
            raise ValueError("Expected bounded 8-bit PGM")
        if magic == b"P5":
            pixels = np.frombuffer(f.read(), np.uint8)
        else:
            pixels = np.array([int(token()) for _ in range(width*height)], dtype=np.uint8)
        if len(pixels) != width*height:
            raise ValueError("PGM size mismatch")
    return pixels.reshape(height, width)[::-1]


def free_cells(yaml_path, clearance):
    meta = yaml.safe_load(Path(yaml_path).read_text())
    resolution = float(meta["resolution"])
    if resolution <= 0 or clearance < 0:
        raise ValueError("Invalid occupancy resolution/clearance")
    image = pgm(Path(yaml_path).parent/meta["image"])
    probability = image/255. if meta.get("negate", 0) else (255.-image)/255.
    free = probability < float(meta["free_thresh"])
    # Treat the outer boundary as unknown as well as walls/unknown pixels.
    padded = np.pad(free, 1, constant_values=False)
    free &= distance_transform_edt(padded)[1:-1, 1:-1]*resolution > clearance+resolution/np.sqrt(2)
    indices = np.argwhere(free)
    if len(indices) == 0:
        raise ValueError("No free cells remain after robot clearance; inspect occupancy map")
    origin = np.asarray(meta["origin"], dtype=float)
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("Invalid map origin")
    xy = (indices[:, ::-1]+.5)*resolution
    cs, sn = np.cos(origin[2]), np.sin(origin[2])
    xy = xy @ np.array([[cs, sn], [-sn, cs]]) + origin[:2]
    return xy, resolution


def prepare(map_path, occupancy_yaml, cache_root, settings, executable=None):
    executable = executable or ["ros2", "run", "mgts_localization", "prepare_vdb"]
    meta = yaml.safe_load(Path(occupancy_yaml).read_text())
    image = Path(occupancy_yaml).parent/meta["image"]
    leaf = float(settings.get("map_resolution_m", .15))
    clearance = float(settings.get("robot_clearance_m", .25))
    if not .02 <= leaf <= 2 or not np.isfinite(clearance) or clearance < 0:
        raise ValueError("Invalid Beluga map resolution/clearance")
    contract = {"format": "beluga-vdb-level-set-v1", "world_transform": [0,0,0,0,0,0,1],
                "map_sha256": sha256(map_path), "occupancy_yaml_sha256": sha256(occupancy_yaml),
                "occupancy_image_sha256": sha256(image), "resolution_m": leaf, "clearance_m": clearance,
                "beluga_commit": "e6323da8c7bee863e29950910737bb3502af1306"}
    cache = Path(cache_root)/fingerprint(contract)
    if cache.exists():
        saved = json.loads((cache/"manifest.json").read_text())
        if saved["contract"] != contract or any(sha256(cache/k) != v for k,v in saved["artifacts"].items()):
            raise ValueError("Corrupt Beluga map cache; move it aside before rebuilding")
        return cache
    cache.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="beluga-map-", dir=cache.parent) as temporary:
        stage = Path(temporary)
        points = pcd.xyz(map_path)
        if len(points) < 5:
            raise ValueError("Insufficient finite PCD geometry")
        points.astype("<f4").tofile(stage/"points.f32")
        xy, resolution = free_cells(occupancy_yaml, clearance)
        np.savetxt(stage/"free_cells.csv", xy, fmt="%.9f")
        subprocess.run([*executable, str(stage/"points.f32"), str(stage/"map.vdb"), str(leaf)], check=True)
        (stage/"points.f32").unlink()
        if not (stage/"map.vdb").is_file() or (stage/"map.vdb").stat().st_size == 0:
            raise RuntimeError("VDB converter did not produce a map")
        write_json(stage/"manifest.json", {"contract": contract, "free_resolution_m": resolution,
            "free_cells": len(xy), "pcd_points": len(points),
            "artifacts": {k: sha256(stage/k) for k in ("map.vdb", "free_cells.csv")}})
        shutil.copytree(stage, cache)
    return cache
