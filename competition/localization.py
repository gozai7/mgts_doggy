"""Shared experiment contracts; mapping and relative odometry remain independent."""
import copy
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .config import fingerprint, pose7, transform, write_json
from .trajectory import save

PROFILES = {
    "baseline": ("ndt_gicp", "existing"),
    "beluga": ("beluga", "existing"),
    "ndt_kiss": ("ndt_gicp", "kiss_icp"),
    "beluga_kiss": ("beluga", "kiss_icp"),
}


def select(c, profile=None):
    c = copy.deepcopy(c)
    localizer = c["localization"].get("backend", "ndt_gicp")
    source = c.get("odometry", {}).get("source", "existing")
    if profile:
        localizer, source = PROFILES[profile]
    if (localizer, source) not in PROFILES.values():
        raise ValueError("Choose ndt_gicp|beluga and existing|kiss_icp")
    c["localization"]["backend"] = localizer
    c.setdefault("odometry", {})["source"] = source
    for key in ("max_gap_s","voxel_size_m","position_variance_m2","orientation_variance_rad2"):
        value=c["odometry"].get(key)
        if value is not None and (not np.isfinite(value) or value<=0):
            raise ValueError(f"odometry.{key} must be finite and positive")
    c["localization"]["profile"] = next(k for k, v in PROFILES.items() if v == (localizer, source))
    if source == "kiss_icp":
        if c.get("odometry",{}).get("deskew") and (c["point_time"]["mode"]=="none" or c["point_time"].get("header_reference")!="end"):
            raise ValueError("KISS deskew uses the sweep end: set verified point_time.header_reference:end or leave odometry.deskew:false")
        c["topics"]["odometry"] = None
        c["localization"]["prediction"] = "external_tf"
    elif localizer == "beluga":
        if not c["topics"].get("odometry"):
            raise ValueError("Beluga + existing requires measured kinematic Odometry; constant_velocity is not an odometry stream")
        c["localization"]["prediction"] = "external_tf"
    if localizer == "beluga":
        b = c.get("beluga", {})
        for key in ("min_particles","max_particles","max_points","seed","particle_export_every_n"):
            if key in b and (not isinstance(b[key],int) or isinstance(b[key],bool) or b[key] < (0 if key=="seed" else 1)):
                raise ValueError(f"beluga.{key} must be a valid nonnegative seed or positive integer count")
        if b.get("min_particles",2000)>b.get("max_particles",12000):
            raise ValueError("Beluga minimum particle count exceeds maximum")
        for key in ("motion_noise","sigma_hit_m","max_xy_variance_m2","max_yaw_variance_rad2"):
            if key in b and (not np.isfinite(b[key]) or b[key]<=0):
                raise ValueError(f"beluga.{key} must be finite and positive")
        if not 0 < b.get("min_overlap",.65) <= 1:
            raise ValueError("beluga.min_overlap must be in (0,1]")
        if b.get("initial_base_height_m") is None or not np.isfinite(b["initial_base_height_m"]):
            raise ValueError("Set beluga.initial_base_height_m in map coordinates (measured floor + initial body height)")
        if c["odometry"].get("gravity_aligned", False) is not True:
            raise ValueError("Beluga requires odometry.gravity_aligned: true and a verified gravity reference")
        tilt = c["odometry"].get("initial_roll_pitch_rad")
        if source == "kiss_icp" and (tilt is None or np.asarray(tilt).shape != (2,) or not np.isfinite(tilt).all()):
            raise ValueError("Set measured odometry.initial_roll_pitch_rad for KISS's initial base frame")
        if c["localization"].get("manual_initial_pose"):
            raise ValueError("Use an explicit approximate initial_pose for the separate known-start Beluga experiment")
    return c


def sha256(path):
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def mapping_backend(c, default="glim"):
    backend = c["mapping"].get("backend", default).replace("-", "_")
    if backend not in ("glim", "rko_lio"):
        raise ValueError("mapping.backend must be glim or rko_lio")
    return "rko-lio" if backend == "rko_lio" else backend


class Guard:
    """Accept corrections only after confirmation; preserve the last trusted map/odom anchor."""
    def __init__(self, settings=None):
        s = settings or {}
        self.confirmations = int(s.get("confirmations", 3))
        self.max_translation = float(s.get("max_motion_disagreement_m", 1.0))
        self.max_rotation = float(s.get("max_motion_disagreement_rad", .7))
        self.max_gap = float(s.get("max_correction_gap_s", 1.0))
        self.fallback_limit = float(s.get("max_fallback_s", 5.0))
        self.fallback = s.get("odometry_fallback", True)
        self.max_odom_step = float(s.get("max_odometry_step_m",2.))
        self.max_odom_angle = float(s.get("max_odometry_step_rad",1.5))
        values=np.array([self.max_translation,self.max_rotation,self.max_gap,self.fallback_limit,self.max_odom_step,self.max_odom_angle])
        if self.confirmations < 1 or not np.isfinite(values).all() or np.any(values<=0):
            raise ValueError("Invalid recovery thresholds")
        self.anchor = self.last_odom = self.candidate = None
        self.last_good_ns = self.previous_ns = None
        self.streak = 0
        self.tracking = False

    def update(self, n, candidate, odom, accepted):
        if self.previous_ns is not None and n <= self.previous_ns:
            raise ValueError("Supervisor timestamps must increase")
        gap = self.previous_ns is not None and (n-self.previous_ns)*1e-9 > self.max_gap
        self.previous_ns = n
        o = transform(odom) if odom is not None else None
        p = transform(candidate) if candidate is not None else None
        if o is not None and self.last_odom is not None and not gap:
            step=np.linalg.inv(self.last_odom)@o
            if np.linalg.norm(step[:3,3])>self.max_odom_step or Rotation.from_matrix(step[:3,:3]).magnitude()>self.max_odom_angle:
                # An odometry reset/jump invalidates the old map/odom anchor, not only this scan.
                self.anchor=self.candidate=None
                self.last_good_ns=None; self.streak=0; self.tracking=False; self.last_odom=o
                return "LOST","missing",None
        if gap:
            self.streak = 0
            self.candidate = None
            self.tracking = False
        if accepted and p is not None:
            reference = self.candidate
            if self.tracking and self.anchor is not None and o is not None:
                reference = self.anchor @ o
            elif reference is not None and o is not None and self.last_odom is not None:
                reference = reference @ np.linalg.inv(self.last_odom) @ o
            consistent = reference is None or (
                np.linalg.norm((np.linalg.inv(reference) @ p)[:3, 3]) <= self.max_translation
                and Rotation.from_matrix((np.linalg.inv(reference) @ p)[:3, :3]).magnitude() <= self.max_rotation)
            if consistent:
                self.streak += 1
                self.candidate = p
            else:
                self.streak = 0
                self.candidate = p
                self.tracking = False
            required = self.confirmations if self.anchor is None or self.tracking else max(2, self.confirmations)
            if self.streak >= required:
                self.last_good_ns = n
                if o is not None:
                    self.anchor = p @ np.linalg.inv(o)
                self.last_odom = o
                self.tracking = True
                return "TRACKING", "map_based", pose7(p)
        else:
            self.streak = 0
            self.candidate = None
            self.tracking = False
        self.last_odom = o
        if self.anchor is not None and o is not None and self.fallback and (n-self.last_good_ns)*1e-9 <= self.fallback_limit:
            return "ODOMETRY_ONLY", "odometry_only", pose7(self.anchor @ o)
        return ("LOST" if self.anchor is not None and (n-self.last_good_ns)*1e-9 > self.fallback_limit else
                "RELOCALIZING" if self.anchor is not None else "GLOBAL_SEARCH"), "unconfirmed" if p is not None else "missing", None


def export_results(folder, events, poses, odometry, settings=None, guard=True, availability_after_s=0.):
    folder = Path(folder)
    supervisor = Guard(settings)
    rows, status, odo_rows, relocalizations = [], [], [], []
    previous_state = "UNINITIALIZED"
    for event in events:
        n = event["stamp_ns"]
        p, o = poses.get(n), odometry.get(n)
        if guard:
            state, source, output = supervisor.update(n, p, o, event["accepted"])
        else:
            state, source, output = ("TRACKING", "map_based", p) if event["accepted"] else ("LOST", "missing", None)
        if event.get("state")=="UNINITIALIZED" and output is None:
            state="UNINITIALIZED"
        record = {"stamp_ns": n, "t": f"{n//10**9}.{n%10**9:09d}", "state": state, "source": source,
                  "upstream_accepted": event["accepted"], "pose_available": output is not None,
                  "available_after_sensor_ns":max(n,events[0]["stamp_ns"]+round(availability_after_s*1e9)) if output is not None else None}
        status.append(record)
        if state == "TRACKING" and previous_state != "TRACKING":
            relocalizations.append({"stamp_ns": n, "from_state": previous_state, "to_state": state})
        previous_state = state
        if output is not None:
            rows.append([n*1e-9, *output])
        if o is not None:
            odo_rows.append([n*1e-9, *o])
        record["pose"] = output
    # Keep raw upstream output for diagnosis; final trajectory includes only explicitly labelled estimates.
    if (folder/"trajectory.tum").exists():
        (folder/"trajectory.tum").rename(folder/"upstream_trajectory.tum")
    if rows:
        save(folder/"trajectory.tum", np.asarray(rows), csv=False)
    else:
        (folder/"trajectory.tum").write_text("# No confirmed global poses\n")
    if odo_rows:
        save(folder/"odometry.tum", np.asarray(odo_rows), csv=False)
    else:
        (folder/"odometry.tum").write_text("# No external odometry available; internal constant-velocity prediction is not an odometry trajectory\n")
    with (folder/"trajectory.csv").open("w") as f:
        w = csv.writer(f); w.writerow(["stamp_ns", "t", "x", "y", "z", "qx", "qy", "qz", "qw", "source", "state","available_after_sensor_ns"])
        for record in status:
            w.writerow([record["stamp_ns"], record["t"], *(record["pose"] or [None]*7), record["source"], record["state"],record["available_after_sensor_ns"]])
    with (folder/"localization_status.csv").open("w") as f:
        keys = ["stamp_ns", "t", "state", "source", "upstream_accepted", "pose_available","available_after_sensor_ns"]
        w = csv.DictWriter(f, keys, extrasaction="ignore"); w.writeheader(); w.writerows(status)
    with (folder/"relocalization_events.csv").open("w") as f:
        w = csv.DictWriter(f, ["stamp_ns", "from_state", "to_state"]); w.writeheader(); w.writerows(relocalizations)
    duration = (events[-1]["stamp_ns"]-events[0]["stamp_ns"])*1e-9 if len(events)>1 else 0.
    # Integrate only bounded intervals: a LiDAR outage must not become confirmed coverage.
    times = {key: 0. for key in ("map_based", "odometry_only", "unconfirmed", "missing")}
    online_confirmed=0.
    for a,b in zip(status,status[1:]):
        dt = (b["stamp_ns"]-a["stamp_ns"])*1e-9
        times[a["source"] if dt<=supervisor.max_gap else "missing"] += dt
        if a["source"]=="map_based" and dt<=supervisor.max_gap:
            online_confirmed+=max(0,b["stamp_ns"]-a["available_after_sensor_ns"])*1e-9
    result = {"scans": len(events), "confirmed_poses": sum(s["source"]=="map_based" for s in status),
              "duration_s": duration, "time_by_source_s": times,
              "confirmed_time_fraction": times["map_based"]/duration if duration else None,
              "online_confirmed_time_fraction":online_confirmed/duration if duration else None,
              "initialization_time_s": max(availability_after_s,(relocalizations[0]["stamp_ns"]-events[0]["stamp_ns"])*1e-9) if relocalizations else None,
              "tracking_entries": len(relocalizations), "absolute_accuracy": None,
              "missing_poses_are_not_interpolated": True}
    write_json(folder/"metrics.json",result)
    return result
