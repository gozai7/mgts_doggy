"""Bounded structural audit; heuristics never become a calibration."""
from collections import defaultdict
from pathlib import Path
import numpy as np
from .bagio import reader, ns, vec
from .cloud import unpack, TIME_FIELDS
from .config import write_json


def summary(a):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return None
    return dict(zip(("min", "median", "p95", "p99", "max"), map(float, np.percentile(a, [0, 50, 95, 99, 100]))))


def audit(path, output, max_messages=200000, cloud_samples=50):
    stats = defaultdict(lambda: {"header": [], "bag": [], "frames": set(), "clouds": [], "acc": [], "gyro": [], "positions": []})
    edges, errors, warnings, assumptions = set(), [], [], []
    seen = 0
    with reader(path) as r:
        counts = defaultdict(int)
        types = {}
        for c in r.connections:
            counts[c.topic] += c.msgcount
            types[c.topic] = c.msgtype
        for conn, t, raw in r.messages():
            if seen >= max_messages:
                break
            seen += 1
            typ = conn.msgtype
            if typ not in ("sensor_msgs/msg/PointCloud2", "sensor_msgs/msg/Imu", "nav_msgs/msg/Odometry", "tf2_msgs/msg/TFMessage"):
                continue
            try:
                m = r.deserialize(raw, typ)
                if typ.endswith("TFMessage"):
                    for tf in m.transforms:
                        edges.add((tf.header.frame_id, tf.child_frame_id, conn.topic.endswith("tf_static")))
                    continue
                s = stats[conn.topic]
                s["header"].append(ns(m)); s["bag"].append(t); s["frames"].add(m.header.frame_id)
                if typ.endswith("PointCloud2"):
                    stride = max(1, counts[conn.topic] // cloud_samples)
                    if (len(s["header"]) - 1) % stride == 0 and len(s["clouds"]) < cloud_samples:
                        xyz, a = unpack(m)
                        valid = np.isfinite(xyz).all(axis=1)
                        times = {k: summary(a[k]) for k in TIME_FIELDS if k in a.dtype.names}
                        s["clouds"].append({"points": len(xyz), "invalid": int((~valid).sum()),
                           "fields": [{"name": f.name, "datatype": f.datatype, "count": f.count} for f in m.fields],
                           "range_raw_units": summary(np.linalg.norm(xyz[valid], axis=1)), "point_time_candidates": times})
                elif typ.endswith("Imu"):
                    if not np.isfinite(np.r_[vec(m.linear_acceleration), vec(m.angular_velocity)]).all():
                        raise ValueError("Nonfinite IMU values")
                    s["acc"].append(float(np.linalg.norm(vec(m.linear_acceleration))))
                    s["gyro"].append(float(np.linalg.norm(vec(m.angular_velocity))))
                elif typ.endswith("Odometry"):
                    from .bagio import pose_values
                    from .config import transform
                    transform(pose_values(m.pose.pose))
                    s["positions"].append(vec(m.pose.pose.position).tolist())
            except (ValueError, TypeError) as e:
                errors.append(f"{conn.topic}: {e}")
        duration = (r.end_time - r.start_time) * 1e-9
    result = {"bag": str(path), "duration_s": duration, "read_messages": seen,
              "complete_header_audit": seen >= sum(counts.values()), "topics": {},
              "tf_edges": sorted(edges), "errors": errors, "warnings": warnings, "assumptions": assumptions}
    for topic, count in counts.items():
        row = {"type": types[topic], "messages": count}
        s = stats.get(topic)
        if s and s["header"]:
            t = np.array(s["header"], dtype=np.int64)
            dt = np.diff(t) * 1e-9
            posdt = dt[dt > 0]
            row.update({"examined": len(t), "frame_ids": sorted(s["frames"]),
                        "header_span_s": [float(t.min() * 1e-9), float(t.max() * 1e-9)],
                        "nonmonotonic_or_duplicate": int((dt <= 0).sum()), "interval_s": summary(dt),
                        "rate_hz": float(1 / np.median(posdt)) if len(posdt) else None,
                        "header_minus_record_s": summary((t - np.array(s["bag"], dtype=np.int64)) * 1e-9),
                        "large_gaps_gt_2_5_median": int((dt > 2.5 * np.median(posdt)).sum()) if len(posdt) else 0})
            if row["nonmonotonic_or_duplicate"]:
                errors.append(f"{topic}: duplicate/decreasing header timestamps")
            if row["large_gaps_gt_2_5_median"]:
                warnings.append(f"{topic}: large timestamp gaps (heuristic, not a sensor fault diagnosis)")
            if s["clouds"]:
                row["sampled_clouds"] = s["clouds"]
                assumptions.append(f"{topic}: point time units and XYZ units require dataset documentation")
            if s["acc"]:
                row["acceleration_norm_raw_units"] = summary(s["acc"])
                row["gyro_norm_raw_units"] = summary(s["gyro"])
                assumptions.append(f"{topic}: plateau/acceleration norms alone do not prove clipping or unit scale")
            if s["positions"]:
                row["odometry_path_length_raw_units"] = float(np.linalg.norm(np.diff(s["positions"], axis=0), axis=1).sum())
        result["topics"][topic] = row
    spans = [v["header_span_s"] for v in result["topics"].values() if "header_span_s" in v and v["type"].startswith("sensor_msgs/")]
    result["sensor_overlap_s"] = max(0.0, min(x[1] for x in spans) - max(x[0] for x in spans)) if spans else None
    # Detect multiple parents and cycles in the observed TF topology.
    parents = defaultdict(set)
    graph = defaultdict(set)
    for a, b, _ in edges:
        parents[b].add(a); graph[a].add(b)
    for child, ps in parents.items():
        if len(ps) > 1:
            errors.append(f"TF multiple parents: {child}: {sorted(ps)}")
    def visit(n, active, done):
        if n in active:
            return True
        if n in done:
            return False
        active.add(n)
        cyclic = any(visit(x, active, done) for x in graph.get(n, []))
        active.remove(n); done.add(n)
        return cyclic
    if any(visit(n, set(), set()) for n in list(graph)):
        errors.append("TF cycle detected")
    if not edges:
        warnings.append("No TF found; measured extrinsics must be supplied explicitly")
    undirected = defaultdict(set)
    for a, b, _ in edges:
        undirected[a].add(b); undirected[b].add(a)
    unseen = set(undirected); components = []
    while unseen:
        stack = [min(unseen)]; component = set()
        while stack:
            node = stack.pop()
            if node in component:
                continue
            component.add(node); stack.extend(undirected[node] - component)
        unseen -= component; components.append(sorted(component))
    result["tf_connected_components"] = components
    if len(components) > 1:
        warnings.append("Observed TF has disconnected components; verify sensor-to-base calibration")
    result["tf_note"] = "Topology only: valid transforms at every scan time are not established by this audit"
    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    write_json(output / "bag_report.json", result)
    lines = ["# Bag Doctor", f"Bag: `{path}`; duration {duration:.3f} s.",
             "", "| Topic | Type | Messages | Hz |", "|---|---|---:|---:|"]
    for topic, s in result["topics"].items():
        lines.append(f"| {topic} | {s['type']} | {s['messages']} | {s.get('rate_hz')} |")
    for label, values in (("Errors", errors), ("Warnings", warnings), ("Assumptions to verify", assumptions)):
        lines.extend(["", "## " + label, ""] + ["- " + v for v in values])
    (output / "bag_report.md").write_text("\n".join(lines) + "\n")
    return result
