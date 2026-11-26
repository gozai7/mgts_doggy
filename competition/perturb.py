"""Reproducible corruption of a COPY; independent record/header/point clocks."""
from pathlib import Path
import sqlite3
import numpy as np
from rosbags.rosbag2 import Writer
from . import bagio, cloud
from .config import write_json


def perturb(source, output, *, topic, seed=42, header_shift_s=0., record_shift_s=0.,
            point_shift=0., point_scale=1., time_field="time", keep_points=1., drop_scans=0.,
            gap_s=None, gyro_bias=(0., 0., 0.), gyro_noise=0., accel_noise=0., gyro_clip=None,
            remove_time=False, tf_child=None, tf_translation=(0., 0., 0.), tf_rpy_deg=(0., 0., 0.)):
    from scipy.spatial.transform import Rotation
    source, output = Path(source), Path(output)
    if output.exists() or output.resolve() == source.resolve() or source.resolve() in output.resolve().parents:
        raise ValueError("Choose a new output directory outside the source bag")
    if not 0 < keep_points <= 1 or not 0 <= drop_scans < 1:
        raise ValueError("Invalid keep/drop fractions")
    options = {k: v for k, v in locals().copy().items() if k not in ("Rotation", "source", "output")}
    options = {k: str(v) if isinstance(v, Path) else v for k, v in options.items()}
    rng = np.random.default_rng(seed)
    output.parent.mkdir(parents=True, exist_ok=True)
    staging = output.with_name(output.name + ".partial"); staging.mkdir()
    db = sqlite3.connect(staging / "sort.sqlite")
    db.execute("CREATE TABLE m(t INTEGER, seq INTEGER, connection INTEGER, data BLOB)")
    dropped, changed = 0, 0
    with bagio.reader(source) as r, Writer(staging / "bag", version=9) as w:
        conns = {x.id: w.add_connection(x.topic, x.msgtype, typestore=r.typestore) for x in r.connections}
        if topic not in {x.topic for x in r.connections}:
            raise ValueError(f"Missing perturbation topic {topic}")
        for i, (conn, t, raw) in enumerate(r.messages()):
            if conn.topic == topic:
                if gap_s is not None and gap_s[0] <= (t - r.start_time) * 1e-9 <= gap_s[1]:
                    dropped += 1; continue
                if conn.msgtype.endswith("PointCloud2") and rng.random() < drop_scans:
                    dropped += 1; continue
                m = r.deserialize(raw, conn.msgtype)
                t += round(record_shift_s * 1e9)
                if hasattr(m, "header"):
                    bagio.set_stamp(m, bagio.ns(m) + round(header_shift_s * 1e9))
                if conn.msgtype.endswith("PointCloud2"):
                    a = cloud.fields(m).copy()
                    if time_field in a.dtype.names:
                        if remove_time:
                            m.fields = [f for f in m.fields if f.name != time_field]
                        else:
                            values = a[time_field].astype(float) * point_scale + point_shift
                            if np.issubdtype(a[time_field].dtype, np.integer):
                                limits = np.iinfo(a[time_field].dtype)
                                if np.any((values < limits.min) | (values > limits.max)):
                                    raise ValueError("Point time perturbation overflows source field")
                            a[time_field] = values
                    a = a[rng.random(len(a)) < keep_points]
                    m.height, m.width = 1, len(a); m.row_step = len(a) * m.point_step
                    m.data = np.frombuffer(a.tobytes(), dtype=np.uint8).copy()
                elif conn.msgtype.endswith("Imu"):
                    omega = bagio.vec(m.angular_velocity) + gyro_bias + rng.normal(0, gyro_noise, 3)
                    if gyro_clip is not None:
                        omega = np.clip(omega, -gyro_clip, gyro_clip)
                    bagio.assign_vec(m.angular_velocity, omega)
                    bagio.assign_vec(m.linear_acceleration, bagio.vec(m.linear_acceleration) + rng.normal(0, accel_noise, 3))
                elif conn.msgtype.endswith("TFMessage") and tf_child:
                    for tf in m.transforms:
                        if tf.child_frame_id == tf_child:
                            bagio.assign_vec(tf.transform.translation, bagio.vec(tf.transform.translation) + tf_translation)
                            q = tf.transform.rotation
                            v = (Rotation.from_euler("xyz", tf_rpy_deg, degrees=True) * Rotation.from_quat([q.x,q.y,q.z,q.w])).as_quat()
                            q.x,q.y,q.z,q.w = map(float, v)
                raw = r.typestore.serialize_cdr(m, conn.msgtype)
                changed += 1
            if t < 0:
                raise ValueError("Negative record time after perturbation")
            db.execute("INSERT INTO m VALUES(?,?,?,?)", (int(t), i, conn.id, bytes(raw)))
        db.commit()
        for t, _, conn, raw in db.execute("SELECT * FROM m ORDER BY t,seq"):
            w.write(conns[conn], t, raw)
    db.close(); (staging / "sort.sqlite").unlink()
    write_json(staging / "manifest.json", {"source": str(source), "seed": seed, "changes": options,
               "modified_messages": changed, "dropped_messages": dropped, "original_modified": False})
    staging.rename(output)
