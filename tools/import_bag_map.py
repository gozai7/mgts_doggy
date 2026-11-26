#!/usr/bin/env python3
"""Import an explicitly world-framed scan stream; this does not run or claim SLAM optimization."""
import argparse
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

import numpy as np
import yaml
from competition import bagio,cloud
from competition.config import load,write_json,source_identity
from competition.trajectory import save
from competition.occupancy import build


def import_map(config, world_topic, output, *, every=5, leaf=.10, drop_zero_padding=False):
    c=load(config); output=Path(output)
    if output.exists():raise FileExistsError(output)
    if every<1 or leaf<=0:raise ValueError("Positive scan stride and voxel size required")
    output.mkdir(parents=True); folder=output/"mapping"; folder.mkdir()
    points=np.empty((0,3),np.float32); rows=[]; frame=None; scans=0
    with bagio.reader(c["bags"]["mapping"]) as reader:
        topics={x.topic for x in reader.connections}
        if world_topic not in topics or c["topics"]["odometry"] not in topics:
            raise ValueError("Explicit world cloud and base odometry topics required")
        selected=[x for x in reader.connections if x.topic in (world_topic,c["topics"]["odometry"])]
        for conn,_,raw in reader.messages(connections=selected):
            m=reader.deserialize(raw,conn.msgtype)
            if frame is None:frame=m.header.frame_id
            if m.header.frame_id!=frame:raise ValueError("World cloud and odometry must share one frame")
            if conn.topic==c["topics"]["odometry"]:
                if m.child_frame_id!=c["source_frames"]["odometry_child"]:
                    raise ValueError("Odometry child must be the physical base frame")
                rows.append([bagio.ns(m)*1e-9,*bagio.pose_values(m.pose.pose)])
            else:
                scans+=1
                if (scans-1)%every:continue
                xyz,_=cloud.unpack(m); xyz=xyz[np.isfinite(xyz).all(axis=1)]
                if drop_zero_padding:xyz=xyz[np.any(xyz!=0,axis=1)]
                points=np.vstack([points,xyz])
                _,ids=np.unique(np.floor(points/leaf).astype(np.int64),axis=0,return_index=True)
                points=points[np.sort(ids)]
    if len(points)<5 or len(rows)<2:raise ValueError("Insufficient world map/trajectory observations")
    header=(f"VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n")
    (folder/"map.pcd").write_bytes(header.encode()+points.astype("<f4").tobytes())
    save(folder/"trajectory.tum",np.asarray(rows))
    write_json(folder/"frames.json",{"map_frame":c["frames"]["map"],"source_world_frame":frame,
        "T_map_source_world":[0,0,0,0,0,0,1],"body_frame":c["source_frames"]["odometry_child"],
        "optimization":"none; supplied onboard odometry","provenance":"provided_bag_world_cloud"})
    (folder/"config_resolved.yaml").write_text(yaml.safe_dump(c))
    # Occupancy must use raw sensor-frame scans and the measured mount, not world-frame scans as local hits.
    from competition.normalize import normalize_bag
    mapping_config=json.loads(json.dumps(c)); mapping_config["topics"]["imu"]=None
    normalize_bag(c["bags"]["mapping"],output/"normalized/mapping",mapping_config,require_imu=False)
    build(output/"normalized/mapping/bag",folder/"trajectory.tum",output/"occupancy",mapping_config)
    report=json.loads((output/"occupancy/report.json").read_text())
    report["method"]="raycasting using supplied onboard base odometry and measured LiDAR mount; no offline graph optimization"
    write_json(output/"occupancy/report.json",report)
    write_json(folder/"complete.json",{"complete":True,"backend":"provided_bag_world_cloud","source":str(c["bags"]["mapping"]),
        "source_identity":source_identity(c["bags"]["mapping"]),"world_topic":world_topic,"world_frame":frame,
        "scans":scans,"points":len(points),"voxel_m":leaf,"every_n_scans":every,
        "optimization":"none; not a GLIM or RKO result","drop_zero_padding":drop_zero_padding})
    return output


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("--config",required=True);p.add_argument("--world-cloud",required=True)
    p.add_argument("--output",required=True);p.add_argument("--every",type=int,default=5);p.add_argument("--leaf",type=float,default=.1)
    p.add_argument("--drop-zero-padding",action="store_true")
    a=p.parse_args(); print(import_map(a.config,a.world_cloud,a.output,every=a.every,leaf=a.leaf,drop_zero_padding=a.drop_zero_padding))
