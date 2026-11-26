#!/usr/bin/env python3
"""Verify an existing rigid mount from exact point correspondences in paired raw/base topics."""
import argparse
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import numpy as np
from scipy.spatial.transform import Rotation
from competition import bagio,cloud
from competition.config import write_json,source_identity


def fit(raw,base):
    def keys(fields):
        return [tuple(float(fields[k][i]) for k in ("ring","time","intensity")) for i in range(len(fields))]
    a,fa=raw; b,fb=base
    raw_keys,base_keys=keys(fa),keys(fb)
    ids={key:i for i,key in enumerate(raw_keys)}
    pairs=[(ids[key],i) for i,key in enumerate(base_keys) if key in ids]
    if len(pairs)<50:raise ValueError("Fewer than 50 identical ring/time/intensity point identities")
    aa=a[[i for i,j in pairs]];bb=b[[j for i,j in pairs]]
    u,_,vt=np.linalg.svd((aa-aa.mean(0)).T@(bb-bb.mean(0)));vt[-1]*=np.linalg.det(vt.T@u.T)
    rotation=vt.T@u.T;translation=bb.mean(0)-rotation@aa.mean(0)
    error=np.linalg.norm(aa@rotation.T+translation-bb,axis=1)
    if np.max(error)>1e-4:
        raise ValueError("Paired point identities do not define one rigid mount within 0.1 mm")
    return [*translation,*Rotation.from_matrix(rotation).as_quat()],float(np.sqrt(np.mean(error**2))),len(pairs)


def infer(bag,raw_topic,base_topic,count=10):
    pending={};estimates=[]
    with bagio.reader(bag) as reader:
        selected=[c for c in reader.connections if c.topic in (raw_topic,base_topic)]
        if len({c.topic for c in selected})!=2:raise ValueError("Both raw and base cloud topics required")
        for conn,_,data in reader.messages(connections=selected):
            m=reader.deserialize(data,conn.msgtype); n=bagio.ns(m)
            pending.setdefault(n,{})[conn.topic]=cloud.unpack(m)
            if len(pending[n])==2:
                mount,error,matches=fit(pending[n][raw_topic],pending[n][base_topic])
                estimates.append({"stamp_ns":n,"T_base_lidar":mount,"rmse_m":error,"correspondences":matches})
                del pending[n]
                if len(estimates)>=count:break
            while len(pending)>100:del pending[min(pending)]
    if len(estimates)<count:raise ValueError("Insufficient paired scans")
    values=np.asarray([e["T_base_lidar"] for e in estimates])
    rotation=Rotation.from_quat(values[:,3:]).mean()
    if np.ptp(values[:,:3],axis=0).max()>1e-4 or (rotation.inv()*Rotation.from_quat(values[:,3:])).magnitude().max()>1e-4:
        raise ValueError("Mount is not constant across paired scans")
    return {"T_base_lidar":[*values[:,:3].mean(0),*rotation.as_quat()],"method":"exact paired point identity + rigid least squares; checked across scans",
            "not_an_imu_calibration":True,"bag":str(bag),"source_identity":source_identity(bag),"checks":estimates}


if __name__=="__main__":
    p=argparse.ArgumentParser(description=__doc__);p.add_argument("bag");p.add_argument("--raw-topic",required=True)
    p.add_argument("--base-topic",required=True);p.add_argument("--output",required=True);p.add_argument("--count",type=int,default=10)
    a=p.parse_args();write_json(a.output,infer(a.bag,a.raw_topic,a.base_topic,a.count))
