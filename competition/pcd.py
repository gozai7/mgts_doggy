"""Read XYZ from the ASCII/binary PCD exports produced by the selected backends."""
from pathlib import Path
import numpy as np


def xyz(path):
    with Path(path).open('rb') as f:
        header = {}
        for _ in range(100):
            line = f.readline().decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split(); header[key.upper()] = values
            if key.upper() == 'DATA':
                break
        else:
            raise ValueError('PCD header not terminated')
        names, sizes, types = header['FIELDS'], list(map(int, header['SIZE'])), header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(names))))
        if not len(names) == len(sizes) == len(types) == len(counts):
            raise ValueError('Invalid PCD fields')
        if any(k not in names or counts[names.index(k)] != 1 for k in ('x','y','z')):
            raise ValueError('PCD must contain scalar x,y,z')
        n = int(header['POINTS'][0]); encoding = header['DATA'][0]
        if encoding == 'binary':
            codes = {'F':'f', 'U':'u', 'I':'i'}
            dtype = np.dtype([(k, '<' + codes[t] + str(s), (c,)) for k,s,t,c in zip(names,sizes,types,counts)])
            data = f.read()
            if len(data) != n * dtype.itemsize:
                raise ValueError('PCD binary size does not match POINTS')
            a = np.frombuffer(data, dtype=dtype)
            result = np.column_stack([a[k][:, 0] for k in ('x','y','z')])
        elif encoding == 'ascii':
            a = np.loadtxt(f, ndmin=2)
            if a.shape != (n, sum(counts)):
                raise ValueError('PCD ASCII size does not match header')
            offsets = np.cumsum([0]+counts[:-1])
            result = a[:, [offsets[names.index(k)] for k in ('x','y','z')]]
        else:
            raise ValueError('Viewer accepts ASCII/binary PCD; convert binary_compressed with pcl_convert_pcd_ascii_binary')
        return result[np.isfinite(result).all(axis=1)].astype(np.float32)
