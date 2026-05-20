# -*- coding: utf-8 -*-
# =====================================================================
# Abaqus 2021 (Python 2.7) ODB -> topo_stress_result.mat
# =====================================================================
#
# Reads an Abaqus ODB after static analysis, extracts per-element first
# principal stress directions, and writes a topo_stress_result.mat
# matching the format expected by voxel_refinement_from_test.m.
#
# Works with both voxelize.py modes:
#   source_mode == 'stl'  : xPhys = 0/1 occupancy mask
#   source_mode == 'topo' : xPhys = continuous SIMP density field
#                           (read from npz, NOT computed from stress)
# The mode is auto-detected from the .npz produced by voxelize.py.
#
# Angle convention (must match voxel_refinement_from_test.m):
#   u = cosd(t_xoz) * cosd(t_xoy)
#   v = cosd(t_xoz) * sind(t_xoy)
#   w = sind(t_xoz)
#   => t_xoy = atan2(vy, vx) [deg]
#      t_xoz = atan2(vz, sqrt(vx^2+vy^2)) = asin(vz) [deg]
#
# Usage:
#   abaqus python abaqus_odb_to_mat.py --odb Job.odb --npz voxel_grid.npz
#
# Windows cmd users: do NOT use bash-style '\' line continuation.
# Either keep the command on a single line, or use '^' (cmd) / '`' (PowerShell).
#
# Constraints (Albert's environment):
#   - Line 1 declares UTF-8.
#   - Source is ASCII only.
#   - Python 2-compatible syntax.
# =====================================================================

from __future__ import print_function
import os
import sys
import math
import argparse

import numpy as np
from scipy.io import savemat

# Abaqus ODB API (only available under 'abaqus python')
try:
    from odbAccess import openOdb
    from abaqusConstants import (CENTROID, INTEGRATION_POINT, ELEMENT_NODAL)
except ImportError:
    sys.stderr.write(
        'ERROR: must be run with "abaqus python ..." '
        '(odbAccess not importable).\n')
    sys.exit(1)


# =====================================================================
# Helpers
# =====================================================================

def log(msg):
    print('[odb_to_mat] ' + str(msg))


def principal_eigvec(s11, s22, s33, s12, s13, s23, mode='max'):
    """
    Return (eigvec, eigval) for the chosen principal direction.
    mode: 'max' = largest |sigma|, 'tension' = largest +sigma,
          'compression' = smallest -sigma.
    """
    T = np.array([[s11, s12, s13],
                  [s12, s22, s23],
                  [s13, s23, s33]], dtype=float)
    try:
        w, V = np.linalg.eigh(T)
    except Exception:
        return None, 0.0
    if mode == 'tension':
        k = int(np.argmax(w))
    elif mode == 'compression':
        k = int(np.argmin(w))
    else:  # 'max'
        k = int(np.argmax(np.abs(w)))
    return V[:, k], w[k]


def dir_to_angles_deg(d):
    """
    Convert direction (dx,dy,dz) to (t_xoy, t_xoz) in degrees.
    Convention matches voxel_refinement_from_test.m:
        u = cosd(t_xoz)*cosd(t_xoy)
        v = cosd(t_xoz)*sind(t_xoy)
        w = sind(t_xoz)              [NOTE: positive sin]
    => t_xoy = atan2(vy, vx),  t_xoz = atan2(vz, sqrt(vx^2+vy^2))
    """
    n = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
    if n < 1e-12:
        return 0.0, 0.0
    dx, dy, dz = d[0] / n, d[1] / n, d[2] / n
    r_xy = math.sqrt(dx * dx + dy * dy)
    t_xoy = math.degrees(math.atan2(dy, dx))
    t_xoz = math.degrees(math.atan2(dz, r_xy))
    return t_xoy, t_xoz


# =====================================================================
# Core extraction
# =====================================================================

def extract(odb_path, npz_path, output_mat, frame_idx=-1,
            principal_mode='max', instance_name=None, step_name=None,
            verbose=True):
    # ---- Load metadata ----
    if verbose:
        log('Loading metadata: ' + npz_path)
    meta = np.load(npz_path, allow_pickle=False)

    # source_mode: 'stl' or 'topo'
    if 'source_mode' in meta.files:
        source_mode = str(meta['source_mode'])
    else:
        source_mode = 'stl'   # backward compat (pre-merge npz files)
    if verbose:
        log('Source mode: ' + source_mode)

    nelx = int(meta['nelx'])
    nely = int(meta['nely'])
    nelz = int(meta['nelz'])
    voxel_size = float(meta['voxel_size'])
    origin_xyz = np.asarray(meta['origin_xyz'])
    occupancy_xyz = meta['occupancy_xyz'].astype(bool)
    e2ijk = meta['elem_label_to_ijk']
    n_elements = int(meta['n_elements'])

    # Topo-mode extras
    xPhys_full_yxz = None
    density_threshold = None
    iter_data = None
    if source_mode == 'topo':
        if 'xPhys_full_yxz' in meta.files:
            xPhys_full_yxz = np.asarray(meta['xPhys_full_yxz'])
            if verbose:
                log('Loaded continuous xPhys from npz, shape=%s, range=[%.4f, %.4f]'
                    % (str(xPhys_full_yxz.shape),
                       float(xPhys_full_yxz.min()),
                       float(xPhys_full_yxz.max())))
        if 'density_threshold' in meta.files:
            density_threshold = float(meta['density_threshold'])
        if 'iter_data' in meta.files:
            iter_data = np.asarray(meta['iter_data'], dtype=float).flatten()

    if verbose:
        log('Grid: %d x %d x %d (nele=%d)' % (nelx, nely, nelz, n_elements))

    # ---- Open ODB ----
    if verbose:
        log('Opening ODB: ' + odb_path)
    odb = openOdb(odb_path, readOnly=True)

    instances = odb.rootAssembly.instances
    if instance_name is None:
        cand = [n for n in instances.keys() if n.upper() != 'ASSEMBLY']
        instance_name = cand[0] if cand else list(instances.keys())[0]
    if verbose:
        log('Instance: ' + instance_name)
    inst = instances[instance_name]

    # Step + frame
    step_keys = list(odb.steps.keys())
    if not step_keys:
        raise RuntimeError('No steps in ODB.')
    if step_name is None:
        step_name = step_keys[-1]
    step = odb.steps[step_name]
    if verbose:
        log('Step: ' + step_name)

    n_frames = len(step.frames)
    if frame_idx < 0:
        frame_idx = n_frames + frame_idx
    if frame_idx < 0 or frame_idx >= n_frames:
        raise RuntimeError('frame_idx %d out of range [0, %d)'
                           % (frame_idx, n_frames))
    frame = step.frames[frame_idx]
    if verbose:
        log('Frame: %d / %d (time = %g)'
            % (frame_idx, n_frames, frame.frameValue))

    # ---- Extract stress ----
    if 'S' not in frame.fieldOutputs:
        raise RuntimeError("Field 'S' not in this frame. Did the job complete?")
    s_field = frame.fieldOutputs['S']
    s_sub = s_field.getSubset(region=inst, position=CENTROID)

    # Iterate (works in Abaqus python where len/index sometimes fail)
    label_to_sum = {}
    label_to_count = {}
    n_vals = 0
    for v in s_sub.values:
        eid = int(v.elementLabel)
        if len(v.data) < 6:
            continue
        if eid not in label_to_sum:
            label_to_sum[eid] = [0.0] * 6
            label_to_count[eid] = 0
        for j in range(6):
            label_to_sum[eid][j] += float(v.data[j])
        label_to_count[eid] += 1
        n_vals += 1
        if verbose and n_vals % 50000 == 0:
            log('  read %d stress values...' % n_vals)

    # Average
    label_to_stress = {}
    for eid, sv in label_to_sum.items():
        c = float(label_to_count[eid])
        label_to_stress[eid] = tuple(sv[j] / c for j in range(6))

    if verbose:
        log('Stress samples: %d values -> %d elements averaged'
            % (n_vals, len(label_to_stress)))

    # ---- Build [nelx,nely,nelz] xyz arrays ----
    t_xoy_xyz = np.zeros((nelx, nely, nelz), dtype=np.float64)
    t_xoz_xyz = np.zeros((nelx, nely, nelz), dtype=np.float64)
    n_missing = 0

    for e in range(n_elements):
        eid = e + 1
        i, j, k = int(e2ijk[e, 0]), int(e2ijk[e, 1]), int(e2ijk[e, 2])
        if eid in label_to_stress:
            s11, s22, s33, s12, s13, s23 = label_to_stress[eid]
            d, _ = principal_eigvec(s11, s22, s33, s12, s13, s23,
                                    mode=principal_mode)
            if d is not None:
                ty, tz = dir_to_angles_deg(d)
                t_xoy_xyz[i, j, k] = ty
                t_xoz_xyz[i, j, k] = tz
        else:
            n_missing += 1

    if n_missing > 0 and verbose:
        log('WARN: %d elements had no stress data' % n_missing)

    odb.close()

    # ---- Build xPhys based on mode ----
    # Both modes also expose a binary 'mask' = "this element actually exists
    # in the Abaqus model and has stress data". Downstream pipelines can use
    # either xPhys > threshold (their existing convention) OR this mask
    # (direct, no thresholding ambiguity).
    if source_mode == 'topo' and xPhys_full_yxz is not None:
        # Continuous SIMP density field (already in [nely,nelx,nelz])
        xPhys = xPhys_full_yxz.astype(np.float64)
        xPhys_semantics = 'simp_continuous_density'
    else:
        # STL mode: xPhys = valid voxel mask, 0 or 1
        xPhys_xyz = occupancy_xyz.astype(np.float64)
        xPhys = np.transpose(xPhys_xyz, (1, 0, 2))   # [nely,nelx,nelz]
        xPhys_semantics = 'valid_voxel_mask'

    # Build explicit material mask in [nely,nelx,nelz] (which elements were
    # actually sent to Abaqus and have stress results).
    mask_xyz = occupancy_xyz.astype(np.float64)
    mask = np.transpose(mask_xyz, (1, 0, 2))   # [nely,nelx,nelz]

    # ---- Reorder direction arrays to [nely,nelx,nelz] ----
    t_xoy = np.transpose(t_xoy_xyz, (1, 0, 2))
    t_xoz = np.transpose(t_xoz_xyz, (1, 0, 2))

    # Direction at masked-out positions is meaningless: explicitly zero it.
    # (It was already zero from initialization; this just makes the contract
    # explicit and protects against future changes to the loop.)
    masked_out = mask < 0.5
    t_xoy[masked_out] = 0.0
    t_xoz[masked_out] = 0.0

    # ---- Write .mat ----
    out = {
        'nelx': float(nelx),
        'nely': float(nely),
        'nelz': float(nelz),
        'nele': float(nelx * nely * nelz),
        'xPhys': xPhys,
        'mask': mask,                        # 1 = active element (sent to Abaqus)
        't_xoy': t_xoy,
        't_xoz': t_xoz,
        'Iter': float(frame_idx + 1),
        # Metadata (downstream ignores extras)
        'voxel_size': float(voxel_size),
        'origin_xyz': np.asarray(origin_xyz, dtype=float),
        'source_mode': source_mode,
        'xPhys_semantics': xPhys_semantics,
        'source_odb': os.path.abspath(odb_path),
        'source_npz': os.path.abspath(npz_path),
        'principal_mode': principal_mode,
    }
    if density_threshold is not None:
        out['density_threshold'] = float(density_threshold)
    if iter_data is not None and iter_data.size > 0:
        out['Iter_history'] = iter_data

    savemat(output_mat, out, do_compression=True)
    if verbose:
        sz = os.path.getsize(output_mat) / 1024.0 / 1024.0
        log('Saved: %s (%.2f MB)' % (output_mat, sz))
        log('xPhys: shape=%s, semantics=%s, range=[%.4f, %.4f]'
            % (str(xPhys.shape), xPhys_semantics,
               float(xPhys.min()), float(xPhys.max())))
        log('mask (active elements): %d / %d  (%.2f%%)'
            % (int(mask.sum()), int(mask.size),
               100.0 * float(mask.sum()) / float(mask.size)))
        log('t_xoy range: [%.2f, %.2f] deg  (zero outside mask)'
            % (float(t_xoy.min()), float(t_xoy.max())))
        log('t_xoz range: [%.2f, %.2f] deg  (zero outside mask)'
            % (float(t_xoz.min()), float(t_xoz.max())))


# =====================================================================
# CLI
# =====================================================================

def _build_parser():
    p = argparse.ArgumentParser(
        description='Extract per-element first principal stress directions '
                    'from an Abaqus ODB into topo_stress_result.mat. '
                    'xPhys is auto-selected: occupancy mask if source_mode=stl, '
                    'continuous SIMP density if source_mode=topo.')
    p.add_argument('--odb', required=True, help='Path to .odb file.')
    p.add_argument('--npz', required=True,
                   help='Metadata .npz from voxelize.py.')
    p.add_argument('--output', default='topo_stress_result.mat',
                   help='Output .mat path.')
    p.add_argument('--frame', type=int, default=-1,
                   help='Frame index (default last).')
    p.add_argument('--principal', default='max',
                   choices=['max', 'tension', 'compression'],
                   help='Which principal eigenvector of stress.')
    p.add_argument('--instance', default=None,
                   help='Override instance name.')
    p.add_argument('--step', default=None,
                   help='Override step name (default: last).')
    return p


def main():
    args = _build_parser().parse_args()
    extract(
        odb_path=args.odb,
        npz_path=args.npz,
        output_mat=args.output,
        frame_idx=args.frame,
        principal_mode=args.principal,
        instance_name=args.instance,
        step_name=args.step,
        verbose=True,
    )


if __name__ == '__main__':
    main()
