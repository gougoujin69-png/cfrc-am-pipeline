# -*- coding: utf-8 -*-
"""
========================================================================
voxelize.py - Unified Voxelization Preprocessor for CFRC Pipeline
========================================================================

把任意几何输入转成 Abaqus 可直接计算的等大六面体网格.
两种输入模式:

  from-stl   : 从 STL 几何体素化 (xPhys = 0/1 mask, 实心结构)
  from-topo  : 从 SIMP 拓扑优化结果 .mat 直接转 (xPhys = 连续密度)

输出 (两种模式相同, 模式信息保存在 npz 中):
  voxel_grid.inp   - Abaqus 输入 (C3D8R), 含 6 个边界 NSET 供 BC 拾取
  voxel_grid.npz   - 元数据 + 模式标识 + (topo 模式额外保存连续密度)

完整管线:
  STL 或 SIMP mat
      |
      v  voxelize.py from-stl  /  from-topo
  voxel_grid.inp + voxel_grid.npz
      |
      v  Abaqus/CAE 加 BC / Load / Step / Job
  job.odb
      |
      v  abaqus python abaqus_odb_to_mat.py ...
  topo_stress_result.mat
      |
      v  voxel_refinement_from_test.m -> ...

依赖:
  pip install trimesh numpy scipy
"""

from __future__ import print_function
import argparse
import math
import os
import sys
import time

import numpy as np


# ============================================================
# 旋转预设 (STL 模式专用)
# ============================================================

_PI = 3.141592653589793
_ROTATION_PRESETS = {
    'y-to-z': (_PI / 2,  [1.0, 0.0, 0.0]),
    'z-to-y': (-_PI / 2, [1.0, 0.0, 0.0]),
    'x-to-z': (-_PI / 2, [0.0, 1.0, 0.0]),
    'z-to-x': (_PI / 2,  [0.0, 1.0, 0.0]),
    'x-to-y': (_PI / 2,  [0.0, 0.0, 1.0]),
    'y-to-x': (-_PI / 2, [0.0, 0.0, 1.0]),
    'flip-z': (_PI,      [1.0, 0.0, 0.0]),
    'flip-x': (_PI,      [0.0, 1.0, 0.0]),
    'flip-y': (_PI,      [0.0, 0.0, 1.0]),
}
ROTATE_AXIS_CHOICES = ['none'] + sorted(_ROTATION_PRESETS.keys())


def get_rotation_matrix(mode):
    """4x4 transform for axis-remap preset, or None for 'none'."""
    if mode is None or mode == '' or mode == 'none':
        return None
    import trimesh
    if mode not in _ROTATION_PRESETS:
        raise ValueError("Unknown rotate-axis mode: %r" % mode)
    angle, axis = _ROTATION_PRESETS[mode]
    return trimesh.transformations.rotation_matrix(angle, axis)


# ============================================================
# 公共写出工具: .inp + .npz
# ============================================================

C3D8_LOCAL_NODES = np.array([
    [0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0],
    [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1],
], dtype=np.int8)


def write_abaqus_inp(occupancy_xyz, voxel_size, origin, output_path,
                     element_type='C3D8R', youngs_modulus=70000.0,
                     poisson_ratio=0.3, density=2.7e-9,
                     part_name='VOXEL_PART', instance_name='PART-1',
                     material_name='MAT1', write_face_sets=True,
                     verbose=True):
    """Write voxel grid as Abaqus .inp. Returns metadata dict."""
    nx, ny, nz = occupancy_xyz.shape
    nxn, nyn, nzn = nx + 1, ny + 1, nz + 1
    if verbose:
        print('  [inp] occupancy grid: %d x %d x %d' % (nx, ny, nz))
        print('  [inp] element type:   %s' % element_type)

    t0 = time.time()
    # 1) Mark needed nodes
    need = np.zeros((nxn, nyn, nzn), dtype=bool)
    idxs = np.argwhere(occupancy_xyz)
    for n in range(idxs.shape[0]):
        i, j, k = idxs[n]
        need[i:i + 2, j:j + 2, k:k + 2] = True

    # 2) Assign node IDs (k slow, j mid, i fast)
    node_id_grid = np.zeros((nxn, nyn, nzn), dtype=np.int64)
    nid = 1
    for k in range(nzn):
        for j in range(nyn):
            for i in range(nxn):
                if need[i, j, k]:
                    node_id_grid[i, j, k] = nid
                    nid += 1
    n_nodes = nid - 1

    # 3) Element connectivity
    n_elem = idxs.shape[0]
    elem_label_to_ijk = np.zeros((n_elem, 3), dtype=np.int32)
    elem_conn = np.zeros((n_elem, 8), dtype=np.int64)
    for e in range(n_elem):
        i, j, k = idxs[e]
        for n in range(8):
            di, dj, dk = C3D8_LOCAL_NODES[n]
            elem_conn[e, n] = node_id_grid[i + di, j + dj, k + dk]
        elem_label_to_ijk[e] = (i, j, k)

    if verbose:
        print('  [inp] nodes: %d, elements: %d (built in %.2fs)'
              % (n_nodes, n_elem, time.time() - t0))

    # 4) Face node sets
    face_sets = {}
    if write_face_sets:
        xmin = set(); xmax = set()
        for j in range(nyn):
            for k in range(nzn):
                col = node_id_grid[:, j, k]
                pos = np.where(col > 0)[0]
                if pos.size > 0:
                    xmin.add(int(col[pos[0]]))
                    xmax.add(int(col[pos[-1]]))
        ymin = set(); ymax = set()
        for i in range(nxn):
            for k in range(nzn):
                col = node_id_grid[i, :, k]
                pos = np.where(col > 0)[0]
                if pos.size > 0:
                    ymin.add(int(col[pos[0]]))
                    ymax.add(int(col[pos[-1]]))
        zmin = set(); zmax = set()
        for i in range(nxn):
            for j in range(nyn):
                col = node_id_grid[i, j, :]
                pos = np.where(col > 0)[0]
                if pos.size > 0:
                    zmin.add(int(col[pos[0]]))
                    zmax.add(int(col[pos[-1]]))
        face_sets = {
            'N_XMIN': sorted(xmin), 'N_XMAX': sorted(xmax),
            'N_YMIN': sorted(ymin), 'N_YMAX': sorted(ymax),
            'N_ZMIN': sorted(zmin), 'N_ZMAX': sorted(zmax),
        }
        if verbose:
            for k, v in face_sets.items():
                print('  [inp] %s: %d nodes' % (k, len(v)))

    # 5) Write .inp
    t0 = time.time()
    with open(output_path, 'w') as f:
        f.write('*HEADING\n')
        f.write('Voxel grid generated by voxelize.py\n')
        f.write('Grid: %d x %d x %d (nele=%d), voxel size = %g\n'
                % (nx, ny, nz, n_elem, voxel_size))
        f.write('Origin: %.6f %.6f %.6f\n' % tuple(origin))
        f.write('**\n')
        f.write('*PART, NAME=%s\n' % part_name)
        # Nodes
        f.write('*NODE\n')
        for k in range(nzn):
            for j in range(nyn):
                for i in range(nxn):
                    nid_ = node_id_grid[i, j, k]
                    if nid_ > 0:
                        x = origin[0] + i * voxel_size
                        y = origin[1] + j * voxel_size
                        z = origin[2] + k * voxel_size
                        f.write('%8d, %16.8e, %16.8e, %16.8e\n'
                                % (nid_, x, y, z))
        # Elements
        f.write('*ELEMENT, TYPE=%s, ELSET=ALL_VOXELS\n' % element_type)
        for e in range(n_elem):
            eid = e + 1
            c = elem_conn[e]
            f.write('%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d,%8d\n'
                    % (eid, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]))
        # Face NSETs
        if write_face_sets:
            for name, nodes in face_sets.items():
                f.write('*NSET, NSET=%s\n' % name)
                _write_list(f, nodes, per_line=8)
        # Solid section
        f.write('*SOLID SECTION, ELSET=ALL_VOXELS, MATERIAL=%s\n' % material_name)
        f.write('1.0,\n')
        f.write('*END PART\n**\n')
        # Assembly
        f.write('*ASSEMBLY, NAME=Assembly\n')
        f.write('*INSTANCE, NAME=%s, PART=%s\n' % (instance_name, part_name))
        f.write('*END INSTANCE\n')
        f.write('*END ASSEMBLY\n**\n')
        # Material
        f.write('*MATERIAL, NAME=%s\n' % material_name)
        f.write('*ELASTIC\n')
        f.write('%g, %g\n' % (youngs_modulus, poisson_ratio))
        if density and density > 0:
            f.write('*DENSITY\n')
            f.write('%g,\n' % density)
        f.write('**\n')
        # User hint
        f.write('** Next: Import this .inp in Abaqus/CAE, apply BCs using\n')
        f.write('**       N_XMIN/XMAX/YMIN/YMAX/ZMIN/ZMAX sets, create Step\n')
        f.write('**       and Job. After job completes, run:\n')
        f.write('**         abaqus python abaqus_odb_to_mat.py --odb ... --npz ...\n')

    if verbose:
        sz = os.path.getsize(output_path) / 1024.0 / 1024.0
        print('  [inp] written: %s (%.2f MB, %.2fs)'
              % (output_path, sz, time.time() - t0))

    return {
        'n_nodes': n_nodes, 'n_elements': n_elem,
        'nelx': nx, 'nely': ny, 'nelz': nz,
        'voxel_size': voxel_size,
        'origin': np.asarray(origin),
        'elem_label_to_ijk': elem_label_to_ijk,
    }


def _write_list(f, items, per_line=8):
    items = list(items)
    for i in range(0, len(items), per_line):
        chunk = items[i:i + per_line]
        f.write(', '.join('%d' % x for x in chunk) + '\n')


def write_metadata_npz(npz_path, meta, occupancy_xyz, source_mode,
                       source_path, xPhys_full_yxz=None,
                       density_threshold=None, iter_data=None):
    """Write metadata + mode-specific extras to .npz.

    Always-present fields:
        source_mode, source_path
        nelx, nely, nelz, n_elements, n_nodes
        voxel_size, origin_xyz
        occupancy_xyz   [nelx, nely, nelz]  uint8
        elem_label_to_ijk  (n_elements, 3) int32

    Topo-only extras:
        xPhys_full_yxz   [nely, nelx, nelz]  continuous density
        density_threshold (scalar)
        iter_data (optional 1-D array)
    """
    fields = dict(
        source_mode=np.asarray(source_mode),
        source_path=np.asarray(source_path),
        nelx=np.int64(meta['nelx']),
        nely=np.int64(meta['nely']),
        nelz=np.int64(meta['nelz']),
        n_elements=np.int64(meta['n_elements']),
        n_nodes=np.int64(meta['n_nodes']),
        voxel_size=np.float64(meta['voxel_size']),
        origin_xyz=np.asarray(meta['origin'], dtype=np.float64),
        occupancy_xyz=occupancy_xyz.astype(np.uint8),
        elem_label_to_ijk=meta['elem_label_to_ijk'].astype(np.int32),
    )
    if xPhys_full_yxz is not None:
        fields['xPhys_full_yxz'] = np.asarray(xPhys_full_yxz, dtype=np.float64)
    if density_threshold is not None:
        fields['density_threshold'] = np.float64(density_threshold)
    if iter_data is not None and len(np.asarray(iter_data).flatten()) > 0:
        fields['iter_data'] = np.asarray(iter_data, dtype=np.float64).flatten()
    np.savez_compressed(npz_path, **fields)


# ============================================================
# STL voxelization
# ============================================================

def voxelize_stl_mesh(stl_path, voxel_size, rotate_axis=None,
                      padding=1, fix_mesh=True, verbose=True):
    """STL -> (occupancy_xyz, origin)."""
    import trimesh
    from scipy.ndimage import binary_fill_holes

    if verbose:
        print('[stl] loading: %s' % stl_path)
    mesh = trimesh.load(stl_path, force='mesh')
    if not isinstance(mesh, trimesh.Trimesh):
        raise RuntimeError('Failed to load mesh: %s' % stl_path)
    if verbose:
        print('  bounds before any transform: %s'
              % np.array2string(mesh.bounds, precision=3))

    R = get_rotation_matrix(rotate_axis)
    if R is not None:
        mesh.apply_transform(R)
        if verbose:
            print('  applied rotation: %s' % rotate_axis)
            print('  bounds after rotation: %s'
                  % np.array2string(mesh.bounds, precision=3))

    if fix_mesh:
        try:
            mesh.process(validate=True)
        except Exception:
            try: mesh.merge_vertices()
            except Exception: pass
        for fn in ('nondegenerate_faces', 'unique_faces'):
            try: mesh.update_faces(getattr(mesh, fn)())
            except Exception: pass
        try: mesh.remove_unreferenced_vertices()
        except Exception: pass
        if not mesh.is_watertight:
            try:
                trimesh.repair.fill_holes(mesh)
                trimesh.repair.fix_normals(mesh)
            except Exception as ex:
                if verbose: print('  [warn] repair: %s' % ex)
        if verbose:
            print('  watertight: %s' % mesh.is_watertight)

    if verbose:
        print('[stl] voxelizing at pitch=%g...' % voxel_size)
    vg = mesh.voxelized(pitch=voxel_size)
    try:
        vg = vg.fill(method='holes')
    except Exception:
        try: vg = vg.fill()
        except Exception:
            m = np.asarray(vg.matrix).copy()
            for k in range(m.shape[2]):
                m[:, :, k] = binary_fill_holes(m[:, :, k])
            vg = trimesh.voxel.VoxelGrid(encoding=m, transform=vg.transform)

    occ = np.asarray(vg.matrix, dtype=bool)
    center_origin = np.asarray(vg.transform[:3, 3], dtype=float)
    corner_origin = center_origin - 0.5 * voxel_size

    if padding > 0:
        occ = np.pad(occ, ((padding,) * 2,) * 3,
                     mode='constant', constant_values=False)
        corner_origin = corner_origin - padding * voxel_size

    if verbose:
        n_occ = int(occ.sum())
        print('  [stl] final grid %d x %d x %d, occupied %d (%.2f%%)'
              % (occ.shape[0], occ.shape[1], occ.shape[2],
                 n_occ, 100.0 * n_occ / occ.size))
    return occ, corner_origin


def cmd_from_stl(args):
    """Subcommand: from-stl."""
    if args.verbose:
        print('=' * 62)
        print('  voxelize.py from-stl')
        print('=' * 62)

    occ_xyz, origin = voxelize_stl_mesh(
        args.stl_path, args.voxel_size,
        rotate_axis=args.rotate_axis, padding=args.padding,
        fix_mesh=not args.no_fix, verbose=args.verbose)

    if args.verbose:
        print('\n[inp] writing %s...' % args.output_inp)
    meta = write_abaqus_inp(
        occ_xyz, args.voxel_size, origin, args.output_inp,
        element_type=args.element_type,
        youngs_modulus=args.youngs, poisson_ratio=args.poisson,
        density=args.density, verbose=args.verbose)

    if args.verbose:
        print('\n[npz] writing %s...' % args.output_npz)
    write_metadata_npz(args.output_npz, meta, occ_xyz,
                       source_mode='stl', source_path=args.stl_path)

    if args.verbose:
        _print_next_steps(args.output_inp, args.output_npz)


# ============================================================
# Topology optimization voxelization
# ============================================================

def load_topo_mat(path, verbose=True):
    """
    Load SIMP topo opt mat. Returns dict with xPhys_yxz, nelx/nely/nelz, Iter.

    Grid dimensions are read from the mat file's nelx/nely/nelz fields when
    available (SIMP results normally carry them). If absent, fall back to
    inferring from xPhys.shape. For 1-D xPhys the explicit fields are required.
    """
    try:
        from scipy.io import loadmat
        data = loadmat(path, squeeze_me=True)
    except (NotImplementedError, ValueError):
        # MATLAB v7.3 = HDF5
        try:
            import h5py
        except ImportError:
            raise RuntimeError(
                "Cannot read %s (likely MATLAB v7.3 HDF5). Install h5py "
                "or in MATLAB: save(..., '-v7')." % path)
        if verbose:
            print('  [topo] loading v7.3 HDF5 mat via h5py')
        data = {}
        with h5py.File(path, 'r') as f:
            for key in f.keys():
                if key.startswith('#'):
                    continue
                arr = np.array(f[key])
                if arr.ndim >= 2:
                    arr = arr.T   # undo v7.3 transposition
                data[key] = arr

    if verbose:
        print('  [topo] keys in mat: %s' % sorted(
            [k for k in data.keys() if not k.startswith('__')]))

    # Find density field
    xPhys = None
    for name in ('xPhys', 'x', 'rho', 'density'):
        if name in data:
            xPhys = np.asarray(data[name], dtype=np.float64)
            if verbose:
                print('  [topo] density field: %r' % name)
            break
    if xPhys is None:
        raise RuntimeError(
            'No density field found in %s. Expected one of: xPhys, x, rho.' % path)

    # Read explicit grid dimensions from mat (the SIMP output normally carries them)
    def _scalar(v):
        if v is None:
            return None
        arr = np.asarray(v).flatten()
        return int(arr[0]) if arr.size > 0 else None

    nelx_field = _scalar(data.get('nelx'))
    nely_field = _scalar(data.get('nely'))
    nelz_field = _scalar(data.get('nelz'))

    # Reshape if necessary, and verify consistency
    if xPhys.ndim == 1:
        if None in (nelx_field, nely_field, nelz_field):
            raise RuntimeError(
                '1D density vector but nelx/nely/nelz missing in mat. '
                'Need explicit grid dimensions.')
        xPhys = xPhys.reshape((nely_field, nelx_field, nelz_field))
        nelx, nely, nelz = nelx_field, nely_field, nelz_field
        if verbose:
            print('  [topo] reshaped 1D vector to (nely=%d, nelx=%d, nelz=%d)'
                  % (nely, nelx, nelz))
    elif xPhys.ndim == 2:
        xPhys = xPhys[:, :, None]
        nely, nelx, nelz = xPhys.shape
    elif xPhys.ndim == 3:
        nely, nelx, nelz = xPhys.shape
    else:
        raise RuntimeError('Unsupported xPhys ndim=%d' % xPhys.ndim)

    # If the mat had explicit fields, trust them but warn on mismatch
    if (nelx_field is not None and nely_field is not None
            and nelz_field is not None):
        decl_shape = (nely_field, nelx_field, nelz_field)
        if decl_shape != xPhys.shape:
            print('  [topo] WARN: declared shape %s != xPhys shape %s. '
                  'Using xPhys shape.' % (decl_shape, xPhys.shape))
        else:
            if verbose:
                print('  [topo] grid dims from mat fields: '
                      '(nely=%d, nelx=%d, nelz=%d)' % (nely, nelx, nelz))
    else:
        if verbose:
            print('  [topo] grid dims inferred from xPhys.shape: '
                  '(nely=%d, nelx=%d, nelz=%d)' % (nely, nelx, nelz))

    iter_data = None
    if 'Iter' in data:
        iter_data = np.asarray(data['Iter'], dtype=np.float64).flatten()

    if verbose:
        print('  [topo] xPhys range [%.4f, %.4f]'
              % (float(xPhys.min()), float(xPhys.max())))

    return dict(xPhys_yxz=xPhys, nelx=nelx, nely=nely, nelz=nelz,
                iter_data=iter_data)


def cmd_from_topo(args):
    """
    Subcommand: from-topo

    This is NOT geometric voxelization. The grid is already defined by the
    SIMP optimizer; xPhys carries the per-element relative density. What
    this command does is apply a *material mask*: only elements with
    xPhys > density_threshold are written as Abaqus C3D8R elements (and
    later receive BCs / loads / stress in Abaqus). The full continuous
    density field is preserved in the npz so that the downstream pipeline
    sees the original SIMP result, not a binarized version.
    """
    if args.verbose:
        print('=' * 62)
        print('  voxelize.py from-topo  (SIMP density-field mask application)')
        print('=' * 62)
        print('  input mat        : %s' % args.mat_path)
        print('  density threshold: %g  (elements with xPhys > threshold are kept)'
              % args.density_threshold)
        print('  voxel size       : %g' % args.voxel_size)
        print('')

    topo = load_topo_mat(args.mat_path, verbose=args.verbose)
    xPhys_yxz = topo['xPhys_yxz']    # [nely, nelx, nelz]
    nelx = topo['nelx']
    nely = topo['nely']
    nelz = topo['nelz']

    # Apply material mask
    threshold = float(args.density_threshold)
    mask_yxz = xPhys_yxz > threshold
    n_active = int(mask_yxz.sum())
    n_total = int(mask_yxz.size)
    if args.verbose:
        print('\n[mask] applying material mask: xPhys > %.4f' % threshold)
        print('  active elements (sent to Abaqus): %d / %d  (%.2f%%)'
              % (n_active, n_total, 100.0 * n_active / n_total))
        print('  masked-out (xPhys <= threshold) : %d / %d  (%.2f%%)'
              % (n_total - n_active, n_total,
                 100.0 * (n_total - n_active) / n_total))
    if n_active == 0:
        raise RuntimeError('No active elements after masking. Lower threshold.')

    # Convert mask to (nelx, nely, nelz) for inp writer
    occ_xyz = np.transpose(mask_yxz, (1, 0, 2)).copy()
    origin = (0.0, 0.0, 0.0)
    voxel_size = float(args.voxel_size)

    if args.verbose:
        print('\n[inp] writing %s (only active elements)...' % args.output_inp)
    meta = write_abaqus_inp(
        occ_xyz, voxel_size, origin, args.output_inp,
        element_type=args.element_type,
        youngs_modulus=args.youngs, poisson_ratio=args.poisson,
        density=args.density, verbose=args.verbose)

    if args.verbose:
        print('\n[npz] writing %s (full continuous density preserved)...'
              % args.output_npz)
    write_metadata_npz(
        args.output_npz, meta, occ_xyz,
        source_mode='topo', source_path=args.mat_path,
        xPhys_full_yxz=xPhys_yxz,
        density_threshold=threshold,
        iter_data=topo.get('iter_data'))

    if args.verbose:
        _print_next_steps(args.output_inp, args.output_npz)


# ============================================================
# Common: epilogue
# ============================================================

def _print_next_steps(inp_path, npz_path):
    print('\nDone. Next steps:')
    print('  1) Open Abaqus/CAE -> File -> Import -> Model -> %s'
          % os.path.basename(inp_path))
    print('  2) Apply BCs (use N_XMIN/XMAX/YMIN/YMAX/ZMIN/ZMAX node sets)')
    print('  3) Apply loads, create Step, submit Job')
    print('  4) abaqus python abaqus_odb_to_mat.py --odb job.odb --npz %s '
          '--output topo_stress_result.mat'
          % os.path.basename(npz_path))


# ============================================================
# CLI
# ============================================================

def _add_common_output_args(p):
    p.add_argument('-i', '--output-inp', default='voxel_grid.inp',
                   help='Output Abaqus .inp path.')
    p.add_argument('-m', '--output-npz', default='voxel_grid.npz',
                   help='Output metadata .npz path.')
    p.add_argument('-e', '--element-type', default='C3D8R',
                   choices=['C3D8', 'C3D8R', 'C3D8I'])
    p.add_argument('-E', '--youngs', type=float, default=70000.0,
                   help='Youngs modulus (MPa).')
    p.add_argument('-v', '--poisson', type=float, default=0.3)
    p.add_argument('-d', '--density', type=float, default=2.7e-9)
    p.add_argument('-q', '--quiet', dest='verbose', action='store_false',
                   default=True)


def _build_parser():
    p = argparse.ArgumentParser(
        prog='voxelize.py',
        description='Unified voxelization preprocessor for CFRC pipeline. '
                    'Two input modes: from-stl, from-topo.')
    sub = p.add_subparsers(dest='source', metavar='{from-stl,from-topo}')

    # ---- from-stl ----
    ps = sub.add_parser('from-stl',
                        help='Voxelize a STL geometry (xPhys = 0/1 mask).')
    ps.add_argument('stl_path', help='Input STL file.')
    ps.add_argument('-s', '--voxel-size', type=float, required=True,
                    help='Voxel edge length (same unit as STL).')
    ps.add_argument('-r', '--rotate-axis', default='none',
                    choices=ROTATE_AXIS_CHOICES,
                    help="Axis remap BEFORE voxelization. 'y-to-z' is the "
                         "common fix for SolidWorks Y-up STL.")
    ps.add_argument('-p', '--padding', type=int, default=1,
                    help='Empty voxel layers around bbox.')
    ps.add_argument('--no-fix', action='store_true',
                    help='Skip mesh repair.')
    _add_common_output_args(ps)
    ps.set_defaults(func=cmd_from_stl)

    # ---- from-topo ----
    pt = sub.add_parser('from-topo',
                        help='Convert SIMP topology optimization mat '
                             '(xPhys = continuous density).')
    pt.add_argument('mat_path', help='Input .mat file containing xPhys.')
    pt.add_argument('-s', '--voxel-size', type=float, default=1.0,
                    help='Voxel edge length (default 1.0).')
    pt.add_argument('-t', '--density-threshold', type=float, default=0.5,
                    help='Elements with xPhys > threshold become solid.')
    _add_common_output_args(pt)
    pt.set_defaults(func=cmd_from_topo)

    return p


def main(argv=None):
    parser = _build_parser()
    args = parser.parse_args(argv)
    if not hasattr(args, 'func'):
        parser.print_help()
        sys.exit(1)
    args.func(args)


if __name__ == '__main__':
    main()
