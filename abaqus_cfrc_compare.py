# -*- coding: utf-8 -*-
"""
Abaqus CFRC 5-Way FEA Comparison Driver (v4, orphan-mesh beams)
================================================================

v4 CHANGES vs v3:
- Beams are created via ORPHAN MESH INP IMPORT (not WirePolyLine)
- All beam elements live in a SINGLE beam part per config
  (was 888 parts per config, each with WirePolyLine geometry)
- SPRING1 elements are also written directly into the beam inp
  (no more keywordBlock injection)
- End-to-end config build time: ~30 sec (was: never completes)

Two-phase workflow for 4-way structural stiffness comparison.

Phase 1 - Template Build (one-time):
    execfile('abaqus_cfrc_compare.py')
    step1_build_template()
        -> creates host-only model with step + materials + sections
        -> saves to template.cae

    User then opens template.cae in CAE, sets BCs and Concentrated Force,
    creates Assembly Set named 'LoadPoint' on the load node, saves CAE.

Phase 2 - Batch Run:
    step2_run_batch_comparison()

Config layout on disk:
    C:/temp/cfrc_fea/
      host/
        mesh_params.txt
        valid_elements.txt
      mine_stream/      beam_paths/path_0001.txt ...
      mine_stream/      beam_paths/path_0001.txt ...
      mine_offset/      beam_paths/path_0001.txt ...
      planar_stream/    beam_paths/path_0001.txt ...
      planar_offset/    beam_paths/path_0001.txt ...
      template.cae

Abaqus 2021 / Python 2.7 compatible (no non-ASCII in source).
"""

from abaqus import *
from abaqusConstants import *
from caeModules import *
import regionToolset
import mesh
import os
import math
import sys

if sys.version_info[0] == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')


# ============================================================================
# Configuration
# ============================================================================
class Config:
    BASE_DIR       = 'C:/temp/cfrc_fea'
    HOST_SUBDIR    = 'host'
    TEMPLATE_CAE   = 'C:/temp/cfrc_fea/template.cae'
    JOBS_DIR       = 'C:/temp/cfrc_fea/jobs'
    RESULTS_DIR    = 'C:/temp/cfrc_fea/results'

    # Abaqus job 输出目录 (Job_*.dat / .inp / .odb 实际写入的位置).
    # auto_diagnose 会按顺序在这些目录里查找 dat/inp 文件.
    # cwd 默认会被搜索, 这里只填额外路径 (用 / 而不是 \).
    JOB_OUTPUT_DIRS = []

    CONFIGS = [
        ('mine_stream',   'mine_stream'),
        ('mine_offset',   'mine_offset'),
        ('planar_stream', 'planar_stream'),
        ('planar_offset', 'planar_offset'),
    ]

    ELEMENT_SIZE_X = 1.0
    ELEMENT_SIZE_Y = 1.0
    ELEMENT_SIZE_Z = 1.0

    TRANSLATE_X = 0.0
    TRANSLATE_Y = 0.0
    TRANSLATE_Z = 0.0

    # Beam-to-host 坐标对齐偏移
    #   None     -> 自动计算 (推荐, 让 beam 中心对齐 host 中心)
    #   (x,y,z)  -> 手动指定 (mm), 例: (0.5, 0.0, 0.5)
    BEAM_MANUAL_OFFSET = None

    BEAM_MAJOR_AXIS = 0.6
    BEAM_MINOR_AXIS = 0.15

    HOST_E         = 2500.0
    HOST_NU        = 0.38
    HOST_DENSITY   = 1.14e-9
    BEAM_E_RATIO   = 92.0

    # Beam element resampling:
    # If a path has many points (hundreds), the B31 mesh density will be high
    # without benefit. Resample each path to at most this many segments.
    # None = use all original segments (no resampling).
    BEAM_MAX_SEGMENTS_PER_PATH = 30   # was None/all. 30 keeps beam direction
                                      # fidelity while cutting element count.

    # Beam section orientation n1 (single source of truth).
    # 必须同时给:
    #   (a) _write_beam_inp 用来过滤共线 segment (避免 ErrElemNormal)
    #   (b) assignBeamSectionOrientation 用来设置 *Beam Section 方向
    # 这两处之前用了不同的 n1 -> 共线 segment 漏过 -> Abaqus 报零长/法线错.
    # 选一个非对称、不与任何主轴对齐、不与典型路径方向重合的方向.
    # 归一化后 ≈ (0.309, 0.619, 0.722).
    BEAM_N1_RAW = (0.3, 0.6, 0.7)
    BEAM_N1_PARALLEL_COSINE = 0.99    # |dot(t_unit, n1_unit)| > 此值 = 过滤

    # Path blacklist per config.
    # Some paths contain geometric degeneracies that trigger Abaqus internal bugs
    # in embedded-region processing (fake "zero length element" errors). Dropping
    # these paths is acceptable as they represent < 2% of total paths.
    # path_idx is 1-based (matches BEAMPATH_NNNN naming in inp).
    #
    # planar_stream blacklist: 自动从 dat 文件 ErrElemNormal/ErrElemZeroLength/
    # ErrElemBeamSecDirVect 三个 elset 反查得到 (2026-05-20).
    # 这 40 个 path (占 1.8%) 含有全部 79 个错误 element. 其中:
    #   - BEAMPATH_1307 (4/4), BEAMPATH_1310 (3/3): 整条 path 全错
    #   - BEAMPATH_0634 (10/30), 0907 (6/30), 0909 (5/30): 含病变 cluster
    #   - 其余 path 各只有 1-2 个 error element
    # 不是简单的零长 / n1 共线问题, 是 v6 streamline 在工件内特定区域生成的
    # 局部几何 noise (Abaqus B31 preprocessor 内部容差不接受). 黑名单是最实用解.
    # planar_stream blacklist (v14 corrected): 
    # 上一版 (v13) 我把 v12 dat 里的 BEAMPATH_NNNN 当作了原始 path 索引,
    # 但 v12 inp 已经因为旧 blacklist [10,11,77,79,...] 做了索引偏移,
    # BEAMPATH_NNNN 实际是 "过滤后第 NNNN 个 path", 不是原始 NNNN.
    # 这一版做了两件事:
    #   (a) 修了 _write_beam_inp, BEAMPATH_NNNN 从此以后用原始 path index
    #   (b) 把 v12 BEAMPATH list 反向映射回原始 path index (shift +2 ~ +10)
    # 同时保留旧 10-entry blacklist 作为防御 (即使现在不需要也没坏处).
    BLACKLIST_PATH_IDX = {
        'mine_stream': [],
        'mine_offset': [],
        'planar_stream': [
            # 旧 (历史遗留, 防御性保留)
            10, 11, 77, 79, 136, 139, 146, 150, 175, 181,
            # 新增 40 个 (从 v12 BEAMPATH 反向映射到原始 path index)
            23, 24, 52, 96, 193, 198, 271, 370, 372, 488,
            532, 542, 547, 590, 644, 647, 694, 695, 731, 738,
            865, 868, 917, 919, 921, 923, 1074, 1125, 1284, 1313,
            1314, 1317, 1319, 1320, 1323, 1410, 1433, 1437, 1447, 1493,
        ],
        'planar_offset': [57],
    }

    SPRING_ROT_STIFFNESS  = 1e-3

    STEP_NAME         = 'LoadStep'
    STEP_TIME_PERIOD  = 1.0
    STEP_INITIAL_INC  = 0.05
    STEP_MAX_INC      = 0.05
    STEP_MIN_INC      = 1e-12
    STEP_MAX_NUM_INC  = 1000
    STEP_USE_NLGEOM   = OFF

    LOAD_POINT_SET_NAME = 'LoadPoint'

    HOST_PART_NAME       = 'HostPart'
    BEAM_PART_NAME       = 'AllBeams'     # single part for all beams
    TEMPLATE_MODEL_NAME  = 'Template'
    MODEL_PREFIX         = 'Cfg_'

    NUM_CPUS = 4


# ============================================================================
# Data reading
# ============================================================================
def read_mesh_params(data_dir):
    params = {}
    fp = os.path.join(data_dir, 'mesh_params.txt')
    with open(fp, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 2:
                key = parts[0]
                try:
                    val = int(parts[1])
                except ValueError:
                    val = float(parts[1])
                params[key] = val
    return params


def read_valid_elements(data_dir):
    elems = []
    fp = os.path.join(data_dir, 'valid_elements.txt')
    with open(fp, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            if len(parts) >= 4:
                ix = int(parts[0]); iy = int(parts[1]); iz = int(parts[2])
                density = float(parts[3])
                if len(parts) >= 7:
                    xc = float(parts[4]); yc = float(parts[5]); zc = float(parts[6])
                else:
                    xc = (ix + 0.5) * Config.ELEMENT_SIZE_X
                    yc = (iy + 0.5) * Config.ELEMENT_SIZE_Y
                    zc = (iz + 0.5) * Config.ELEMENT_SIZE_Z
                elems.append((ix, iy, iz, density, xc, yc, zc))
    return elems


def read_beam_paths(data_dir):
    paths = []
    paths_dir = os.path.join(data_dir, 'beam_paths')
    if not os.path.isdir(paths_dir):
        print 'WARNING: beam_paths dir not found: %s' % paths_dir
        return paths
    names = [n for n in os.listdir(paths_dir)
             if n.startswith('path_') and n.endswith('.txt')]
    names.sort()
    for name in names:
        pts = []
        with open(os.path.join(paths_dir, name), 'r') as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith('#'):
                    continue
                parts = line.split()
                if len(parts) >= 3:
                    pts.append((float(parts[0]), float(parts[1]), float(parts[2])))
        if len(pts) >= 2:
            paths.append(pts)
    return paths


def resample_path(pts, max_segments):
    """按 ** 弧长 ** 均匀降采样, 不再按索引采样.
    
    旧的按索引采样在 S 形 / 折返路径上会跨过转折点, 产生数十毫米的
    跨越式 segment, 直接导致 Abaqus B31 单元 ErrElemNormal /
    ErrElemBeamSecDirVect 等错误.
    
    新算法:
      1. 去掉相邻近重合点 (< 0.01 mm)
      2. 计算累积弧长
      3. 在 [0, total_length] 上均分 max_segments+1 个站点
      4. 对每个站点在路径上线性内插得到坐标
      5. 再去重一次防止内插出来的点过近
    """
    MIN_SEG_LEN_SQ = 1e-4  # (0.01 mm)^2
    def dedup(seq):
        if len(seq) < 2:
            return list(seq)
        out = [seq[0]]
        for p in seq[1:]:
            last = out[-1]
            dx = p[0] - last[0]; dy = p[1] - last[1]; dz = p[2] - last[2]
            if dx*dx + dy*dy + dz*dz > MIN_SEG_LEN_SQ:
                out.append(p)
        return out

    pts = dedup(pts)
    if len(pts) < 2:
        return pts

    if max_segments is None or len(pts) <= max_segments + 1:
        return pts

    n = len(pts)
    # 累积弧长
    cum_len = [0.0]
    for i in range(1, n):
        p1 = pts[i-1]; p2 = pts[i]
        dx = p2[0]-p1[0]; dy = p2[1]-p1[1]; dz = p2[2]-p1[2]
        cum_len.append(cum_len[-1] + (dx*dx + dy*dy + dz*dz) ** 0.5)
    total_len = cum_len[-1]
    if total_len < 1e-6:
        return [pts[0], pts[-1]]

    target = max_segments + 1   # 站点数 = 段数 + 1
    out = [pts[0]]
    j = 1
    for i in range(1, target - 1):
        t = total_len * i / float(target - 1)
        # 找包含弧长 t 的 segment
        while j < n - 1 and cum_len[j] < t:
            j += 1
        # 在 (pts[j-1], pts[j]) 之间线性内插
        seg_len = cum_len[j] - cum_len[j-1]
        if seg_len < 1e-9:
            out.append(pts[j-1])
        else:
            alpha = (t - cum_len[j-1]) / seg_len
            p1 = pts[j-1]; p2 = pts[j]
            out.append((p1[0] + alpha*(p2[0]-p1[0]),
                        p1[1] + alpha*(p2[1]-p1[1]),
                        p1[2] + alpha*(p2[2]-p1[2])))
    out.append(pts[-1])
    return dedup(out)


# ============================================================================
# Host mesh (orphan mesh via inp file)
# ============================================================================
def create_host_from_voxels(model, params, elements, part_name=None):
    if part_name is None:
        part_name = Config.HOST_PART_NAME

    print '\n--- Creating host orphan mesh ---'

    nelx = params.get('nelx', 20)
    nely = params.get('nely', 40)
    nelz = params.get('nelz', 20)

    dx = Config.ELEMENT_SIZE_X
    dy = Config.ELEMENT_SIZE_Y
    dz = Config.ELEMENT_SIZE_Z

    print 'Grid: %d x %d x %d, valid elements: %d' % (
        nelx, nely, nelz, len(elements))

    node_positions = set()
    for e in elements:
        ix, iy, iz = e[0], e[1], e[2]
        for di in (0, 1):
            for dj in (0, 1):
                for dk in (0, 1):
                    node_positions.add((ix + di, iy + dj, iz + dk))

    node_positions = sorted(list(node_positions))
    node_map = {}
    node_coords = []
    for idx, (i, j, k) in enumerate(node_positions):
        node_id = idx + 1
        node_map[(i, j, k)] = node_id
        node_coords.append((i * dx, j * dy, k * dz))

    conn_list = []
    for e in elements:
        ix, iy, iz = e[0], e[1], e[2]
        n1 = node_map[(ix,     iy,     iz    )]
        n2 = node_map[(ix + 1, iy,     iz    )]
        n3 = node_map[(ix + 1, iy + 1, iz    )]
        n4 = node_map[(ix,     iy + 1, iz    )]
        n5 = node_map[(ix,     iy,     iz + 1)]
        n6 = node_map[(ix + 1, iy,     iz + 1)]
        n7 = node_map[(ix + 1, iy + 1, iz + 1)]
        n8 = node_map[(ix,     iy + 1, iz + 1)]
        conn_list.append((n1, n2, n3, n4, n5, n6, n7, n8))

    inp_path = os.path.join(Config.BASE_DIR, 'temp_host_mesh.inp')
    with open(inp_path, 'w') as f:
        f.write('*Heading\n** Host orphan mesh\n')
        f.write('*Node\n')
        for nid, (x, y, z) in enumerate(node_coords, start=1):
            f.write('%d, %.6f, %.6f, %.6f\n' % (nid, x, y, z))
        f.write('*Element, type=C3D8, elset=AllElements\n')
        for eid, c in enumerate(conn_list, start=1):
            f.write('%d, %d, %d, %d, %d, %d, %d, %d, %d\n' %
                    (eid, c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7]))
        f.write('*Nset, nset=AllNodes, generate\n')
        f.write('1, %d, 1\n' % len(node_coords))

    tmp_model_name = '_tmp_host_import'
    if tmp_model_name in mdb.models.keys():
        del mdb.models[tmp_model_name]
    mdb.ModelFromInputFile(name=tmp_model_name, inputFileName=inp_path)
    tmp_model = mdb.models[tmp_model_name]
    tmp_part_name = tmp_model.parts.keys()[0]
    tmp_part = tmp_model.parts[tmp_part_name]

    if part_name in model.parts.keys():
        del model.parts[part_name]
    part = model.Part(name=part_name, objectToCopy=tmp_part)

    del mdb.models[tmp_model_name]
    try: os.remove(inp_path)
    except: pass

    print 'Host mesh imported: %d nodes, %d elements' % (
        len(part.nodes), len(part.elements))
    xs = [c[0] for c in node_coords]
    ys = [c[1] for c in node_coords]
    zs = [c[2] for c in node_coords]
    print 'Bounding box: X[%.2f, %.2f] Y[%.2f, %.2f] Z[%.2f, %.2f]' % (
        min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))

    return part


def setup_host_material(model):
    name = 'HostMaterial'
    if name in model.materials.keys():
        del model.materials[name]
    mat = model.Material(name=name)
    mat.Elastic(table=((Config.HOST_E, Config.HOST_NU),))
    mat.Density(table=((Config.HOST_DENSITY,),))
    print 'Host material: E=%.1f MPa, nu=%.3f' % (Config.HOST_E, Config.HOST_NU)
    return name


def setup_host_section(model, host_material_name):
    name = 'HostSolidSection'
    if name in model.sections.keys():
        del model.sections[name]
    model.HomogeneousSolidSection(
        name=name, material=host_material_name, thickness=None)
    return name


def assign_host_section(model, host_section_name):
    part = model.parts[Config.HOST_PART_NAME]
    all_elems = part.elements
    region = part.Set(elements=all_elems, name='AllElements')
    part.SectionAssignment(
        region=region, sectionName=host_section_name, offset=0.0,
        offsetType=MIDDLE_SURFACE, offsetField='',
        thicknessAssignment=FROM_SECTION)
    print 'Host section assigned: %d elements' % len(all_elems)


# ============================================================================
# Beam: single orphan-mesh part containing all beams + springs
# ============================================================================
def _write_beam_inp(inp_path, beam_paths_resampled, orig_path_indices=None):
    """Write an inp file with:
      - All beam nodes (deduplicated not needed -- unique per beam)
      - B31 elements per beam
      - One elset per beam (BeamPath_0001, ...) NAMED BY ORIGINAL PATH INDEX
      - SPRING1 elements on each beam's two endpoints (DOF 4, 5, 6)
      - End-of-beam nset for quick visualization

    orig_path_indices: optional list of 1-based original path indices,
      length == len(beam_paths_resampled). If provided, BEAMPATH_NNNN
      naming uses these instead of sequential filtered indices.
      This is critical for blacklist consistency across runs: BEAMPATH_0023
      always refers to the same original path data regardless of which
      paths are dropped by blacklist.

    Returns dict:
      'n_nodes', 'n_b31', 'n_springs', 'n_paths'
    """
    # Default: sequential 1-based numbering = old behavior
    if orig_path_indices is None:
        orig_path_indices = list(range(1, len(beam_paths_resampled) + 1))
    assert len(orig_path_indices) == len(beam_paths_resampled), \
        'orig_path_indices length must match beam_paths_resampled'

    node_lines = []
    elem_b31_lines = []         # B31 elements
    elset_per_path_lines = []   # separate *Elset blocks per path
    spring_elements = []        # SPRING1 rows (eid, nid)
    spring_elsets = { 4: [], 5: [], 6: [] }  # dof -> list of eids

    # Pass 1: collect all nodes/elements per path
    paths_data = []   # list of (orig_path_idx, pts, path_node_ids_tmp, path_elems, used_local)
    next_node_id = 1
    next_elem_id = 1

    n_paths = 0
    n_zero_seg_skipped = 0
    for path_idx, pts in enumerate(beam_paths_resampled):
        orig_idx = orig_path_indices[path_idx]   # 1-based ORIGINAL path index
        if len(pts) < 2:
            continue

        # Allocate node ids (tentatively)
        path_node_ids_tmp = []
        for _ in pts:
            path_node_ids_tmp.append(next_node_id)
            next_node_id += 1

        # Build element list, skipping zero-length, short, and n1-degenerate
        # Abaqus ModelFromInputFile 会自动合并 <~0.01mm 的节点; 此外,
        # 如果 beam 切线向量与 Config.BEAM_N1 近似平行, Abaqus 会报
        # "beam direction vectors coincide" -> 伪装的 "zero length" 错误. 过滤这些.
        path_elems = []  # list of (n1_local_idx, n2_local_idx)
        MIN_SEG_LEN_SQ = 0.01  # (0.1 mm)^2 = 0.01 mm^2
        # n1 方向 (与 assignBeamSectionOrientation 必须一致, 用 Config 单一真实源)
        import math as _m
        _nx0, _ny0, _nz0 = Config.BEAM_N1_RAW
        _norm0 = _m.sqrt(_nx0*_nx0 + _ny0*_ny0 + _nz0*_nz0)
        N1_VEC = (_nx0/_norm0, _ny0/_norm0, _nz0/_norm0)
        # 切线和 n1 的夹角余弦绝对值 > MAX_COSINE 则剔除 (1.0 = parallel)
        MAX_PARALLEL_COSINE = getattr(Config, 'BEAM_N1_PARALLEL_COSINE', 0.99)
        last_kept_idx = 0
        for i in range(1, len(pts)):
            p1 = pts[last_kept_idx]; p2 = pts[i]
            dx = p2[0] - p1[0]; dy = p2[1] - p1[1]; dz = p2[2] - p1[2]
            seg_len_sq = dx*dx + dy*dy + dz*dz
            if seg_len_sq < MIN_SEG_LEN_SQ:
                n_zero_seg_skipped += 1
                continue
            # 检查切线和 n1 的平行度
            L = seg_len_sq ** 0.5
            tx = dx / L; ty = dy / L; tz = dz / L
            cosine = abs(tx * N1_VEC[0] + ty * N1_VEC[1] + tz * N1_VEC[2])
            if cosine > MAX_PARALLEL_COSINE:
                n_zero_seg_skipped += 1  # 复用计数器
                continue
            path_elems.append((last_kept_idx, i))
            last_kept_idx = i

        if not path_elems:
            # whole path dropped (all segments zero length)
            next_node_id -= len(pts)  # reclaim ids
            continue

        # Collect used local node indices
        used_local = set()
        for (a, b) in path_elems:
            used_local.add(a); used_local.add(b)

        n_paths += 1
        # paths_data tuple: (orig_idx_1based, pts, tmp_ids, elems, used)
        paths_data.append((orig_idx, pts, path_node_ids_tmp, path_elems, used_local))

    # Pass 2: 节点编号
    # 关键策略: path 之间的节点完全独立 (各用自己的 id), 并对每条 path 的坐标施加
    # 微小扰动 (μm 级), 让不同 path 在物理上也不重合. 这样 Abaqus ModelFromInputFile
    # 的自动节点合并就没有可合并的目标, 每条 path 的拓扑保持完整.
    # 扰动量级 1e-5 mm (10 nm), 远小于 beam 截面 0.6mm, 对任何工程分析无影响.
    node_id_map = {}
    final_node_id = 1
    PERTURB_SCALE = 1e-5  # 10 nm per path index

    for pdi, (orig_idx, pts, tmp_ids, elems, used) in enumerate(paths_data):
        # 每条 path 独立扰动 (prime 乘积让扰动各向异性, 避免偶然共线)
        # pdi=0 无扰动, pdi=1 在 x+13nm y+17nm z+19nm 依次类推
        perturb_x = PERTURB_SCALE * pdi * 1.3
        perturb_y = PERTURB_SCALE * pdi * 1.7
        perturb_z = PERTURB_SCALE * pdi * 1.9
        for li in sorted(used):
            (x, y, z) = pts[li]
            # 施加扰动
            x2 = x + perturb_x
            y2 = y + perturb_y
            z2 = z + perturb_z
            nid = final_node_id
            final_node_id += 1
            node_id_map[(pdi, li)] = nid
            node_lines.append('%d, %.6f, %.6f, %.6f' % (nid, x2, y2, z2))

    total_nodes = final_node_id - 1
    print '    [Pass 2] independent nodes per path, +micro-perturbation to avoid merging'

    # Pass 3: write elements with compact node ids, build elsets, collect spring endpoints
    final_elem_id = 1
    spring_start_id = 900001
    next_spring_id = spring_start_id
    n_merged_zero = 0

    for pdi, (orig_idx, pts, tmp_ids, elems, used) in enumerate(paths_data):
        path_elem_ids = []
        for (a, b) in elems:
            n1 = node_id_map[(pdi, a)]
            n2 = node_id_map[(pdi, b)]
            if n1 == n2:
                # Pass 2 合并后两端变同一节点, 跳过 (避免零长度 element)
                n_merged_zero += 1
                continue
            elem_b31_lines.append('%d, %d, %d' % (final_elem_id, n1, n2))
            path_elem_ids.append(final_elem_id)
            final_elem_id += 1

        # Elset for this path -- 用原始 path index (1-based), 不是过滤后的位置.
        # 这样 BEAMPATH_NNNN 跨 blacklist 配置都指向同一条物理路径,
        # 极大便于错误诊断和 blacklist 迭代.
        elset_name = 'BeamPath_%04d' % orig_idx
        if path_elem_ids:
            chunks = []
            cur = []
            for eid in path_elem_ids:
                cur.append(str(eid))
                if len(cur) >= 10:
                    chunks.append(', '.join(cur))
                    cur = []
            if cur:
                chunks.append(', '.join(cur))
            elset_per_path_lines.append('*Elset, elset=%s' % elset_name)
            elset_per_path_lines.extend(chunks)

        # SPRING1 at path endpoints (first used local idx, last used local idx)
        sorted_used = sorted(used)
        first_local = sorted_used[0]
        last_local = sorted_used[-1]
        endpoint_nids = [node_id_map[(pdi, first_local)],
                         node_id_map[(pdi, last_local)]]
        for dof in (4, 5, 6):
            for nid in endpoint_nids:
                spring_eid = next_spring_id
                next_spring_id += 1
                spring_elements.append((dof, spring_eid, nid))
                spring_elsets[dof].append(spring_eid)

    # Build inp text
    out = []
    out.append('*Heading')
    out.append('** Beam orphan mesh (all paths in one part)')
    out.append('*Part, name=%s' % Config.BEAM_PART_NAME)
    out.append('*Node')
    out.extend(node_lines)
    out.append('*Element, type=B31, elset=AllBeamElements')
    out.extend(elem_b31_lines)
    out.extend(elset_per_path_lines)

    # Nset all beam nodes (for mass/section verification)
    out.append('*Nset, nset=AllBeamNodes, generate')
    out.append('1, %d, 1' % total_nodes)

    # SPRING1 elements: grouped by DOF
    for dof in (4, 5, 6):
        if not spring_elsets[dof]:
            continue
        elset_name = 'RotSprings_DOF%d' % dof
        out.append('*Element, type=SPRING1, elset=%s' % elset_name)
        for (d2, eid, nid) in spring_elements:
            if d2 == dof:
                out.append('%d, %d' % (eid, nid))
        out.append('*Spring, elset=%s' % elset_name)
        out.append('%d' % dof)
        out.append('%.6e' % Config.SPRING_ROT_STIFFNESS)

    out.append('*End Part')

    with open(inp_path, 'w') as f:
        f.write('\n'.join(out))
        f.write('\n')

    return {
        'n_nodes': total_nodes,
        'n_b31': len(elem_b31_lines),
        'n_springs': len(spring_elements),
        'n_paths': n_paths,
        'n_zero_seg_skipped': n_zero_seg_skipped,
    }


def setup_beam_material_profile_section(model):
    """Material + elliptical profile + beam section (shared by all paths)."""
    bm_name = 'BeamMaterial'
    if bm_name in model.materials.keys():
        del model.materials[bm_name]
    E_beam = Config.HOST_E * Config.BEAM_E_RATIO
    mat = model.Material(name=bm_name)
    mat.Elastic(table=((E_beam, Config.HOST_NU),))
    mat.Density(table=((Config.HOST_DENSITY,),))

    a = Config.BEAM_MAJOR_AXIS / 2.0
    b = Config.BEAM_MINOR_AXIS / 2.0
    area = math.pi * a * b
    i11 = math.pi * a * (b ** 3) / 4.0
    i22 = math.pi * b * (a ** 3) / 4.0
    j = math.pi * (a ** 3) * (b ** 3) / (a ** 2 + b ** 2)

    prof_name = 'EllipticalProfile'
    if prof_name in model.profiles.keys():
        del model.profiles[prof_name]
    model.GeneralizedProfile(name=prof_name, area=area,
                             i11=i11, i12=0.0, i22=i22, j=j,
                             gammaO=0.0, gammaW=0.0)

    sec_name = 'BeamSection'
    if sec_name in model.sections.keys():
        del model.sections[sec_name]
    G = E_beam / (2.0 * (1.0 + Config.HOST_NU))
    model.BeamSection(name=sec_name, integration=BEFORE_ANALYSIS,
                      profile=prof_name, poissonRatio=Config.HOST_NU,
                      thermalExpansion=OFF, temperatureVar=LINEAR,
                      consistentMassMatrix=False,
                      density=Config.HOST_DENSITY,
                      table=((E_beam, G),))

    print 'Beam material: E=%.1f MPa (%.1fx host)' % (E_beam, Config.BEAM_E_RATIO)
    print 'Beam section: A=%.4e mm^2, I11=%.4e mm^4' % (area, i11)
    return sec_name


def _filter_extreme_outliers(beam_paths, host_bbox, max_excess=2.0):
    """
    过滤掉偏离 host bbox 超过 max_excess 毫米的 beam 点。
    host_bbox: (xmin, xmax, ymin, ymax, zmin, zmax)
    对每条 path，只保留连续在 tol 内的子段。
    """
    xmin, xmax, ymin, ymax, zmin, zmax = host_bbox
    out_paths = []
    n_removed_pts = 0
    n_orig_pts = 0
    for pts in beam_paths:
        n_orig_pts += len(pts)
        good = []
        for (x, y, z) in pts:
            inside = (xmin - max_excess <= x <= xmax + max_excess and
                      ymin - max_excess <= y <= ymax + max_excess and
                      zmin - max_excess <= z <= zmax + max_excess)
            if inside:
                good.append((x, y, z))
            else:
                n_removed_pts += 1
        if len(good) >= 2:
            out_paths.append(good)
    return out_paths, n_removed_pts, n_orig_pts


def create_beams_from_inp(model, beam_paths, cfg_name):
    """Resample paths, write inp, import as single beam part,
    create material + section, assign section + orientation."""
    if not beam_paths:
        return None

    # 跟踪每条 path 的 ORIGINAL 1-based index. 这样无论后面 blacklist /
    # strict filter / resample 怎么删, BEAMPATH_NNNN 在 inp 里始终对应
    # 原始 path 数据, blacklist 迭代时不会因为编号偏移而失效.
    orig_indices = list(range(1, len(beam_paths) + 1))

    # --- [NEW] 应用 per-config path blacklist ---
    blacklist = Config.BLACKLIST_PATH_IDX.get(cfg_name, [])
    if blacklist:
        blacklist_set = set(blacklist)  # 1-based
        n_before = len(beam_paths)
        # 同步过滤 beam_paths 和 orig_indices
        kept = [(oi, p) for oi, p in zip(orig_indices, beam_paths) if oi not in blacklist_set]
        orig_indices = [t[0] for t in kept]
        beam_paths = [t[1] for t in kept]
        print '[Blacklist] Dropped %d paths for %s: %s' % (
            n_before - len(beam_paths), cfg_name, blacklist)

    # --- [NEW] Beam 坐标偏移（自动对齐 host 的半格偏移）---
    # v2 脚本里手动 TRANSLATE_Y=0.5，这里用"自动模式": 根据 host 和 beam 的 bbox
    # 计算让 beam 中心对齐 host 中心所需的偏移，然后应用到所有 beam 点。
    host_part = None
    if Config.HOST_PART_NAME in model.parts.keys():
        host_part = model.parts[Config.HOST_PART_NAME]

    auto_offset = (0.0, 0.0, 0.0)
    if host_part is not None and len(host_part.nodes) > 0 and beam_paths:
        hn = host_part.nodes
        hxs = [n.coordinates[0] for n in hn]
        hys = [n.coordinates[1] for n in hn]
        hzs = [n.coordinates[2] for n in hn]
        h_cx = 0.5 * (min(hxs) + max(hxs))
        h_cy = 0.5 * (min(hys) + max(hys))
        h_cz = 0.5 * (min(hzs) + max(hzs))

        all_bx = []
        all_by = []
        all_bz = []
        for pts in beam_paths:
            for (x, y, z) in pts:
                all_bx.append(x); all_by.append(y); all_bz.append(z)
        b_cx = 0.5 * (min(all_bx) + max(all_bx))
        b_cy = 0.5 * (min(all_by) + max(all_by))
        b_cz = 0.5 * (min(all_bz) + max(all_bz))

        # 自动偏移: 让 beam 中心对齐 host 中心
        auto_offset = (h_cx - b_cx, h_cy - b_cy, h_cz - b_cz)
        print '[Beam offset] Auto-computed shift (beam -> host frame):'
        print '  dx=%.3f, dy=%.3f, dz=%.3f' % auto_offset
        print '  Host center: (%.3f, %.3f, %.3f)' % (h_cx, h_cy, h_cz)
        print '  Beam center: (%.3f, %.3f, %.3f)' % (b_cx, b_cy, b_cz)

    # 允许用户在 Config 里强制指定偏移（覆盖自动值）
    manual_offset = getattr(Config, 'BEAM_MANUAL_OFFSET', None)
    if manual_offset is not None:
        auto_offset = manual_offset
        print '[Beam offset] Using manual offset from Config: %s' % (manual_offset,)

    # 应用偏移到所有 beam 点
    ox, oy, oz = auto_offset
    if abs(ox) > 1e-6 or abs(oy) > 1e-6 or abs(oz) > 1e-6:
        beam_paths = [[(x + ox, y + oy, z + oz) for (x, y, z) in pts]
                      for pts in beam_paths]
        print '  Applied offset to %d paths.' % len(beam_paths)

    # --- [NEW] 根据 host 的实际范围过滤极端 outlier ---
    if host_part is not None and len(host_part.nodes) > 0:
        hn = host_part.nodes
        host_coords = [(n.coordinates[0], n.coordinates[1], n.coordinates[2]) for n in hn]
        xs = [c[0] for c in host_coords]
        ys = [c[1] for c in host_coords]
        zs = [c[2] for c in host_coords]
        bbox = (min(xs), max(xs), min(ys), max(ys), min(zs), max(zs))
        beam_paths_clean, n_rm, n_tot = _filter_extreme_outliers(
            beam_paths, bbox, max_excess=2.0)
        if n_rm > 0:
            print '[Filter] Removed %d/%d extreme outlier points (>2mm beyond host bbox)' % (
                n_rm, n_tot)
        beam_paths = beam_paths_clean

        # [STRICT FILTER] 基于有效 voxel 的精确检查
        # element 的任一端点若不在任何 valid voxel 的 0.5mm 邻域内, 剔除整条 path
        # (这些点是 host 非凸凹陷边界, 嵌入时可能触发 Abaqus 的投影 bug)
        try:
            host_elem_coords = set()
            for e in host_part.elements:
                conn = e.connectivity
                if len(conn) < 8: continue
                ec = [host_part.nodes[nid].coordinates for nid in conn]
                cx = sum([c[0] for c in ec]) / 8.0
                cy = sum([c[1] for c in ec]) / 8.0
                cz = sum([c[2] for c in ec]) / 8.0
                # 以 voxel 中心 index 存入 set
                host_elem_coords.add((int(round(cx)), int(round(cy)), int(round(cz))))
            print '  Host valid voxels (indexed): %d' % len(host_elem_coords)
            
            # 对每条 path, 检查所有点是否在 host 内
            beam_paths_strict = []
            orig_indices_strict = []
            n_path_dropped = 0
            n_point_dropped = 0
            for orig_idx, pts in zip(orig_indices, beam_paths):
                good_pts = []
                for (x, y, z) in pts:
                    # 找点 p 所在 voxel 是否有效
                    ix, iy, iz = int(round(x)), int(round(y)), int(round(z))
                    # 检查 3x3x3 邻域里是否至少一个是 valid voxel
                    found = False
                    for di in range(-1, 2):
                        for dj in range(-1, 2):
                            for dk in range(-1, 2):
                                if (ix+di, iy+dj, iz+dk) in host_elem_coords:
                                    found = True; break
                            if found: break
                        if found: break
                    if found:
                        good_pts.append((x, y, z))
                    else:
                        n_point_dropped += 1
                if len(good_pts) >= 2:
                    beam_paths_strict.append(good_pts)
                    orig_indices_strict.append(orig_idx)   # 同步保留原始 index
                else:
                    n_path_dropped += 1
            print '  [Strict filter] dropped %d points, %d whole paths (out of host valid voxels + 1 ring)' % (
                n_point_dropped, n_path_dropped)
            beam_paths = beam_paths_strict
            orig_indices = orig_indices_strict
        except Exception, e:
            print '  [WARN] strict filter failed: %s' % str(e)

    max_seg = Config.BEAM_MAX_SEGMENTS_PER_PATH
    beam_paths_rs = [resample_path(p, max_seg) for p in beam_paths]
    total_orig_pts = sum([len(p) for p in beam_paths])
    total_rs_pts = sum([len(p) for p in beam_paths_rs])

    print '\n--- Creating beam orphan mesh ---'
    print 'Paths: %d, original points: %d, resampled points: %d (max %s per path)' % (
        len(beam_paths), total_orig_pts, total_rs_pts,
        'N/A' if max_seg is None else str(max_seg + 1))

    inp_path = os.path.join(Config.BASE_DIR, 'temp_beam_%s.inp' % cfg_name)
    stats = _write_beam_inp(inp_path, beam_paths_rs, orig_path_indices=orig_indices)
    print 'Wrote inp: %d nodes, %d B31, %d SPRING1 (%d valid paths)' % (
        stats['n_nodes'], stats['n_b31'], stats['n_springs'], stats['n_paths'])
    if stats['n_zero_seg_skipped'] > 0:
        print '  [warn] Skipped %d zero-length segments' % stats['n_zero_seg_skipped']

    tmp_model_name = '_tmp_beam_import_' + cfg_name
    if tmp_model_name in mdb.models.keys():
        del mdb.models[tmp_model_name]
    mdb.ModelFromInputFile(name=tmp_model_name, inputFileName=inp_path)
    tmp_model = mdb.models[tmp_model_name]
    tmp_part_name = tmp_model.parts.keys()[0]
    tmp_part = tmp_model.parts[tmp_part_name]

    # [DIAGNOSTIC] 导入后立即检查 element 长度分布
    print '\n[DIAG] Post-import element length check:'
    tmp_nodes = {n.label: n.coordinates for n in tmp_part.nodes}
    tmp_elems = tmp_part.elements
    zero_eids = []
    short_eids = []
    lens = []
    for e in tmp_elems:
        conn = e.connectivity
        if len(conn) != 2:
            continue  # not a 2-node element (SPRING1 or other)
        try:
            c1 = tmp_part.nodes[conn[0]].coordinates
            c2 = tmp_part.nodes[conn[1]].coordinates
        except:
            continue
        dx = c2[0]-c1[0]; dy = c2[1]-c1[1]; dz = c2[2]-c1[2]
        L2 = dx*dx + dy*dy + dz*dz
        if L2 < 1e-8:
            zero_eids.append(e.label)
        elif L2 < 1e-2:
            short_eids.append(e.label)
        lens.append(L2 ** 0.5)
    if lens:
        lens.sort()
        print '  %d 2-node elements, min L=%.6f  median=%.6f  max=%.6f' % (
            len(lens), lens[0], lens[len(lens)//2], lens[-1])
    print '  Zero-length (<1e-4 mm): %d' % len(zero_eids)
    print '  Short (<0.1 mm):        %d' % len(short_eids)
    if zero_eids[:5]:
        print '  First zero eids: %s' % zero_eids[:5]

    if Config.BEAM_PART_NAME in model.parts.keys():
        del model.parts[Config.BEAM_PART_NAME]
    beam_part = model.Part(name=Config.BEAM_PART_NAME, objectToCopy=tmp_part)

    del mdb.models[tmp_model_name]
    # [DEBUG] 暂时不删 inp, 方便诊断零长度 element 问题
    # try: os.remove(inp_path)
    # except: pass
    print '  [DEBUG] beam inp kept at: %s' % inp_path

    print 'Beam part imported: %d nodes, %d elements (B31+SPRING1)' % (
        len(beam_part.nodes), len(beam_part.elements))

    # Material/profile/section
    sec_name = setup_beam_material_profile_section(model)

    # Assign beam section to all B31 elements
    # B31 elements are in elset 'AllBeamElements'
    if 'AllBeamElements' in beam_part.sets.keys():
        b31_region = beam_part.sets['AllBeamElements']
    else:
        # Fallback: collect B31s by filtering
        all_b31 = [e for e in beam_part.elements if e.type == B31]
        b31_region = beam_part.Set(elements=mesh.MeshElementArray(all_b31),
                                    name='AllBeamElements')

    beam_part.SectionAssignment(
        region=b31_region, sectionName=sec_name, offset=0.0,
        offsetType=MIDDLE_SURFACE, offsetField='',
        thicknessAssignment=FROM_SECTION)

    # Beam orientation: 必须显式指定, 否则 Abaqus 默认 n1=(0,0,-1) 会和沿 Z 轴
    # 的 beam 共线 -> "direction vectors coincide" 错误.
    # 用 Config.BEAM_N1_RAW (单一真实源), 与 _write_beam_inp 过滤用的 n1 完全一致.
    import math as _m
    _nx, _ny, _nz = Config.BEAM_N1_RAW
    _norm = _m.sqrt(_nx*_nx + _ny*_ny + _nz*_nz)
    n1_vec = (_nx/_norm, _ny/_norm, _nz/_norm)
    try:
        beam_part.assignBeamSectionOrientation(
            region=b31_region, method=N1_COSINES, n1=n1_vec)
        print '  [beam orientation] n1 = (%.4f, %.4f, %.4f)' % n1_vec
    except Exception, e:
        print '  [WARN] beam orientation failed: %s' % str(e)

    return beam_part


# ============================================================================
# Assembly + embedded
# ============================================================================
def instance_host_in_assembly(model):
    assembly = model.rootAssembly
    assembly.DatumCsysByDefault(CARTESIAN)
    iname = Config.HOST_PART_NAME + '-1'
    if Config.HOST_PART_NAME in model.parts.keys():
        host_part = model.parts[Config.HOST_PART_NAME]
        if iname not in assembly.instances.keys():
            assembly.Instance(name=iname, part=host_part, dependent=ON)
    if (Config.TRANSLATE_X != 0 or
        Config.TRANSLATE_Y != 0 or
        Config.TRANSLATE_Z != 0):
        assembly.translate(
            instanceList=(iname,),
            vector=(Config.TRANSLATE_X, Config.TRANSLATE_Y, Config.TRANSLATE_Z))
    return iname


def instance_beams_and_embed(model):
    """Instance beam part (single), create ONE EmbeddedRegion for all B31 elements."""
    assembly = model.rootAssembly
    host_iname = Config.HOST_PART_NAME + '-1'
    beam_iname = Config.BEAM_PART_NAME + '-1'

    if host_iname not in assembly.instances.keys():
        raise RuntimeError('Host instance not found')
    host_inst = assembly.instances[host_iname]

    if Config.BEAM_PART_NAME not in model.parts.keys():
        print '  no beam part to instance'
        return 0

    if beam_iname not in assembly.instances.keys():
        assembly.Instance(name=beam_iname,
                          part=model.parts[Config.BEAM_PART_NAME], dependent=ON)
    beam_inst = assembly.instances[beam_iname]

    # Host region: all elements
    host_region = assembly.Set(elements=host_inst.elements, name='HostRegion')

    # Embedded region: only B31 elements in the beam instance
    # PartInstance has no elementSets attribute. Filter by element type.
    beam_elems = beam_inst.elements
    b31_list = []
    for e in beam_elems:
        try:
            if e.type == B31:
                b31_list.append(e)
        except:
            pass

    if not b31_list:
        print '  [WARN] no B31 elements found in beam instance'
        return 0

    emb_elems = mesh.MeshElementArray(b31_list)
    emb_region = assembly.Set(elements=emb_elems, name='EmbeddedBeams')

    constraint_name = 'EmbedBeams'
    if constraint_name in model.constraints.keys():
        del model.constraints[constraint_name]
    # absoluteTolerance: 1.5 mm (1.5 x voxel size)
    # 之前 1.0 还有 ~300 节点越界（非凸 host 的深凹处 + bbox 中心法偏移不够完美）
    # 1.5 足够吃掉这些残余。Abaqus 会把越界节点自动拉到最近的 host 表面。
    model.EmbeddedRegion(
        name=constraint_name,
        embeddedRegion=emb_region, hostRegion=host_region,
        weightFactorTolerance=1e-06,
        absoluteTolerance=2.5,
        fractionalTolerance=0.05,
        toleranceMethod=ABSOLUTE)

    print 'Embedded region: %d B31 elements embedded in host (%d elements)' % (
        len(emb_elems), len(host_inst.elements))
    print '  (absoluteTolerance=2.5 mm)'
    return len(emb_elems)


# ============================================================================
# Analysis step and output
# ============================================================================
def create_analysis_step(model, step_name=None):
    if step_name is None:
        step_name = Config.STEP_NAME
    if step_name in model.steps.keys():
        del model.steps[step_name]
    model.StaticStep(
        name=step_name, previous='Initial',
        description='CFRC 4-way comparison load step',
        timePeriod=Config.STEP_TIME_PERIOD,
        initialInc=Config.STEP_INITIAL_INC,
        maxInc=Config.STEP_MAX_INC,
        minInc=Config.STEP_MIN_INC,
        maxNumInc=Config.STEP_MAX_NUM_INC,
        nlgeom=Config.STEP_USE_NLGEOM)

    if 'F-Output-1' in model.fieldOutputRequests.keys():
        del model.fieldOutputRequests['F-Output-1']
    model.FieldOutputRequest(
        name='F-Output-1', createStepName=step_name,
        variables=('S', 'E', 'U', 'RF', 'CF', 'SF', 'SE'))
    print 'Step created: %s, nlgeom=%s, maxInc=%.3f (~%d increments)' % (
        step_name,
        'ON' if Config.STEP_USE_NLGEOM == ON else 'OFF',
        Config.STEP_MAX_INC,
        int(Config.STEP_TIME_PERIOD / Config.STEP_MAX_INC))
    return step_name


def add_history_output_for_loadpoint(model, step_name=None):
    if step_name is None:
        step_name = Config.STEP_NAME
    assembly = model.rootAssembly
    set_name = Config.LOAD_POINT_SET_NAME

    if set_name not in assembly.sets.keys():
        print 'WARNING: Assembly Set "%s" not found. Template missing LoadPoint.' % set_name
        return False

    h_name = 'H-LoadPoint'
    if h_name in model.historyOutputRequests.keys():
        del model.historyOutputRequests[h_name]

    region = assembly.sets[set_name]
    model.HistoryOutputRequest(
        name=h_name, createStepName=step_name,
        variables=('U1', 'U2', 'U3', 'RF1', 'RF2', 'RF3', 'CF1', 'CF2', 'CF3'),
        region=region, sectionPoints=DEFAULT, rebar=EXCLUDE)
    print 'History output added for set "%s"' % set_name
    return True


# ============================================================================
# PHASE 1: Build host-only template
# ============================================================================
def step1_build_template(host_data_dir=None, output_cae=None, make_step=True):
    if host_data_dir is None:
        host_data_dir = os.path.join(Config.BASE_DIR, Config.HOST_SUBDIR)
    if output_cae is None:
        output_cae = Config.TEMPLATE_CAE

    print '\n' + '=' * 70
    print 'PHASE 1: Build host-only template'
    print '=' * 70
    print 'Host data: %s' % host_data_dir
    print 'Output CAE: %s' % output_cae

    if not os.path.isdir(host_data_dir):
        print 'ERROR: host data directory not found'
        return None

    model_name = Config.TEMPLATE_MODEL_NAME
    if model_name in mdb.models.keys():
        del mdb.models[model_name]
    model = mdb.Model(name=model_name)

    if 'Model-1' in mdb.models.keys() and 'Model-1' != model_name:
        try: del mdb.models['Model-1']
        except: pass

    params = read_mesh_params(host_data_dir)
    elements = read_valid_elements(host_data_dir)

    create_host_from_voxels(model, params, elements)

    host_mat = setup_host_material(model)
    host_sec = setup_host_section(model, host_mat)
    assign_host_section(model, host_sec)

    instance_host_in_assembly(model)

    if make_step:
        create_analysis_step(model)

    try:
        session.viewports['Viewport: 1'].setValues(
            displayedObject=model.rootAssembly)
        session.viewports['Viewport: 1'].view.fitView()
    except:
        pass

    out_dir = os.path.dirname(output_cae)
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir)
    mdb.saveAs(pathName=output_cae)

    print '\n' + '=' * 70
    print 'TEMPLATE READY. Next steps:'
    print '=' * 70
    print '  1. In CAE: Load module'
    print '  2. Set BCs (supports, rigid body restraint)'
    print '  3. Apply Concentrated Force at load point (-Z)'
    print '  4. CRITICAL: Create Assembly Set "%s" on load node' % Config.LOAD_POINT_SET_NAME
    print '  5. File -> Save'
    print '  6. Back in Python: step2_run_batch_comparison()'
    print '=' * 70

    return model


# ============================================================================
# PHASE 2: Batch comparison run
# ============================================================================
def get_config_status(cfg_name, base_dir=None):
    """检查一个 config 的运行状态.
    
    判据 (按优先级):
      - 找不到 Job_<cfg>.odb -> NOT_RUN
      - 有 .lck 文件          -> RUNNING (或上次崩了)
      - 没有 .sta 文件        -> FAILED  (分析未启动)
      - .sta 含 "COMPLETED SUCCESSFULLY" -> COMPLETED
      - 其他                   -> FAILED  (.sta 有但没成功标记)
    
    搜索路径: cwd, base_dir, base_dir 下所有子目录, Config.JOB_OUTPUT_DIRS.
    
    Returns:
        (status_str, odb_path_or_none, message_str)
        status_str in {'NOT_RUN', 'RUNNING', 'FAILED', 'COMPLETED', 'UNKNOWN'}
    """
    if base_dir is None:
        base_dir = Config.BASE_DIR
    
    job_name = 'Job_' + cfg_name
    
    # 搜索目录
    search_dirs = [os.getcwd()]
    extra = getattr(Config, 'JOB_OUTPUT_DIRS', [])
    if extra:
        search_dirs.extend(extra)
    if base_dir:
        search_dirs.append(base_dir)
        try:
            for n in os.listdir(base_dir):
                p = os.path.join(base_dir, n)
                if os.path.isdir(p):
                    search_dirs.append(p)
        except: pass
    
    odb_dir = None
    odb_path = None
    for d in search_dirs:
        if not d or not os.path.isdir(d): continue
        p = os.path.join(d, job_name + '.odb')
        if os.path.exists(p):
            odb_dir = d
            odb_path = p
            break
    
    if odb_dir is None:
        return 'NOT_RUN', None, 'no ODB found'
    
    lck = os.path.join(odb_dir, job_name + '.lck')
    if os.path.exists(lck):
        return 'RUNNING', odb_path, 'LCK present (running or crashed)'
    
    sta = os.path.join(odb_dir, job_name + '.sta')
    if not os.path.exists(sta):
        return 'FAILED', odb_path, 'no STA (analysis did not start)'
    
    try:
        f = open(sta, 'r')
        content = f.read()
        f.close()
    except Exception, e:
        return 'UNKNOWN', odb_path, 'cannot read STA: %s' % str(e)
    
    if 'COMPLETED SUCCESSFULLY' in content.upper():
        return 'COMPLETED', odb_path, 'OK'
    else:
        return 'FAILED', odb_path, 'STA missing COMPLETED marker'


def is_config_done(cfg_name, base_dir=None):
    """是否已成功完成. 简单的 True/False, 见 get_config_status."""
    status, _, _ = get_config_status(cfg_name, base_dir)
    return status == 'COMPLETED'


def list_status(configs=None, base_dir=None):
    """打印所有 (或指定) config 的运行状态总览."""
    if configs is None:
        configs = Config.CONFIGS
    if base_dir is None:
        base_dir = Config.BASE_DIR
    
    print '\n' + '=' * 78
    print 'FEA config status (cwd=%s)' % os.getcwd()
    print '=' * 78
    print '  %-18s %-12s %-10s  %s' % ('Config', 'Status', 'ODB(MB)', 'Detail')
    print '  ' + '-' * 74
    
    n_done = 0; n_failed = 0; n_not_run = 0; n_running = 0
    
    for item in configs:
        if isinstance(item, tuple):
            cfg_name = item[0]
        else:
            cfg_name = item
        status, odb_path, msg = get_config_status(cfg_name, base_dir)
        sz_str = '--'
        if odb_path and os.path.exists(odb_path):
            sz_mb = os.path.getsize(odb_path) / 1024.0 / 1024.0
            sz_str = '%.1f' % sz_mb
        print '  %-18s %-12s %-10s  %s' % (cfg_name, status, sz_str, msg)
        if status == 'COMPLETED': n_done += 1
        elif status == 'NOT_RUN': n_not_run += 1
        elif status == 'RUNNING': n_running += 1
        else: n_failed += 1
    
    print '  ' + '-' * 74
    print '  Total: %d   COMPLETED: %d   FAILED: %d   RUNNING: %d   NOT_RUN: %d' % (
        len(configs), n_done, n_failed, n_running, n_not_run)
    print '=' * 78


def clean_config(cfg_name, base_dir=None, dry_run=False):
    """清掉一个 config 的所有 Job_<cfg>.* 残留 (odb/dat/sta/msg/lck/com/inp/...).
    
    用于强制重跑前的"清场". dry_run=True 只打印不真删.
    """
    if base_dir is None:
        base_dir = Config.BASE_DIR
    
    job_name = 'Job_' + cfg_name
    exts = ['.odb', '.dat', '.sta', '.msg', '.lck', '.com', '.inp',
            '.prt', '.sim', '.023', '.mdl', '.stt', '.rec', '.SMABulk',
            '.abq', '.pac', '.sel']
    
    search_dirs = [os.getcwd()]
    extra = getattr(Config, 'JOB_OUTPUT_DIRS', [])
    if extra:
        search_dirs.extend(extra)
    if base_dir:
        search_dirs.append(base_dir)
        try:
            for n in os.listdir(base_dir):
                p = os.path.join(base_dir, n)
                if os.path.isdir(p):
                    search_dirs.append(p)
        except: pass
    
    removed = []
    errors = []
    seen = set()
    for d in search_dirs:
        if not d or not os.path.isdir(d): continue
        if d in seen: continue
        seen.add(d)
        for ext in exts:
            p = os.path.join(d, job_name + ext)
            if os.path.exists(p):
                if dry_run:
                    removed.append(p)
                else:
                    try:
                        os.remove(p)
                        removed.append(p)
                    except Exception, e:
                        errors.append('%s (%s)' % (p, str(e)))
    
    action = '[DRY-RUN] would remove' if dry_run else 'Removed'
    print '\n[clean_config %s] %s %d files' % (cfg_name, action, len(removed))
    for r in removed[:20]:
        print '    %s' % r
    if len(removed) > 20:
        print '    ... (%d more)' % (len(removed) - 20)
    if errors:
        print '  ERRORS:'
        for e in errors:
            print '    %s' % e
    return removed


# ============================================================================
# Phase 2: Batch comparison run (one model per config, sequential)
# ============================================================================
def step2_run_batch_comparison(template_cae=None, configs=None,
                                base_dir=None, submit=True,
                                wait_each=True, skip_done=False):
    if template_cae is None:
        template_cae = Config.TEMPLATE_CAE
    if configs is None:
        configs = Config.CONFIGS
    if base_dir is None:
        base_dir = Config.BASE_DIR

    print '\n' + '=' * 70
    print 'PHASE 2: Batch comparison run (%d configs)' % len(configs)
    print '=' * 70

    if not os.path.exists(template_cae):
        print 'ERROR: template CAE not found. Run step1_build_template() first.'
        return

    openMdb(pathName=template_cae)
    print 'Opened template.'

    if Config.TEMPLATE_MODEL_NAME not in mdb.models.keys():
        src = None
        for mn in mdb.models.keys():
            if Config.HOST_PART_NAME in mdb.models[mn].parts.keys():
                src = mn; break
        if src is None:
            print 'ERROR: no model with HostPart found in template CAE'
            return
        template_model_name = src
    else:
        template_model_name = Config.TEMPLATE_MODEL_NAME

    tpl = mdb.models[template_model_name]

    if Config.LOAD_POINT_SET_NAME not in tpl.rootAssembly.sets.keys():
        print 'WARNING: LoadPoint set missing in template.'
    if len(tpl.boundaryConditions) == 0:
        print 'WARNING: no BCs in template!'
    if len(tpl.loads) == 0:
        print 'WARNING: no loads in template!'
    print 'Template: %d BCs, %d loads, LoadPoint=%s' % (
        len(tpl.boundaryConditions),
        len(tpl.loads),
        'YES' if Config.LOAD_POINT_SET_NAME in tpl.rootAssembly.sets.keys() else 'NO')

    if not os.path.exists(Config.JOBS_DIR):
        os.makedirs(Config.JOBS_DIR)

    if skip_done:
        print 'skip_done=True: 已 COMPLETED 的 config 会被跳过.'

    job_names = []
    n_skipped_done = 0
    for cfg_name, subdir in configs:
        data_dir = os.path.join(base_dir, subdir)
        if not os.path.isdir(data_dir):
            print '\nSKIP %s: data dir missing' % cfg_name
            continue
        if not os.path.exists(os.path.join(data_dir, 'beam_paths_summary.txt')):
            print '\nSKIP %s: no beam_paths' % cfg_name
            continue

        # 新增: 已成功的就跳过
        if skip_done:
            st, odb_path, _msg = get_config_status(cfg_name, base_dir)
            if st == 'COMPLETED':
                print '\nSKIP %s: already COMPLETED  (ODB: %s)' % (cfg_name, odb_path)
                n_skipped_done += 1
                continue
            elif st == 'RUNNING':
                print '\nSKIP %s: %s  (用 clean_config(\'%s\') 清场后再跑)' % (
                    cfg_name, _msg, cfg_name)
                n_skipped_done += 1
                continue

        print '\n' + '-' * 70
        print 'Processing config: %s' % cfg_name
        print '-' * 70

        jn = _process_single_config(
            template_model_name, cfg_name, data_dir,
            submit=submit, wait=wait_each)
        if jn:
            job_names.append(jn)

    print '\n' + '=' * 70
    print 'Batch finished. Jobs run: %s' % (', '.join(job_names) if job_names else '(none)')
    if skip_done and n_skipped_done > 0:
        print '  Skipped (already COMPLETED): %d' % n_skipped_done
    print '=' * 70
    if submit and job_names:
        print 'Next: abaqus cae noGUI=extract_fea_results.py'
    return job_names


def _process_single_config(template_model_name, cfg_name, data_dir,
                           submit=True, wait=True):
    new_model_name = Config.MODEL_PREFIX + cfg_name

    if new_model_name in mdb.models.keys():
        del mdb.models[new_model_name]
    mdb.Model(name=new_model_name,
              objectToCopy=mdb.models[template_model_name])
    new_model = mdb.models[new_model_name]

    beam_paths = read_beam_paths(data_dir)
    print 'Read %d beam paths' % len(beam_paths)

    if not beam_paths:
        print '  no beams, submitting host-only job'
    else:
        create_beams_from_inp(new_model, beam_paths, cfg_name)
        instance_beams_and_embed(new_model)

    add_history_output_for_loadpoint(new_model)

    job_name = 'Job_' + cfg_name
    if job_name in mdb.jobs.keys():
        del mdb.jobs[job_name]
    mdb.Job(name=job_name, model=new_model_name, type=ANALYSIS,
            numCpus=Config.NUM_CPUS, numDomains=Config.NUM_CPUS,
            multiprocessingMode=DEFAULT,
            resultsFormat=ODB,
            echoPrint=OFF, modelPrint=OFF,
            contactPrint=OFF, historyPrint=OFF)

    if submit:
        print 'Submitting job: %s' % job_name
        print '  Job output will be in: %s' % os.getcwd()
        mdb.jobs[job_name].submit(consistencyChecking=OFF)
        if wait:
            mdb.jobs[job_name].waitForCompletion()
            status = mdb.jobs[job_name].status
            print 'Job %s status: %s' % (job_name, status)
            # 检查 dat 是否生成
            dat_check = os.path.join(os.getcwd(), '%s.dat' % job_name)
            if os.path.exists(dat_check):
                print '  dat file: %s (size=%d)' % (dat_check, os.path.getsize(dat_check))

    return job_name


# ============================================================================
# Diagnostics
# ============================================================================
def print_model_summary(model_name=None):
    if model_name is None:
        model_name = Config.TEMPLATE_MODEL_NAME
    if model_name not in mdb.models.keys():
        print 'Model not found: %s' % model_name
        return
    m = mdb.models[model_name]
    print '\nModel: %s' % model_name
    print '  Parts: %s' % ', '.join(m.parts.keys())
    print '  Instances: %s' % ', '.join(m.rootAssembly.instances.keys())
    print '  Constraints: %s' % ', '.join(m.constraints.keys())
    print '  BCs: %s' % ', '.join(m.boundaryConditions.keys())
    print '  Loads: %s' % ', '.join(m.loads.keys())
    print '  Steps: %s' % ', '.join(m.steps.keys())
    print '  Assembly sets: %s' % ', '.join(m.rootAssembly.sets.keys())


def run_single(cfg_name, submit=True):
    return step2_run_batch_comparison(
        configs=[(cfg_name, cfg_name)], submit=submit, wait_each=True)


def run_remaining(cfg_names, submit=True):
    """跑指定的几个 config 子集 (例: run_remaining(['planar_stream', 'planar_offset']))."""
    if isinstance(cfg_names, str):
        cfg_names = [cfg_names]
    sub_configs = [(c, c) for c in cfg_names]
    return step2_run_batch_comparison(
        configs=sub_configs, submit=submit, wait_each=True)


def run_only(cfg_names, submit=True, skip_done=False, clean_first=False):
    """选择性地跑一个 (或一组) config. 推荐用这个代替 run_remaining.
    
    Args:
        cfg_names    : 'planar_stream'  或  ['planar_stream', 'planar_offset']
        submit       : True 提交, False 只建模不跑
        skip_done    : True 时已成功的 config 跳过 (默认 False, 强跑)
        clean_first  : True 时跑之前先 clean_config (清掉旧 job 文件)
    
    Examples:
        run_only('planar_stream')                            # 重跑 planar_stream
        run_only('planar_stream', clean_first=True)          # 清场再跑
        run_only(['planar_stream','planar_offset'])          # 跑两个
        run_only(['mine_stream','mine_offset','planar_stream','planar_offset'],
                 skip_done=True)                             # 跑剩下的, 跳过已成功的
    """
    if isinstance(cfg_names, str):
        cfg_names = [cfg_names]
    
    if clean_first:
        print '\n--- clean_first=True: removing old job files ---'
        for cfg in cfg_names:
            clean_config(cfg)
    
    sub_configs = [(c, c) for c in cfg_names]
    return step2_run_batch_comparison(
        configs=sub_configs, submit=submit,
        wait_each=True, skip_done=skip_done)


def run_pending(submit=True):
    """自动跑所有 NOT_RUN 或 FAILED 的 config (跳过 COMPLETED).
    
    相当于 step2_run_batch_comparison(skip_done=True). 适合多次迭代场景:
    先跑一遍, 看 list_status, 修一下失败的, run_pending() 接着跑剩下的.
    """
    return step2_run_batch_comparison(submit=submit, wait_each=True, skip_done=True)


# ============================================================================
# Auto-diagnose & retry (for stubborn 9-zero-length-bug paths)
# ============================================================================
def _parse_dat_errors(dat_path):
    """从 dat 提取 (problem_eids, embedded_node_nids).
    problem_eids 包含: zero-length, normal-cannot-compute, direction-coincide
    各种 problem element 的 id 集合 (任何一种就触发 path 剔除).
    """
    import re as _re
    if not os.path.exists(dat_path):
        return [], []
    with open(dat_path, 'rb') as f:
        raw = f.read()
    text = raw.decode('latin-1', 'replace')
    eids = []
    bad_nids = []
    # zero length
    for m in _re.finditer(
            r'ELEMENT\s+(\d+)\s+INSTANCE\s+ALLBEAMS-1\s+IS OF ZERO LENGTH',
            text, _re.IGNORECASE):
        eids.append(int(m.group(1)))
    # "NORMAL TO ELEMENT N INSTANCE ALLBEAMS-1 CANNOT BE CALCULATED"
    for m in _re.finditer(
            r'NORMAL TO ELEMENT\s+(\d+)\s+INSTANCE\s+ALLBEAMS-1.{0,80}?CANNOT BE',
            text, _re.IGNORECASE | _re.DOTALL):
        eids.append(int(m.group(1)))
    # "ELEMENT N INSTANCE ALLBEAMS-1 ... NORMAL CANNOT BE COMPUTED" (旧格式兜底)
    for m in _re.finditer(
            r'ELEMENT\s+(\d+)\s+INSTANCE\s+ALLBEAMS-1.{0,80}?NORMAL CANNOT BE',
            text, _re.IGNORECASE | _re.DOTALL):
        eids.append(int(m.group(1)))
    # beam direction vectors coincide
    for m in _re.finditer(
            r'BEAM CROSS-SECTION DIRECTION VECTORS COINCIDE.{0,200}?ELEMENT\s+(\d+)',
            text, _re.IGNORECASE | _re.DOTALL):
        eids.append(int(m.group(1)))
    # Element ID inside element-level error blocks (general fallback)
    for m in _re.finditer(
            r'\*\*\*ERROR:\s*ELEMENT\s+(\d+)\s+INSTANCE\s+ALLBEAMS-1',
            text, _re.IGNORECASE):
        eids.append(int(m.group(1)))
    # embedded node not in host
    for m in _re.finditer(
            r'NODE\s+(\d+)\s+INSTANCE\s+ALLBEAMS-1\s+ON AN EMBEDDED ELEMENT DOES NOT LIE IN',
            text, _re.IGNORECASE):
        bad_nids.append(int(m.group(1)))
    return sorted(set(eids)), sorted(set(bad_nids))


def _parse_inp_path_info(inp_path):
    """从 Job inp 提取 (path_ranges, node_to_paths).
    path_ranges: {inp_path_id: (lo_eid, hi_eid)}
    node_to_paths: {node_id: set(inp_path_id)}  -- AllBeams part 内
    """
    import re as _re
    if not os.path.exists(inp_path):
        return {}, {}
    with open(inp_path, 'rb') as f:
        raw = f.read()
    text = raw.decode('latin-1', 'replace')
    lines = text.split('\n')

    # 1) path_ranges (从 Assembly-level *Elset, generate=BEAMPATH_NNNN)
    ranges = {}
    current_pid = None; in_gen = False
    for line in lines:
        ln = line.strip()
        if ln.lower().startswith('*elset'):
            m = _re.match(r'\*Elset,\s+elset=(?:assembly_)?(?:allbeams-1_)?BEAMPATH_(\d+),?\s*(generate)?',
                          ln, _re.IGNORECASE)
            if m:
                current_pid = int(m.group(1))
                in_gen = 'generate' in ln.lower()
                continue
            current_pid = None; continue
        elif ln.startswith('*'):
            current_pid = None; continue
        if current_pid is not None and in_gen:
            parts = [x.strip() for x in ln.split(',')]
            if len(parts) >= 3:
                try:
                    lo, hi = int(parts[0]), int(parts[1])
                    if current_pid not in ranges:
                        ranges[current_pid] = (lo, hi)
                except: pass

    # 2) AllBeams part 里的 B31 element node 引用
    elems = {}
    in_part_allbeams = False
    in_b31 = False
    for line in lines:
        ln = line.strip()
        low = ln.lower()
        if low.startswith('*part'):
            in_part_allbeams = ('name=allbeams' in low)
            in_b31 = False; continue
        if low.startswith('*end part'):
            in_part_allbeams = False; in_b31 = False; continue
        if not in_part_allbeams: continue
        if ln.startswith('*'):
            in_b31 = low.startswith('*element, type=b31')
            continue
        if in_b31:
            parts = [x.strip() for x in ln.split(',')]
            if len(parts) >= 3:
                try:
                    elems[int(parts[0])] = (int(parts[1]), int(parts[2]))
                except: pass

    # 3) 反向 mapping node -> path
    node_to_paths = {}
    for pid, (lo, hi) in ranges.items():
        for eid in range(lo, hi + 1):
            if eid in elems:
                n1, n2 = elems[eid]
                node_to_paths.setdefault(n1, set()).add(pid)
                node_to_paths.setdefault(n2, set()).add(pid)
    return ranges, node_to_paths


def _map_inp_pid_to_orig(inp_pid, current_blacklist):
    """v14+: BEAMPATH_NNNN 直接就是原始 path_idx (不再因 blacklist 过滤偏移).
    保留此函数签名为兼容性, 但实现简化为 identity."""
    return inp_pid


def _find_job_files(cfg_name, base_dir=None):
    """搜索 Job_<cfg>.dat 和 .inp, 返回 (dat_path, inp_path) 或 (None, None).
    搜索顺序: CAE cwd, Config.JOB_OUTPUT_DIRS (用户配置), base_dir, base_dir/jobs, base_dir 子目录"""
    candidates = []
    cwd = os.getcwd()
    candidates.append(cwd)
    
    # 用户在 Config 里指定的额外搜索路径
    extra = getattr(Config, 'JOB_OUTPUT_DIRS', [])
    if extra:
        candidates.extend(extra)
    
    if base_dir:
        candidates.append(base_dir)
        candidates.append(os.path.join(base_dir, 'jobs'))
        try:
            for n in os.listdir(base_dir):
                p = os.path.join(base_dir, n)
                if os.path.isdir(p):
                    candidates.append(p)
        except: pass
    
    dat_target = 'Job_%s.dat' % cfg_name
    inp_target = 'Job_%s.inp' % cfg_name
    
    found_dat = None
    found_inp = None
    for d in candidates:
        if not d or not os.path.isdir(d): continue
        dp = os.path.join(d, dat_target)
        ip = os.path.join(d, inp_target)
        if os.path.exists(dp) and found_dat is None:
            found_dat = dp
        if os.path.exists(ip) and found_inp is None:
            found_inp = ip
        if found_dat and found_inp: break
    return found_dat, found_inp


def auto_diagnose(cfg_name, base_dir=None):
    """读 dat+inp, 算出新需要加入 blacklist 的原始 path_idx 列表."""
    if base_dir is None:
        base_dir = Config.BASE_DIR
    
    dat_path, inp_path = _find_job_files(cfg_name, base_dir)
    
    print '\n[auto_diagnose] %s' % cfg_name
    print '  dat: %s' % (dat_path if dat_path else '(NOT FOUND)')
    print '  inp: %s' % (inp_path if inp_path else '(NOT FOUND)')
    print '  cwd: %s' % os.getcwd()

    if not dat_path or not inp_path:
        # 列出可能的位置, 帮助调试
        print '  Searched in:'
        cwd = os.getcwd()
        print '    cwd: %s' % cwd
        if base_dir:
            print '    base_dir: %s' % base_dir
        # 在 cwd 找所有 Job_*.dat
        try:
            jobs = [f for f in os.listdir(cwd) if f.startswith('Job_') and f.endswith('.dat')]
            print '  Job dat in cwd: %s' % jobs
        except: pass
        return []

    zero_eids, bad_nids = _parse_dat_errors(dat_path)
    print '  zero-length eids: %s' % zero_eids
    print '  bad-embed nids:   %s' % bad_nids

    if not zero_eids and not bad_nids:
        return []

    ranges, node_to_paths = _parse_inp_path_info(inp_path)
    print '  parsed %d BEAMPATH ranges, %d node-path mappings' % (
        len(ranges), len(node_to_paths))

    bad_inp_pids = set()
    for eid in zero_eids:
        for pid, (lo, hi) in ranges.items():
            if lo <= eid <= hi:
                bad_inp_pids.add(pid); break
    for nid in bad_nids:
        if nid in node_to_paths:
            for pid in node_to_paths[nid]:
                bad_inp_pids.add(pid)

    print '  bad inp BEAMPATH ids: %s' % sorted(bad_inp_pids)

    current_blacklist = Config.BLACKLIST_PATH_IDX.get(cfg_name, [])
    new_orig_pids = []
    for inp_pid in sorted(bad_inp_pids):
        orig = _map_inp_pid_to_orig(inp_pid, current_blacklist)
        if orig is not None and orig not in current_blacklist:
            new_orig_pids.append(orig)
    print '  new original path_idx to add: %s' % new_orig_pids
    return new_orig_pids


def _cleanup_job_files(cfg_name, max_wait=5):
    """清理上次 Job 的 lck/odb/sta/msg 残留, 避免 'Detected lock file' 错误.
    
    .lck 可能被 Abaqus 进程占用导致删除失败, 这里加重试机制.
    max_wait: 最多等待秒数让 Abaqus 释放 lck."""
    import time as _time
    cwd = os.getcwd()
    job = 'Job_' + cfg_name
    extensions = ['.lck', '.odb', '.sta', '.msg', '.com', '.log',
                  '.prt', '.sim', '.023', '.mdl', '.stt', '.abq', '.pac', '.sel']
    cleaned = []
    failed = []
    for ext in extensions:
        p = os.path.join(cwd, job + ext)
        if not os.path.exists(p):
            continue
        # 重试删除最多 max_wait 秒
        deleted = False
        for retry in range(max_wait * 2):
            try:
                os.remove(p)
                cleaned.append(ext)
                deleted = True
                break
            except OSError:
                _time.sleep(0.5)
            except: break
        if not deleted:
            failed.append(ext)
    if cleaned:
        print '  Cleaned residual files: %s' % cleaned
    if failed:
        print '  [WARN] Could not clean (still locked): %s' % failed
        print '         Try waiting a few seconds or close any running Abaqus jobs.'


def run_with_auto_retry(cfg_names, max_retries=4):
    """跑指定 config, 每次失败自动诊断 + 扩 blacklist + 重试.
    
    Args:
        cfg_names: list of config names ('planar_stream' / etc.)  or  single string
        max_retries: 每个 config 最多重试次数 (默认 4)
    
    Returns:
        dict: {cfg_name: (final_status, final_blacklist)}
    """
    if isinstance(cfg_names, str):
        cfg_names = [cfg_names]
    
    results = {}
    for cfg in cfg_names:
        print '\n' + '=' * 70
        print 'AUTO-RETRY: %s' % cfg
        print '=' * 70
        
        for attempt in range(1, max_retries + 2):
            print '\n--- Attempt %d/%d for %s ---' % (
                attempt, max_retries + 1, cfg)
            print 'Current blacklist: %s' % Config.BLACKLIST_PATH_IDX.get(cfg, [])
            
            # 清理上次的 lck/odb 残留
            _cleanup_job_files(cfg)
            
            # 跑 single config
            try:
                step2_run_batch_comparison(
                    configs=[(cfg, cfg)], submit=True, wait_each=True)
            except Exception, e:
                print 'Submit failed: %s' % str(e)
                results[cfg] = ('EXCEPTION', Config.BLACKLIST_PATH_IDX.get(cfg, []))
                break
            
            # 检查 status
            job_name = 'Job_' + cfg
            if job_name not in mdb.jobs.keys():
                print 'Job %s does not exist' % job_name
                break
            status = mdb.jobs[job_name].status
            print 'Job %s final status: %s' % (job_name, status)
            
            if status == COMPLETED:
                print '*** SUCCESS for %s after %d attempt(s) ***' % (cfg, attempt)
                results[cfg] = ('COMPLETED', Config.BLACKLIST_PATH_IDX.get(cfg, []))
                break
            
            # 失败 -> 自动诊断
            if attempt > max_retries:
                print '*** GIVE UP after %d attempts ***' % attempt
                results[cfg] = ('FAILED_OUT_OF_RETRIES',
                                 Config.BLACKLIST_PATH_IDX.get(cfg, []))
                break
            
            new_pids = auto_diagnose(cfg)
            if not new_pids:
                print '*** No new path_idx found, cannot recover ***'
                results[cfg] = ('FAILED_NO_DIAGNOSE',
                                 Config.BLACKLIST_PATH_IDX.get(cfg, []))
                break
            
            # 扩 blacklist
            old_bl = list(Config.BLACKLIST_PATH_IDX.get(cfg, []))
            new_bl = sorted(set(old_bl + new_pids))
            Config.BLACKLIST_PATH_IDX[cfg] = new_bl
            print '  Updated %s blacklist: %s -> %s' % (cfg, old_bl, new_bl)
    
    # Summary
    print '\n' + '=' * 70
    print 'AUTO-RETRY SUMMARY'
    print '=' * 70
    for cfg, (status, bl) in results.items():
        print '  %-20s: %-25s blacklist=%s' % (cfg, status, bl)
    print
    print 'IMPORTANT: blacklists are in-memory only.'
    print 'For permanence, copy these into Config.BLACKLIST_PATH_IDX in source:'
    for cfg, (status, bl) in results.items():
        if bl and status == 'COMPLETED':
            print "  '%s': %s," % (cfg, bl)
    print '=' * 70
    
    return results


def dump_blacklist():
    """打印当前内存里的 Config.BLACKLIST_PATH_IDX, 格式可以直接粘贴回源码.
    用法: auto_retry 跑完后, 把这里的输出复制到 Config 里, 让 blacklist 持久化."""
    print '\n# Paste this back into Config.BLACKLIST_PATH_IDX:'
    print 'BLACKLIST_PATH_IDX = {'
    for cfg in ['mine_stream', 'mine_offset', 'planar_stream', 'planar_offset']:
        bl = Config.BLACKLIST_PATH_IDX.get(cfg, [])
        if not bl:
            print "    '%s': []," % cfg
        else:
            print "    '%s': %s," % (cfg, sorted(bl))
    print '}'


def inp_health_check(cfg_name):
    """对已生成的 inp 做 pre-flight 健康检查 (不需要跑 Abaqus job).
    
    报告: 段长分布, 与 n1 共线 segment 数, 重合节点数, 可疑 path.
    在 step2 / run_only 之前调用以判断 inp 是否健康.
    """
    import re as _re
    import math as _math
    
    # Find inp
    inp_target = 'Job_%s.inp' % cfg_name
    inp_path = None
    for d in [os.getcwd(), Config.BASE_DIR] + list(getattr(Config, 'JOB_OUTPUT_DIRS', [])):
        if not d or not os.path.isdir(d): continue
        p = os.path.join(d, inp_target)
        if os.path.exists(p):
            inp_path = p; break
    if not inp_path:
        # Try temp_beam (before job submission)
        for d in [Config.BASE_DIR]:
            p = os.path.join(d, 'temp_beam_%s.inp' % cfg_name)
            if os.path.exists(p):
                inp_path = p; break
    if not inp_path:
        print '[health_check] inp not found for %s' % cfg_name
        return
    
    print '\n[health_check] %s' % inp_path
    
    with open(inp_path, 'rb') as f:
        text = f.read().decode('latin-1', 'replace')
    
    # Parse nodes + B31 from AllBeams part
    nodes = {}
    elems = []
    in_part = False; in_node = False; in_b31 = False
    for line in text.splitlines():
        low = line.strip().lower()
        if low.startswith('*part'):
            in_part = 'name=allbeams' in low; continue
        if in_part and low.startswith('*end part'):
            break
        if not in_part: continue
        if low.startswith('*node'):
            in_node = True; in_b31 = False; continue
        if low.startswith('*element, type=b31'):
            in_b31 = True; in_node = False; continue
        if line.strip().startswith('*'):
            in_node = False; in_b31 = False; continue
        if in_node:
            p = line.split(',')
            if len(p) >= 4:
                try: nodes[int(p[0])] = (float(p[1]), float(p[2]), float(p[3]))
                except: pass
        elif in_b31:
            p = line.split(',')
            if len(p) >= 3:
                try: elems.append((int(p[0]), int(p[1]), int(p[2])))
                except: pass
    
    # Parse BEAMPATH ranges
    ranges = {}
    lines = text.splitlines()
    for i in range(len(lines) - 1):
        m = _re.match(r'\*Elset,\s*elset=BEAMPATH_(\d+)', lines[i], _re.I)
        if m:
            pn = int(m.group(1))
            # try generate format
            if 'generate' in lines[i].lower():
                data = lines[i+1].split(',')
                if len(data) >= 2:
                    try: ranges[pn] = (int(data[0]), int(data[1]))
                    except: pass
            else:
                # Explicit list -> approximate range
                eids_in_set = []
                j = i + 1
                while j < len(lines) and not lines[j].strip().startswith('*'):
                    for tok in lines[j].split(','):
                        tok = tok.strip()
                        if tok.isdigit(): eids_in_set.append(int(tok))
                    j += 1
                if eids_in_set:
                    ranges[pn] = (min(eids_in_set), max(eids_in_set))
    
    # n1 from Config
    _nx, _ny, _nz = Config.BEAM_N1_RAW
    _norm = _math.sqrt(_nx*_nx + _ny*_ny + _nz*_nz)
    n1 = (_nx/_norm, _ny/_norm, _nz/_norm)
    cos_thr = getattr(Config, 'BEAM_N1_PARALLEL_COSINE', 0.99)
    
    # Compute stats
    lens = []
    bad_short = []
    bad_long = []
    bad_parallel = []
    for eid, na, nb in elems:
        if na not in nodes or nb not in nodes: continue
        x1,y1,z1 = nodes[na]; x2,y2,z2 = nodes[nb]
        dx,dy,dz = x2-x1, y2-y1, z2-z1
        L = _math.sqrt(dx*dx + dy*dy + dz*dz)
        lens.append(L)
        if L < 0.05: bad_short.append((eid, L))
        if L > 5.0: bad_long.append((eid, L))
        if L > 1e-9:
            cosv = abs((dx*n1[0] + dy*n1[1] + dz*n1[2]) / L)
            if cosv > cos_thr:
                bad_parallel.append((eid, cosv))
    
    # Coincident nodes
    coord_to_nids = {}
    for nid, (x,y,z) in nodes.items():
        key = (round(x,5), round(y,5), round(z,5))
        coord_to_nids.setdefault(key, []).append(nid)
    n_coincident_pairs = sum(1 for v in coord_to_nids.values() if len(v) > 1)
    
    # Report
    print '  Total nodes:    %d' % len(nodes)
    print '  Total B31:      %d' % len(elems)
    print '  Total paths:    %d' % len(ranges)
    if lens:
        lens_sorted = sorted(lens)
        print '  Segment lens:   min=%.4f  max=%.4f  median=%.4f' % (
            lens_sorted[0], lens_sorted[-1], lens_sorted[len(lens_sorted)//2])
    print '  Short (<0.05mm): %d' % len(bad_short)
    print '  Long  (>5mm):    %d  %s' % (
        len(bad_long), '(may cause B31 normal error)' if bad_long else '')
    print '  N1-parallel:     %d  (cos>%.2f)' % (len(bad_parallel), cos_thr)
    print '  Coincident node pairs: %d' % n_coincident_pairs
    
    # Identify suspect paths
    bad_eids = set([e[0] for e in bad_short + bad_long + bad_parallel])
    if bad_eids:
        bad_paths = set()
        for eid in bad_eids:
            for pid, (lo, hi) in ranges.items():
                if lo <= eid <= hi:
                    bad_paths.add(pid); break
        print '  Suspect BEAMPATH (= original path index): %s' % sorted(bad_paths)
    
    health = 'HEALTHY' if (not bad_long and not bad_parallel) else 'SUSPECT'
    print '  Overall: %s' % health
    return {
        'n_paths': len(ranges),
        'bad_paths': sorted(bad_paths) if bad_eids else [],
        'health': health,
    }


print '\n' + '=' * 70
print 'abaqus_cfrc_compare.py v5 (skip-done + selective re-run) loaded.'
print '  VERSION TAG: SKIP-DONE+RUN-ONLY  (2026-05-20-v15)  [4-way, no s3_mine]'
print '=' * 70
print 'Phase 1: step1_build_template()  -> edit template.cae, save'
print 'Phase 2: step2_run_batch_comparison()              # 跑全部 (覆盖)'
print '         step2_run_batch_comparison(skip_done=True)# 跑全部, 跳过已成功'
print ''
print 'Status:  list_status()                             # 看每个 config 的状态'
print '         get_config_status("planar_stream")        # 看单个'
print ''
print 'Re-run:  run_only("planar_stream")                 # 强跑一个'
print '         run_only("planar_stream", clean_first=True)# 清场后强跑'
print '         run_only(["planar_stream","planar_offset"])# 跑多个'
print '         run_pending()                             # 跑所有未完成的'
print ''
print 'Clean:   clean_config("planar_stream")             # 清掉旧 Job 文件'
print '         clean_config("planar_stream", dry_run=True)# 只看不删'
print ''
print 'Debug:   inp_health_check("mine_stream")           # pre-flight 检查 inp'
print '         auto_diagnose("planar_stream")            # 从 dat 反查问题 path'
print '         run_with_auto_retry(["mine_stream"])      # 自动诊断+重试'
print '         dump_blacklist()                          # 打印 in-memory blacklist'
print '=' * 70
