# -*- coding: utf-8 -*-
"""
Extract F-U history from CFRC comparison job ODBs.
====================================================

v2 (2026-05-20): Field-output fallback.
  The original script only read History Output (Node-level time series),
  but the inp uses *Output, history, variable=PRESELECT which does NOT
  include per-node U/RF/CF histories -> CSV came out empty.
  This version tries History Output first; if it's empty, FALLS BACK to
  Field Output (iterates over frames, picks LoadPoint nodes per frame).

Reads Job_<config>.odb for each of the 5 configurations, extracts:
  - Time history at LoadPoint set nodes
  - U3, CF3, RF3 histories
  - Stiffness K = F / u via linear regression
  - Max mises in host, max axial SF1 in beams
  - Element counts

Writes results to CSV per-config and summary.txt that MATLAB's
compare_fea_results.m parses (no scipy dependency required).

Run as:
    abaqus cae noGUI=extract_fea_results.py
OR inside Abaqus CAE Python command:
    execfile('extract_fea_results.py')
    run_extract()
"""

from odbAccess import *
from abaqusConstants import *
import os
import sys


# --- Must match abaqus_cfrc_compare.py Config ---
BASE_DIR       = 'C:/temp/cfrc_fea'
RESULTS_DIR    = 'C:/temp/cfrc_fea/results'
STEP_NAME      = 'LoadStep'
LOAD_POINT_SET = 'LoadPoint'

CONFIGS = [
    ('mine_stream',   'Job_mine_stream.odb'),
    ('mine_offset',   'Job_mine_offset.odb'),
    ('planar_stream', 'Job_planar_stream.odb'),
    ('planar_offset', 'Job_planar_offset.odb'),
]


def find_odb(odb_filename):
    """Search common locations for the odb."""
    candidates = [
        os.path.join(BASE_DIR, odb_filename),
        os.path.abspath(odb_filename),
        odb_filename,
    ]
    for c in candidates:
        if os.path.exists(c):
            return c
    for root, dirs, files in os.walk(BASE_DIR):
        if odb_filename in files:
            return os.path.join(root, odb_filename)
    return None


def get_load_point_node_labels(odb, set_name=LOAD_POINT_SET):
    """Extract node labels from LoadPoint assembly set.
    Assembly node sets in odb are typically uppercased."""
    labels = set()
    for trial_name in (set_name.upper(), set_name, set_name.lower()):
        try:
            ns = odb.rootAssembly.nodeSets[trial_name]
        except KeyError:
            continue
        try:
            for inst_nodes in ns.nodes:
                for node in inst_nodes:
                    labels.add(node.label)
            if labels:
                return labels
        except:
            pass
    return labels


def collect_history_for_nodes(step, target_labels):
    """Try to extract U/RF/CF from History Output (per-node time series).
    Returns (time_vec, {var: list_of_values}) or (None, {}) if no data.
    
    Multiple nodes: U values averaged; RF/CF summed.
    """
    result = {}
    time_vec = None
    counts = {}

    for hr_name, hr in step.historyRegions.items():
        if not hr_name.startswith('Node'):
            continue
        try:
            label = int(hr_name.split('.')[-1])
        except (ValueError, IndexError):
            continue
        if target_labels and label not in target_labels:
            continue

        ho = hr.historyOutputs
        for var in ('U1', 'U2', 'U3', 'RF1', 'RF2', 'RF3', 'CF1', 'CF2', 'CF3'):
            if var not in ho.keys():
                continue
            data = ho[var].data
            if time_vec is None:
                time_vec = [d[0] for d in data]
            vals = [d[1] for d in data]
            if var not in result:
                result[var] = list(vals)
                counts[var] = 1
            else:
                for i in range(len(result[var])):
                    result[var][i] += vals[i]
                counts[var] += 1

    # U values: average across nodes (they should be ~equal anyway)
    for var in ('U1', 'U2', 'U3'):
        if var in result and counts.get(var, 1) > 1:
            c = float(counts[var])
            result[var] = [v / c for v in result[var]]

    return time_vec, result


def collect_from_field_output(step, target_labels):
    """[NEW] Extract U/RF/CF by iterating frames of Field Output.
    
    Works even when *Output, history wasn't requested per-node
    (i.e. when only *Output, history, variable=PRESELECT was used).
    
    For target_labels: U values averaged; RF/CF summed (consistent with
    'total applied/reaction force on the set').
    
    Returns (time_vec, {var: list_of_values_per_frame}).
    """
    n_frames = len(step.frames)
    if n_frames == 0:
        return None, {}
    
    if not target_labels:
        print '    [field fallback] WARNING: no target_labels, will use all nodes'
    target_set = set(target_labels) if target_labels else None
    
    time_vec = []
    result = {}
    
    # First check which field variables exist (use last frame as reference)
    last_fr = step.frames[-1]
    available_fields = set()
    for var_name in ('U', 'RF', 'CF'):
        if var_name in last_fr.fieldOutputs.keys():
            available_fields.add(var_name)
    print '    [field fallback] available field outputs: %s' % sorted(available_fields)
    
    if not available_fields:
        print '    [field fallback] no U/RF/CF field outputs found'
        return None, {}
    
    for fr in step.frames:
        time_vec.append(fr.frameValue)
        
        for var_name in sorted(available_fields):
            field = fr.fieldOutputs[var_name]
            
            # Filter to target node labels
            if target_set is not None:
                values_filtered = [v for v in field.values if v.nodeLabel in target_set]
            else:
                values_filtered = list(field.values)
            
            if not values_filtered:
                # Append zeros for this frame to keep arrays aligned
                for comp_idx, suffix in enumerate(('1', '2', '3')):
                    key = var_name + suffix
                    if key not in result: result[key] = []
                    result[key].append(0.0)
                continue
            
            # Each value has .data = (component1, component2, component3) for vectors
            for comp_idx, suffix in enumerate(('1', '2', '3')):
                key = var_name + suffix
                if key not in result: result[key] = []
                vals = []
                for v in values_filtered:
                    try:
                        if v.data is not None and len(v.data) > comp_idx:
                            vals.append(v.data[comp_idx])
                    except: pass
                if not vals:
                    result[key].append(0.0)
                elif var_name == 'U':
                    # Average displacements across the set
                    result[key].append(sum(vals) / float(len(vals)))
                else:
                    # Sum forces (total applied/reaction at set)
                    result[key].append(sum(vals))
    
    return time_vec, result


def extract_one(odb_path, cfg_name):
    """Open ODB, extract F-U history and scalar metrics. Returns dict."""
    print '\n--- Extracting: %s ---' % cfg_name
    print '  ODB: %s' % odb_path

    result = {
        'config_name': cfg_name,
        'odb_path': odb_path,
        'status': 'FAIL',
        'time': [],
        'u1': [], 'u2': [], 'u3': [],
        'rf1': [], 'rf2': [], 'rf3': [],
        'cf1': [], 'cf2': [], 'cf3': [],
        'F_applied': [],
        'u_loadpt': [],
        'stiffness_K': 0.0,
        'stiffness_K_end': 0.0,
        'max_u3_abs': 0.0,
        'max_u_mag': 0.0,
        'max_mises_host': 0.0,
        'max_sf1_beam': 0.0,
        'num_host_elems': 0,
        'num_beam_elems': 0,
        'num_load_nodes': 0,
        'data_source': 'NONE',   # 'history' / 'field' / 'NONE'
    }

    try:
        odb = openOdb(path=odb_path, readOnly=True)
    except Exception, e:
        print '  ERROR opening odb: %s' % str(e)
        result['status'] = 'OPEN_FAILED'
        return result

    try:
        # Step
        try:
            step = odb.steps[STEP_NAME]
        except KeyError:
            keys = odb.steps.keys()
            if not keys:
                print '  ERROR: no steps'
                return result
            step = odb.steps[keys[0]]

        # Load point node labels
        lp_labels = get_load_point_node_labels(odb, LOAD_POINT_SET)
        result['num_load_nodes'] = len(lp_labels)
        if not lp_labels:
            print '  WARNING: "%s" node set not found' % LOAD_POINT_SET
        else:
            preview = sorted(list(lp_labels))[:10]
            print '  LoadPoint nodes: %d total, first 10: %s' % (
                len(lp_labels), preview)

        # =====================================================
        # STAGE 1: try History Output (per-node time series)
        # =====================================================
        print '  [Stage 1] Trying History Output...'
        time_vec, vardict = collect_history_for_nodes(step, lp_labels)
        used_source = None
        if time_vec is not None and vardict and any(
                v and any(abs(x) > 1e-20 for x in v) for v in vardict.values()):
            print '    [history] OK -- %d time points, vars=%s' % (
                len(time_vec), sorted(vardict.keys()))
            used_source = 'history'
        else:
            print '    [history] empty or zeros only -- falling back'

            # =================================================
            # STAGE 2: Field Output frame-by-frame
            # =================================================
            print '  [Stage 2] Trying Field Output (per-frame)...'
            time_vec, vardict = collect_from_field_output(step, lp_labels)
            if time_vec is not None and vardict:
                print '    [field] OK -- %d frames, vars=%s' % (
                    len(time_vec), sorted(vardict.keys()))
                used_source = 'field'
            else:
                print '    [field] also empty'

        if used_source is None:
            print '  WARNING: no time-history data could be extracted'
        else:
            result['data_source'] = used_source
            result['time'] = time_vec
            for var in ('U1', 'U2', 'U3', 'RF1', 'RF2', 'RF3', 'CF1', 'CF2', 'CF3'):
                if var in vardict:
                    result[var.lower()] = vardict[var]

            # Applied force: prefer CF3, then -RF3, then time-ramp fallback
            cf3 = result['cf3']
            F_app = None
            if cf3 and any([abs(x) > 1e-12 for x in cf3]):
                F_app = cf3
                print '  F_applied: using CF3 (max |CF3|=%.4e N)' % max(
                    [abs(x) for x in cf3])
            else:
                rf3 = result['rf3']
                if rf3 and any([abs(x) > 1e-12 for x in rf3]):
                    F_app = [-x for x in rf3]
                    print '  F_applied: using -RF3 (CF3 zero, assuming balance, max=%.4e N)' % max(
                        [abs(x) for x in rf3])
                else:
                    tp = step.timePeriod
                    F_app = [t / tp for t in time_vec]
                    print '  WARNING: F_applied = normalized ramp (no CF/RF data)'
            result['F_applied'] = F_app

            # u at load point (U3 per user spec)
            u_lp = result['u3'] if result['u3'] else (
                result['u1'] if result['u1'] else [0.0] * len(time_vec))
            result['u_loadpt'] = u_lp

            # Stiffness K = F / u (linear least-squares through origin)
            abs_u = [abs(x) for x in u_lp]
            abs_F = [abs(x) for x in F_app]
            num = sum([F * u for F, u in zip(abs_F, abs_u)])
            den = sum([u * u for u in abs_u])
            if den > 1e-20:
                result['stiffness_K'] = num / den
            if abs_u and abs_u[-1] > 1e-10:
                result['stiffness_K_end'] = abs_F[-1] / abs_u[-1]

            result['max_u3_abs'] = max([abs(x) for x in u_lp]) if u_lp else 0.0

            if result['u1'] and result['u2'] and result['u3']:
                umag = [
                    (u1**2 + u2**2 + u3**2) ** 0.5
                    for u1, u2, u3 in zip(result['u1'], result['u2'], result['u3'])
                ]
                result['max_u_mag'] = max(umag)

            print '  Data source: %s' % used_source
            print '  K (linear fit)    = %.4f N/mm' % result['stiffness_K']
            print '  K (end-point)     = %.4f N/mm' % result['stiffness_K_end']
            print '  |U3|_max          = %.4e mm' % result['max_u3_abs']
            if abs_F:
                print '  F_applied_max     = %.4e N' % max(abs_F)

        # --- Field output: max Mises and max SF1 at last frame ---
        try:
            fr = step.frames[-1]
            if 'S' in fr.fieldOutputs.keys():
                S = fr.fieldOutputs['S']
                max_mises = 0.0
                for v in S.values:
                    if hasattr(v, 'mises') and v.mises is not None:
                        if v.mises > max_mises:
                            max_mises = v.mises
                result['max_mises_host'] = max_mises
                print '  Max Mises: %.2f MPa' % max_mises

            if 'SF' in fr.fieldOutputs.keys():
                SF = fr.fieldOutputs['SF']
                max_sf1 = 0.0
                for v in SF.values:
                    if hasattr(v, 'data') and v.data is not None and len(v.data) >= 1:
                        if abs(v.data[0]) > max_sf1:
                            max_sf1 = abs(v.data[0])
                result['max_sf1_beam'] = max_sf1
                print '  Max |SF1| (beam): %.4f N' % max_sf1
        except Exception, e:
            print '  Field extraction error: %s' % str(e)

        # Element counts
        try:
            for iname, inst in odb.rootAssembly.instances.items():
                U = iname.upper()
                if U.startswith('HOSTPART') or 'HOST' in U:
                    result['num_host_elems'] += len(inst.elements)
                elif U.startswith('ALLBEAMS') or 'BEAM' in U:
                    n_b31 = 0
                    for e in inst.elements:
                        try:
                            if e.type == 'B31':
                                n_b31 += 1
                        except:
                            pass
                    result['num_beam_elems'] += n_b31
        except Exception, e:
            print '  Element count error: %s' % str(e)

        result['status'] = 'OK'
    finally:
        odb.close()

    return result


def run_extract():
    if not os.path.exists(RESULTS_DIR):
        os.makedirs(RESULTS_DIR)

    all_results = {}
    for cfg_name, odb_fname in CONFIGS:
        odb_path = find_odb(odb_fname)
        if odb_path is None:
            print '\nSKIP %s: %s not found' % (cfg_name, odb_fname)
            all_results[cfg_name] = {'status': 'NO_ODB', 'config_name': cfg_name}
            continue
        r = extract_one(odb_path, cfg_name)
        all_results[cfg_name] = r

    # --- Write per-config time history CSV ---
    for cfg_name, _ in CONFIGS:
        r = all_results.get(cfg_name, {})
        if r.get('status') != 'OK':
            continue
        csv_path = os.path.join(RESULTS_DIR, '%s_time_history.csv' % cfg_name)
        _write_csv(csv_path, r)
        print '\nWrote: %s (%d rows, source=%s)' % (
            csv_path, len(r.get('time', [])), r.get('data_source', 'NONE'))

    # --- Write summary table ---
    summary_path = os.path.join(RESULTS_DIR, 'summary.txt')
    _write_summary(summary_path, all_results)
    print '\nWrote summary: %s' % summary_path

    # --- Print final summary to console ---
    print '\n' + '=' * 95
    print 'EXTRACTION SUMMARY'
    print '=' * 95
    print '%-18s %-9s %-8s %-12s %-14s %-10s %-10s' % (
        'Config', 'Status', 'Source', 'K (N/mm)', '|U3|_max (mm)', 'Mises', 'Beams')
    for cfg_name, _ in CONFIGS:
        r = all_results.get(cfg_name, {})
        st = r.get('status', 'MISSING')
        if st == 'OK':
            print '%-18s %-9s %-8s %-12.4f %-14.4e %-10.2f %-10d' % (
                cfg_name, st, r.get('data_source', '?'),
                r['stiffness_K'], r['max_u3_abs'],
                r['max_mises_host'], r['num_beam_elems'])
        else:
            print '%-18s %-9s' % (cfg_name, st)
    print '=' * 95
    print '\nNow in MATLAB: run compare_fea_results.m  (or run_compare in C:\\temp\\cfrc_fea)'

    return all_results


def _write_csv(csv_path, r):
    t = r['time']
    F = r['F_applied']
    u = r['u_loadpt']
    u3 = r['u3'] if r['u3'] else u
    rf3 = r['rf3'] if r['rf3'] else [0.0] * len(t)
    cf3 = r['cf3'] if r['cf3'] else [0.0] * len(t)

    def g(lst, i):
        return lst[i] if i < len(lst) else 0.0

    n = len(t)
    with open(csv_path, 'w') as f:
        f.write('time,F_applied_N,u_loadpt_mm,U3_mm,RF3_N,CF3_N\n')
        for i in range(n):
            f.write('%.6e,%.6e,%.6e,%.6e,%.6e,%.6e\n' % (
                t[i], g(F, i), g(u, i), g(u3, i), g(rf3, i), g(cf3, i)))


def _write_summary(path, all_results):
    with open(path, 'w') as f:
        f.write('# CFRC 4-way FEA comparison summary\n')
        f.write('# config, status, K_linear_Nmm, K_end_Nmm, max_u3_abs_mm, '
                'max_u_mag_mm, max_mises_MPa, max_sf1_N, num_beam_elems, '
                'num_host_elems, num_load_nodes\n')
        for cfg_name, _ in CONFIGS:
            r = all_results.get(cfg_name, {})
            st = r.get('status', 'MISSING')
            if st == 'OK':
                f.write('%s, %s, %.6e, %.6e, %.6e, %.6e, %.6e, %.6e, %d, %d, %d\n' % (
                    cfg_name, st,
                    r['stiffness_K'], r['stiffness_K_end'],
                    r['max_u3_abs'], r['max_u_mag'],
                    r['max_mises_host'], r['max_sf1_beam'],
                    r['num_beam_elems'], r['num_host_elems'],
                    r['num_load_nodes']))
            else:
                f.write('%s, %s, 0, 0, 0, 0, 0, 0, 0, 0, 0\n' % (cfg_name, st))


# Auto-run when launched via `abaqus cae noGUI=extract_fea_results.py`
if __name__ == '__main__':
    run_extract()
else:
    print 'extract_fea_results.py v2 loaded. Call: run_extract()'
