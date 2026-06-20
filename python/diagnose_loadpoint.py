# -*- coding: utf-8 -*-
"""
diagnose_loadpoint.py
=====================
检查 LoadPoint (Set-1) 节点集到底包含了哪些节点, 它们的位置分布,
以及在末帧的 U3 / U_magnitude 是多少.

输出会同时打印到 console AND 写入 diagnose_output.txt (在 BASE_DIR).

用法 (Windows cmd 在 C:\\temp\\cfrc_fea):
    abaqus cae noGUI=diagnose_loadpoint.py
然后打开:
    C:\\temp\\cfrc_fea\\diagnose_output.txt
"""
from odbAccess import *
import os
import sys

BASE_DIR = 'C:/temp/cfrc_fea'
STEP_NAME = 'LoadStep'
LOAD_POINT_SET = 'LoadPoint'
LOG_PATH = os.path.join(BASE_DIR, 'diagnose_output.txt')

CONFIGS = ['mine_stream', 'mine_offset', 'planar_stream', 'planar_offset']

# Global log file handle (opened in diagnose_all)
_LOG_FH = None


def log(msg=''):
    """打印到 console 同时写入 log 文件."""
    print msg
    if _LOG_FH is not None:
        try:
            _LOG_FH.write(msg + '\n')
            _LOG_FH.flush()
        except: pass


def find_odb(cfg):
    fn = 'Job_%s.odb' % cfg
    for d in [BASE_DIR, os.getcwd()]:
        p = os.path.join(d, fn)
        if os.path.exists(p):
            return p
    return None


def diagnose_one(cfg):
    odb_path = find_odb(cfg)
    if not odb_path:
        log('\n[%s] ODB not found' % cfg)
        return
    log('\n' + '=' * 70)
    log('DIAGNOSING:  %s' % cfg)
    log('=' * 70)
    log('ODB: %s' % odb_path)

    odb = openOdb(path=odb_path, readOnly=True)
    try:
        step = odb.steps.values()[0]

        # 1. Find LoadPoint set
        ns = None
        for nm in (LOAD_POINT_SET.upper(), LOAD_POINT_SET, 'Set-1', 'SET-1'):
            if nm in odb.rootAssembly.nodeSets.keys():
                ns = odb.rootAssembly.nodeSets[nm]
                log('  [Set] Using node set: %s' % nm)
                break
        if ns is None:
            log('  [ERROR] No LoadPoint / Set-1 found')
            log('  Available node sets: %s' % sorted(odb.rootAssembly.nodeSets.keys()))
            return

        # 2. Enumerate nodes in the set with their coordinates
        set_labels = set()
        node_coords = {}   # label -> (x,y,z)
        for inst_nodes in ns.nodes:
            for node in inst_nodes:
                set_labels.add(node.label)
                node_coords[node.label] = node.coordinates

        log('  [Set] Total nodes: %d' % len(set_labels))
        if not set_labels:
            return

        # Z distribution
        zs = sorted([c[2] for c in node_coords.values()])
        log('  [Set] Z range: %.3f to %.3f mm' % (zs[0], zs[-1]))
        z_buckets = {}
        for z in zs:
            zk = round(z * 2) / 2.0
            z_buckets[zk] = z_buckets.get(zk, 0) + 1
        log('  [Set] Z bucket counts:')
        for zk in sorted(z_buckets.keys()):
            log('          z=%.1f  -> %d nodes' % (zk, z_buckets[zk]))

        # 3. Get U at last frame for these nodes
        fr = step.frames[-1]
        if 'U' not in fr.fieldOutputs.keys():
            log('  [ERROR] No U field output')
            return

        U_field = fr.fieldOutputs['U']
        U3_list = []
        Umag_list = []
        per_node_data = []
        for v in U_field.values:
            if v.nodeLabel in set_labels:
                u3 = v.data[2] if len(v.data) >= 3 else 0.0
                umag = (v.data[0]**2 + v.data[1]**2 + v.data[2]**2) ** 0.5
                U3_list.append(u3)
                Umag_list.append(umag)
                per_node_data.append((v.nodeLabel,
                                      node_coords.get(v.nodeLabel, (0,0,0)),
                                      u3, umag))

        if not U3_list:
            log('  [ERROR] No U values matched')
            return

        n = len(U3_list)
        u3_abs = [abs(x) for x in U3_list]
        log('')
        log('  [U3 stats at LAST frame]')
        log('    n samples:    %d' % n)
        log('    |U3| mean:    %.4e mm   <- extract.py uses this' % (sum(u3_abs)/n))
        log('    |U3| max:     %.4e mm   <- physical "max deflection"' % max(u3_abs))
        log('    |U3| min:     %.4e mm' % min(u3_abs))
        log('    |U3| median:  %.4e mm' % sorted(u3_abs)[n//2])
        log('')
        log('    |U|mag mean:  %.4e mm' % (sum(Umag_list)/n))
        log('    |U|mag max:   %.4e mm   <- matches CAE contour peak (red)' % max(Umag_list))

        # F applied
        F_total = None
        if 'CF' in fr.fieldOutputs.keys():
            CF = fr.fieldOutputs['CF']
            cf3_sum = 0.0
            cf3_count = 0
            cf3_per_node = []
            for v in CF.values:
                if v.nodeLabel in set_labels:
                    cf = v.data[2] if len(v.data) >= 3 else 0.0
                    cf3_sum += cf
                    cf3_count += 1
                    if abs(cf) > 1e-10:
                        cf3_per_node.append(cf)
            F_total = cf3_sum
            log('')
            log('  [Applied force at LAST frame] (over LoadPoint set)')
            log('    Sum CF3:           %.4e N  (%d nodes contributed)' % (
                F_total, cf3_count))
            if cf3_per_node:
                log('    Non-zero per-node: %d nodes' % len(cf3_per_node))
                log('    CF3 per-node min:  %.4e N' % min(cf3_per_node))
                log('    CF3 per-node max:  %.4e N' % max(cf3_per_node))
                avg_cf = sum(cf3_per_node) / len(cf3_per_node)
                log('    CF3 per-node avg:  %.4e N  <- *Cload value' % avg_cf)

        # RF over BC nodes
        bc_set = None
        for trial in ('SET-2', 'Set-2', 'BCSet', 'ENCASTRESET'):
            if trial in odb.rootAssembly.nodeSets.keys():
                bc_set = odb.rootAssembly.nodeSets[trial]
                log('')
                log('  [BC set found]: %s' % trial)
                break

        if bc_set is not None and 'RF' in fr.fieldOutputs.keys():
            bc_labels = set()
            for inst_nodes in bc_set.nodes:
                for node in inst_nodes:
                    bc_labels.add(node.label)
            log('    BC node count:     %d' % len(bc_labels))

            RF = fr.fieldOutputs['RF']
            rf3_sum = 0.0
            rf3_all = []
            for v in RF.values:
                if v.nodeLabel in bc_labels:
                    rf = v.data[2] if len(v.data) >= 3 else 0.0
                    rf3_sum += rf
                    if abs(rf) > 1e-9:
                        rf3_all.append(rf)
            log('    Sum RF3 over BC:   %.4e N' % rf3_sum)
            log('    (should equal -Sum_CF3 by Newton balance)')
            if rf3_all:
                log('    RF3 max single:    %.4e N  (this matches CAE contour max)' % max(rf3_all))
                log('    RF3 min single:    %.4e N' % min(rf3_all))
            if F_total is not None and abs(F_total) > 1e-10:
                ratio = -rf3_sum / F_total
                log('    RF_sum / -CF_sum:  %.4f  (should be ~1.0)' % ratio)

        # K candidates
        log('')
        log('  [K = F / U candidates]')
        if F_total is not None and abs(F_total) > 1e-10:
            if sum(u3_abs)/n > 1e-12:
                K_avg = abs(F_total) / (sum(u3_abs)/n)
                log('    K_avg  = |F_total| / mean(|U3|) = %.2f N/mm   (extract.py current)' % K_avg)
            if max(u3_abs) > 1e-12:
                K_max = abs(F_total) / max(u3_abs)
                log('    K_max  = |F_total| / max (|U3|) = %.2f N/mm   (more physical)' % K_max)
            if max(Umag_list) > 1e-12:
                K_umag = abs(F_total) / max(Umag_list)
                log('    K_umag = |F_total| / max(|U|mag) = %.2f N/mm' % K_umag)

        # Per-node table
        log('')
        log('  [Per-node U3, sorted by |U3| descending, top 5 + bottom 5]')
        per_node_data.sort(key=lambda r: -abs(r[2]))
        head = '%-8s %-12s %-12s %-12s %-12s %-12s' % (
            'nodeLab', 'x(mm)', 'y(mm)', 'z(mm)', '|U3|(mm)', '|U|mag(mm)')
        log('    ' + head)
        for r in per_node_data[:5]:
            lab, (x,y,z), u3, um = r
            log('    %-8d %-12.3f %-12.3f %-12.3f %-12.4e %-12.4e' % (
                lab, x, y, z, abs(u3), um))
        log('    ...')
        for r in per_node_data[-5:]:
            lab, (x,y,z), u3, um = r
            log('    %-8d %-12.3f %-12.3f %-12.3f %-12.4e %-12.4e' % (
                lab, x, y, z, abs(u3), um))

        # Diagnosis
        log('')
        log('  [DIAGNOSIS]')
        ratio = max(u3_abs) / max((sum(u3_abs)/n), 1e-12)
        log('    max(|U3|) / mean(|U3|) = %.2f' % ratio)
        if ratio < 1.5:
            log('    -> Set-1 looks GOOD (uniform U3)')
        elif ratio < 5:
            log('    -> Set-1 is mixed (some high-U + some low-U nodes)')
        else:
            log('    -> Set-1 is HETEROGENEOUS (many low-U nodes pulling avg down)')

    finally:
        odb.close()


def diagnose_all():
    global _LOG_FH
    try:
        _LOG_FH = open(LOG_PATH, 'w')
    except Exception, e:
        print 'WARNING: cannot open log file %s: %s' % (LOG_PATH, str(e))
        _LOG_FH = None
    
    log('=' * 70)
    log('CFRC LoadPoint Diagnostic Output')
    log('Working dir: %s' % os.getcwd())
    log('BASE_DIR:    %s' % BASE_DIR)
    log('=' * 70)
    
    for cfg in CONFIGS:
        try:
            diagnose_one(cfg)
        except Exception, e:
            log('\n[%s] EXCEPTION: %s' % (cfg, str(e)))
    
    log('')
    log('=' * 70)
    log('DONE. Output saved to: %s' % LOG_PATH)
    log('=' * 70)
    
    if _LOG_FH is not None:
        _LOG_FH.close()
        _LOG_FH = None
    
    print ''
    print '################################################################'
    print '#'
    print '#  Diagnostic complete.  Open this file to see the results:'
    print '#'
    print '#     %s' % LOG_PATH
    print '#'
    print '################################################################'


if __name__ == '__main__':
    diagnose_all()
else:
    print 'diagnose_loadpoint.py loaded. Call:'
    print '  diagnose_all()                  -- all 4 configs'
    print '  diagnose_one("mine_stream")     -- single config'
    print 'Output goes to console AND %s' % LOG_PATH
