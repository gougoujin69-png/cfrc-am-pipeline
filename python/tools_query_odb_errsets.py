# -*- coding: utf-8 -*-
# tools_query_odb_errsets.py
# 诊断工具: 查询 (datacheck/失败) odb 里 Abaqus 自己存的 ErrElem* 单元集,
# 打印这些单元的"真实"编号 / 连接 / 节点坐标 / 段长.
# 用途: .dat 文本里的单元号可能与 inp 部件级编号存在错位 (Abaqus 内部
# 坍缩/重排), 静态解析 inp 找不到坏单元时, 以 odb 为准是唯一可靠手段.
#
# 用法:
#   1) 在失败作业目录 (或对某 inp 单独 datacheck: abaqus job=chk input=xx.inp
#      datacheck interactive) 得到 odb
#   2) abaqus python tools_query_odb_errsets.py <job.odb> [instance名, 默认 ALLBEAMS-1]
from odbAccess import openOdb
import math, sys

odb_path = sys.argv[1] if len(sys.argv) > 1 else 'psc.odb'
inst_name = sys.argv[2] if len(sys.argv) > 2 else 'ALLBEAMS-1'

odb = openOdb(odb_path, readOnly=True)
ra = odb.rootAssembly
print 'instances:', ra.instances.keys()

inst = ra.instances[inst_name]
ncoord = {}
for n in inst.nodes:
    ncoord[n.label] = tuple(n.coordinates)
econn = {}
for e in inst.elements:
    econn[e.label] = e.connectivity


def flatten(elems):
    out = []
    for e in elems:
        if hasattr(e, 'label'):
            out.append(e)
        else:
            for e2 in e:
                out.append(e2)
    return out


def dump_set(name, elset):
    labels = [e.label for e in flatten(elset.elements)]
    print '\n=== set %s : %d elements ===' % (name, len(labels))
    for lab in labels[:20]:
        conn = econn.get(lab)
        if conn and len(conn) >= 2:
            a, b = conn[0], conn[1]
            ca, cb = ncoord.get(a), ncoord.get(b)
            if ca and cb:
                d = math.sqrt(sum((ca[i]-cb[i])**2 for i in range(3)))
                print '  elem %d conn=%s  %s -> %s  len=%.8f' % (
                    lab, tuple(conn), ca, cb, d)
            else:
                print '  elem %d conn=%s (missing coords)' % (lab, tuple(conn))
        else:
            print '  elem %d conn=%s' % (lab, tuple(conn) if conn else '?')


for src, holder in [('instance', inst), ('assembly', ra)]:
    try:
        keys = holder.elementSets.keys()
    except:
        keys = []
    for k in keys:
        if 'ERR' in k.upper():
            dump_set('%s.%s' % (src, k), holder.elementSets[k])
odb.close()
print '\nDONE'
