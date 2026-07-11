# -*- coding: utf-8 -*-
# grind_planar_stream.py  (run: abaqus cae noGUI=grind_planar_stream.py)
# Serial, single-process auto-retry grind for planar_stream: no interactive CAE,
# no overlapping attempts, so the .dat always matches the .inp (no retry race).
# auto_diagnose accumulates the blacklist round by round (the proven mechanism
# that got the 0.4mm planar_stream through). max_retries default is 80.
from abaqus import *
from abaqusConstants import *
import os, sys

execfile('abaqus_cfrc_compare.py')

print '================ GRIND START: planar_stream (serial auto-retry) ================'
sys.__stdout__.write('GRIND START\n'); sys.__stdout__.flush()

results = run_with_auto_retry(['planar_stream'])

print 'RESULTS:', results
try:
    dump_blacklist()
except Exception, e:
    print 'dump_blacklist failed:', e

try:
    st, odb, msg = get_config_status('planar_stream')
    print 'FINAL planar_stream: status=%s  msg=%s  odb=%s' % (st, msg, odb)
    open('grind_result.txt', 'w').write('status=%s\nmsg=%s\nodb=%s\nblacklist=%s\n' % (
        st, msg, odb, Config.BLACKLIST_PATH_IDX.get('planar_stream', [])))
except Exception, e:
    print 'status check failed:', e

print '================ GRIND DONE ================'
