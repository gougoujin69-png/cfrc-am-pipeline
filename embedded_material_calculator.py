#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
embedded_material_calculator.py

Convert printed-path geometry (line width w, layer height t) plus the
measured homogenized 3D-printed CFRC properties (Table 3.1) into the
embedded-element material parameters used by abaqus_cfrc_compare.py
(isotropic host solid + embedded B31 beams).

Theory: Abaqus embedded elements SUPERPOSE stiffness (host stiffness is
NOT removed where beams exist). Along the fiber direction, for the
material cross-section represented by one printed path
(A_cell = w * t), the calibration equation is:

    E11_meas * A_cell = E_host * A_cell + E_beam * A_beam
    =>  E_beam = (E11_meas - E_host) * A_cell / A_beam

The host must reproduce the matrix/interface-dominated block of the
measured table on its own (E22/E33 and the shear moduli), because the
beams contribute nothing transversely:

    E_host  = E22            (or (E22+E33)/2 with --host-avg)
    nu_host = nu23           (check: G_iso = E/(2(1+nu)) must fall
                              inside the measured G band)

Compatible with Python 2.7 (including 'abaqus python') and Python 3.
Source is ASCII-only on purpose.

Quick start:
    python embedded_material_calculator.py -w 0.8 -t 0.25
"""

from __future__ import print_function
import argparse
import math
import sys


# ---------------------------------------------------------------------------
# Measured 3D-printed CFRC properties (Table 3.1), MPa / dimensionless.
# Override any of them from the command line if the table changes.
# ---------------------------------------------------------------------------
DEFAULTS = {
    'e11': 42360.2,
    'e22': 2012.9,
    'e33': 2146.1,
    'g12': 698.2,
    'g13': 758.4,
    'g23': 658.1,
    'nu12': 0.29,
    'nu13': 0.287,
    'nu23': 0.39,
}

# Current beam profile in abaqus_cfrc_compare.py (full axes, mm)
DEFAULT_BEAM_MAJOR = 0.6
DEFAULT_BEAM_MINOR = 0.15

# Reference raw-fiber longitudinal modulus (T300-class), MPa.
# Only used for the informational Vf estimate and the 'fiber' convention.
DEFAULT_EF = 230000.0


def ellipse_profile(major, minor):
    """Section properties of a full ellipse given FULL axes (mm)."""
    a = major / 2.0
    b = minor / 2.0
    area = math.pi * a * b
    i11 = math.pi * a * (b ** 3) / 4.0
    i22 = math.pi * b * (a ** 3) / 4.0
    j = math.pi * (a ** 3) * (b ** 3) / (a ** 2 + b ** 2)
    return area, i11, i22, j


def rect_profile(w, t):
    """Section properties of a w x t rectangle (w = width, t = height)."""
    area = w * t
    i11 = w * (t ** 3) / 12.0   # bending about the in-plane (width) axis
    i22 = t * (w ** 3) / 12.0
    a = max(w, t) / 1.0
    b = min(w, t) / 1.0
    # Roark's approximation for the torsion constant of a solid rectangle
    j = a * (b ** 3) * (1.0 / 3.0 - 0.21 * (b / a) *
                        (1.0 - (b ** 4) / (12.0 * (a ** 4))))
    return area, i11, i22, j


def parse_args(argv):
    p = argparse.ArgumentParser(
        description='Path geometry -> embedded-element material calculator '
                    'for the CFRC AM pipeline.')
    p.add_argument('-w', '--width', type=float, required=True,
                   help='printed line width / in-plane path spacing, mm')
    p.add_argument('-t', '--height', type=float, required=True,
                   help='layer height / inter-layer spacing, mm')

    g = p.add_argument_group('measured composite properties (MPa / -)')
    for k, v in sorted(DEFAULTS.items()):
        g.add_argument('--' + k, type=float, default=v,
                       help='default %g' % v)

    b = p.add_argument_group('beam profile / convention')
    b.add_argument('--beam-major', type=float, default=DEFAULT_BEAM_MAJOR,
                   help='ellipse full major axis, mm (default %g)'
                        % DEFAULT_BEAM_MAJOR)
    b.add_argument('--beam-minor', type=float, default=DEFAULT_BEAM_MINOR,
                   help='ellipse full minor axis, mm (default %g)'
                        % DEFAULT_BEAM_MINOR)
    b.add_argument('--convention', choices=['ellipse', 'cell', 'fiber'],
                   default='ellipse',
                   help="'ellipse' = keep current elliptical profile "
                        "(default, no code change needed); "
                        "'cell' = beam area equals A_cell = w*t; "
                        "'fiber' = beam represents raw fiber only "
                        "(area = Vf*A_cell, needs --ef)")
    b.add_argument('--ef', type=float, default=DEFAULT_EF,
                   help='raw fiber longitudinal modulus for Vf estimate '
                        'and fiber convention, MPa (default %g)' % DEFAULT_EF)

    h = p.add_argument_group('host overrides')
    h.add_argument('--host-avg', action='store_true',
                   help='use (E22+E33)/2 instead of E22 for HOST_E')
    h.add_argument('--host-e', type=float, default=None,
                   help='force HOST_E, MPa (overrides table)')
    h.add_argument('--host-nu', type=float, default=None,
                   help='force HOST_NU (overrides nu23)')
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])

    w, t = args.width, args.height
    if w <= 0.0 or t <= 0.0:
        print('ERROR: width and height must be positive.')
        return 1

    a_cell = w * t

    # ---------------- host ----------------
    if args.host_e is not None:
        e_host = args.host_e
        host_src = 'user override'
    elif args.host_avg:
        e_host = 0.5 * (args.e22 + args.e33)
        host_src = '(E22+E33)/2'
    else:
        e_host = args.e22
        host_src = 'E22'
    nu_host = args.host_nu if args.host_nu is not None else args.nu23
    g_host = e_host / (2.0 * (1.0 + nu_host))

    g_meas = [args.g12, args.g13, args.g23]
    g_lo, g_hi = min(g_meas), max(g_meas)
    g_mean = sum(g_meas) / 3.0

    # ---------------- Vf estimate (informational) ----------------
    vf = (args.e11 - e_host) / (args.ef - e_host)

    # ---------------- beam ----------------
    conv = args.convention
    if conv == 'ellipse':
        a_beam, i11, i22, jj = ellipse_profile(args.beam_major,
                                               args.beam_minor)
        prof_note = ('elliptical profile %.3g x %.3g mm '
                     '(matches current Config, no code change)'
                     % (args.beam_major, args.beam_minor))
    elif conv == 'cell':
        a_beam, i11, i22, jj = rect_profile(w, t)
        prof_note = ('rectangular cell profile %.3g x %.3g mm '
                     '(update GeneralizedProfile: area/i11/i22/j below)'
                     % (w, t))
    else:  # fiber
        a_beam = vf * a_cell
        r_eq = math.sqrt(a_beam / math.pi)
        i11 = i22 = math.pi * (r_eq ** 4) / 4.0
        jj = 2.0 * i11
        prof_note = ('fiber-only circular profile, equivalent radius '
                     '%.4g mm (area = Vf*A_cell)' % r_eq)

    e_beam = (args.e11 - e_host) * a_cell / a_beam
    ratio = e_beam / e_host

    # verification: reconstruct E11
    e11_check = e_host + e_beam * a_beam / a_cell

    # ---------------- report ----------------
    line = '-' * 68
    print(line)
    print('EMBEDDED-ELEMENT MATERIAL CALCULATOR  (CFRC AM pipeline)')
    print(line)
    print('INPUT')
    print('  line width  w        = %10.4f mm' % w)
    print('  layer height t       = %10.4f mm' % t)
    print('  A_cell = w*t         = %10.6f mm^2' % a_cell)
    print('  convention           = %s' % conv)
    print('  beam profile         : %s' % prof_note)
    print('  A_beam               = %10.6f mm^2' % a_beam)
    print('')
    print('HOST (isotropic, transverse/shear behavior)')
    print('  HOST_E               = %10.2f MPa   (%s)' % (e_host, host_src))
    print('  HOST_NU              = %10.4f       (nu23)' % nu_host)
    print('  implied G_host       = %10.2f MPa' % g_host)
    print('  measured G band      = [%.1f .. %.1f], mean %.1f MPa'
          % (g_lo, g_hi, g_mean))
    print('')
    print('BEAM (isotropic, only E matters for B31 axial response)')
    print('  E_beam = (E11-E_host)*A_cell/A_beam')
    print('         = %10.2f MPa' % e_beam)
    print('  BEAM_E_RATIO         = %10.4f  (= E_beam / HOST_E)' % ratio)
    print('  profile area/i11/i22/j = %.6g / %.6g / %.6g / %.6g'
          % (a_beam, i11, i22, jj))
    print('')
    print('CHECKS')
    print('  reconstructed E11    = %10.2f MPa (target %.1f)  %s'
          % (e11_check, args.e11,
             'OK' if abs(e11_check - args.e11) < 1e-6 * args.e11
             else 'MISMATCH'))
    print('  Vf estimate (Ef=%.0f) = %8.4f  (%.1f %%)'
          % (args.ef, vf, 100.0 * vf))

    warns = []
    if not (g_lo <= g_host <= g_hi):
        warns.append('implied host shear G=%.1f MPa is outside the '
                     'measured band [%.1f, %.1f]; consider tuning '
                     '--host-nu (nu = E/(2G)-1 = %.3f matches the mean).'
                     % (g_host, g_lo, g_hi, e_host / (2.0 * g_mean) - 1.0))
    if e_beam > args.ef:
        warns.append('E_beam exceeds the raw fiber modulus Ef=%.0f MPa. '
                     'Beam area is too small for this cell; check w, t '
                     'and the beam profile.' % args.ef)
    if a_beam > a_cell:
        warns.append('A_beam > A_cell: beam volume exceeds the material '
                     'volume it represents; geometry inconsistent.')
    if not (0.05 <= vf <= 0.6):
        warns.append('Vf estimate %.3f is outside the plausible printed-'
                     'CFRC range (0.05..0.6); verify E11/E22 inputs.' % vf)
    if warns:
        for msg in warns:
            print('  WARNING: %s' % msg)
    else:
        print('  no warnings')

    print('')
    print('PASTE INTO abaqus_cfrc_compare.py  class Config')
    print(line)
    print('    HOST_E = %.1f' % e_host)
    print('    HOST_NU = %.2f' % nu_host)
    print('    BEAM_E_RATIO = %.3f' % ratio)
    if conv == 'ellipse':
        print('    BEAM_MAJOR_AXIS = %g' % args.beam_major)
        print('    BEAM_MINOR_AXIS = %g' % args.beam_minor)
    else:
        print('    # NOTE: convention %r does not use the elliptical'
              % conv)
        print('    # profile. Update setup_beam_material_profile_section')
        print('    # with area/i11/i22/j printed above.')
    print(line)
    print('REMINDER: A_cell must match the spacing of the beam paths')
    print('actually exported to the FEA model (in-plane spacing = w,')
    print('layer spacing = t). If the FEA paths were generated with a')
    print('different LINE_WIDTH, rerun this tool with that value.')
    print(line)
    return 0


if __name__ == '__main__':
    sys.exit(main())
