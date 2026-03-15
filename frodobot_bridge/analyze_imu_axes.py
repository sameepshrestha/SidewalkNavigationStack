"""
Advanced IMU Axis Calibration for Earth Rover (FrodoBots)
=========================================================
Determines the IMU XYZ → Robot Body Frame (X=fwd, Y=left, Z=up) mapping
using 6 cross-validated methods.
"""

import math
import os

ARROW_FILE = "/mnt/windows_storage/datasets/frodobots_dataset/train/data-00000-of-00001.arrow"
SEPARATOR = "=" * 70


def read_data():
    import pyarrow as pa
    with open(ARROW_FILE, 'rb') as f:
        reader = pa.ipc.open_stream(f)
        table = reader.read_all()
    print(f"  Columns: {table.column_names}")
    print(f"  Rows: {len(table)}")
    return {
        'acc': table.column('observation.accelerometer').to_pylist(),
        'gyro': table.column('observation.gyroscope').to_pylist(),
        'mag': table.column('observation.magnetometer').to_pylist(),
        'mag_f': table.column('observation.magnetometer_filtered').to_pylist(),
        'ep': table.column('episode_index').to_pylist(),
        'fi': table.column('frame_index').to_pylist(),
        'act': table.column('action').to_pylist(),
        'rpm': table.column('observation.wheel_rpm').to_pylist(),
    }


# ── Vector math helpers ─────────────────────────────────────────────────
def vmag(v):   return math.sqrt(sum(x*x for x in v))
def vmean(vs): return [sum(v[i] for v in vs)/len(vs) for i in range(len(vs[0]))]
def vstd(vs, m): return [math.sqrt(sum((v[i]-m[i])**2 for v in vs)/max(1,len(vs)-1)) for i in range(len(m))]
def vdot(a,b): return sum(x*y for x,y in zip(a,b))
def vnorm(v):
    m = vmag(v)
    return [x/m for x in v] if m > 1e-10 else v
def vcross(a,b): return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]
AX = ['X_imu', 'Y_imu', 'Z_imu']


def pearson(xs, ys):
    """Pearson correlation between two lists of scalars."""
    n = len(xs)
    if n < 3: return 0.0
    mx = sum(xs)/n
    my = sum(ys)/n
    num = sum((x-mx)*(y-my) for x,y in zip(xs,ys))
    dx = math.sqrt(sum((x-mx)**2 for x in xs))
    dy = math.sqrt(sum((y-my)**2 for y in ys))
    if dx < 1e-10 or dy < 1e-10: return 0.0
    return num / (dx * dy)


# ── PCA (power iteration, no numpy needed) ──────────────────────────────
def pca_dominant(vecs):
    """Find the dominant principal component direction via power iteration."""
    n = len(vecs)
    dim = len(vecs[0])
    m = vmean(vecs)
    centered = [[v[i]-m[i] for i in range(dim)] for v in vecs]

    # Covariance matrix
    cov = [[0.0]*dim for _ in range(dim)]
    for v in centered:
        for i in range(dim):
            for j in range(dim):
                cov[i][j] += v[i]*v[j]
    for i in range(dim):
        for j in range(dim):
            cov[i][j] /= (n-1)

    # Power iteration
    ev = [1.0, 0.0, 0.0]
    for _ in range(200):
        nv = [sum(cov[i][j]*ev[j] for j in range(dim)) for i in range(dim)]
        m2 = vmag(nv)
        if m2 < 1e-15: break
        ev = [x/m2 for x in nv]

    # Eigenvalue
    Av = [sum(cov[i][j]*ev[j] for j in range(dim)) for i in range(dim)]
    eigenval = vdot(Av, ev)

    # Total variance
    total_var = sum(cov[i][i] for i in range(dim))

    return ev, eigenval, total_var, cov


def analyze():
    print(SEPARATOR)
    print("ADVANCED IMU AXIS CALIBRATION FOR EARTH ROVER")
    print(SEPARATOR)

    print("\nReading data...")
    d = read_data()
    N = len(d['acc'])

    # Group by episode
    episodes = {}
    for i in range(N):
        ep = d['ep'][i]
        if ep not in episodes:
            episodes[ep] = []
        episodes[ep].append(i)
    print(f"  Episodes: {sorted(episodes.keys())} ({len(episodes)} total)")

    # ═══════════════════════════════════════════════════════════════════
    # METHOD 1: Per-Episode Windowed Stationary Detection
    # ═══════════════════════════════════════════════════════════════════
    print(f"\n{SEPARATOR}")
    print("METHOD 1: PER-EPISODE WINDOWED STATIONARY DETECTION")
    print(SEPARATOR)

    WINDOW = 20
    best_windows = []  # (episode, start_idx, variance, acc_mean, gyro_mean)

    for ep_id in sorted(episodes.keys()):
        idxs = episodes[ep_id]

        # Find stationary regions: RPM ≈ 0
        stationary_runs = []
        run_start = None
        for pos, i in enumerate(idxs):
            rpm_sum = sum(abs(x) for x in d['rpm'][i])
            if rpm_sum < 2.0:
                if run_start is None:
                    run_start = pos
            else:
                if run_start is not None and (pos - run_start) >= WINDOW:
                    stationary_runs.append((run_start, pos))
                run_start = None
        if run_start is not None and (len(idxs) - run_start) >= WINDOW:
            stationary_runs.append((run_start, len(idxs)))

        if not stationary_runs:
            continue

        # Find the most stable window within stationary runs
        best_var = float('inf')
        best_w = None
        for rs, re in stationary_runs:
            for ws in range(rs, re - WINDOW + 1):
                window_idxs = [idxs[ws + j] for j in range(WINDOW)]
                window_acc = [d['acc'][i] for i in window_idxs]
                wa_mean = vmean(window_acc)
                wa_std = vstd(window_acc, wa_mean)
                var = sum(s**2 for s in wa_std)  # total variance
                if var < best_var:
                    best_var = var
                    window_gyro = [d['gyro'][i] for i in window_idxs]
                    wg_mean = vmean(window_gyro)
                    best_w = (ep_id, ws, var, wa_mean, wg_mean, vmag(wa_mean))

        if best_w:
            best_windows.append(best_w)

    print(f"  Found stable windows in {len(best_windows)} / {len(episodes)} episodes")
    print(f"\n  Per-episode gravity vectors (accel mean during best stationary window):")
    print(f"  NOTE: Accelerometer measures REACTION force. Reading = UP direction.")
    for ep, ws, var, am, gm, amag in sorted(best_windows, key=lambda x: x[2]):
        gdir = vnorm(am)
        print(f"    ep={ep:2d}: acc=[{am[0]:7.4f}, {am[1]:7.4f}, {am[2]:7.4f}] |{amag:.4f}|g  "
              f"dir=[{gdir[0]:6.3f}, {gdir[1]:6.3f}, {gdir[2]:6.3f}]  var={var:.6f}  "
              f"gyro_bias=[{gm[0]:6.2f}, {gm[1]:6.2f}, {gm[2]:6.2f}]")

    # Take the top N lowest-variance windows
    TOP_N = min(10, len(best_windows))
    best_windows.sort(key=lambda x: x[2])
    top_acc_vecs = [w[3] for w in best_windows[:TOP_N]]
    top_gyro_vecs = [w[4] for w in best_windows[:TOP_N]]

    if TOP_N >= 2:
        # SIGN-ALIGNED GRAVITY CONSENSUS
        # Some episodes may have flipped IMU sign. Align all to the best (lowest variance).
        reference = vnorm(top_acc_vecs[0])  # best window as reference
        aligned_acc = []
        aligned_gyro = []
        for i, av in enumerate(top_acc_vecs):
            av_dir = vnorm(av)
            # Check if this vector agrees or disagrees with reference
            cos_sim = vdot(av_dir, reference)
            if cos_sim < 0:  # flipped sign
                aligned_acc.append([-x for x in av])
                aligned_gyro.append([-x for x in top_gyro_vecs[i]])  # gyro also flips
                print(f"    (ep {best_windows[i][0]}: FLIPPED to align with reference)")
            else:
                aligned_acc.append(av)
                aligned_gyro.append(top_gyro_vecs[i])

        # Weight by closeness to 1.0g (proper gravity magnitude)
        weights = []
        for av in aligned_acc:
            m = vmag(av)
            w = 1.0 / (1.0 + abs(m - 1.0) * 10)  # higher weight if |acc| ≈ 1g
            weights.append(w)
        total_w = sum(weights)
        consensus_gravity = [sum(weights[i]*aligned_acc[i][j] for i in range(len(aligned_acc)))/total_w for j in range(3)]
        consensus_gravity_mag = vmag(consensus_gravity)
        consensus_gravity_dir = vnorm(consensus_gravity)
        gyro_bias = [sum(weights[i]*aligned_gyro[i][j] for i in range(len(aligned_gyro)))/total_w for j in range(3)]

        print(f"\n  ★ CONSENSUS GRAVITY (sign-aligned, magnitude-weighted, top {TOP_N}):")
        print(f"    Mean accel: [{consensus_gravity[0]:.4f}, {consensus_gravity[1]:.4f}, {consensus_gravity[2]:.4f}]")
        print(f"    Magnitude: {consensus_gravity_mag:.4f} g (should be ≈1.0)")
        print(f"    Direction: [{consensus_gravity_dir[0]:.4f}, {consensus_gravity_dir[1]:.4f}, {consensus_gravity_dir[2]:.4f}]")
        print(f"    (This IS the 'up' direction since accel measures reaction force)")

        abs_g = [abs(x) for x in consensus_gravity_dir]
        grav_ax = abs_g.index(max(abs_g))
        grav_sign = '+' if consensus_gravity_dir[grav_ax] > 0 else '-'
        sorted_abs_g = sorted(abs_g, reverse=True)
        grav_ratio = sorted_abs_g[0] / (sorted_abs_g[1] + 1e-10)
        print(f"    Dominant axis: {grav_sign}{AX[grav_ax]} (ratio: {grav_ratio:.1f}x)")
        if grav_ratio < 3:
            print(f"    ⚠ Gravity is NOT cleanly along one axis — IMU is tilted ~45°")
            for ax in range(3):
                angle = math.degrees(math.acos(min(1, abs_g[ax])))
                print(f"      {AX[ax]}: {90-angle:.1f}° from gravity")

        print(f"\n    ★ GYRO BIAS (stationary): [{gyro_bias[0]:.4f}, {gyro_bias[1]:.4f}, {gyro_bias[2]:.4f}]")
    else:
        print("  ⚠ Not enough stable windows! Falling back to global mean.")
        consensus_gravity_dir = None
        gyro_bias = [0, 0, 0]

    # ═══════════════════════════════════════════════════════════════════
    # METHOD 2: Gyro Unit Verification
    # ═══════════════════════════════════════════════════════════════════
    print(f"\n{SEPARATOR}")
    print("METHOD 2: GYRO UNIT VERIFICATION")
    print(SEPARATOR)

    gb_mag = vmag(gyro_bias)
    print(f"  Gyro bias magnitude: {gb_mag:.4f}")
    print(f"  If deg/s: bias = {gb_mag:.2f}°/s (typical for consumer IMU: 0.5-5°/s)")
    print(f"  If rad/s: bias = {math.degrees(gb_mag):.2f}°/s (unusually large)")

    # Check: during the best stationary window, what's the gyro std?
    if TOP_N >= 1:
        best_ep_idxs = episodes[best_windows[0][0]]
        ws = best_windows[0][1]
        best_stat_gyro = [d['gyro'][best_ep_idxs[ws+j]] for j in range(WINDOW)]
        sg_std = vstd(best_stat_gyro, vmean(best_stat_gyro))
        print(f"\n  Gyro std in best stationary window: [{sg_std[0]:.4f}, {sg_std[1]:.4f}, {sg_std[2]:.4f}]")
        sg_std_mag = vmag(sg_std)
        print(f"  If deg/s: noise = {sg_std_mag:.3f}°/s  (BNO055 typical: 0.2-1°/s)")
        print(f"  If rad/s: noise = {math.degrees(sg_std_mag):.3f}°/s  (very high)")

    if gb_mag < 0.2:
        gyro_unit = "rad/s"
        gyro_scale = 1.0
    else:
        gyro_unit = "deg/s"
        gyro_scale = math.pi / 180.0

    print(f"\n  ★ CONCLUSION: Gyroscope is in {gyro_unit}")

    # ═══════════════════════════════════════════════════════════════════
    # METHOD 3: Action-IMU Correlation
    # ═══════════════════════════════════════════════════════════════════
    print(f"\n{SEPARATOR}")
    print("METHOD 3: ACTION-IMU CORRELATION (Pearson)")
    print(SEPARATOR)

    # Bias-corrected gyro
    gyro_bc = [[d['gyro'][i][j] - gyro_bias[j] for j in range(3)] for i in range(N)]

    # Extract action components
    act_linear = [d['act'][i][0] for i in range(N)]
    act_angular = [d['act'][i][1] for i in range(N)]

    # Correlate angular command with each gyro axis
    print("  Correlation: action[1] (angular) vs gyro axes (bias-corrected):")
    gyro_angular_corr = []
    for ax in range(3):
        gyro_ax_vals = [gyro_bc[i][ax] for i in range(N)]
        r = pearson(act_angular, gyro_ax_vals)
        gyro_angular_corr.append(r)
        print(f"    {AX[ax]}: r = {r:+.4f}")

    abs_corr = [abs(r) for r in gyro_angular_corr]
    yaw_ax = abs_corr.index(max(abs_corr))
    yaw_sign = '+' if gyro_angular_corr[yaw_ax] > 0 else '-'
    print(f"\n  ★ YAW AXIS: {yaw_sign}{AX[yaw_ax]} (|r| = {abs_corr[yaw_ax]:.4f})")
    print(f"    In ROS, +angular command = CCW = +Z_body rotation")
    if gyro_angular_corr[yaw_ax] > 0:
        print(f"    → +{AX[yaw_ax]}_imu = +Z_body (yaw)")
    else:
        print(f"    → -{AX[yaw_ax]}_imu = +Z_body (yaw)")

    # Correlate linear command with each accel axis
    print(f"\n  Correlation: action[0] (linear) vs accel axes:")
    acc_linear_corr = []
    for ax in range(3):
        acc_ax_vals = [d['acc'][i][ax] for i in range(N)]
        r = pearson(act_linear, acc_ax_vals)
        acc_linear_corr.append(r)
        print(f"    {AX[ax]}: r = {r:+.4f}")

    abs_acc_corr = [abs(r) for r in acc_linear_corr]
    fwd_ax = abs_acc_corr.index(max(abs_acc_corr))
    # Note: forward DRIVING creates BACKWARD tilt (accel points backward + gravity)
    # This means the correlation may be NEGATIVE (driving forward = accel backward)
    # OR there's also the effect of slope. Need to be careful.
    print(f"\n  Highest |r| on {AX[fwd_ax]}: r = {acc_linear_corr[fwd_ax]:+.4f}")
    print(f"  (Note: forward driving on flat ground: centripetal effects dominate)")

    # Correlate angular command with each accel axis (centripetal accel during turns)
    print(f"\n  Correlation: action[1] (angular) vs accel axes:")
    acc_angular_corr = []
    for ax in range(3):
        acc_ax_vals = [d['acc'][i][ax] for i in range(N)]
        r = pearson(act_angular, acc_ax_vals)
        acc_angular_corr.append(r)
        print(f"    {AX[ax]}: r = {r:+.4f}")

    # ═══════════════════════════════════════════════════════════════════
    # METHOD 4: PCA on Stationary Accelerometer
    # ═══════════════════════════════════════════════════════════════════
    print(f"\n{SEPARATOR}")
    print("METHOD 4: PCA ON STATIONARY ACCELEROMETER")
    print(SEPARATOR)

    # Collect all stationary accel samples
    all_stat_acc = []
    for w in best_windows[:TOP_N]:
        ep_idxs = episodes[w[0]]
        ws = w[1]
        for j in range(WINDOW):
            all_stat_acc.append(d['acc'][ep_idxs[ws+j]])

    if len(all_stat_acc) > 10:
        pc1, eigval1, total_var, cov = pca_dominant(all_stat_acc)
        var_explained = eigval1 / total_var * 100 if total_var > 0 else 0

        print(f"  Stationary samples used: {len(all_stat_acc)}")
        print(f"  Covariance matrix:")
        for i in range(3):
            print(f"    [{cov[i][0]:10.6f}, {cov[i][1]:10.6f}, {cov[i][2]:10.6f}]")
        print(f"\n  PC1 direction: [{pc1[0]:.4f}, {pc1[1]:.4f}, {pc1[2]:.4f}]")
        print(f"  Eigenvalue: {eigval1:.6f}")
        print(f"  Variance explained: {var_explained:.1f}%")

        # PC1 of stationary accel IS the gravity direction (highest variance axis)
        # But wait: if the IMU is stable, all variance is noise → PC1 is noise direction
        # We want the MEAN direction, not the variance direction
        # PCA on raw (not centered) data would give gravity as PC1
        # Let's also do uncentered PCA
        print(f"\n  (Note: For stable stationary data, PCA finds the noise direction,")
        print(f"   not gravity. Using the MEAN of stationary data is more reliable.)")
        print(f"   Gravity from mean: [{consensus_gravity_dir[0]:.4f}, {consensus_gravity_dir[1]:.4f}, {consensus_gravity_dir[2]:.4f}]")

        # UNCENTERED PCA: dominant direction of raw vectors ≈ gravity
        # Compute X^T X directly
        raw_cov = [[0.0]*3 for _ in range(3)]
        for v in all_stat_acc:
            for i in range(3):
                for j in range(3):
                    raw_cov[i][j] += v[i]*v[j]
        for i in range(3):
            for j in range(3):
                raw_cov[i][j] /= len(all_stat_acc)

        # Power iteration on raw covariance
        ev = [1.0, 0.0, 0.0]
        for _ in range(200):
            nv = [sum(raw_cov[i][j]*ev[j] for j in range(3)) for i in range(3)]
            m2 = vmag(nv)
            if m2 < 1e-15: break
            ev = [x/m2 for x in nv]

        print(f"\n  ★ UNCENTERED PCA gravity direction: [{ev[0]:.4f}, {ev[1]:.4f}, {ev[2]:.4f}]")
        abs_ev = [abs(x) for x in ev]
        pca_grav_ax = abs_ev.index(max(abs_ev))
        pca_grav_sign = '+' if ev[pca_grav_ax] > 0 else '-'
        print(f"    Dominant axis: {pca_grav_sign}{AX[pca_grav_ax]}")

    # ═══════════════════════════════════════════════════════════════════
    # METHOD 5: Magnetometer Dip Angle
    # ═══════════════════════════════════════════════════════════════════
    print(f"\n{SEPARATOR}")
    print("METHOD 5: MAGNETOMETER DIP ANGLE CROSS-CHECK")
    print(SEPARATOR)

    # Use filtered magnetometer for less noise
    all_stat_mag_f = []
    for w in best_windows[:TOP_N]:
        ep_idxs = episodes[w[0]]
        ws = w[1]
        for j in range(WINDOW):
            all_stat_mag_f.append(d['mag_f'][ep_idxs[ws+j]])

    if len(all_stat_mag_f) > 5 and consensus_gravity_dir:
        mag_f_mean = vmean(all_stat_mag_f)
        mag_f_mag = vmag(mag_f_mean)
        mag_f_dir = vnorm(mag_f_mean)

        print(f"  Filtered mag mean: [{mag_f_mean[0]:.2f}, {mag_f_mean[1]:.2f}, {mag_f_mean[2]:.2f}]")
        print(f"  Magnitude: {mag_f_mag:.2f}")
        print(f"  Direction:  [{mag_f_dir[0]:.4f}, {mag_f_dir[1]:.4f}, {mag_f_dir[2]:.4f}]")

        # Also raw mag
        all_stat_mag_raw = []
        for w in best_windows[:TOP_N]:
            ep_idxs = episodes[w[0]]
            ws = w[1]
            for j in range(WINDOW):
                all_stat_mag_raw.append(d['mag'][ep_idxs[ws+j]])
        mag_raw_mean = vmean(all_stat_mag_raw)
        mag_raw_dir = vnorm(mag_raw_mean)
        print(f"\n  Raw mag mean: [{mag_raw_mean[0]:.2f}, {mag_raw_mean[1]:.2f}, {mag_raw_mean[2]:.2f}]")
        print(f"  Direction:    [{mag_raw_dir[0]:.4f}, {mag_raw_dir[1]:.4f}, {mag_raw_dir[2]:.4f}]")

        # Dip angle: angle between magnetic field and horizontal plane
        # horizontal plane is perpendicular to gravity
        cos_grav_mag = vdot(mag_f_dir, consensus_gravity_dir)
        angle_from_grav = math.degrees(math.acos(min(1, max(-1, abs(cos_grav_mag)))))
        dip_angle = 90 - angle_from_grav  # angle below horizontal

        print(f"\n  Angle between filtered mag and gravity: {angle_from_grav:.1f}°")
        print(f"  ★ Magnetic dip angle: {dip_angle:.1f}° below horizontal")
        print(f"    Expected for Berkeley/SF Bay Area: ~61°")
        print(f"    Expected range for continental US: 55-75°")

        if 45 < dip_angle < 80:
            print(f"    ✓ Dip angle is physically plausible!")
        else:
            print(f"    ⚠ Dip angle seems off. Check gravity direction.")

        # Cross-check with raw mag
        cos_grav_raw = vdot(mag_raw_dir, consensus_gravity_dir)
        dip_raw = 90 - math.degrees(math.acos(min(1, max(-1, abs(cos_grav_raw)))))
        print(f"    Raw mag dip angle: {dip_raw:.1f}°")

    # ═══════════════════════════════════════════════════════════════════
    # METHOD 6: Consistency Validation & Final Mapping
    # ═══════════════════════════════════════════════════════════════════
    print(f"\n{SEPARATOR}")
    print("METHOD 6: CONSISTENCY VALIDATION & FINAL MAPPING")
    print(SEPARATOR)

    # Gravity axis (from Method 1)
    # Accel measures reaction force: reading = UP direction
    if consensus_gravity_dir:
        abs_g = [abs(x) for x in consensus_gravity_dir]
        grav_ax = abs_g.index(max(abs_g))
        grav_positive = consensus_gravity_dir[grav_ax] > 0
        sorted_abs_g = sorted(abs_g, reverse=True)
        grav_ratio = sorted_abs_g[0] / (sorted_abs_g[1] + 1e-10)
        print(f"\n  Gravity analysis (accel = reaction force = UP direction):")
        print(f"    Accel points along: {'+'if grav_positive else '-'}{AX[grav_ax]}")
        print(f"    → {'+'if grav_positive else '-'}{AX[grav_ax]} = +Z_body (upward)")
        print(f"    Confidence ratio: {grav_ratio:.1f}x")

    # Yaw axis (from Method 3)
    yaw_positive = gyro_angular_corr[yaw_ax] > 0
    print(f"\n  Yaw analysis:")
    print(f"    Yaw rotation axis: {'+'if yaw_positive else '-'}{AX[yaw_ax]}")
    print(f"    → {'+'if yaw_positive else '-'}{AX[yaw_ax]} = +Z_body (CCW positive)")

    # CHECK 1: Gravity axis should == Yaw axis (both define Z_body)
    grav_body_z = (grav_ax, grav_positive)
    yaw_body_z = (yaw_ax, yaw_positive)
    print(f"\n  CHECK 1: Gravity axis == Yaw axis?")
    if grav_body_z == yaw_body_z:
        print(f"    ✓ YES! Both agree: {'+'if grav_positive else '-'}{AX[grav_ax]} = +Z_body")
        z_ax = grav_ax
        z_pos = grav_positive
    elif grav_ax == yaw_ax and grav_positive != yaw_positive:
        print(f"    ✗ SAME AXIS but OPPOSITE SIGN!")
        print(f"      Gravity says {'+'if grav_positive else '-'}{AX[grav_ax]}")
        print(f"      Yaw says {'+'if yaw_positive else '-'}{AX[yaw_ax]}")
        print(f"    Gravity ratio: {grav_ratio:.1f}x, Yaw |r|: {abs_corr[yaw_ax]:.4f}")
        if abs_corr[yaw_ax] > 0.3 and grav_ratio < 2.0:
            print(f"    → TRUSTING YAW (cleaner signal)")
            z_ax = yaw_ax; z_pos = yaw_positive
        else:
            print(f"    → TRUSTING GRAVITY (stronger ratio)")
            z_ax = grav_ax; z_pos = grav_positive
    else:
        print(f"    ✗ DIFFERENT AXES! Gravity: {'+'if grav_positive else '-'}{AX[grav_ax]}, Yaw: {'+'if yaw_positive else '-'}{AX[yaw_ax]}")
        print(f"    Gravity ratio: {grav_ratio:.1f}x, Yaw |r|: {abs_corr[yaw_ax]:.4f}")
        if abs_corr[yaw_ax] > 0.3 and grav_ratio < 2.0:
            print(f"    → TRUSTING YAW (cleaner signal, gravity is ambiguous)")
            z_ax = yaw_ax; z_pos = yaw_positive
        elif grav_ratio > 3.0:
            print(f"    → TRUSTING GRAVITY (very clear dominant axis)")
            z_ax = grav_ax; z_pos = grav_positive
        else:
            print(f"    → TRUSTING YAW (correlation-based, more robust)")
            z_ax = yaw_ax; z_pos = yaw_positive

    # Forward axis: use correlation with linear action
    # Exclude the Z axis from consideration
    remaining = [i for i in range(3) if i != z_ax]
    fwd_candidates = [(abs(acc_linear_corr[i]), i, acc_linear_corr[i]) for i in remaining]
    fwd_candidates.sort(reverse=True)

    # Also look at mean accel difference: forward driving vs stationary
    fwd_drive_indices = [i for i in range(N) if d['act'][i][0] > 0.3 and abs(d['act'][i][1]) < 0.05]
    stat_indices = []
    for w in best_windows[:TOP_N]:
        ep_idxs = episodes[w[0]]
        ws = w[1]
        stat_indices.extend([ep_idxs[ws+j] for j in range(WINDOW)])

    if fwd_drive_indices and stat_indices:
        fwd_acc_mean = vmean([d['acc'][i] for i in fwd_drive_indices])
        stat_acc_mean = vmean([d['acc'][i] for i in stat_indices])
        accel_diff = [fwd_acc_mean[i] - stat_acc_mean[i] for i in range(3)]
        print(f"\n  Forward motion analysis (accel diff: driving - stationary):")
        print(f"    [{accel_diff[0]:+.4f}, {accel_diff[1]:+.4f}, {accel_diff[2]:+.4f}]")

        # The forward axis should show some acceleration when driving starts
        # But steady-state driving = no acceleration → look at action correlation
        # For a ground robot, there might be pitch effect: accelerating forward
        # causes the robot to tilt back slightly → gravity component shifts

        fwd_diff_remaining = [(abs(accel_diff[i]), i, accel_diff[i]) for i in remaining]
        fwd_diff_remaining.sort(reverse=True)
        print(f"    Candidates (excluding Z={AX[z_ax]}):")
        for absval, ax, val in fwd_diff_remaining:
            print(f"      {AX[ax]}: diff = {val:+.4f}")

    # Determine forward axis
    print(f"\n  Forward axis determination:")
    print(f"    Action-accel correlation candidates (excluding Z={AX[z_ax]}):")
    for absval, ax, r in fwd_candidates:
        print(f"      {AX[ax]}: r = {r:+.4f}")

    x_ax = fwd_candidates[0][1]  # highest correlation (excl. Z)
    # Convention: forward command (positive action[0]) should create forward acceleration
    # The sign of correlation tells us: positive correlation = same direction
    # But accelerometer measures force opposite to motion (like gravity)
    # When driving forward, the inertial force pushes backward → accel negative in forward direction
    # So NEGATIVE correlation with action[0] means that axis points FORWARD
    x_corr = acc_linear_corr[x_ax]
    x_pos = x_corr < 0  # negative correlation = forward drives accel negative = axis points forward
    print(f"\n    Best: {AX[x_ax]} (r = {x_corr:+.4f})")
    print(f"    Neg correlation → forward driving pushes accel backward → axis points forward")
    print(f"    → {'+'if x_pos else '-'}{AX[x_ax]} = +X_body (forward)")

    # Y axis: cross product of Z × X gives Y (right-hand rule)
    y_ax = [i for i in range(3) if i != z_ax and i != x_ax][0]
    # Determine Y sign from right-hand rule: Z × X should give -Y (ROS: X-forward, Y-left, Z-up)
    # Actually: in ROS, X×Y=Z, so Z×X=Y, X×Z=-Y
    # If Z points along axis z_ax with sign z_pos, X along x_ax with sign x_pos:
    # We need: cross(Z_body_in_imu, X_body_in_imu) = Y_body_in_imu
    z_vec = [0, 0, 0]
    z_vec[z_ax] = 1.0 if z_pos else -1.0
    x_vec = [0, 0, 0]
    x_vec[x_ax] = 1.0 if x_pos else -1.0
    # Y = Z × X? No. X × Y = Z → Y = Z × X / |Z×X| check...
    # Actually: for right-handed: i×j=k, j×k=i, k×i=j
    # So: X×Y=Z → Y = cross_product_requires: just compute Z×X and see if it gives ±Y_ax
    # Actually simpler: X cross Y = Z, so Y = (Z cross X) if we want right-handed
    # Wait: if X_body cross Y_body gives Z_body, then Y_body = Z_body cross X_body? No.
    # X cross Y = Z; and Z cross X = Y; and Y cross Z = X.
    # So: Y_body = Z_body × X_body... Let me verify: i cross j = k, k cross i = j ✓
    y_vec_calc = vcross(z_vec, x_vec)
    y_pos = y_vec_calc[y_ax] > 0

    # Verify: X × Y should give Z
    y_vec = [0, 0, 0]
    y_vec[y_ax] = 1.0 if y_pos else -1.0
    z_check = vcross(x_vec, y_vec)
    is_right_handed = all(abs(z_check[i] - z_vec[i]) < 0.01 for i in range(3))

    print(f"\n  ★ COMPLETE AXIS MAPPING:")
    print(f"    +X_body (forward) = {'+'if x_pos else '-'}{AX[x_ax]}")
    print(f"    +Y_body (left)    = {'+'if y_pos else '-'}{AX[y_ax]}")
    print(f"    +Z_body (up)      = {'+'if z_pos else '-'}{AX[z_ax]}")
    print(f"    Right-handed check: {'✓ PASS' if is_right_handed else '✗ FAIL'}")

    # ═══════════════════════════════════════════════════════════════════
    # COMPUTE ROTATION MATRIX & RPY
    # ═══════════════════════════════════════════════════════════════════
    print(f"\n{SEPARATOR}")
    print("ROTATION MATRIX: IMU → BODY")
    print(SEPARATOR)

    # The rotation R maps IMU vectors to body vectors:
    # v_body = R @ v_imu
    # Each column of R is the body axis expressed in IMU coordinates:
    #   R[:,0] = X_body in IMU frame
    #   R[:,1] = Y_body in IMU frame
    #   R[:,2] = Z_body in IMU frame
    # Wait, actually: R_body_from_imu means R's columns are the IMU unit vectors in body frame
    # No. Let's think clearly:
    #   The mapping is: v_body = R @ v_imu
    #   If +X_body = -X_imu, then R[0,0] = -1 (X_body gets -1 * X_imu_component)
    #   If +X_body = +Z_imu, then R[0,2] = 1

    R = [[0.0]*3 for _ in range(3)]

    # +X_body = sign * axis_imu
    body_axes = [
        (x_ax, 1.0 if x_pos else -1.0),  # X_body
        (y_ax, 1.0 if y_pos else -1.0),  # Y_body
        (z_ax, 1.0 if z_pos else -1.0),  # Z_body
    ]

    for body_i, (imu_ax, sign) in enumerate(body_axes):
        R[body_i][imu_ax] = sign

    print("  R (IMU → Body):")
    for i in range(3):
        print(f"    [{R[i][0]:+.0f}  {R[i][1]:+.0f}  {R[i][2]:+.0f}]")

    # Verify orthogonality
    det = (R[0][0]*(R[1][1]*R[2][2]-R[1][2]*R[2][1])
          -R[0][1]*(R[1][0]*R[2][2]-R[1][2]*R[2][0])
          +R[0][2]*(R[1][0]*R[2][1]-R[1][1]*R[2][0]))
    print(f"  Determinant: {det:.1f} (should be +1)")

    # Extract RPY (ZYX Euler angles)
    # R = Rz(yaw) @ Ry(pitch) @ Rx(roll)
    # pitch = asin(-R[2][0])
    # roll = atan2(R[2][1], R[2][2])
    # yaw = atan2(R[1][0], R[0][0])
    pitch = math.asin(max(-1, min(1, -R[2][0])))
    roll = math.atan2(R[2][1], R[2][2])
    yaw = math.atan2(R[1][0], R[0][0])

    roll_deg = math.degrees(roll)
    pitch_deg = math.degrees(pitch)
    yaw_deg = math.degrees(yaw)

    print(f"\n  ★ IMU RPY (body ← imu):")
    print(f"    Roll:  {roll_deg:+.1f}°  ({roll:+.4f} rad)")
    print(f"    Pitch: {pitch_deg:+.1f}°  ({pitch:+.4f} rad)")
    print(f"    Yaw:   {yaw_deg:+.1f}°  ({yaw:+.4f} rad)")

    print(f"\n  ★ For bridge_params.yaml:")
    print(f"    imu_rpy_deg: [{roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f}]")

    print(f"\n{SEPARATOR}")
    print("DONE — SUMMARY")
    print(SEPARATOR)
    print(f"""
    Units:
      Accelerometer: g (multiply by 9.81 for m/s²)
      Gyroscope: {gyro_unit}
      Magnetometer: raw ADC / mGauss

    IMU → Body axis mapping:
      +X_body (forward) = {'+'if x_pos else '-'}{AX[x_ax]}
      +Y_body (left)    = {'+'if y_pos else '-'}{AX[y_ax]}
      +Z_body (up)      = {'+'if z_pos else '-'}{AX[z_ax]}

    Rotation (RPY): [{roll_deg:.1f}, {pitch_deg:.1f}, {yaw_deg:.1f}] degrees
    Gyro bias: [{gyro_bias[0]:.3f}, {gyro_bias[1]:.3f}, {gyro_bias[2]:.3f}] {gyro_unit}
    Right-handed: {'✓' if is_right_handed else '✗'}
    Det(R) = {det:.1f}
    """)


if __name__ == "__main__":
    analyze()
