#!/usr/bin/env python3
# imu_yaw_debug.py
# ICM-20948 yaw with mag gating + separate mag rotation + debug.

import time, math, json, os, sys, argparse
import qwiic_icm20948

CAL_FILE = "imu_cal.json"

# Sensor->robot rotation for ACCEL/GYRO
R = [
    [-1.0, 0.0, 0.0],  # x flip (your setup)
    [ 0.0, 1.0, 0.0],
    [ 0.0, 0.0, 1.0],
]

# Additional MAG-only rotation around Z (deg): try 0, 90, 180, 270
RM_Z_DEG = 0

def rotz(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return [[c,-s,0],[s,c,0],[0,0,1]]

def matvec(M, v):
    return [M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
            M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
            M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]]

def norm(v): return math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])

def wrap_pi(a):
    while a <= -math.pi: a += 2*math.pi
    while a >   math.pi: a -= 2*math.pi
    return a

def load_cal():
    return json.load(open(CAL_FILE)) if os.path.exists(CAL_FILE) else None

def save_cal(cal):
    json.dump(cal, open(CAL_FILE,"w"), indent=2)
    print(f"Saved calibration -> {CAL_FILE}")

def quick_calibrate(imu, mag_s=15.0, gyro_rest_s=3.0):
    print("Calibration starting.\n1) Keep still for gyro bias...")
    t0=time.time(); gsum=[0.0,0.0,0.0]; n=0
    while time.time()-t0<gyro_rest_s:
        imu.getAgmt()
        gsum[0]+=math.radians(imu.gxRaw); gsum[1]+=math.radians(imu.gyRaw); gsum[2]+=math.radians(imu.gzRaw)
        n+=1; time.sleep(0.01)
    g_bias = matvec(R, [gsum[i]/max(n,1) for i in range(3)])

    print("2) Mag min/max (~{}s). Rotate slowly through all orientations...".format(int(mag_s)))
    mmin=[1e9]*3; mmax=[-1e9]*3; t0=time.time()
    Rm = rotz(RM_Z_DEG)
    while time.time()-t0<mag_s:
        imu.getAgmt()
        m = matvec(Rm, matvec(R, [imu.mxRaw, imu.myRaw, imu.mzRaw]))
        for i in range(3):
            mmin[i]=min(mmin[i], m[i]); mmax[i]=max(mmax[i], m[i])
        time.sleep(0.01)

    print("3) Quick accel sample...")
    asum=0.0; n=0; t0=time.time()
    while time.time()-t0<1.0:
        imu.getAgmt(); a=matvec(R,[imu.axRaw,imu.ayRaw,imu.azRaw]); asum+=norm(a); n+=1; time.sleep(0.01)
    g_norm = asum/max(n,1)

    cal={"gyro_bias":g_bias,"mag_min":mmin,"mag_max":mmax,"g_norm":g_norm, "rm_z": RM_Z_DEG}
    print("Calibration done.")
    save_cal(cal); return cal

def apply_mag_cal(m, mmin, mmax):
    c=[(mmax[i]+mmin[i])*0.5 for i in range(3)]
    r=[(mmax[i]-mmin[i])*0.5 for i in range(3)]
    r=[ri if abs(ri)>1e-9 else 1.0 for ri in r]
    return [(m[i]-c[i])/r[i] for i in range(3)]

def accel_to_pr(a):
    ax,ay,az=a; n=max(norm(a),1e-9); ax/=n; ay/=n; az/=n
    phi   = math.atan2(ay, az)                             # roll
    theta = math.atan2(-ax, math.sqrt(ay*ay+az*az))        # pitch
    return phi, theta

def tilt_comp_heading(m, phi, theta):
    mx,my,mz=m
    cth,sth=math.cos(theta),math.sin(theta)
    cph,sph=math.cos(phi),math.sin(phi)
    mx2 = mx*cth + mz*sth
    my2 = mx*sph*sth + my*cph - mz*sph*cth
    return wrap_pi(math.atan2(-my2, mx2))

def main():
    ap = argparse.ArgumentParser(description="ICM-20948 yaw with gating + debug")
    ap.add_argument("--recal", action="store_true", help="force new calibration")
    ap.add_argument("--alpha", type=float, default=0.02, help="base mag correction gain (default 0.02)")
    ap.add_argument("--hz", type=float, default=10.0, help="print rate")
    ap.add_argument("--show-mag", action="store_true", help="print mag-only yaw too")
    ap.add_argument("--rmz", type=int, choices=[0,90,180,270], help="override mag Z-rotation deg (0/90/180/270)")
    args = ap.parse_args()

    imu = qwiic_icm20948.QwiicIcm20948()
    if not imu.connected:
        print("ICM-20948 not detected. Check Qwiic wiring/power.")
        sys.exit(1)
    imu.begin()

    cal = None if args.recal else load_cal()
    if cal is None: cal = quick_calibrate(imu)
    g_bias = cal["gyro_bias"]; mmin, mmax = cal["mag_min"], cal["mag_max"]
    rmz = args.rmz if args.rmz is not None else cal.get("rm_z", RM_Z_DEG)
    Rm = rotz(rmz)

    # Trust tracking
    B_ref=None; M_ref=None
    B_alpha=0.02
    B_tol  = 0.05                 # was 0.25
    ang_tol = math.radians(3)     # was 25

    yaw=0.0; yaw_mag_f=None
    last=time.time(); last_out=last
    print("\nStreaming… Ctrl+C to stop.")
    hdr = "yaw     pitch   roll    |B|(mG)  trust  a_eff"
    if args.show_mag: hdr += "   yaw_mag"
    print(hdr)

    try:
        while True:
            t=time.time(); dt=t-last
            if dt<=0: time.sleep(0.001); continue
            last=t

            imu.getAgmt()

            # Raw -> robot frame
            ax,ay,az = matvec(R,[imu.axRaw,imu.ayRaw,imu.azRaw])            # g
            gx,gy,gz = matvec(R,[math.radians(imu.gxRaw),math.radians(imu.gyRaw),math.radians(imu.gzRaw)])  # rad/s
            # MAG uses additional rotation (possible AK09916 axis difference)
            mx,my,mz = matvec(Rm, matvec(R,[imu.mxRaw,imu.myRaw,imu.mzRaw])) # mG likely

            # Gyro bias
            gz -= g_bias[2]

            # Attitude from accel
            phi,theta = accel_to_pr([ax,ay,az])

            # Mag tilt-comp + cal
            m_cal = apply_mag_cal([mx,my,mz], mmin, mmax)
            yaw_mag = tilt_comp_heading(m_cal, phi, theta)

            # Smooth the mag yaw on the circle
            if yaw_mag_f is None:
                yaw_mag_f = yaw_mag
            else:
                # circular EMA
                e = wrap_pi(yaw_mag - yaw_mag_f)
                yaw_mag_f = wrap_pi(yaw_mag_f + 0.2*e)

            # Gating
            B = norm([mx,my,mz])               # mG
            if B_ref is None: B_ref=B
            else: B_ref = (1-B_alpha)*B_ref + B_alpha*B

            if M_ref is None:
                u=m_cal[:]; n=max(norm(u),1e-9); M_ref=[u[0]/n,u[1]/n,u[2]/n]
            u=m_cal[:]; n=max(norm(u),1e-9); u=[u[0]/n,u[1]/n,u[2]/n]
            dotp=max(-1.0,min(1.0,u[0]*M_ref[0]+u[1]*M_ref[1]+u[2]*M_ref[2]))
            ang=math.acos(dotp)

            t_mag = 1.0 if abs(B-B_ref)/max(B_ref,1e-6) < B_tol else 0.0
            t_dir = 1.0 if ang < ang_tol else 0.0
            trust = t_mag * t_dir
            alpha_eff = args.alpha * trust

            # Limit single-step correction so mag can’t snap heading
            yaw_gyro = wrap_pi(yaw + gz*dt)
            err = wrap_pi((yaw_mag_f) - yaw_gyro)
            max_step = math.radians(2.0)  # ≤2° per update
            corr = max(-max_step, min(max_step, alpha_eff*err))
            yaw = wrap_pi(yaw_gyro + corr)

            if (t-last_out) >= 1.0/args.hz:
                last_out=t
                line = f"{math.degrees(yaw):7.2f}  {math.degrees(theta):7.2f}  {math.degrees(phi):7.2f}  {B:7.1f}   {trust:4.1f}   {alpha_eff:5.3f}"
                if args.show_mag:
                    line += f"   {math.degrees(yaw_mag_f):7.2f}"
                print(line)

            time.sleep(0.003)

    except KeyboardInterrupt:
        print("\nStopping.")

if __name__ == "__main__":
    main()
