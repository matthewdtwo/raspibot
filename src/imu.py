#!/usr/bin/env python3
# imu.py — IMU class with fused yaw heading for SparkFun Qwiic ICM-20948
# - Uses qwiic_icm20948.getAgmt()
# - Correct scaling: accel (±2g), gyro (±250 dps), mag (raw counts)
# - Tilt-compass + complementary fuse with gating
# - Stationary gyro-Z bias creep
# - Prime yaw to mag on first call
# - get_heading() returns yaw in degrees

import time, math, json, os
import qwiic_icm20948

# ---- Scale constants (match full-scale ranges we set in __init__) ----
ACC_LSB_PER_G      = 16384.0   # ±2 g
GYRO_LSB_PER_DPS   = 131.0     # ±250 dps

def rotz(deg: float):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
    return [[c,-s,0],[s,c,0],[0,0,1]]

def matvec(M, v):
    return [M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
            M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
            M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]]

def vnorm(v): return math.sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2])
def wrap_pi(a: float) -> float:
    while a <= -math.pi: a += 2*math.pi
    while a >   math.pi: a -= 2*math.pi
    return a

class IMU:
    def __init__(self,
                 alpha: float = 0.25,            # mag correction gain
                 rmz: int = 270,                 # mag-only Z rotation (deg)
                 cal_file: str = "imu_cal.json",
                 recalibrate: bool = False):

        # Sensor->robot rotation for accel/gyro (your setup: flip X)
        self.R = [
            [-1.0, 0.0, 0.0],
            [ 0.0, 1.0, 0.0],
            [ 0.0, 0.0, 1.0],
        ]
        self.Rm = rotz(rmz)

        # Fusion & gating params
        self.alpha_base = alpha
        self.B_alpha = 0.02                # EMA for |B|
        self.B_tol   = 0.05                # ±5% magnitude window
        self.ang_tol = math.radians(3.0)   # 3° direction window
        self.max_step = math.radians(10.0) # per-update correction cap

        # Stationary detection / bias creep
        self.gz_still_thresh = math.radians(2.0)  # rad/s
        self.acc_xy_thresh   = 0.05               # g
        self.still_time_req  = 2.0                # s
        self.bias_learn      = 0.001              # creep

        # Driver
        self.imu = qwiic_icm20948.QwiicIcm20948()
        if not self.imu.connected:
            raise RuntimeError("ICM-20948 not detected. Check Qwiic wiring/power.")
        self.imu.begin()

        # Ensure ranges match our scale constants
        try:
            self.imu.setFullScaleRangeAccel(self.imu.gpm2)   # ±2 g
            self.imu.setFullScaleRangeGyro(self.imu.dps250)  # ±250 dps
        except Exception:
            pass

        self.cal_file = cal_file
        self.cal = None if recalibrate else self._load_cal()
        if self.cal is None:
            self.cal = self._calibrate(rmz)

        self.g_bias = self.cal["gyro_bias"]      # rad/s in robot frame
        self.mmin, self.mmax = self.cal["mag_min"], self.cal["mag_max"]

        # State
        self.last_t = time.time()
        self.yaw = 0.0
        self.yaw_mag_f = None
        self._primed = False

        # Gating refs
        self.B_ref = None
        self.M_ref = None
        self.B_prev = None
        self.u_prev = None

        # Stationary timer
        self._still_t = 0.0

        # Optional external gating hook (e.g., motors running)
        self._motors_on = False

        self._dbg = {}

    # ---------- Calibration ----------
    def _load_cal(self):
        if os.path.exists(self.cal_file):
            with open(self.cal_file,"r") as f: return json.load(f)
        return None

    def _save_cal(self, cal):
        with open(self.cal_file,"w") as f: json.dump(cal,f,indent=2)

    def _calibrate(self, rmz):
        mag_s=15.0; gyro_rest_s=3.0
        print("Calibration starting.")
        print("1) Keep still for gyro bias...")
        gsum=[0.0,0.0,0.0]; n=0; t0=time.time()
        while time.time()-t0<gyro_rest_s:
            self.imu.getAgmt()
            # scale RAW -> rad/s using our configured range
            gx_dps = self.imu.gxRaw / GYRO_LSB_PER_DPS
            gy_dps = self.imu.gyRaw / GYRO_LSB_PER_DPS
            gz_dps = self.imu.gzRaw / GYRO_LSB_PER_DPS
            gsum[0] += math.radians(gx_dps)
            gsum[1] += math.radians(gy_dps)
            gsum[2] += math.radians(gz_dps)
            n+=1; time.sleep(0.01)
        g_bias_sensor = [gsum[i]/max(n,1) for i in range(3)]
        g_bias = matvec(self.R, g_bias_sensor)

        print(f"2) Mag min/max (~{int(mag_s)}s). Rotate slowly through all orientations…")
        mmin=[1e9]*3; mmax=[-1e9]*3; t0=time.time()
        while time.time()-t0<mag_s:
            self.imu.getAgmt()
            m = matvec(self.Rm, matvec(self.R, [self.imu.mxRaw, self.imu.myRaw, self.imu.mzRaw]))
            for i in range(3):
                if m[i]<mmin[i]: mmin[i]=m[i]
                if m[i]>mmax[i]: mmax[i]=m[i]
            time.sleep(0.01)

        print("3) Quick accel sample…")
        asum=0.0; n=0; t0=time.time()
        while time.time()-t0<1.0:
            self.imu.getAgmt()
            ax_g = self.imu.axRaw/ACC_LSB_PER_G
            ay_g = self.imu.ayRaw/ACC_LSB_PER_G
            az_g = self.imu.azRaw/ACC_LSB_PER_G
            a = matvec(self.R, [ax_g, ay_g, az_g])
            asum+=vnorm(a); n+=1; time.sleep(0.01)
        g_norm = asum/max(n,1)

        cal={"gyro_bias":g_bias,"mag_min":mmin,"mag_max":mmax,"g_norm":g_norm,"rm_z":rmz}
        self._save_cal(cal)
        print("Calibration done.")
        return cal

    # ---------- Helpers ----------
    @staticmethod
    def _apply_mag_cal(m, mmin, mmax):
        c=[(mmax[i]+mmin[i])*0.5 for i in range(3)]
        r=[(mmax[i]-mmin[i])*0.5 for i in range(3)]
        r=[ri if abs(ri)>1e-9 else 1.0 for ri in r]
        return [(m[i]-c[i])/r[i] for i in range(3)]

    @staticmethod
    def _accel_to_pr(a):
        ax,ay,az=a; n=max(vnorm(a),1e-9); ax/=n; ay/=n; az/=n
        phi   = math.atan2(ay, az)                          # roll
        theta = math.atan2(-ax, math.sqrt(ay*ay+az*az))     # pitch
        return phi, theta

    @staticmethod
    def _tilt_comp_heading(m, phi, theta):
        mx,my,mz=m
        cth,sth=math.cos(theta),math.sin(theta)
        cph,sph=math.cos(phi),math.sin(phi)
        mx2 = mx*cth + mz*sth
        my2 = mx*sph*sth + my*cph - mz*sph*cth
        return wrap_pi(math.atan2(-my2, mx2))

    # ---------- Optional external hook ----------
    def set_motors_active(self, on: bool):
        self._motors_on = bool(on)

    # ---------- Public API ----------
    def get_heading(self) -> float:
        """Return fused yaw heading (degrees). Call at ~20–100 Hz."""
        t = time.time(); dt = t - self.last_t
        if dt <= 0: dt = 1e-3
        self.last_t = t

        self.imu.getAgmt()

        # ---- scale RAW counts to physical units ----
        # accel in g
        ax_g = self.imu.axRaw/ACC_LSB_PER_G
        ay_g = self.imu.ayRaw/ACC_LSB_PER_G
        az_g = self.imu.azRaw/ACC_LSB_PER_G
        ax, ay, az = matvec(self.R, [ax_g, ay_g, az_g])

        # gyro in rad/s
        gx_dps = self.imu.gxRaw/GYRO_LSB_PER_DPS
        gy_dps = self.imu.gyRaw/GYRO_LSB_PER_DPS
        gz_dps = self.imu.gzRaw/GYRO_LSB_PER_DPS
        gx, gy, gz = matvec(self.R, [math.radians(gx_dps),
                                     math.radians(gy_dps),
                                     math.radians(gz_dps)])

        # mag: raw counts (we normalize & calibrate)
        mx_raw, my_raw, mz_raw = self.imu.mxRaw, self.imu.myRaw, self.imu.mzRaw
        mx, my, mz = matvec(self.Rm, matvec(self.R, [mx_raw, my_raw, mz_raw]))

        # Gyro bias & stillness
        gz -= self.g_bias[2]
        stationary = (abs(gz) < self.gz_still_thresh) and (abs(ax) < self.acc_xy_thresh and abs(ay) < self.acc_xy_thresh)
        self._still_t = self._still_t + dt if stationary else 0.0
        if self._still_t > self.still_time_req:
            # creep bias toward current (very slow)
            self.g_bias[2] = 0.999*self.g_bias[2] + 0.001*(self.g_bias[2] + gz)

        # Pitch/Roll
        phi,theta = self._accel_to_pr([ax,ay,az])

        # Mag tilt-comp + cal + smoothing
        m_cal = self._apply_mag_cal([mx,my,mz], self.mmin, self.mmax)
        yaw_mag = self._tilt_comp_heading(m_cal, phi, theta)

        if self.yaw_mag_f is None:
            self.yaw_mag_f = yaw_mag
        else:
            e = wrap_pi(yaw_mag - self.yaw_mag_f)
            self.yaw_mag_f = wrap_pi(self.yaw_mag_f + 0.2*e)  # circular EMA

        # Prime on first call to avoid wrap flips
        if not self._primed:
            self.yaw = self.yaw_mag_f
            self._primed = True

        # Gating: magnitude + direction + spike slew
        B = vnorm([mx,my,mz])                  # raw-count magnitude (relative only)
        self.B_ref = B if self.B_ref is None else (1.0 - self.B_alpha)*self.B_ref + self.B_alpha*B

        u = m_cal[:]                           # unit direction
        n=max(vnorm(u),1e-9); u=[u[0]/n,u[1]/n,u[2]/n]
        if self.M_ref is None: self.M_ref = u[:]

        t_mag = 1.0 if abs(B - self.B_ref)/max(self.B_ref,1e-6) < self.B_tol else 0.0
        dotp = max(-1.0, min(1.0, u[0]*self.M_ref[0] + u[1]*self.M_ref[1] + u[2]*self.M_ref[2]))
        ang  = math.acos(dotp)
        t_dir = 1.0 if ang < self.ang_tol else 0.0

        # slew gates
        trust = t_mag * t_dir
        if self.B_prev is not None and self.u_prev is not None:
            dB_dt = abs(B - self.B_prev) / max(dt,1e-3)
            cosang = max(-1.0,min(1.0, u[0]*self.u_prev[0]+u[1]*self.u_prev[1]+u[2]*self.u_prev[2]))
            dang_dt = math.acos(cosang) / max(dt,1e-3)
            if dB_dt > 50 or dang_dt > math.radians(10):
                trust = 0.0
        self.B_prev, self.u_prev = B, u

        # optional external gate (e.g., motors on -> gyro-only)
        if self._motors_on:
            trust = 0.0

        alpha_eff = self.alpha_base * trust

        # Complementary fuse + per-step cap
        yaw_gyro = wrap_pi(self.yaw + gz*dt)
        err = wrap_pi(self.yaw_mag_f - yaw_gyro)
        corr = max(-self.max_step, min(self.max_step, alpha_eff*err))
        self.yaw = wrap_pi(yaw_gyro + corr)

        # Debug snapshot
        self._dbg = {
            "yaw_deg": math.degrees(self.yaw),
            "yaw_mag_deg": math.degrees(self.yaw_mag_f),
            "B_raw": B,
            "trust": trust,
            "alpha_eff": alpha_eff,
            "gz_bias": self.g_bias[2],
            "gz_raw": self.imu.gzRaw,           # LSB counts (sanity only)
            "gz_after_bias": gz,                # rad/s after bias subtraction
            "dt": dt
        }

        return math.degrees(self.yaw)
