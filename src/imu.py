#!/usr/bin/env python3
# imu.py
import time, math, json, os
import qwiic_icm20948

CAL_FILE = "imu_cal.json"

R = [
    [-1.0, 0.0, 0.0],  # accel/gyro rotation matrix (sensor->robot frame)
    [ 0.0, 1.0, 0.0],
    [ 0.0, 0.0, 1.0],
]

RMZ = 270  # mag-only rotation about Z (deg)

def rotz(deg):
    r = math.radians(deg); c, s = math.cos(r), math.sin(r)
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

class IMU:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.imu = qwiic_icm20948.QwiicIcm20948()
        if not self.imu.connected:
            raise RuntimeError("ICM-20948 not detected. Check wiring.")
        self.imu.begin()
        self.Rm = rotz(RMZ)
        self.cal = self._load_cal() or self._calibrate()
        self.g_bias = self.cal["gyro_bias"]
        self.mmin, self.mmax = self.cal["mag_min"], self.cal["mag_max"]
        self.yaw = 0.0
        self.yaw_mag_f = None
        self.last_t = time.time()
        # gating refs
        self.B_ref = None
        self.M_ref = None
        # stationary bias estimator
        self.stat_window = []
        self.bias_learn_rate = 0.001  # slow creep

    def _load_cal(self):
        if os.path.exists(CAL_FILE):
            with open(CAL_FILE,"r") as f:
                return json.load(f)
        return None

    def _save_cal(self, cal):
        with open(CAL_FILE,"w") as f:
            json.dump(cal,f,indent=2)

    def _calibrate(self, mag_s=15.0, gyro_rest_s=3.0):
        print("Calibration starting.")
        print("1) Keep still for gyro bias...")
        gsum=[0.0,0.0,0.0]; n=0
        t0=time.time()
        while time.time()-t0<gyro_rest_s:
            self.imu.getAgmt()
            gsum[0]+=math.radians(self.imu.gxRaw)
            gsum[1]+=math.radians(self.imu.gyRaw)
            gsum[2]+=math.radians(self.imu.gzRaw)
            n+=1; time.sleep(0.01)
        g_bias = matvec(R,[gsum[i]/max(n,1) for i in range(3)])

        print("2) Mag min/max (~{}s). Rotate slowly...".format(int(mag_s)))
        mmin=[1e9]*3; mmax=[-1e9]*3; t0=time.time()
        while time.time()-t0<mag_s:
            self.imu.getAgmt()
            m = matvec(self.Rm, matvec(R, [self.imu.mxRaw,self.imu.myRaw,self.imu.mzRaw]))
            for i in range(3):
                mmin[i]=min(mmin[i], m[i]); mmax[i]=max(mmax[i], m[i])
            time.sleep(0.01)

        print("3) Quick accel sample...")
        asum=0.0; n=0; t0=time.time()
        while time.time()-t0<1.0:
            self.imu.getAgmt()
            a = matvec(R,[self.imu.axRaw,self.imu.ayRaw,self.imu.azRaw])
            asum+=norm(a); n+=1; time.sleep(0.01)
        g_norm = asum/max(n,1)

        cal={"gyro_bias":g_bias,"mag_min":mmin,"mag_max":mmax,"g_norm":g_norm,"rm_z":RMZ}
        self._save_cal(cal)
        print("Calibration done.")
        return cal

    def _apply_mag_cal(self, m):
        mmin, mmax = self.mmin, self.mmax
        c=[(mmax[i]+mmin[i])*0.5 for i in range(3)]
        r=[(mmax[i]-mmin[i])*0.5 for i in range(3)]
        r=[ri if abs(ri)>1e-9 else 1.0 for ri in r]
        return [(m[i]-c[i])/r[i] for i in range(3)]

    def _accel_to_pr(self, a):
        ax,ay,az=a; n=max(norm(a),1e-9); ax/=n; ay/=n; az/=n
        phi   = math.atan2(ay, az)
        theta = math.atan2(-ax, math.sqrt(ay*ay+az*az))
        return phi, theta

    def _tilt_comp_heading(self, m, phi, theta):
        mx,my,mz=m
        cth,sth=math.cos(theta),math.sin(theta)
        cph,sph=math.cos(phi),math.sin(phi)
        mx2 = mx*cth + mz*sth
        my2 = mx*sph*sth + my*cph - mz*sph*cth
        return wrap_pi(math.atan2(-my2, mx2))

    def get_heading(self):
        """Returns fused yaw heading in degrees."""
        t = time.time(); dt = t - self.last_t
        if dt <= 0: dt = 1e-3
        self.last_t = t

        self.imu.getAgmt()

        # Sensor->robot
        ax,ay,az = matvec(R, [self.imu.axRaw,self.imu.ayRaw,self.imu.azRaw])
        gx,gy,gz = matvec(R, [math.radians(self.imu.gxRaw),
                              math.radians(self.imu.gyRaw),
                              math.radians(self.imu.gzRaw)])
        mx,my,mz = matvec(self.Rm, matvec(R, [self.imu.mxRaw,self.imu.myRaw,self.imu.mzRaw]))

        # Apply gyro bias
        gz -= self.g_bias[2]

        # Pitch/roll
        phi,theta = self._accel_to_pr([ax,ay,az])

        # Mag tilt comp + cal
        m_cal = self._apply_mag_cal([mx,my,mz])
        yaw_mag = self._tilt_comp_heading(m_cal, phi, theta)

        # Smooth mag yaw
        if self.yaw_mag_f is None:
            self.yaw_mag_f = yaw_mag
        else:
            e = wrap_pi(yaw_mag - self.yaw_mag_f)
            self.yaw_mag_f = wrap_pi(self.yaw_mag_f + 0.2*e)

        # Gating
        B = norm([mx,my,mz])
        if self.B_ref is None:
            self.B_ref = B
        else:
            self.B_ref = 0.98*self.B_ref + 0.02*B

        if self.M_ref is None:
            u = m_cal[:]; n = max(norm(u),1e-9); self.M_ref = [u[0]/n,u[1]/n,u[2]/n]
        u = m_cal[:]; n = max(norm(u),1e-9); u = [u[0]/n,u[1]/n,u[2]/n]
        dotp = max(-1.0,min(1.0,u[0]*self.M_ref[0]+u[1]*self.M_ref[1]+u[2]*self.M_ref[2]))
        ang = math.acos(dotp)

        t_mag = 1.0 if abs(B - self.B_ref)/max(self.B_ref,1e-6) < 0.25 else 0.0
        t_dir = 1.0 if ang < math.radians(25) else 0.0
        trust = t_mag * t_dir
        alpha_eff = self.alpha * trust

        # Fuse
        yaw_gyro = wrap_pi(self.yaw + gz*dt)
        err = wrap_pi(self.yaw_mag_f - yaw_gyro)
        self.yaw = wrap_pi(yaw_gyro + alpha_eff*err)

        # --- Stationary bias estimator ---
        stat_thresh = math.radians(2.0)   # deg/s
        acc_dev = max(abs(ax),abs(ay))    # crude motion check
        if abs(gz) < stat_thresh and acc_dev < 0.05:
            self.g_bias[2] = 0.999*self.g_bias[2] + 0.001*(self.g_bias[2] + gz)

        return math.degrees(self.yaw)
