#!/usr/bin/env python3
"""
Node UTS: menggerakkan robot Robin melewati track KANAN sampai kotak finish.

Hanya memakai dua topic yang diizinkan:
  - publish   /cmd_vel      (geometry_msgs/Twist)
  - subscribe /wheel/speed  (std_msgs/Float64MultiArray)
                            data[0] = roda kiri, data[1] = roda kanan (rad/s)

Posisi robot dihitung sendiri dari kecepatan roda (odometri).
TIDAK membaca data Gazebo secara langsung.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


# ======================================================================
# 1) PARAMETER ROBOT
# ======================================================================
# Ukuran roda ASLI robot (robin_description/.../mobile_base.xacro)
R_FISIK = 0.03          # radius roda (m)
JARAK_RODA = 0.17       # jarak roda kiri-kanan (m)

# Parameter plugin DiffDrive Gazebo (mobile_base_gazebo.xacro).
# Plugin mengubah /cmd_vel jadi kecepatan roda memakai nilai ini,
# sehingga perintah harus dikompensasi (lihat fungsi kirim()).
R_PLUGIN = 0.10
JARAK_PLUGIN = 0.45

# Faktor kalibrasi odometri (1.0 = tanpa koreksi)
K_JARAK = 1.0
K_PUTAR = 1.0

# ======================================================================
# 2) PARAMETER GERAK (satuan: kecepatan SEBENARNYA robot)
# ======================================================================
V_MAX = 0.35            # kecepatan maju maksimum (m/s)
V_BUSUR = 0.25          # kecepatan saat melewati busur (m/s)
W_MAX = 1.0             # kecepatan putar maksimum (rad/s)
W_MIN = 0.2             # kecepatan putar minimum (rad/s)
KP_PUTAR = 2.0          # penguat saat memutar di tempat
KP_ARAH = 2.0           # penguat koreksi arah saat maju
SUDUT_PUTAR_MULAI = math.radians(25)   # error > ini -> putar di tempat
SUDUT_PUTAR_SELESAI = math.radians(3)  # error < ini -> selesai putar
TOL_BERHENTI = 0.05     # toleransi sampai di titik biasa (m)
TOL_LEWAT = 0.15        # toleransi sampai di titik "lewat" (m)


# ======================================================================
# 3) RUTE TRACK KANAN  (x, y, berhenti?)
#    Robot mulai di (0, 0) menghadap +x. Satuan meter.
# ======================================================================
def busur(cx, cy, r, sudut_awal, sudut_akhir, langkah):
    """Titik-titik sepanjang busur lingkaran (sudut dalam derajat)."""
    n = int(round(abs(sudut_akhir - sudut_awal) / langkah))
    hasil = []
    for k in range(1, n + 1):
        a = math.radians(sudut_awal + (sudut_akhir - sudut_awal) * k / n)
        hasil.append((cx + r * math.cos(a), cy + r * math.sin(a), False))
    return hasil


WAYPOINTS = [
    (0.60, -2.50, True),
    (2.40, -2.50, True),
    (2.40, -3.15, True),
    (3.65, -3.15, True),
    (3.65, -0.92, True),
    (7.915, -0.92, True),
    (7.915, -6.00, False),
    *busur(6.38, -6.00, 1.535, 0.0, -180.0, 15.0),
    (4.85, -4.35, True),
    (3.93, -4.35, True),
    (3.93, -6.93, True),
    (2.00, -6.93, True),
    (1.40, -7.65, True),
    (0.30, -7.65, True),
    (0.30, -5.00, True),     # kotak FINISH
]


def normalisasi_sudut(a):
    """Ubah sudut ke rentang -pi .. +pi."""
    return math.atan2(math.sin(a), math.cos(a))


def batasi(x, bawah, atas):
    return max(bawah, min(atas, x))


class UtsNode(Node):
    def __init__(self):
        super().__init__('uts_node')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Float64MultiArray, '/wheel/speed',
                                 self.wheel_callback, qos_profile_sensor_data)

        # posisi robot hasil odometri
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.waktu_terakhir = None

        # status
        self.idx = 0
        self.sedang_putar = False
        self.waktu_mulai = None
        self.selesai = False

        self.create_timer(0.05, self.loop_kontrol)   # 20 Hz
        self.get_logger().info('Node siap, menunggu /wheel/speed ...')

    def sekarang(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- ODOMETRI ----------
    def wheel_callback(self, msg):
        if len(msg.data) < 2:
            return
        t = self.sekarang()
        if self.waktu_terakhir is None:
            self.waktu_terakhir = t
            return
        dt = t - self.waktu_terakhir
        self.waktu_terakhir = t
        if dt <= 0.0 or dt > 0.5:
            return

        w_kiri, w_kanan = msg.data[0], msg.data[1]
        v = K_JARAK * R_FISIK * (w_kanan + w_kiri) / 2.0
        omega = K_PUTAR * R_FISIK * (w_kanan - w_kiri) / JARAK_RODA

        yaw_tengah = self.yaw + omega * dt / 2.0
        self.x += v * math.cos(yaw_tengah) * dt
        self.y += v * math.sin(yaw_tengah) * dt
        self.yaw = normalisasi_sudut(self.yaw + omega * dt)

    # ---------- KIRIM PERINTAH ----------
    def kirim(self, v, w):
        """v, w = yang diinginkan pada robot asli -> dikompensasi ke plugin."""
        msg = Twist()
        msg.linear.x = float(v * R_PLUGIN / R_FISIK)
        msg.angular.z = float(
            w * JARAK_RODA * R_PLUGIN / (R_FISIK * JARAK_PLUGIN))
        self.pub.publish(msg)

    # ---------- LOOP KONTROL ----------
    def loop_kontrol(self):
        if self.selesai or self.waktu_terakhir is None:
            return

        gx, gy, berhenti = WAYPOINTS[self.idx]
        dx, dy = gx - self.x, gy - self.y
        jarak = math.hypot(dx, dy)
        err = normalisasi_sudut(math.atan2(dy, dx) - self.yaw)

        # sudah sampai di titik tujuan?
        tol = TOL_BERHENTI if berhenti else TOL_LEWAT
        if jarak < tol:
            self.get_logger().info(
                f'Titik {self.idx + 1}/{len(WAYPOINTS)} tercapai '
                f'pos=({self.x:.2f}, {self.y:.2f})')
            self.idx += 1
            if self.idx >= len(WAYPOINTS):
                self.finish()
            return

        # putar di tempat jika arah terlalu melenceng (histeresis)
        if abs(err) > SUDUT_PUTAR_MULAI:
            self.sedang_putar = True
        elif abs(err) < SUDUT_PUTAR_SELESAI:
            self.sedang_putar = False

        if self.sedang_putar:
            w = math.copysign(batasi(KP_PUTAR * abs(err), W_MIN, W_MAX), err)
            self.kirim(0.0, w)
            return

        # maju sambil koreksi arah
        v_dasar = V_MAX if berhenti else V_BUSUR
        v = v_dasar * batasi(1.0 - abs(err) / 0.7, 0.2, 1.0)
        if berhenti:
            v = min(v, max(0.08, 1.0 * jarak))   # melambat dekat titik berhenti
        w = batasi(KP_ARAH * err, -W_MAX, W_MAX)

        # stopwatch mulai saat robot pertama kali maju
        if self.waktu_mulai is None:
            self.waktu_mulai = self.sekarang()
            self.get_logger().info('Robot mulai bergerak, stopwatch jalan.')

        self.kirim(v, w)

    def finish(self):
        self.selesai = True
        self.kirim(0.0, 0.0)
        durasi = self.sekarang() - self.waktu_mulai
        self.get_logger().info('=' * 40)
        self.get_logger().info(
            f'FINISH! Waktu tempuh: {durasi:.2f} detik (waktu simulasi)')
        self.get_logger().info('=' * 40)


def main(args=None):
    rclpy.init(args=args)
    node = UtsNode()
    try:
        while rclpy.ok() and not node.selesai:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):          # pastikan robot berhenti
            node.kirim(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
