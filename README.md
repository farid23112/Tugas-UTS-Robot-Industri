# Tugas UTS Robot Industri

Node ROS 2 (Python) untuk menggerakkan robot **Robin** melewati **track kanan** pada simulasi Gazebo, sampai kotak finish, sekaligus mencatat waktu tempuh. Dibuat untuk UTS Robotika.

## Aturan yang dipenuhi
- Hanya memakai dua topic:
  - `/cmd_vel` (`geometry_msgs/msg/Twist`) untuk mengirim perintah gerak
  - `/wheel/speed` (`std_msgs/msg/Float64MultiArray`) untuk membaca kecepatan roda
- **Tidak** membaca data Gazebo secara langsung. Posisi dihitung sendiri dari kecepatan roda (odometri).
- Waktu tempuh dicatat otomatis: dari robot mulai bergerak sampai titik finish tercapai.

## Cara kerja
1. **Odometri.** `/wheel/speed` diintegrasikan menjadi posisi `(x, y)` dan arah `yaw`.
2. **Rute.** Robot mengikuti daftar titik (`WAYPOINTS`) di tengah lorong track kanan.
3. **Kontrol.**
   - Jika arah ke titik tujuan melenceng lebih dari 25 derajat, robot **berputar di tempat**.
   - Jika tidak, robot **maju sambil mengoreksi arah** (kontrol proporsional).
   - Titik pada busur setengah lingkaran dilewati tanpa berhenti.
4. **Kompensasi plugin.** Plugin DiffDrive Gazebo memakai radius dan jarak roda yang berbeda dari ukuran fisik robot, sehingga `/cmd_vel` dikali faktor koreksi di fungsi `kirim()`.

## Persiapan simulasi
```bash
cd ~/ros2_ws/src/robin_bringup
git fetch
git switch uts

cd ~/ros2_ws/src/robin_description
git fetch
git switch uts
```

## Instalasi
```bash
cd ~/ros2_ws/src
git clone https://github.com/USERNAME_KAMU/robin_uts.git

cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Menjalankan
Terminal 1 (simulasi):
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch robin_bringup my_robot_gazebo.launch.py
```

Terminal 2 (node, jalankan setelah robot muncul di Gazebo dan simulasi tidak ter-pause):
```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run robin_uts uts_node --ros-args -p use_sim_time:=true
```

Saat robot sampai di kotak finish, terminal menampilkan:
```
FINISH! Waktu tempuh: XX.XX detik (waktu simulasi)
```

## Hasil
| Percobaan | Waktu tempuh (detik) |
|---|---|
| 1 | - |
| 2 | - |
| 3 | - |

## Penyetelan parameter
Semua parameter ada di bagian atas `robin_uts/uts_node.py`.

| Masalah | Yang diubah |
|---|---|
| Robot menabrak sudut atau dinding | Kecilkan `V_MAX` / `V_BUSUR` |
| Robot bergoyang saat maju | Kecilkan `KP_ARAH` |
| Robot berputar kurang atau lebih dari seharusnya | Ubah `K_PUTAR` |
| Jarak tempuh kurang atau lebih | Ubah `K_JARAK` |
| Ingin lebih cepat | Naikkan `V_MAX` sedikit demi sedikit |
| Robot melenceng di titik tertentu | Geser koordinat di `WAYPOINTS` |

Pastikan juga `R_FISIK`, `JARAK_RODA`, `R_PLUGIN`, dan `JARAK_PLUGIN` sesuai dengan nilai `wheel_radius` dan `wheel_separation` di file xacro `robin_description`.

## Pemecahan masalah
- **Node berhenti di "menunggu /wheel/speed":** pastikan Gazebo berjalan dan tidak ter-pause, lalu cek `ros2 topic echo /wheel/speed`. Subscriber memakai `qos_profile_sensor_data` agar cocok dengan publisher best-effort.
- **`ModuleNotFoundError`:** jalankan ulang `colcon build --packages-select robin_uts` lalu `source install/setup.bash`.
- **Waktu simulasi tidak berjalan:** coba jalankan tanpa `-p use_sim_time:=true` (waktu yang dicetak jadi waktu nyata).

## Struktur repo
```
robin_uts/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/robin_uts
├── robin_uts/
│   ├── __init__.py
│   └── uts_node.py
├── README.md
├── LICENSE
└── .gitignore
```

## Lisensi
MIT, lihat file `LICENSE`.
