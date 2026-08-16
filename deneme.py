#!/usr/bin/env python3

import numpy as np
from pathlib import Path
from scipy.spatial import cKDTree

# ============================================================
# Ayarlar
# ============================================================

MAP_FILE = Path("map.pcd")
TRAJ_FILE = Path("trajectory.pcd")
OUTPUT_FILE = Path("map_clean.pcd")

# Trajectory noktalarına ne kadar yakınlıktaki noktalar silinsin?
# 0.1 = 10 santimetre. Hassasiyeti duruma göre artırıp azaltabilirsiniz.
DISTANCE_THRESHOLD = 1.5 

# ============================================================
# PCD Okuma/Yazma Fonksiyonları (Önceki güncel hali)
# ============================================================

def read_pcd(path: Path):
    with path.open("rb") as f:
        header_lines = []
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError(f"{path} başlığı okunamadı.")
            decoded = line.decode("ascii", errors="ignore").strip()
            header_lines.append(decoded)
            if decoded.startswith("DATA"):
                break

        header = {}
        for line in header_lines:
            parts = line.split(maxsplit=1)
            if len(parts) == 2:
                header[parts[0].upper()] = parts[1]

        data_type = header.get("DATA", "").lower()
        points = int(header["POINTS"])

        dtype = np.dtype([
            ("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4"),
        ])

        if data_type == "binary":
            data = f.read()
            expected_bytes = points * 4 * 4
            cloud = np.frombuffer(data[:expected_bytes], dtype=dtype, count=points)
            
        elif data_type == "ascii":
            raw_data = np.loadtxt(f, dtype=np.float32, max_rows=points)
            cloud = np.empty(points, dtype=dtype)
            cloud["x"] = raw_data[:, 0]
            cloud["y"] = raw_data[:, 1]
            cloud["z"] = raw_data[:, 2]
            
            # Bazı trajectory pcd'lerinde intensity olmayabilir, hata vermemesi için kontrol:
            if raw_data.shape[1] > 3:
                cloud["intensity"] = raw_data[:, 3]
            else:
                cloud["intensity"] = 0.0

        else:
            raise RuntimeError(f"Desteklenmeyen veri tipi: {data_type}")

    return cloud

def write_binary_pcd(path: Path, cloud):
    points = len(cloud)
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        "FIELDS x y z intensity\n"
        "SIZE 4 4 4 4\n"
        "TYPE F F F F\n"
        "COUNT 1 1 1 1\n"
        f"WIDTH {points}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {points}\n"
        "DATA binary\n"
    )
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        f.write(cloud.tobytes())
    print(f"\nKaydedildi: {path}")
    print(f"Kalan Nokta Sayısı: {points}")

# ============================================================
# Ana İşlem: KDTree ile Fark Alma (Çıkarma)
# ============================================================

def main():
    print("Harita (map.pcd) yükleniyor...")
    map_cloud = read_pcd(MAP_FILE)
    
    print("Rota (trajectory.pcd) yükleniyor...")
    traj_cloud = read_pcd(TRAJ_FILE)

    print(f"Harita noktaları: {len(map_cloud):,}")
    print(f"Rota noktaları  : {len(traj_cloud):,}")

    # X, Y, Z koordinatlarını 3D dizi (Nx3) formatına çeviriyoruz
    map_xyz = np.vstack([map_cloud['x'], map_cloud['y'], map_cloud['z']]).T
    traj_xyz = np.vstack([traj_cloud['x'], traj_cloud['y'], traj_cloud['z']]).T

    print("\nKDTree oluşturuluyor (Bu işlem birkaç saniye sürebilir)...")
    # Trajectory noktalarından bir ağaç oluşturuyoruz
    tree = cKDTree(traj_xyz)

    print("Mesafeler hesaplanıyor ve noktalar siliniyor...")
    # Haritadaki her bir noktanın, en yakın trajectory noktasına olan mesafesini buluyoruz
    distances, _ = tree.query(map_xyz, k=1, workers=-1) # workers=-1 tüm işlemci çekirdeklerini kullanır

    # Belirlediğimiz mesafeden UZAKTA olan noktaları (True) tutuyoruz
    # Yani trajectory'ye yakın olanlar False olup silinecek
    mask = distances > DISTANCE_THRESHOLD

    clean_cloud = map_cloud[mask]
    
    removed_count = len(map_cloud) - len(clean_cloud)
    print(f"Silinen nokta sayısı: {removed_count:,}")
    
    write_binary_pcd(OUTPUT_FILE, clean_cloud)
    print("İşlem tamamlandı!")

if __name__ == "__main__":
    main()