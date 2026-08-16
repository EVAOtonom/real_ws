import open3d as o3d
import numpy as np
import cv2
import yaml

def pcd_to_pgm_and_yaml(pcd_path, pgm_path, yaml_path, resolution=0.05, occupied_thresh=0.65, free_thresh=0.25):
    # PCD dosyasını yükle
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    # X ve Y koordinatlarını al
    x = points[:, 0]
    y = points[:, 1]

    # Minimum ve maksimum değerleri bul
    x_min, x_max = x.min(), x.max()
    y_min, y_max = y.min(), y.max()

    # Görüntü boyutunu hesapla
    width = int(np.ceil((x_max - x_min) / resolution)) + 1
    height = int(np.ceil((y_max - y_min) / resolution)) + 1

    # Piksel koordinatlarına dönüştür
    x_pix = ((x - x_min) / resolution).astype(int)
    y_pix = ((y - y_min) / resolution).astype(int)

    # Beyaz arka planlı boş görüntü oluştur
    img = np.ones((height, width), dtype=np.uint8) * 255

    # Noktaları siyah olarak işaretle
    for xi, yi in zip(x_pix, y_pix):
        img[height - 1 - yi, xi] = 0

    # PGM dosyasını kaydet
    cv2.imwrite(pgm_path, img)
    print(f"PGM dosyası kaydedildi: {pgm_path}")

    # YAML dosyasını oluştur
    yaml_data = {
        'image': pgm_path.split('/')[-1],  # Sadece dosya adı
        'resolution': float(resolution),
        'origin': [float(x_min), float(y_min), 0.0],  # PCD'nin min koordinatları
        'negate': 0,
        'occupied_thresh': occupied_thresh,
        'free_thresh': free_thresh,
        'mode': 'trinary'  # Varsayılan mod
    }

    with open(yaml_path, 'w') as file:
        yaml.dump(yaml_data, file, default_flow_style=None)
    print(f"YAML dosyası kaydedildi: {yaml_path}")

# Örnek kullanım
pcd_to_pgm_and_yaml(
    pcd_path="map.pcd",
    pgm_path="harita.pgm",
    yaml_path="harita.yaml",
    resolution=0.05
)
