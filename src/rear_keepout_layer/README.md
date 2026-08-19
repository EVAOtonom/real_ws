# rear_keepout_layer (ROS 2 Humble / Nav2)

Aracla birlikte donen, global costmap uzerine U-seklinde LETHAL_OBSTACLE yazan dinamik costmap pluginidir.

## Varsayilan geometri

Arac frame'i: `base_footprint`, +X ileri, +Y sol.

- Yol / serit genisligi: `6.0 m` (`y = -3.0 ... +3.0`)
- Arac footprint arka siniri: `x = -1.35 m`
- Arka tampon ile yatay bariyer arasi bosluk: `0.80 m`
- Arka yatay bariyer kalinligi: `0.40 m`
- Yan kol kalinligi: `0.35 m`
- Yan kollarin on ucu: `x = +1.80 m`

Yaklasik gorunum:

```text
                       +X / ILERI
                           ^
                           |
        y=+3.0   ##########|##########   <- sol yan kol (0.35 m)
                  #        |        #
                  #     [ ARAC ]    #
                  #        |        #
        y=-3.0   ##########|##########   <- sag yan kol (0.35 m)
                  ###################    <- arka bar x=-2.55..-2.15

                  U'nun ON TARAFI ACIK
```

Not: ASCII cizimde arka bar gorsel olarak altta gosterilmistir; gercek geometri arac frame'inde `x=-2.55 ... -2.15 m` ve `y=-3 ... +3 m` araligindadir.

## Neden sadece global costmap?

Bu katman global planner'in arac arkasina rota cizmesini engellemek icindir. Local costmap'e eklenmez; boylece controller / collision checker arac arkasindaki sentetik duvardan gereksiz etkilenmez.

## Plugin sirasi

`rear_keepout_layer` mutlaka `inflation_layer` sonrasinda, tercihen plugins listesinin EN SONUNDA olsun. Bu sayede U duvari inflation ile arac tarafina sisirilmez.

## Derleme

Paketi workspace'in `src` klasorune kopyalayin:

```bash
cd ~/real_ws/src
# rear_keepout_layer klasoru burada olmali
cd ~/real_ws
colcon build --packages-select rear_keepout_layer --symlink-install
source install/setup.bash
```

Plugin'i kontrol etmek icin:

```bash
ros2 plugin list | grep -i rear_keepout
```

## Runtime ac / kapat

Park veya ozel manevrada arka bolgeye rota cizmek gerekecekse layer kapatilabilir:

```bash
ros2 param set /global_costmap/global_costmap rear_keepout_layer.enabled false
```

Tekrar acmak icin:

```bash
ros2 param set /global_costmap/global_costmap rear_keepout_layer.enabled true
```

Ayni komutlar `scripts/rear_keepout_off.sh` ve `scripts/rear_keepout_on.sh` ile de verilebilir.

## Onemli

Bu paket global costmap update hizina baglidir. Verilen YAML'da global `update_frequency` 1 Hz'den 3 Hz'e cikarildi. 0.8 m/s hizda teorik konum gecikmesi yaklasik 0.27 m seviyesine iner; publish_frequency 1 Hz birakildigi icin RViz/network yuku gereksiz artmaz.

## ParkingGrid notu

Bu layer `global_costmap` icinde oldugu icin ayni global costmap'i kullanan `ParkingGrid` planlarini da etkiler. Park manevrasinda aracin arkasina plan cizilmesi gerekiyorsa park baslamadan once layer'i kapatin ve park bittiginde tekrar acin.

Ayrica verilen mevcut YAML'da park bolumunun yorumlari "ileri + geri" dese de `ParkingGrid.motion_model_for_search` halen `DUBIN` ve `ParkFollowPath.allow_reversing` halen `false`. Bu bundle bu iki park ayarini bilerek degistirmedi; normal navigasyondaki rear-keepout gorevinden ayri tuttuk.
