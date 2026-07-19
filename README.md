# LIO-SAM 6AXIS Setup und Reproduzierbarkeits-Dokumentation

Diese Dokumentation beschreibt, wie das Ouster OS0-32 LiDAR und dessen interne IMU genutzt werden können, um mit LIO-SAM 6AXIS eine konsistente Karte (Point Cloud Map) zu generieren.

## 1. Übersicht der Dateien und Skripte

Folgende Skripte und Konfigurationen arbeiten zusammen, um die Kartierung durchzuführen:

*   **`docker-compose.yml` & `Dockerfile`**: 
    Diese bauen und starten einen isolierten ROS Melodic Container, in dem LIO-SAM 6AXIS installiert ist. Das aktuelle Verzeichnis wird als `/workspace` in den Container eingebunden, um Skripte und Rosbags zu teilen.
*   **`fix_ouster_bag.py`**:
    *(Vorbereitung)* Ein Python-Skript, das die Zeitstempel (Timestamps) der IMU und LiDAR-Daten in der ursprünglichen `rosbag` synchronisiert und anpasst. Ouster-Sensoren ohne PTP-Zeitsynchronisation haben oft Probleme mit stark asynchronen Timestamps, was SLAM-Algorithmen zum Absturz bringt. Das Skript gibt die reparierte Datei `lio_sam_ready.bag` aus.
*   **`patch_yaml.py`**:
    *(Konfiguration)* Dieses Skript wird automatisch beim Start des Containers ausgeführt. Es modifiziert die LIO-SAM-Konfigurationsdatei `indoor_ouster128.yaml` direkt im Container. Es nimmt essenzielle Änderungen vor:
    *   **Extrinsische Rotation & RPY**: Auf die Identitätsmatrix gesetzt (`[1,0,0, 0,1,0, 0,0,1]`), da die Ouster-IMU nicht physisch gedreht ist.
    *   **Gravity (`imuGravity`)**: Auf `-9.80511` gesetzt, da die Ouster-IMU die tatsächliche Schwerkraft nach unten misst (im Gegensatz zur ROS REP-145 Konvention, die die Reaktionskraft nach oben angibt).
    *   **IMU Noise/Bias (`imuAccNoise`, etc.)**: Erhöht, da die interne IMU relativ ungenau ist. Ohne diese Anpassung würde die Optimierung dem IMU-Sensor zu sehr vertrauen und nach einigen Minuten wegen akkumulierter Fehler abdriften (Divergenz).
*   **`run_lio_sam.sh`**:
    *(Ausführung)* Das Hauptskript, das innerhalb des Docker-Containers gestartet wird. Es übernimmt folgende Aufgaben:
    1.  Spielt `use_sim_time` in die LIO-SAM Launch-Datei ein, damit LIO-SAM die Zeitstempel der Rosbag anstelle der echten Computer-Uhr verwendet.
    2.  Startet LIO-SAM und RViz im Hintergrund.
    3.  Startet den Playback der `lio_sam_ready.bag` und biegt die Topic-Namen per Remap auf die von LIO-SAM erwarteten Namen um (`/ouster/points` -> `/os_cloud_node/points`, `/ouster/imu_meas` -> `/stim300/imu/data_raw`).
    4.  Führt automatisch einen Speichervorgang (`rosservice call ...`) aus, sobald die Rosbag fertig abgespielt wurde.

---

## 2. Schritt-für-Schritt Anleitung

### Schritt 1: Rosbag vorbereiten (Nur beim ersten Mal oder bei neuen Daten)
Da Ouster-Rohdaten oft problematische Zeitstempel haben, muss die Rosbag zuerst bereinigt werden. Stelle sicher, dass deine rohe Rosbag-Datei im Ordner liegt (z.B. `rosbag2_2024_...`).
Führe das Skript auf deinem Host-PC (mit installiertem `rosbags` Paket für Python) aus:
```bash
python fix_ouster_bag.py <deine_input_bag.bag> lio_sam_ready.bag
```
*Dieses Skript wurde bereits ausgeführt und die `lio_sam_ready.bag` liegt bereit.*

### Schritt 2: Docker Container starten
Stelle sicher, dass Docker Desktop läuft. Öffne ein Terminal (PowerShell) in diesem Ordner und führe aus:
```bash
docker compose up
```
*(Um ihn im Hintergrund laufen zu lassen, nutze `docker compose up -d`, aber ohne `-d` siehst du direkt alle Outputs und Fehlermeldungen).*

### Schritt 3: Der automatische Ablauf
Sobald `docker compose up` läuft, passiert folgendes automatisch:
1. Der Container führt `run_lio_sam.sh` aus.
2. Das Skript patcht die Konfiguration (`patch_yaml.py`).
3. LIO-SAM und RViz werden gestartet. RViz öffnet sich auf deinem Rechner über X11 (VcXsrv).
4. Die `lio_sam_ready.bag` wird abgespielt. Du siehst in RViz, wie sich die Map langsam aufbaut.

### Schritt 4: Ergebnisse speichern
Sobald die Rosbag komplett durchgelaufen ist (nach ca. 13-14 Minuten), wird im Terminal die Meldung angezeigt:
`Rosbag finished playing. Automatically saving the map to /workspace/maps/...`

Das Skript ruft dann den ROS Service `/lio_sam_6axis/save_map` auf. 
**Die gespeicherten `.pcd` Punktwolken-Dateien findest du direkt auf deinem Windows-Desktop im neuen Ordner `maps/`.**

> **Manuelles Speichern:** Falls du den Vorgang vorzeitig abbrechen und den bisherigen Stand speichern möchtest, öffne ein zweites Terminal in diesem Ordner, hänge dich in den laufenden Container und rufe den Service manuell auf:
> `docker exec lio_sam_6axis /bin/bash -c "source /opt/ros/melodic/setup.bash && rosservice call /lio_sam_6axis/save_map"`

---

## 3. Fehlerbehebung

*   **RViz öffnet sich nicht:** Stelle sicher, dass VcXsrv (Xming) auf Windows läuft und `Disable access control` in den Einstellungen aktiviert ist.
*   **"Large velocity" Warnungen:** Wenn diese Fehler in den Logs auftauchen und die Map explodiert, stimmt die IMU-Konfiguration nicht. Prüfe, ob `patch_yaml.py` korrekt aufgerufen wurde.
*   **Map bleibt leer in RViz:** Klicke in RViz links auf "Add" -> "PointCloud2" und setze das Topic auf `/lio_sam_6axis/mapping/map_global` oder `/lio_sam_6axis/deskew/cloud_deskewed`.

## Control bag play
* Pause: docker exec lio_sam_6axis /bin/bash -c "source /opt/ros/melodic/setup.bash && rosservice call /bag_player/pause_playback true"

* Weiter: docker exec lio_sam_6axis /bin/bash -c "source /opt/ros/melodic/setup.bash && rosservice call /bag_player/pause_playback false"

---

## 4. Erweiterte GNSS (GPS) Integration

Um die Genauigkeit der Karte über lange Strecken zu gewährleisten und den Lidar-Drift auszugleichen, wurde das System um eine fortschrittliche GNSS-Integration erweitert. Die LIO-SAM-Standardimplementierung hatte in unserem Fall Probleme mit dem stark rauschenden GPS am Startpunkt (Spaghetti-Knoten).

Diese Probleme wurden mit folgenden **C++-Patches** in LIO-SAM behoben:
*   **`simpleGpsOdom_patched.cpp`**: Die Odometrie-Knoten-Berechnung wartet nun, bis sich der Roboter (basierend auf reiner Lidar-Odometrie) **5 Meter** vom Startpunkt wegbewegt hat. Erst dann wird eine stabile, verlässliche Ausrichtung (Yaw) aus der zurückgelegten Strecke berechnet und der Ursprung für das GNSS gesetzt. Dies verhindert fehlerhafte Startwinkel durch stehendes Sensorrauschen.
*   **`mapOptmizationGps.cpp`**: Anstatt darauf zu warten, dass das GPS-Signal initialisiert wird (was nun 5 Meter dauern würde und den gesamten SLAM-Prozess blockieren würde), startet der Lidar-Mapping-Prozess sofort und unabhängig. Sobald nach 5 Metern die ersten verlässlichen GPS-Koordinaten eintreffen, fügt LIO-SAM diese nachträglich als Optimierungs-Faktoren (GTSAM-Graph) ein und richtet die bisher erstellte Karte nahtlos am GNSS aus.

> **Hinweis:** Da `mapOptmizationGps.cpp` modifiziert wurde, wird diese Datei nun direkt über die `docker-compose.yml` als Volume in den Container eingebunden, damit sie beim Container-Start (durch `catkin build`) live mitkompiliert wird.

### 4.1. LIO-SAM mit GNSS starten
Zusätzlich zum regulären Skript existiert ein eigenes Start-Skript für den Betrieb mit GNSS-Korrektur:
```bash
docker compose run --rm lio_sam_6axis /workspace/run_lio_sam_gnss.sh
```
*(Oder falls du in der `docker-compose.yml` den `command` auf `./run_lio_sam_gnss.sh` geändert hast, reicht ein einfaches `docker compose up`)*.

Dieses Skript speichert die optimierte Trajektorie (die exakt an die echten GPS-Punkte angepasst ist) nach Abschluss im Ordner `maps_gnss_<Datum>/`.

---

## 5. Visualisierung & Evaluation (Plotly)

Um die Ergebnisse auszuwerten und verschiedene Sensordaten visuell miteinander zu vergleichen, stehen folgende Python-Skripte zur Verfügung:

*   **`extract_origin.py`**: Ein ROS-Python-Skript, das im Container ausgeführt werden kann, um aus den rohen Bag-Daten (`/gps/fix`) den echten initialen Referenzwinkel nach 5 Metern zurückgelegter Strecke (Yaw) sowie den LLA-Ursprung zu extrahieren. Die extrahierten Werte werden in `gnss_origin.json` und `raw_gnss.json` (für die Plot-Linie) gespeichert.
*   **`plot_comparison.py`**: Das Hauptskript zur Evaluierung. Es läuft komplett außerhalb von ROS und benötigt nur Standard-Python-Bibliotheken (wie Numpy). Es extrahiert und vergleicht automatisch:
    1.  Die **Ground Truth** aus dem `Outdoor1`-Datensatz (automatisch gefiltert und synchronisiert nach Lidar-Zeitstempeln).
    2.  Das **rohe eingebaute GNSS-Signal** (aus `raw_gnss.json`).
    3.  Die freie, driftende **LIO-SAM Trajektorie ohne GNSS** (`maps/garden_day/optimized_odom_tum.txt`).
    4.  Die **GNSS-optimierte LIO-SAM Trajektorie** (aus dem neusten `maps_gnss_...` Ordner).

Das Skript konvertiert alle lokalen Lidar-Koordinaten zurück in globale GPS-Koordinaten (WGS84 LLA) und berechnet automatisch Korrekturwinkel (Rotationen) anhand einer 60-Meter Strecke, um die Lidar-Karten und Ground Truth perfekt auf der echten Weltkarte übereinander zu legen. 

Das Ergebnis wird am Ende als interaktive HTML-Karte generiert:
**`plot_viewer_map.html`**: Diese Datei lässt sich einfach in jedem Browser (Chrome, Firefox, Safari) per Doppelklick öffnen. Sie bietet eine vollständig zoombare OpenStreetMap-Ebene, auf der alle Linien (Ground Truth, GNSS-Rohdaten, Lidar-Drift, Lidar+GNSS-Optimierung) interaktiv ein- und ausgeblendet werden können.