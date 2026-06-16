# GO1 Unitree – Obstacle Avoidance

**Projektpraktikum – Systematische Analyse der Sicherheit in Embedded und AI-basierten Systemen (CYSEC)**
TU Darmstadt · Ridwan Islam, Mateo Tabak, Ahmet Yetisir & Vincent Rauscher

Autonome, rein lokale Korridor-Navigation für den **Unitree GO1 EDU**: Der Roboter
folgt einem Gang, hält sich selbstständig mittig, erkennt Hindernisse über einen
LiDAR-Sensor, umfährt sie reaktiv und stoppt gezielt an echten Korridorecken –
ganz ohne globale Karte oder SLAM.

> Der vollständige technische Bericht liegt in
> [`Go1_Unitree_Obstacle_Avoidance.pdf`](./Go1_Unitree_Obstacle_Avoidance.pdf).

---

## Aufgabe

Ziel des Versuchs war es, den GO1 auf eine festgelegte Route zu schicken und dabei
auftretende Hindernisse mittels **Obstacle Avoidance** durch Auslesen des
LiDAR-Sensors zu umgehen.

Dafür wurden zwei Ansätze betrachtet:

- **Ansatz 1 – Manuelles Codieren:** Die Route wird manuell vorgegeben, das
  Ausweichen erfolgt durch direktes Auslesen und Auswerten der LiDAR-Daten. Der
  Roboter folgt einer definierten Strecke und weicht gleichzeitig dynamisch aus.
- **Ansatz 2 – SLAM-basierte Navigation:** Mit HectorSLAM sollte zunächst eine
  Karte erstellt, der Roboter darauf lokalisiert und eine Route vom Start- zum
  Zielpunkt geplant werden. Geplant war ein Raspberry Pi als zentrale Plattform.

**Ergebnis:** Ansatz 1 wurde erfolgreich umgesetzt. Bei Ansatz 2 traten hardware-
und softwareseitige Hürden auf (instabile Netzwerkkette, headless laufender
LiDAR-Treiber, fehlende Odometrie für AMCL), die eine vollständige Implementierung
im Zeitrahmen verhinderten. Dieses Repo dokumentiert den Entwicklungsprozess, die
Herausforderungen und die finale, funktionierende lokale Hindernisumfahrung.

---

## Demos

### Obstacle Avoidance
<!-- VIDEO 1: Hier "go1 obstacle avoidance.mp4" per Drag & Drop reinziehen.
     GitHub ersetzt diese Zeile auto-->

https://github.com/user-attachments/assets/5558c97a-2d4c-4d2a-aed0-b5dc3ced77ce

### Automatischer Stopp bei Ecke
<!-- VIDEO 2: Hier "automatischerStopBeiEcke.mp4" per Drag & Drop reinziehen. -->


https://github.com/user-attachments/assets/3d5a3151-895a-48b2-a2d8-73792aa02032



### Obstacle Avoidance + Stopp bei Ecke (kombiniert)
<!-- VIDEO 3: Hier "obstacleAvoidanceUndStopBeiEcke.mp4" per Drag & Drop reinziehen. -->


https://github.com/user-attachments/assets/45e9e3b6-9757-4bb5-9861-5b51c887bd94


---

## Systemübersicht

| Komponente | Wert |
|------------|------|
| Roboter | Unitree GO1 EDU, High-Level-Control über ROS (`unitree_legged_sdk`) |
| Roboter-IP | LAN `192.168.123.161` · WLAN-Hotspot `192.168.12.107` |
| LiDAR | RoboSense RS-Helios 16P, IP `192.168.1.200` (Host: `192.168.1.210`) |
| LiDAR-Ports | MSOP (Messdaten) UDP `6699` · DIFOP (Konfig) UDP `7788` |
| OS / Middleware | Ubuntu 20.04 LTS · ROS Noetic · Workspace `~/catkin_ws` |
| Kommunikation | UDP über ROS-Bridge |

**Verwendete ROS-Pakete:** `rslidar_sdk` (LiDAR-Treiber),
`pointcloud_to_laserscan` (Punktwolke → 2D-Laserscan), `hector_slam` (Kartierung,
nur Ansatz 2), `unitree_legged_sdk` (GO1-Integration).

> **Hinweis zur Entwicklungsumgebung:** Eine virtuelle Maschine erwies sich als
> unzuverlässig (instabiles WLAN, falsch geroutete UDP-Ports, IP-Konflikte). Es
> wurde daher auf eine native Ubuntu-Installation gewechselt.

---

## Implementierung

Die finale Navigation (Ansatz 1) ist ein rein lokales, reaktives Verfahren. Der
Roboter steuert ausschließlich die Geschwindigkeiten **vx** (vorwärts) und **vy**
(seitwärts) und hält die Gierachse (yaw) konstant auf 0.

**Sensorik & Ausgänge.** Ausgewertet werden zwei Laser-Topics (`/scan` und
`/scan_low`). Für die Frontdetektion wird konservativ die kleinere Distanz aus
beiden Scans genutzt, damit auch flache Objekte sicher erkannt werden.
Steuerbefehle (vx, vy) gehen über die Unitree-SDK per UDP an den Roboter.

**Sektorbasierte Auswertung.** Die Scans werden in Sektoren unterteilt: Front
(±30°), Links/Rechts (je ≈ 90° zur Wandabstandsschätzung) und diagonale Sektoren
zur frühzeitigen Hinderniserkennung. Statt Minimalwerten werden Quantile (z. B.
20 %) verwendet, um einzelne Ausreißer zu dämpfen.

**Seitennachführung & Glättung.** Ein Wandfolger hält den Roboter mittig: Aus der
Differenz der Seitenabstände (dL, dR) wird ein Center-Error berechnet und über
einen gleitenden Mittelwert gefiltert. Daraus entsteht eine sanfte
Korrekturgeschwindigkeit vy, während vx über eine Rampe hochläuft. Eine Hysterese
sorgt dafür, dass nur bei kurzzeitig stabil freiem Frontbereich weitergefahren wird.

**Reaktive Umfahrung (zweistufig).**
1. *Vorwarn-Bias:* Schließt sich der Frontbereich langsam, wird ein kleiner
   seitlicher Bias zur freieren Seite addiert und vx leicht reduziert.
2. *Nahbereichs-Reaktion:* Bei engem Abstand wird vx stark begrenzt und eine
   gezielte seitliche Bewegung (vy) ausgelöst, bis die Front wieder frei ist.

**Corner-Stop.** Um eine echte Gang-Ecke von einer Tür zu unterscheiden, müssen
drei Bedingungen gleichzeitig erfüllt sein: Die Front schließt sich deutlich, rechts
bleibt eine Wandstruktur erkennbar und links öffnet sich ein breiter,
zusammenhängender Bereich. Treffen alle zu, stoppt der Roboter gezielt (GOAL-Zustand).

**Dynamischer Drift.** Der GO1 zeigte einen lastabhängigen Seitendrift (Schrittgröße,
Geschwindigkeit, v. a. Akkustand). Da yaw=0 gehalten wird, wirkt sich jede
Asymmetrie direkt lateral aus. Kompensiert wurde über asymmetrische Zielabstände
(`left_target`/`right_target`) entgegen der Driftrichtung, reduzierte
Reglerverstärkung (`k_center`) mit größerem `deadband_center` sowie eine optionale
batteriespannungsabhängige Korrektur.

---

## Setup & Ausführung

> Die Startskripte `run_go1.sh` und `run_invcenter.sh` werden **nicht** genutzt –
> die Befehle werden manuell im Terminal ausgeführt.

**1. ROS-Umgebung laden**
```bash
source /opt/ros/noetic/setup.bash
```

**2. Ins Beispielverzeichnis wechseln**
```bash
cd ~/unitree_legged_sdk/example_py
```

### Demo 1 – Corner / Gap-Aware Navigation

Realisiert die vollständige autonome Korridornavigation: mittig halten,
Hindernisse und Ecken erkennen, seitlich umfahren und danach re-zentrieren.

```bash
python3 go1_corridor_bypass_corner_pausealign_gapaware2.py \
  _dest_ip:=192.168.123.161 _rate:=50 _verbose:=true \
  _scan_topic:=/scan _scan_low_topic:=/scan_low \
  _vx_nom:=0.18 _ramp_time_s:=2.0 \
  _vx_abs_max:=0.30 _vy_abs_max:=0.30 \
  _front_width_deg:=60.0 _front_quantile:=p20 \
  _side_width_deg:=50.0 \
  _alpha_center_long:=0.03 _deadband_center:=0.07 _k_center:=0.55 \
  _vy_max:=0.08 _alpha_vy_out:=0.20 _max_dvy_s:=1.2 \
  _stop_thresh:=0.55 _resume_thresh:=0.80 _clear_needed_s:=0.80 \
  _min_front_clear:=0.15 _min_left_clear:=0.20 _min_right_clear:=0.20 _min_back_clear:=0.35 \
  _pass_diag_center_deg:=25.0 _pass_diag_width_deg:=35.0 \
  _pass_side_center_deg:=60.0 _pass_side_width_deg:=40.0 _pass_need:=0.26 \
  _side_min_clear:=0.20 _wall_min_clear:=0.22 \
  _left_target:=0.26 _right_target:=0.26 _k_wall:=0.55 \
  _vy_bypass_max:=0.20 _vy_min_side:=0.12 _wall_hard_min:=0.20 _vy_min_away:=0.10 \
  _vx_creep:=0.10 _recenter_tol:=0.05 \
  _bypass_min_time:=1.6 _bypass_timeout_s:=12.0 \
  _vy_min_center:=0.10 _vy_center_max:=0.18 _vx_center:=0.12 \
  _recenter_inv_timeout_s:=6.0 _recenter_inv_front_guard:=0.90 _pass_opp_clear:=0.26 \
  _left_front_center_deg:=60.0 _left_front_width_deg:=50.0 \
  _left_left_center_deg:=100.0 _left_left_width_deg:=40.0 \
  _corner_front_block:=1.05 _corner_open_r:=1.65 _corner_open_min_deg:=40.0 _corner_hold_s:=0.5 \
  _preavoid_thresh:=0.95 _k_preavoid:=0.35 _vy_bias_max:=0.12 \
  _align_enable:=true _align_period_s:=5.0 _align_max_time_s:=1.2 \
  _align_hold_tol_deg:=2.0 _align_hold_time_s:=0.2 \
  _align_front_guard_m:=0.9 _align_kp:=0.9 _align_max_rate:=0.18 _align_alpha:=0.12 \
  _align_pick_side:=auto _yaw_min_points:=12 _yaw_min_span_x:=0.18 _yaw_min_R2:=0.70
```

**Wichtige Parametergruppen**

| Parameter | Funktion |
|-----------|----------|
| `_dest_ip` | IP-Adresse des GO1 |
| `_vx_nom`, `_vy_max` | maximale lineare Geschwindigkeiten |
| `_deadband_center`, `_k_center` | Tuning der Zentrierung |
| `_stop_thresh`, `_resume_thresh` | Schwellen für Anhalten / Weiterfahren |
| `_corner_*` | Verhalten an Ecken (Corner-Stop) |
| `_align_*` | automatische Ausrichtung |
| `_min*_clear`, `_left_target`/`_right_target` | Sicherheitsabstände / Driftkompensation |

### Demo 2 – Inverse-Center & Diagnostic Watch

Vereinfachte Variante mit versetzter („inverser") Zentrierung und aktiver
LiDAR-Diagnose – zum schnellen Prüfen von Sensorik, Reglerstabilität und
Datenkonsistenz zwischen `/scan` und `/scan_low`.

```bash
python3 go1_corridor_bypass_invcenter_diagwatch_ultra_lowguard.py \
  _dest_ip:=192.168.123.161 \
  _rate:=50 _verbose:=true \
  _scan_topic:=/scan _scan_low_topic:=/scan_low
```

| Parameter | Funktion |
|-----------|----------|
| `_dest_ip` | IP-Adresse des GO1 |
| `_rate` | Ausführungsfrequenz in Hz |
| `_verbose` | zusätzliche Debug-Informationen |
| `_scan_topic`, `_scan_low_topic` | Lidar-Eingabedaten |

---

## Bekannte Probleme & Lösungen

| Problem | Ursache | Lösung |
|---------|---------|--------|
| Roboter zögert / bleibt vor Hindernissen stehen | Sicherheitsschwellen zu konservativ, oder `/scan_low` erkennt flaches Hindernis | `stop_thresh`/`resume_thresh` anpassen, Gewichtung des bodennahen Scans reduzieren |
| Nervöses Zick-Zack im Korridor | Zu aggressive Mittelhalte-Regelung oder LiDAR-Rauschen | `deadband_center` vergrößern, `alpha_center_long` erhöhen |
| Fährt zu nah an Wänden vorbei | Sicherheitsabstände zu klein, Drift unzureichend kompensiert | `left_target`/`right_target` asymmetrisch setzen, `k_center` reduzieren |
| Dynamischer Drift (meist nach rechts) | Sinkende Akkuspannung, variierende Schrittgröße | Reglerparameter dynamisch anpassen oder festen vy-Offset setzen |
| Frontkamera ohne Bilddaten | Kamera-Hardware während der Tests defekt | Im Projekt nicht nutzbar; künftig externe USB-Kamera mit eigener ROS-Node |
| AMCL / Lokalisierung instabil (Ansatz 2) | Fehlende Odometrie, fehlerhafte Scan-Zeitstempel, unvollständige TF-Kette | Eigene Odometrie schätzen (IMU + Velocity-Integration) oder GMapping/Cartographer/RTAB-Map testen |
| Instabile Kommunikation in VM | Ports/IPs (UDP 6699, 7788) blockiert oder falsch geroutet | Native Ubuntu-Installation; bei VM Bridged Adapter + offene UDP-Ports |
