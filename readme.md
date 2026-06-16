

https://github.com/user-attachments/assets/1dc43d60-319c-4f6e-a0d0-ffc84de5b7b0



https://github.com/user-attachments/assets/4ca346ea-7c53-405b-8373-54fdbda97b9f





https://github.com/user-attachments/assets/0fe455fd-f4b9-4483-94c2-b8955bd9502d

Diese Datei beschreibt, welche Befehle in den beiden Startskripten ausgeführt werden und welche Funktion sie haben.
Die Skripte selbst (run_go1.sh und run_invcenter.sh) werden nicht genutzt –
stattdessen können die Befehle manuell im Terminal ausgeführt werden.

⸻

	1.	ROS-Umgebung laden

source /opt/ros/noetic/setup.bash

Dieser Befehl lädt die ROS-Umgebung (Robot Operating System).
Damit werden alle Pfade und Tools verfügbar, die nötig sind,
um ROS-basierte Python-Skripte auszuführen.
Ohne diesen Schritt funktionieren viele ROS-Kommandos nicht.

⸻

	2.	In das Beispielverzeichnis wechseln

cd …/unitree_legged_sdk/example_py


⸻

	3.	Python-Skripte ausführen

DEMO 1 – Corner / Gap-Aware Navigation

python3 go1_corridor_bypass_corner_pausealign_gapaware2.py 
_dest_ip:=192.168.123.161 _rate:=50 _verbose:=true 
_scan_topic:=/scan _scan_low_topic:=/scan_low 
_vx_nom:=0.18 _ramp_time_s:=2.0 
_vx_abs_max:=0.30 _vy_abs_max:=0.30 
_front_width_deg:=60.0 _front_quantile:=p20 
_side_width_deg:=50.0 
_alpha_center_long:=0.03 _deadband_center:=0.07 _k_center:=0.55 
_vy_max:=0.08 _alpha_vy_out:=0.20 _max_dvy_s:=1.2 
_stop_thresh:=0.55 _resume_thresh:=0.80 _clear_needed_s:=0.80 
_min_front_clear:=0.15 _min_left_clear:=0.20 _min_right_clear:=0.20 _min_back_clear:=0.35 
_pass_diag_center_deg:=25.0 _pass_diag_width_deg:=35.0 
_pass_side_center_deg:=60.0 _pass_side_width_deg:=40.0 _pass_need:=0.26 
_side_min_clear:=0.20 _wall_min_clear:=0.22 
_left_target:=0.26 _right_target:=0.26 _k_wall:=0.55 
_vy_bypass_max:=0.20 _vy_min_side:=0.12 _wall_hard_min:=0.20 _vy_min_away:=0.10 
_vx_creep:=0.10 _recenter_tol:=0.05 
_bypass_min_time:=1.6 _bypass_timeout_s:=12.0 
_vy_min_center:=0.10 _vy_center_max:=0.18 _vx_center:=0.12 
_recenter_inv_timeout_s:=6.0 _recenter_inv_front_guard:=0.90 _pass_opp_clear:=0.26 
_left_front_center_deg:=60.0 _left_front_width_deg:=50.0 
_left_left_center_deg:=100.0 _left_left_width_deg:=40.0 
_corner_front_block:=1.05 _corner_open_r:=1.65 _corner_open_min_deg:=40.0 _corner_hold_s:=0.5 
_preavoid_thresh:=0.95 _k_preavoid:=0.35 _vy_bias_max:=0.12 
_align_enable:=true _align_period_s:=5.0 _align_max_time_s:=1.2 
_align_hold_tol_deg:=2.0 _align_hold_time_s:=0.2 
_align_front_guard_m:=0.9 _align_kp:=0.9 _align_max_rate:=0.18 _align_alpha:=0.12 
_align_pick_side:=auto _yaw_min_points:=12 _yaw_min_span_x:=0.18 _yaw_min_R2:=0.70

Dieses Kommando startet ein Python-Programm zur autonomen Korridornavigation.
Der GO1 nutzt Lidar-Daten von /scan und /scan_low, um Hindernisse zu erkennen,
sich zu zentrieren, Ecken zu erkennen und Hindernisse zu umgehen.

Parametergruppen:
	•	_dest_ip – IP-Adresse des GO1-Roboters
	•	_vx_nom, _vy_max – maximale lineare Geschwindigkeiten
	•	_deadband_center, _k_center – Tuningparameter für die Zentrierung
	•	corner* – Verhalten an Ecken
	•	align* – automatische Ausrichtung
	•	min*_clear – Abstandsgrenzen für Kollisionserkennung

Diese Parameter steuern das Bewegungsverhalten, die Ausweichlogik und das Re-Zentrieren im Korridor.

⸻

DEMO 2 – Inverse-Center & Diagnostic Watch

python3 go1_corridor_bypass_invcenter_diagwatch_ultra_lowguard.py 
_dest_ip:=192.168.123.161 
_rate:=50 _verbose:=true 
_scan_topic:=/scan _scan_low_topic:=/scan_low

Dieses Kommando startet eine vereinfachte Variante, bei der:
	•	nur wenige Parameter notwendig sind,
	•	die Lidar-Daten überwacht werden,
	•	der Roboter eine versetzte („inverse“) Zentrierung nutzt, um Engstellen zu umgehen.

Parameter:
	•	_dest_ip – IP-Adresse des GO1
	•	_rate – Ausführungsfrequenz in Hz
	•	_verbose – zeigt zusätzliche Debug-Informationen
	•	_scan_topic, _scan_low_topic – Lidar-Eingabedaten
