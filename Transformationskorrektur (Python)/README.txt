Skripte zur Korrektur von Rohdatensätzen in ROS (aus meiner Masterarbeit).
    Rohdaten stammen aus Aufzeichnungen von Sensordaten eines mobilen Roboters (Laserscans, Odometrie) und einer Motion-Capturing-Anlage (Ground Truth). Die Datenkorrektur dient der Herstellung des Datenformats, welches die SLAM-Algorithmen voraussetzen, die im Rahmen der Evaluation ausgewertet werden sollten.

01_schaubild_trafo_correction.svg - Darstellung der zugrunde liegenden Problematik als Schaubild

02_flowchart_trafo_correction.svg - Lösungsansatz als UML-Flowchart

03_clean_and_rename.py            - Erstes Skript zur Vereinfachung der Rohdaten

04_crop_and_reset.py              - Zweites Skript zum Ausschneiden von Teildatensätzen und Nullen der entsprechenden Koordinatentransformationen
