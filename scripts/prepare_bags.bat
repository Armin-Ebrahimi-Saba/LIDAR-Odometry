@echo off
echo =======================================================
echo  LIO-SAM Bag Preparation Script gestartet
echo =======================================================

echo.
echo [1/3] Extrahiere GNSS und konvertiere ASPN IMU...
python bag_preparation\convert_bag.py
if %ERRORLEVEL% neq 0 goto error

echo.
echo [2/3] Konvertiere ROS2 Bag in ROS1 Bag...
if exist "rosbag_new.bag" del /F /Q "rosbag_new.bag"
python -m rosbags.convert --src rosbag_new --dst rosbag_new.bag
if %ERRORLEVEL% neq 0 goto error

echo.
echo [3/3] Optimiere Punktwolken (Rings/Timestamp) fuer LIO-SAM...
python bag_preparation\fix_ouster_bag.py
if %ERRORLEVEL% neq 0 goto error

echo.
echo =======================================================
echo  Erfolgreich abgeschlossen! 
echo  Die Datei lio_sam_ready.bag ist nun einsatzbereit.
echo =======================================================
pause
exit /b 0

:error
echo.
echo Ein Fehler ist aufgetreten! Abbruch.
pause
exit /b 1
