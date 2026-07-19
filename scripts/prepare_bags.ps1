<#
.SYNOPSIS
Dieses Skript führt alle notwendigen Schritte zur Vorbereitung des originalen ROS-Bags für LIO-SAM aus.
#>

Write-Host "=======================================================" -ForegroundColor Cyan
Write-Host " LIO-SAM Bag Preparation Script gestartet" -ForegroundColor Cyan
Write-Host "=======================================================" -ForegroundColor Cyan

# Schritt 1
Write-Host "`n[1/3] Extrahiere GNSS und konvertiere ASPN IMU..." -ForegroundColor Yellow
python bag_preparation/convert_bag.py
if ($LASTEXITCODE -ne 0) {
    Write-Host "Fehler bei convert_bag.py. Abbruch." -ForegroundColor Red
    exit 1
}

# Schritt 2
Write-Host "`n[2/3] Konvertiere ROS2 Bag in ROS1 Bag..." -ForegroundColor Yellow
# Falls rosbag_new.bag bereits existiert, löschen (rosbags-convert mag keine existierenden Dateien)
if (Test-Path "rosbag_new.bag") {
    Remove-Item "rosbag_new.bag" -Force
}
python -m rosbags.convert --src rosbag_new --dst rosbag_new.bag
if ($LASTEXITCODE -ne 0) {
    Write-Host "Fehler bei rosbags-convert. Ist die rosbags Bibliothek installiert?" -ForegroundColor Red
    exit 1
}

# Schritt 3
Write-Host "`n[3/3] Optimiere Punktwolken (Rings/Timestamp) für LIO-SAM..." -ForegroundColor Yellow
python bag_preparation/fix_ouster_bag.py
if ($LASTEXITCODE -ne 0) {
    Write-Host "Fehler bei fix_ouster_bag.py. Abbruch." -ForegroundColor Red
    exit 1
}

Write-Host "`n=======================================================" -ForegroundColor Green
Write-Host " Erfolgreich abgeschlossen!" -ForegroundColor Green
Write-Host " Die Datei lio_sam_ready.bag ist nun einsatzbereit." -ForegroundColor Green
Write-Host "=======================================================" -ForegroundColor Green
