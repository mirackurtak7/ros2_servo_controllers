# Servo Motor ve Odometry Test Scripti
Write-Host "Servo Motor ve Odometry Test Scripti" -ForegroundColor Green
Write-Host "=====================================" -ForegroundColor Green

# WSL2'de ROS 2 durumunu kontrol et
Write-Host "1. WSL2'de ROS 2 durumu kontrol ediliyor..." -ForegroundColor Yellow
$wslTest = wsl bash -c "source /opt/ros/humble/setup.bash && ros2 --version"
if ($LASTEXITCODE -eq 0) {
    Write-Host "ROS 2 çalışıyor: $wslTest" -ForegroundColor Green
} else {
    Write-Host "ROS 2 kurulu değil veya çalışmıyor!" -ForegroundColor Red
    Write-Host "WSL2'de setup_wsl.sh scriptini çalıştırın." -ForegroundColor Yellow
    exit 1
}

# ESP32 bağlantısını kontrol et
Write-Host "2. ESP32 bağlantısı kontrol ediliyor..." -ForegroundColor Yellow
$comPorts = Get-WmiObject -Class Win32_SerialPort | Select-Object Name, DeviceID, Description
Write-Host "Mevcut seri portlar:" -ForegroundColor Cyan
$comPorts | Format-Table -AutoSize

# WSL2'de USB cihazlarını kontrol et
Write-Host "3. WSL2'de USB cihazları kontrol ediliyor..." -ForegroundColor Yellow
$usbDevices = wsl bash -c "ls /dev/tty* 2>/dev/null || echo 'USB cihazı bulunamadı'"
Write-Host "WSL2'deki seri portlar: $usbDevices" -ForegroundColor Cyan

# Test komutları
Write-Host "4. Test komutları hazırlanıyor..." -ForegroundColor Yellow

$testCommands = @"

# Micro-ROS Agent'ı başlatmak için:
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 baudrate=115200"

# Topic'leri listelemek için:
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"

# Odometry mesajlarını dinlemek için:
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /odom"

# Velocity mesajlarını dinlemek için:
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 topic echo /velocity"

# Cmd_vel mesajı göndermek için (ileri hareket):
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'"

# Cmd_vel mesajı göndermek için (dönüş):
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}'"

# Servo kontrolü için:
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub /servo_control std_msgs/msg/Int32 '{data: 90}'"

# PID gains ayarlamak için:
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 topic pub /pid_gains std_msgs/msg/Float64 '{data: 2.5}'"

# Node'ları listelemek için:
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 node list"

# Service'leri listelemek için:
wsl bash -c "source /opt/ros/humble/setup.bash && ros2 service list"

"@

Write-Host "Test komutları:" -ForegroundColor Cyan
Write-Host $testCommands -ForegroundColor White

# Otomatik test başlatma
Write-Host "5. Otomatik test başlatılıyor..." -ForegroundColor Yellow
Write-Host "Micro-ROS Agent'ı başlatmak için Enter'a basın..." -ForegroundColor Yellow
Read-Host

# WSL2'de Micro-ROS Agent'ı başlat
Write-Host "Micro-ROS Agent başlatılıyor..." -ForegroundColor Green
Start-Process -FilePath "wsl" -ArgumentList "bash", "-c", "source /opt/ros/humble/setup.bash && ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 baudrate=115200" -WindowStyle Normal

# Test senaryoları
Write-Host "6. Test senaryoları hazırlanıyor..." -ForegroundColor Yellow

$testScenarios = @"

=== TEST SENARYOLARI ===

1. Temel Servo Kontrolü:
   - ESP32'yi bağlayın ve firmware'i yükleyin
   - Micro-ROS Agent'ı başlatın
   - Servo kontrolü test edin:
     ros2 topic pub /servo_control std_msgs/msg/Int32 '{data: 0}'
     ros2 topic pub /servo_control std_msgs/msg/Int32 '{data: 90}'
     ros2 topic pub /servo_control std_msgs/msg/Int32 '{data: 180}'

2. Hareket Kontrolü:
   - Cmd_vel ile hareket test edin:
     ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'
     ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}'

3. Odometry Testi:
   - Odometry mesajlarını dinleyin:
     ros2 topic echo /odom
   - Robot hareket ederken pozisyon değişimini gözlemleyin

4. PID Tuning:
   - PID parametrelerini ayarlayın:
     ros2 topic pub /pid_gains std_msgs/msg/Float64 '{data: 1.5}'
     ros2 topic pub /pid_gains std_msgs/msg/Float64 '{data: 3.0}'

5. Gerçek Zamanlı Kontrol:
   - Teleop ile gerçek zamanlı kontrol:
     ros2 run teleop_twist_keyboard teleop_twist_keyboard

"@

Write-Host $testScenarios -ForegroundColor Cyan

Write-Host "Test tamamlandı!" -ForegroundColor Green
Write-Host "ESP32'nizi USB ile bağlayın ve firmware'i yükleyin." -ForegroundColor Cyan
Write-Host "Sonra test senaryolarını sırayla deneyin." -ForegroundColor Yellow 