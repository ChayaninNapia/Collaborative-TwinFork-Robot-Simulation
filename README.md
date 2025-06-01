# Collaborative-TwinFork-Robot-Simulation

Twinfork robot ได้แรงบันดาลใจมาจาก https://filics.eu/en/ ลักษณะคือ หุ่นยอน forklift 2 ที่ทำงานร่วมกัน
ยก pallet

```
mkdir -p ~/6509_ws/src
cd ~/6509_ws/src/
git clone https://github.com/ChayaninNapia/Collaborative-TwinFork-Robot-Simulation.git
cd ~/6509_ws
colcon build
source install/setup.bash
```

## Launch the Simulation

```
ros2 launch twinfork_gazebo multi_sim.launch.py 
```

note: ถ้ารันแล้ว controller fail หรือ โหลดไม่ครบ ให้ทำการ kill procress แล้ว รันอีกครั้ง 

```
killall -9 gzserver gzclient
```

![bruh](image/red_log.png)
![bruh](image/green_log.png)

ถ้ามีข้อความว่า activated controller ได้แล้ว 2 ครั้งแบบนี้ แสดงว่าใช้งานได้แล้ว

## run the GUI

เปิด termianl ขึ้นมาอีก หน้าแล้วรันคำสั่ง

```
ros2 run twinfork_controller taskspace_server.py 
```

## Let's Test

1. ให้ไปที่เมนู insert ของ gazebo ในรายการให้หา path ที่ถูก add ใน project ชื่อ prefix จะขึ้นอยู่กับ ชื่อ user ของผู้ใช้ ตามภาพตัวอย่างก็คือตรงไฮไลท์สีส้ม 

2. ให้กดที่ euro pallet เพื่อ spawn pallet ขึ้นมา คลิกซ้าย ที่ pallet แล้วจะมี เมนูทางแถบด้านข้างขึ้นมา ให้ไปที่ pose เพื่อเลือก pose ที่เราต้องการ x y yaw

3. ไปที่ taskserver gui แล้วนำ pose ที่ตั้งไว้ใส่ไปใน lift command แล้วกด send 

4. รอจนกว่าหุ่นจะไปถึง แล้วอยู่ใน สถานะ Lifting ทั้งคู่ จากนั้นใส่ ตำแหน่งที่เราต้องการจะย้ายไปที่ Drop command