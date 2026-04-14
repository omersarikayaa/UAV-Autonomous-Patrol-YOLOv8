import airsim
import cv2
import numpy as np
from ultralytics import YOLO
import math 

# 1. Sistem Kurulumu
print("Hocam CityEnvironment Operasyon Merkezi hazırlanıyor...")
model = YOLO('yolov8n.pt')
client = airsim.MultirotorClient()
client.confirmConnection()

# ROTA TANIMLAMASI
devriye_noktalari = [
    (100, 0, -27, 5),    
    (100, 40, -27, 5),   
    (10, 80, -27, 5),    
    (-70, 80, -27, 5),   
    (-70, -10, -27, 5),  
    (0, 0, -27, 5)       
]

# --- HAZIRLIK, KALKIŞ VE KAMERA AÇISI ---
for drone_name in ["D1", "D2"]:
    client.enableApiControl(True, vehicle_name=drone_name)
    client.armDisarm(True, vehicle_name=drone_name)
    client.takeoffAsync(vehicle_name=drone_name).join()

    print(f"Hocam {drone_name} havalandı, 10 metre sola taktiksel kayış yapıyor...")
    client.moveToPositionAsync(0, -10, -22, 3, vehicle_name=drone_name).join()

    print(f"Hocam {drone_name} kamerası sokak taraması için aşağı eğiliyor...")
    radyan_aci = math.radians(-45) 
    client.simSetCameraPose("0", airsim.Pose(airsim.Vector3r(0, 0, 0), airsim.to_quaternion(radyan_aci, 0, 0)), vehicle_name=drone_name)

print("Hocam hazırlıklar bitti! Ekran açılıyor ve devriye başlıyor...")

win_name = "SAVASAN IHA - CITY DEVRIYE"
cv2.namedWindow(win_name, cv2.WND_PROP_FULLSCREEN)
cv2.setWindowProperty(win_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

try:
    point_index = 0
    while True:
        # 2. Otonom Hareket
        for drone_name in ["D1", "D2"]:
            x, y, z, speed = devriye_noktalari[point_index]
            offset = 5 if drone_name == "D2" else 0
            client.moveToPositionAsync(x + offset, y + offset, z, speed, vehicle_name=drone_name)

        # 3. Görüntü İşleme
        processed_frames = []
        for drone_name in ["D1", "D2"]:
            responses = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
            ], vehicle_name=drone_name)
            
            if not responses:
                continue

            res = responses[0]
            img = np.frombuffer(res.image_data_uint8, dtype=np.uint8).reshape(res.height, res.width, 3)
            img = cv2.resize(img, (960, 540))
            
            results = model(img, verbose=False)
            processed_frames.append(results[0].plot())

        if len(processed_frames) == 2:
            top_row = np.hstack((processed_frames[0], processed_frames[1]))
            cv2.putText(top_row, f"HEDEF: {point_index + 1}/6 | OPERASYON AKTIF", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(win_name, top_row)

        # 4. AKILLI TUŞ KONTROLÜ
        time_check = cv2.waitKey(1) & 0xFF
        
        # 'n' tuşu: Bir sonraki hamle, 6'dan sonra milimetrik iniş
        if time_check == ord('n'): 
            if point_index < len(devriye_noktalari) - 1:
                point_index += 1
                print(f"Hocam yeni hedefe yönlenildi: {devriye_noktalari[point_index]}")
            else:
                print("Hocam 6 hamle bitti, milimetrik iniş protokolü başlıyor...")
                break

       
        if time_check == ord('q'):
            print("Hocam operasyon iptal! Geldiğimiz yollardan üsse dönüyoruz...")
            
            # 1. Aşama: Geldiğin yolları sondan başa (0. indekse kadar) geri yürü
            for i in range(point_index, -1, -1):
                x_rev, y_rev, z_rev, s_rev = devriye_noktalari[i]
                print(f"Geri dönüş: {i+1}. durak üzerinden geçiliyor...")
                
                for drone_name in ["D1", "D2"]:
                    offset = 5 if drone_name == "D2" else 0
                    client.moveToPositionAsync(x_rev + offset, y_rev + offset, z_rev, s_rev, vehicle_name=drone_name)
                
                # Kamera donmasın ve hedefe varıldığı anlaşılsın diye Mini-Radar Döngüsü
                while True:
                    frames_q = []
                    for drone_name in ["D1", "D2"]:
                        resp = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name=drone_name)
                        if resp:
                            img_q = np.frombuffer(resp[0].image_data_uint8, dtype=np.uint8).reshape(resp[0].height, resp[0].width, 3)
                            frames_q.append(cv2.resize(img_q, (960, 540)))
                    
                    if len(frames_q) == 2:
                        disp_q = np.hstack((frames_q[0], frames_q[1]))
                        cv2.putText(disp_q, f"SISTEM: GERI DONUS YAPIYOR... ROTA {i+1}/6", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
                        cv2.imshow(win_name, disp_q)
                    cv2.waitKey(1)
                    
                    # Hedefe 3 metre kaldıysa bu durağı bitir, diğerine geç
                    durum = client.getMultirotorState(vehicle_name="D1")
                    anlik_x = durum.kinematics_estimated.position.x_val
                    anlik_y = durum.kinematics_estimated.position.y_val
                    mesafe = math.sqrt((x_rev - anlik_x)**2 + (y_rev - anlik_y)**2)
                    if mesafe < 3.0:
                        break

            # 2. Aşama: Duraklar bitti, şimdi "Sıfır" noktasına (tam eve) dönüş
            print("Hocam duraklar bitti, gerçek ana merkeze (0,0) uçuluyor...")
            client.moveToPositionAsync(0, 0, -27, 5, vehicle_name="D1")
            client.moveToPositionAsync(5, 5, -27, 5, vehicle_name="D2")
            
            while True:
                frames_q = []
                for drone_name in ["D1", "D2"]:
                    resp = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name=drone_name)
                    if resp:
                        img_q = np.frombuffer(resp[0].image_data_uint8, dtype=np.uint8).reshape(resp[0].height, resp[0].width, 3)
                        frames_q.append(cv2.resize(img_q, (960, 540)))
                
                if len(frames_q) == 2:
                    disp_q = np.hstack((frames_q[0], frames_q[1]))
                    cv2.putText(disp_q, "SISTEM: EVE YANASIYOR...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
                    cv2.imshow(win_name, disp_q)
                cv2.waitKey(1)
                
                # Gerçek merkeze 2 metre kalana kadar bekle
                durum = client.getMultirotorState(vehicle_name="D1")
                mesafe = math.sqrt((0 - durum.kinematics_estimated.position.x_val)**2 + (0 - durum.kinematics_estimated.position.y_val)**2)
                if mesafe < 2.0:
                    break
            
            print("Hocam tam merkeze ulaştık, inişe geçiliyor...")
            break 
       

except Exception as e:
    print(f"Hocam hata oluştu: {e}")

finally:
    
    print("Hocam droneların ataleti sıfırlanıyor, kayma engellenecek...")
    import time
    start_land_time = time.time()

    for drone_name in ["D1", "D2"]:
        client.hoverAsync(vehicle_name=drone_name).join()
    
    time.sleep(2)

    for drone_name in ["D1", "D2"]:
        state = client.getMultirotorState(vehicle_name=drone_name)
        x_kilit = state.kinematics_estimated.position.x_val
        y_kilit = state.kinematics_estimated.position.y_val
        
        client.moveToPositionAsync(x_kilit, y_kilit, 2, 2, vehicle_name=drone_name)

    print("Hocam konum kilitlendi, milimetrik asansör inişi başladı. Ekran açık...")

    while True:
        frames = []
        altitudes = []
        
        for drone_name in ["D1", "D2"]:
            responses = client.simGetImages([airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)], vehicle_name=drone_name)
            if responses:
                res = responses[0]
                img = np.frombuffer(res.image_data_uint8, dtype=np.uint8).reshape(res.height, res.width, 3)
                frames.append(cv2.resize(img, (960, 540)))
            
            state = client.getMultirotorState(vehicle_name=drone_name)
            altitudes.append(state.kinematics_estimated.position.z_val)

        if len(frames) == 2:
            display = np.hstack((frames[0], frames[1]))
            cv2.putText(display, "SISTEM: KILITLI VE MILIMETRIK INIS YAPILIYOR...", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            cv2.imshow(win_name, display)
        
        cv2.waitKey(1)

        current_time = time.time()
        if all(z > -0.5 for z in altitudes) or (current_time - start_land_time > 12):
            print("Hocam dronelar sıfır hatayla yere oturdu. Motorlar kapatılıyor.")
            break

    for drone_name in ["D1", "D2"]:
        client.armDisarm(False, vehicle_name=drone_name)
        client.enableApiControl(False, vehicle_name=drone_name)
    
    cv2.destroyAllWindows()