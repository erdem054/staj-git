
from turtle import goto
from unittest import result
import cv2
import numpy as np
import time
from scipy.spatial import distance as dist
from dronekit import connect, VehicleMode , LocationGlobalRelative
from pymavlink import mavutil 

iha = connect ("/dev/ttyTHS1",wait_ready=True)
kernel = np.ones((5,5), dtype = np.uint8)
cam = cv2.VideoCapture(0)
pixelsPerMetric = 0.955     # irtifa değerine göre değişecek !!!

def kamera_ters_cevirme(frame, x,y):
    point1 = np.float32([[0,0], [x, 0], [0,y], [x,y]])
    point2 = np.float32([[x,y], [0,y], [x, 0], [0,0]])
    matrix = cv2.getPerspectiveTransform(point1, point2)
    dondurme = cv2.warpPerspective(frame, matrix, (x, y))
    frame = dondurme

def ortalama_fonk(xx, yy):
    print("x", xx)          # x 0dan küçükse negatifse doğuya gidilicek, pozitifse batıya
    print("y", yy)          # y 0 dan küçükse kuzeye doğuya gidilicek, pozitifse guneye
    iha_hareket(iha.location.global_relative_frame.lat + xx, iha.location.global_relative_frame.lon + yy, - iha.location.global_relative_frame.alt)
    #goto_position_target_local_ned( (iha.location.global_relative_frame.lon + xx), (iha.location.global_relative_frame.lat + yy),-7)        ####
    #goto_position_target_local_ned(iha.location.global_relative_frame.lon, iha.location.global_relative_frame.lat,-5)   

def ilac_teslimi():
    ## servo kontrol ##
    pass

def inis_pedi_ortalama(cam, pixelsPerMetric):
    blueLower = (84,  98,  65)
    blueUpper = (179, 255, 255)         
    i = 0
    
    while(True):
        ret, frame = cam.read()
        #cv2.imshow('frame', frame)      ####

        if i == 0:
            print("shape ", frame.shape)
            print("kamera genişlik (x)", frame.shape[0])
            print("kamera yükseklik (y)", frame.shape[1])
            i += 1

        yukseklik = frame.shape[0]
        ort_yukseklik =int(yukseklik/2)

        genislik = frame.shape[1]
        ort_genislik = int(genislik/2)

        #kamera_ters_cevirme(frame, genislik, yukseklik)

        cv2.circle(frame, (ort_genislik, ort_yukseklik), 5, (94,224,69), -1)

        frame = cv2.flip(frame, 1)

        blur_frame = cv2.GaussianBlur(frame, (11,11), 0) 
        hsv_frame = cv2.cvtColor(blur_frame, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv_frame, blueLower, blueUpper)
        mask = cv2.erode(mask, None, iterations = 2)
        result = cv2.dilate(mask, None, iterations = 2)

        (contours,_) = cv2.findContours(result.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        center = None
        if len(contours) > 0:
            
            # en buyuk konturu al
            c = max(contours, key = cv2.contourArea)
            
            # dikdörtgene çevir 
            rect = cv2.minAreaRect(c)
            
            ((x,y), (width,height), rotation) = rect

            s = "x: {}, y: {}, width: {}, height: {}, rotation: {}".format(np.round(x),np.round(y),np.round(width),np.round(height),np.round(rotation))
            #print(s)
            box = cv2.boxPoints(rect)
            box = np.int64(box)
            
            # moment
            M = cv2.moments(c)
            center = (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))
            
            # konturu çizdir: sarı
            cv2.drawContours(frame, [box], 0, (0,255,255),2)
            
            # merkere bir tane nokta çizelim: pembe
            cv2.circle(frame, center, 5, (255,0,255),-1)
            
            # bilgileri ekrana yazdır
            cv2.putText(frame, s, (25,50), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, (255,255,255), 2)

            dA = dist.euclidean((ort_genislik, ort_yukseklik), (center))
            dimA = dA / pixelsPerMetric
            cv2.putText(frame, "{:.1f}mm".format(dimA),
                                        (int(ort_genislik - 15), int(ort_yukseklik - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                                        0.65, (255, 255, 255), 2)

            x_degeri = ort_genislik - center[0]
            y_degeri = ort_yukseklik - center[1]

            if x_degeri >= 0 :      # inis pedi batida kalmistir, batiya gidilecek..
                print("inis pedi batida kalmistir, batiya gidilecek..")
            elif x_degeri <0 :      # inis pedi doguda kalmistir, doguya gidilecek..
                print("inis pedi doguda kalmistir, doguya gidilecek..")

            if y_degeri >= 0 :      # inis pedi guneyde kalmistir, guneye gidilecek..
                print("inis pedi guneyde kalmistir, guneye gidilecek..")
            elif y_degeri <0 :      # inis pedi kuzeyde kalmistir, kuzey gidilecek..
                print("inis pedi kuzeyde kalmistir, kuzey gidilecek..")
            
            time.sleep(0.5)

        cv2.imshow("Goruntu isleme",frame)
        ortalama_fonk(x_degeri, y_degeri)

        if cv2.waitKey(1) & 0xFF == ord("q"): break
    cv2.destroyAllWindows()

def takeoff(irtifa): #Fonksiyon
    while iha.is_armable is not True: #iha arm edilmiş mi sorgusu
        print("iha arm edilebilir durumda değil!")

    print("iha arm edilebilir.")

    iha.mode = VehicleMode("GUIDED")
    iha.armed = True

    while iha.armed is not True: #iha henüz arm olmamışsa
        print("iha arm ediliyor...")  
        time.sleep(0.5)   

    print("iha arm edildi.")

    iha.simple_takeoff(irtifa)

    while iha.location.global_relative_frame.alt < irtifa * 0.9:
            print("iha hedefe yükseliyor. ({})" .format(iha.location.global_relative_frame.alt))
            time.sleep(1)
    if iha.location.global_relative_frame.alt >= irtifa * 0.8 and iha.location.global_relative_frame.alt <= irtifa*1.1:
        print("IRTIFA DEGERI : ", iha.location.global_relative_frame.alt)
    else:
        if iha.location.global_relative_frame.alt > irtifa*1.3:
            print("kontrolsuz irtifa artisi.. inis yapiliyor..")
            land_modu()
        elif iha.location.global_relative_frame.alt < 0.8:
            print("istinilen irtifaya ulasilamamistir.. tekrar deneniyor")
            iha.simple_takeoff(irtifa)

def goto_position_target_local_ned(north, east, down):
    
    msg = iha.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to iha
    ##   iha.send_mavlink(msg)

def land_modu():
    
    time.sleep(1)
    print("LAND Moda geciliyor...")
    iha.mode = VehicleMode("LAND")
    time.sleep(2)
        
    while iha.mode.name != "LAND":
        print("LAND moduna geciliyor...")
        time.sleep(1)

    while True:
        print("IRTIFA: ",iha.location.local_frame.down)
        time.sleep(1)

        if round(iha.location.local_frame.down) == 0:
            print("basariyla inis yapildi...")
            break

    while True:
        print("DISARM ediliyor...")
        time.sleep(1)

        if iha.armed == False:
            break
    
    print("BASARILI ...")
    iha.close()

def iha_hareket(x, y, z):
    goto_position_target_local_ned(x, y, -z)        #z negatif olmali
    irtifa_kontrol = iha.location.global_relative_frame.alt
    x_kontrol = iha.location.global_relative_frame.lat 
    y_kontrol = iha.location.global_relative_frame.lon 
    
    while x_kontrol < x *0.9 :
        print("iha hedef konuma gidiyor... ({}, {})" .format(iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon ))
        time.sleep(1)

    if iha.location.global_relative_frame.lat >= x -1 and (iha.location.global_relative_frame.lat) <= x + 1 :
        print("ENLEM (X) DEGERI : ", iha.location.global_relative_frame.lat)
        print("hedefe ulasilmis olmali")

    """if iha.location.global_relative_frame.lat >= x * 0.8 and iha.location.global_relative_frame.lat <= x *1.1:
        print("ENLEM (X) DEGERI : ", iha.location.global_relative_frame.lat)
    else:
        if iha.location.global_relative_frame.lat > x*1.15:
            print("kontrolsuz enlem artisi.. duzeltiliyor..")
            goto_position_target_local_ned(x, y_kontrol, irtifa_kontrol)
          
        elif iha.location.global_relative_frame.lat < 0.8:
            print("istinilen eneleme ulasilamamistir.. tekrar deneniyor")
            goto_position_target_local_ned(x, y_kontrol, irtifa_kontrol)"""

    """if iha.location.global_relative_frame.lon >= y * 0.8 and iha.location.global_relative_frame.lon <= y *1.1:
        print("BOYLAM (Y) DEGERI : ", iha.location.global_relative_frame.lon)
    else:
        if iha.location.global_relative_frame.lon > y*1.15:
            print("kontrolsuz boylam artisi.. duzeltiliyor..")
            goto_position_target_local_ned(x_kontrol, y, irtifa_kontrol)
        
        elif iha.location.global_relative_frame.alt < 0.8:
            print("istinilen boylama ulasilamamistir.. tekrar deneniyor")
            goto_position_target_local_ned(x_kontrol, y, irtifa_kontrol)"""


## gorev baslangici ##

takeoff(7)

iha_hareket(5,0,7)      # zdegerini pozitif ver fonksiyon icinde negatifleniyor
time.sleep(1)

"""print("goruntu isleme cagriliyo")
inis_pedi_ortalama(cam, pixelsPerMetric)
time.sleep(1)
print("tespit gerceklesti, ortalama yapildi, irtifa dusurulecek")"""


"""time.sleep(1)
print("mevcut irtifa: ", iha.location.global_relative_frame.alt)
iha_hareket(iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon, - (iha.location.global_relative_frame.alt + 2))
print("irtifa dusuruldu: ", iha.location.global_relative_frame.alt)
ilac_teslimi()
time.sleep(1)
print("varsayilan irtifaya cikiliyor..")
iha_hareket(iha.location.global_relative_frame.lat, iha.location.global_relative_frame.lon, - (iha.location.global_relative_frame.alt - 2))
"""

"""print("eve donus")
iha_hareket(0, 0, iha.location.global_relative_frame.alt)"""


time.sleep(1)
print("inis yapilacak")
land_modu()
