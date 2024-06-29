from picamera2 import Picamera2
import time
import cv2 as cv
import numpy as np
import serial

picam2 = Picamera2()
camera_config = picam2.create_still_configuration()
picam2.configure(camera_config)
picam2.start()
time.sleep(2)
file_path = '/home/adc/Desktop/photo.jpg'
picam2.capture_file(file_path)
picam2.stop()
print(f"Foto kaydedildi: {file_path}")




ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
ser.write("SHOT\n".encode())
print("Gonderilen veri: DONE")
yatay = []
dikey = []


def tiklama(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        yatay.append(x)
        dikey.append(y)
        print(f"Tiklandi - X:{x}, Y:{y}")
    elif event == cv.EVENT_RBUTTONDOWN:
        daire_ok = False
        for i in range(len(yatay)):
            if np.abs(yatay[i] - x) < 10 and np.abs(dikey[i] - y) < 10:
                print("Sayi bulundu", x, y, "Indeks:", i)
                print(yatay[i], dikey[i], "Kaldirildi.")
                cv.circle(overlay, (yatay[i], dikey[i]), 9, (0, 0, 0), 2)
                yatay.pop(i)
                dikey.pop(i)
                daire_ok = True
                break
        if not daire_ok:
            print("Daire Bulunamadi.")


image = cv.imread('/home/adc/Desktop/photo.jpg')
image = cv.resize(image, (540, 450))
gray = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
blur = cv.GaussianBlur(gray, (9, 9), 2)
daireler = cv.HoughCircles(blur, cv.HOUGH_GRADIENT, dp=1, minDist=10, param1=100, param2=17, minRadius=5, maxRadius=10)

overlay = np.zeros_like(image)


def process():
    global daireler
    if daireler is not None:
        daireler = np.uint16(np.around(daireler))
        for i in daireler[0, :]:
            yatay.append(i[0])
            dikey.append(i[1])
        cv.namedWindow('Gorsel')
        cv.setMouseCallback('Gorsel', tiklama)

        while True:
            overlay[:] = 0  
            for a in range(len(yatay)):
                center = (yatay[a], dikey[a])
                cv.circle(overlay, center, 9, (0, 0, 255), 2)

            combined_image = cv.addWeighted(image, 1, overlay, 0.5, 0)
            cv.imshow('Gorsel', combined_image)

            if cv.waitKey(1) & 0xFF == ord('q'):
                break

        cv.destroyAllWindows()

        if yatay and dikey:  
            data = f"x:{yatay[0]},y:{dikey[0]}\n"
            ser.write(data.encode())
            print(f"Gonderilen veri: {data}")
            time.sleep(0.1)
            send_coordinates()
            ser.write("DONE\n".encode())
            print("Gonderilen veri: DONE")


def send_coordinates():
    i = 1
    while i < len(yatay):
        response = ser.readline().decode().strip()
        if response == 'OK':
            data = f"x:{yatay[i]},y:{dikey[i]}\n"
            ser.write(data.encode())
            print(f"Gonderilen veri: {data}")
            time.sleep(0.1)
            i += 1


process()
