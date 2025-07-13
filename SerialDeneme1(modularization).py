import pyzed.sl as sl
import cv2
import numpy as np
import math
import time
import os
import select, sys
import tty
import termios
import threading
import queue
import datetime
from colorama import Fore, Back, Style, init
import serial

import config as cfg
import navigasyon as nav
import kamera
from kamera import TimestampHandler


# from ultralytics import YOLO
# import supervision as sv
# import torch , torchvision; from torch.autograd import backward kaldırıldı
from headingFilter import KalmanFilter  # todo: heading filtresi testi yap
from MainSystem2 import USVController


controller = USVController("/dev/ttyACM0", baud=57600)
print("Arming vehicle...")
controller.arm_vehicle()
print("Vehicle armed!")
print("Setting mode...")
controller.set_mode("MANUAL")
print("Mode set!")

serial_port_name = '/dev/ttyUSB0'
baud_rate = 57600

# flags n variables
width = None  # Başlangıç değeri
manual_mode = False  # Windows tarafından gelen komutla değişecek
key = None
mission_started = False

try:
    telemetri = serial.Serial(serial_port_name, baud_rate, timeout=1)
    print('Telemetriye erişim var.')
except Exception as e:
    print("Seri port açılırken hata:", e)


def clear_terminal():
    os.system('cls' if os.name == 'nt' else 'clear')


def send_log_data(telemetri,
                  zed,
                  magnetic_heading,
                  adviced_course,
                  magnetometer_data,
                  current_goal,
                  hedefe_mesafe,
                  manual_mode):

    heading_fark = abs(magnetic_heading - adviced_course) % 360
    if heading_fark > 180:
        heading_fark = 360 - heading_fark

    # Her satır için uygun formatta string oluşturup gönderiyoruz
    telemetri.write(f"ZAMAN:{datetime.datetime.now().strftime('%H:%M:%S')}\n".encode())
    telemetri.write(f"FPS:{(zed.get_current_fps())}\n".encode())
    telemetri.write(f"SOL_PWM:{controller.get_servo_pwm(cfg.SOL_MOTOR)}\n".encode())
    telemetri.write(f"SAG_PWM:{controller.get_servo_pwm(cfg.SAG_MOTOR)}\n".encode())
    if controller.get_horizontal_speed() is not None:
        telemetri.write(f"HIZ:{controller.get_horizontal_speed():.2f}\n".encode())
    telemetri.write(f"GERCEK_HEADING:{int(magnetic_heading)}\n".encode())
    telemetri.write(f"HEDEF_HEADING:{int(adviced_course)}\n".encode())
    telemetri.write(f"HEADING_HATASI:{int(round(heading_fark))}\n".encode())
    telemetri.write(f"HEADING_SAGLIGI:{magnetometer_data.magnetic_heading_state}\n".encode())
    telemetri.write(f"DOGRULUK_ORANI:{magnetometer_data.magnetic_heading_accuracy:.1f}\n".encode())
    telemetri.write(f"GOREV_NOKTASI:{current_goal}\n".encode())
    telemetri.write(f"KONUM:{controller.get_current_position()}\n".encode())
    telemetri.write(f"KALAN_MESAFE:{int(hedefe_mesafe)}\n".encode())
    telemetri.write(f"MANUEL_MOD:{manual_mode}\n".encode())

def main():
    global width, manual_mode, magnetic_heading, mission_started

    print("Initializing Camera...")
    zed = kamera.initialize_camera()
    print("Camera initialized! Initializing positional tracking...")
    kamera.initialize_positional_tracking(zed)
    print("Tracking initialized! Initializing spatial mapping...")
    kamera.initialize_spatial_mapping(zed)
    print("Mapping initialized!")

    ts_handler = TimestampHandler()     # Used to store the sensors timestamp to know if the sensors_data is a new one or not

    sensors_data = sl.SensorsData()    # Sensör verisi al

    current_goal = cfg.GPS1
    adviced_course = 0
    error = 0
    hedefe_mesafe = 1000

    while True:
            
            ida_enlem, ida_boylam = controller.get_current_position()

            decoded = "NONE"
            print(decoded)
            
            if telemetri.in_waiting:
                raw = telemetri.readline()
                decoded = raw.decode().strip()

            if decoded == "START":
                mission_started = True

            elif decoded == "EXIT":
                mission_started = False
                controller.set_servo(cfg.SOL_MOTOR, 1500)
                controller.set_servo(cfg.SAG_MOTOR, 1500)
                break
            elif decoded == "SET_MODE:MANUAL":
                manual_mode = True
            elif decoded == "SET_MODE:AUTONOMY":
                manual_mode = False
            elif decoded.startswith("CMD:"):
                key = decoded.split(":")[1].lower()
                if key == 'w':
                    print("İleri komutu geldi")
                    # controller.set_servo(...) gibi işlemler yapılır
                elif key == 's':
                    print("Geri komutu geldi")
                elif key == 'a':
                    print("Sola dönüş")
                elif key == 'd':
                    print("Sağa dönüş")
            elif decoded.startswith("GPS_UPDATE:"):
                gps_string = decoded.split(":")[1]   # ":" karakterinden sonrasını al
                coords = gps_string.split(",")       # virgülle ayır → liste yap

                if len(coords) == 10:
                    GPS1_lat, GPS1_lon = float(coords[0]), float(coords[1])
                    GPS2_lat, GPS2_lon = float(coords[2]), float(coords[3])
                    GPS3_lat, GPS3_lon = float(coords[4]), float(coords[5])
                    GPS4_lat, GPS4_lon = float(coords[6]), float(coords[7])
                    GPS5_lat, GPS5_lon = float(coords[8]), float(coords[9])

                    print("GPS noktaları güncellendi.")
                else:
                    print("GPS formatı hatalı!")


            # retrieve the current sensors sensors_data
            if zed.get_sensors_data(sensors_data,
                                    sl.TIME_REFERENCE.CURRENT):  # time_reference.image for synchorinzed timestamps
                # Check if the data has been updated since the last time
                # IMU is the sensor with the highest rate
                if ts_handler.is_new(sensors_data.get_imu_data()):
                    magnetometer_data = sensors_data.get_magnetometer_data()
                    # Get the raw magnetic heading  # Apply low-pass filter

                    magnetic_filter = KalmanFilter(process_variance=1e-3, measurement_variance=1e-1)
                    # magnetic_heading = magnetic_filter.update(sensors_data.get_magnetometer_data().magnetic_heading)

                    magnetic_heading = sensors_data.get_magnetometer_data().magnetic_heading

            if current_goal == cfg.GPS1 and mission_started and not manual_mode:
                adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS1_enlem, cfg.GPS1_boylam)
                error = nav.signed_angle_difference(magnetic_heading, adviced_course)
                # Calculate the error using: #negatif deger tavsiye rotanın iskelede, pozitif deger tavsiye rotanın sancakta kaldıgı anlamına gelir

                controller.set_servo(cfg.SOL_MOTOR, cfg.BASE_PWM +error)
                controller.set_servo(cfg.SAG_MOTOR, cfg.BASE_PWM -error)

                hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS1_enlem, cfg.GPS1_boylam)

                if hedefe_mesafe < 1:
                    current_goal = cfg.GPS2

            elif current_goal == cfg.GPS2 and mission_started and not manual_mode:
                adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS2_enlem, cfg.GPS2_boylam)
                error = nav.signed_angle_difference(magnetic_heading, adviced_course)

                controller.set_servo(cfg.SOL_MOTOR, cfg.BASE_PWM + error)
                controller.set_servo(cfg.SAG_MOTOR, cfg.BASE_PWM - error)

                hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS2_enlem, cfg.GPS2_boylam)
                if hedefe_mesafe < 1:
                    current_goal = cfg.GPS3

            elif current_goal == cfg.GPS3 and mission_started and not manual_mode:
                adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS3_enlem, cfg.GPS3_boylam)
                error = nav.signed_angle_difference(magnetic_heading, adviced_course)

                controller.set_servo(cfg.SOL_MOTOR, cfg.BASE_PWM + error)
                controller.set_servo(cfg.SAG_MOTOR, cfg.BASE_PWM - error)

                hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS3_enlem, cfg.GPS3_boylam)
                if hedefe_mesafe < 1:
                    current_goal = cfg.GPS4

            elif current_goal == cfg.GPS4 and mission_started and not manual_mode:
                adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS4_enlem, cfg.GPS4_boylam)
                error = nav.signed_angle_difference(magnetic_heading, adviced_course)

                controller.set_servo(cfg.SOL_MOTOR, cfg.BASE_PWM + error)
                controller.set_servo(cfg.SAG_MOTOR, cfg.BASE_PWM - error)

                hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS4_enlem, cfg.GPS4_boylam)
                if hedefe_mesafe < 1:
                    current_goal = cfg.GPS5

            elif current_goal == cfg.GPS5 and mission_started and not manual_mode:
                adviced_course = nav.calculate_bearing(ida_enlem, ida_boylam, cfg.GPS5_enlem, cfg.GPS5_boylam)
                error = nav.signed_angle_difference(magnetic_heading, adviced_course)

                controller.set_servo(cfg.SOL_MOTOR, cfg.BASE_PWM + error)
                controller.set_servo(cfg.SAG_MOTOR, cfg.BASE_PWM - error)

                hedefe_mesafe = nav.haversine(ida_enlem, ida_boylam, cfg.GPS5_enlem, cfg.GPS5_boylam)
                if hedefe_mesafe < 1:
                    controller.set_servo(cfg.SOL_MOTOR, 1500)
                    controller.set_servo(cfg.SAG_MOTOR, 1500)
                    current_goal = 3169


            # Manuel mod windows tarafından alınan komutla aktif edilmişse, yine windows tarafında WASD tuşlarıyla(WASD tuşlarına windows tarafında basılacak) kontrol yapılır.
            if manual_mode is True:
                # CHATGPT BURAYI SERIAL HABERLEŞMESİNE UYGUN DÜZENLEMELİ
                if key == 'w':
                    # İleri hareket: her iki motor ileri
                    print("manual")
                    controller.set_servo(cfg.SOL_MOTOR, 1600)
                    controller.set_servo(cfg.SAG_MOTOR, 1600)
                elif key == 's':
                    # Geri hareket: her iki motor geri
                    print("manual")
                    controller.set_servo(cfg.SOL_MOTOR, 1300)
                    controller.set_servo(cfg.SAG_MOTOR, 1300)
                elif key == 'a':
                    # Sola dönüş: sol motor yavaş, sağ motor hızlı
                    print("manual")
                    controller.set_servo(cfg.SOL_MOTOR, 1420)
                    controller.set_servo(cfg.SAG_MOTOR, 1580)
                elif key == 'd':
                    # Sağa dönüş: sol motor hızlı, sağ motor yavaş
                    print("manual")
                    controller.set_servo(cfg.SOL_MOTOR, 1580)
                    controller.set_servo(cfg.SAG_MOTOR, 1420)

            send_log_data(telemetri,
                  zed,
                  magnetic_heading,
                  adviced_course,
                  magnetometer_data,
                  current_goal,
                  hedefe_mesafe,
                  manual_mode)

    # Kaynakları serbest bırak ve kamerayı kapat
    zed.close()


if __name__ == "__main__":
    main()