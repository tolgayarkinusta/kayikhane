from pymavlink import mavutil
import math
import cv2
import numpy
import math
import time
import threading

def map_value_to_range(value, in_min=-1, in_max=1, out_min=-2000, out_max=2000):
    return int((value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


class USVController:
    def __init__(self, connection_string="COM10", baud=57600):
        # connecting to the vehicle
        print("trying to connect")
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        print("Connected to MAVLINK")
        print("Heartbeat waiting...")
        self.master.wait_heartbeat()
        print("Heartbeat found!")

    def arm_vehicle(self):
        # arming vehicle
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("Waiting for the vehicle to arm...")
        self.master.motors_armed_wait()  # Waiting for arming the vehicle
        print('Armed:!!!!!!!!!!!!!!!!!!!!!!!!!')

    def disarm_vehicle(self):
        # Disarming the vehicle
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("Waiting for the vehicle to disarm")
        self.master.motors_disarmed_wait()  # Waiting for disarming the vehicle
        print('Disarmed!')

    def print_motor_outputs(self):
        print(self.master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True))

    def get_servo_pwm(self, servo_number):
        """
        Belirtilen servo numarasına (1–8 arası) ait PWM değerini döndürür.
        Dönüş değeri: 4 basamaklı int (örneğin 1500)
        """
        if not 1 <= servo_number <= 8:
            print("Servo numarası 1 ile 8 arasında olmalıdır.")
            return None

        msg = self.master.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        if msg:
            pwm_value = getattr(msg, f'servo{servo_number}_raw')
            return pwm_value
        else:
            print("PWM mesajı alınamadı.")
            return None

    def get_horizontal_speed(self):
        """
        Yalnızca yüzey (2D) ani hız değerini döndürür (vz hariç).
        Dönüş: float (m/s cinsinden)
        """
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            vx = msg.vx / 100.0  # cm/s → m/s
            vy = msg.vy / 100.0

            horizontal_speed = math.sqrt(vx**2 + vy**2)
            return horizontal_speed
        else:
            print("Hız verisi alınamadı.")
            return None

    def set_servo(self, servo_pin, pwm_value, see_motor_output=0):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_pin,
            pwm_value,
            0, 0, 0, 0, 0
        )
        if see_motor_output:
            self.print_motor_outputs()


    def set_mode(self, mode_name):
        # (example: 'STABILIZE', 'MANUAL', 'DEPTH_HOLD')

        mode_id = self.master.mode_mapping()[mode_name]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print("Aracın modu " + mode_name + " olarak değiştirildi.")

    def print_gps_info(self): #mevut gps mevki görüntüle
        # GPS_GLOBAL_INT mesajını al
        print("getting gps info")
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        print("GPS info received")
        if msg:
            lat = msg.lat / 1e7  # Enlem (derece cinsinden)
            lon = msg.lon / 1e7  # Boylam (derece cinsinden)
            alt = msg.alt / 1e3  # Yükseklik (metre cinsinden)


            # GPS bilgilerini yazdır
            print(f"Enlem: {lat:.6f}°, Boylam: {lon:.6f}°, Yükseklik: {alt:.2f} metre")

    def get_current_position(self): #mevcut gps mevkii al
        #anlık GPS konumu alınır
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            enlem = msg.lat / 1e7
            boylam = msg.lon / 1e7
            return enlem, boylam
        return None, None

    def check_distance(self, threshold):    #todo: add to git if success
        if self.first_position:
            self.update_current_position()
            if self.current_position:
                distance = self.haversine(
                    self.first_position[0], self.first_position[1],
                    self.current_position[0], self.current_position[1]
                )
                print(f"Mesafe: {distance:.2f} metre")
                if distance >= threshold:
                    print("STOP")
                    self.set_servo(1,1500)
                    self.set_servo(3,1500)
                    return True
        return False

    def calculate_heading(self): #başlangıç ve bitiş gps mevkiileri arasındaki headingi hesapla
        #ilk ve son GPS konumları arasındaki heading açısını hesapla
        if not self.first_position or not self.current_position:
            print("Başlangıç veya Bitiş konumu eksik, heading hesaplanamıyor.")
            return None

        enlem1, boylam1 = self.first_position
        enlem2, boylam2 = self.current_position

        #enlem ve boylamları radyan cinsine çevirme
        enlem1_radyan = math.radians(enlem1)
        enlem2_radyan = math.radians(enlem2)
        delta_boylam = math.radians(boylam2 - boylam1)

        #heading hesaplama
        x = math.sin(delta_boylam) * math.cos(enlem2_radyan)
        y = math.cos(enlem1_radyan) * math.sin(enlem2_radyan) - math.sin(enlem1_radyan) * math.cos(enlem2_radyan) * math.cos(delta_boylam)
        initial_kerteriz = math.atan2(x, y)

        #açıyı dereceye çevirme
        initial_kerteriz = math.degrees(initial_kerteriz)
        pusula_kerteriz = (initial_kerteriz + 360) % 360
        return pusula_kerteriz

    def give_desired_heading(self): #hesaplanan mevkiyi 'kerteriz' olarak kaydet ve terminale yazdır
        kerteriz = self.calculate_heading()
        print(f"Evin kerterizi: {kerteriz}")

'''
    def set_servo(self, servo_pin, pwm_value, see_motor_output=0):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_pin,
            pwm_value,
            0, 0, 0, 0, 0
        )
        if see_motor_output:
            self.print_motor_outputs()
'''









