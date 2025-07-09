from pymavlink import mavutil
import math
import cv2
import numpy
import math
import time
import threading


class USVController:
    def __init__(self, connection_string="COM10", baud=57600):
        # connecting to the vehicle
        print("trying to connect")
        self.master = mavutil.mavlink_connection(connection_string, baud=baud)
        print("Connected to MAVLINK")
        print("Heartbeat waiting...")
        self.master.wait_heartbeat()
        print("Heartbeat found!")

        # Arka planda dinlenecek mesajlar için veri yapısı
        self.msg_dict = {}  # Örn: {"GLOBAL_POSITION_INT": msg, "SERVO_OUTPUT_RAW": msg}
        self.running = True

        # Dinleyici thread başlat
        self.listener_thread = threading.Thread(target=self._listen_messages)
        self.listener_thread.daemon = True
        self.listener_thread.start()

    def _listen_messages(self):
        """Arka planda MAVLink mesajlarını dinler ve msg_dict'te saklar."""
        while self.running:
            msg = self.master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                self.msg_dict[msg_type] = msg
            time.sleep(0.01)  # Çok küçük delay ile CPU koruma

    def get_current_position(self):
        msg = self.msg_dict.get('GLOBAL_POSITION_INT')
        if msg:
            return msg.lat / 1e7, msg.lon / 1e7
        return None, None

    def stop_listener(self):
        """
        Thread'i güvenli şekilde durdur.
        """
        self.running = False
        self.listener_thread.join()

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
        msg = self.msg_dict.get('SERVO_OUTPUT_RAW')
        if msg:
            print(msg)
        else:
            print("Motor çıkışı henüz alınamadı.")

    def get_servo_pwm(self, servo_number):
        if not 1 <= servo_number <= 8:
            print("Servo numarası 1-8 arasında olmalıdır.")
            return None

        msg = self.msg_dict.get('SERVO_OUTPUT_RAW')
        if msg:
            return getattr(msg, f'servo{servo_number}_raw')
        return None

    def get_horizontal_speed(self):
        msg = self.msg_dict.get('GLOBAL_POSITION_INT')
        if msg:
            vx = msg.vx / 100.0
            vy = msg.vy / 100.0
            return math.sqrt(vx ** 2 + vy ** 2)
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

    def print_gps_info(self):
        msg = self.msg_dict.get('GLOBAL_POSITION_INT')
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1e3
            print(f"Enlem: {lat:.6f}°, Boylam: {lon:.6f}°, Yükseklik: {alt:.2f} m")
        else:
            print("GPS verisi henüz alınamadı.")


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









