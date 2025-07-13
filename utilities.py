
serial_port_name = '/dev/ttyUSB0'
baud_rate = 57600    

from MainSystem2 import USVController
import datetime

SOL_MOTOR = 5
SAG_MOTOR = 6

controller = USVController("/dev/ttyACM0", baud=57600)

def log(msg):
    try:
        if serial_port_name and serial_port_name.is_open:
            serial_port_name.write((msg + "\n").encode())
    except:
        pass


def log_status(zed, adviced_course, hedefe_mesafe, current_goal, manual_mode, magnetic_heading,
               magnetometer_data):


    heading_fark = abs(magnetic_heading - adviced_course) % 360
    if heading_fark > 180:
        heading_fark = 360 - heading_fark

    log("========== İDA DURUMU ==========")
    log(f"ZAMAN: {datetime.datetime.now().strftime('%H:%M:%S')}")
    log(f"FPS: {int(zed.get_current_fps())}")

    log(f"SOL İTİCİ İTKİ İSTEĞİ (PWM): {controller.get_servo_pwm(SOL_MOTOR)}")
    log(f"SAĞ İTİCİ İTKİ İSTEĞİ (PWM): {controller.get_servo_pwm(SAG_MOTOR)}")

    log(f"İDA GERÇEK HIZ: {controller.get_horizontal_speed()} m/s")

    log(f"GERÇEK HEADING: {int(magnetic_heading)}°")
    log(f"HEDEF HEADING: {int(adviced_course)}°")
    log(f"HEADING HATASI: {heading_fark:.0f}°")
    log(f"HEADING SAĞLIĞI: {magnetometer_data.magnetic_heading_state}")
    log(f"DOĞRULUK ORANI: {magnetometer_data.magnetic_heading_accuracy:.1f}")

    log(f"GÖREV NOKTASI: {current_goal}")

    log(f"KONUM: {controller.get_current_position()}")
    log(f"KALAN MESAFE: {int(hedefe_mesafe)} m")

    log(f"MANUEL MOD: {manual_mode}")
    log("================================")