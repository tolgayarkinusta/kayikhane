import pyzed.sl as sl
import cv2
import numpy as np
import math
import time
# from ultralytics import YOLO
# import supervision as sv
# import torch , torchvision; from torch.autograd import backward kaldırıldı
from headingFilter import KalmanFilter  # todo: heading filtresi testi
from MainSystem import USVController

# Harita için ayarlar:
map_width = 800  # Harita genişliği (piksel)
map_height = 800  # Harita yüksekliği (piksel)
scale = 20  # 1 metre = 50 piksel (uyarlamayı ihtiyaç duyarsan değiştirebilirsin)
# Başlangıç konumunu harita merkezine yerleştiriyoruz:
map_center = (map_width // 2, map_height // 2)
# Kalıcı harita (persistent map): başlangıçta mavi bir tuval
light_blue = (173, 216, 230)  # Açık mavi renk (R, G, B)
map_image = np.full((map_height, map_width, 3), light_blue, dtype=np.uint8) * 255

magnetic_filter = KalmanFilter(process_variance=1e-3, measurement_variance=1e-1)

# bounding_box_annotator = sv.BoundingBoxAnnotator()
# label_annotator = sv.LabelAnnotator()

controller = USVController("COM10", baud=57600)
print("Arming vehicle...")
controller.arm_vehicle()
print("Vehicle armed!")
print("Setting mode...")
controller.set_mode("MANUAL")
print("Mode set!")

# Constants
SOL_MOTOR = 5
SAG_MOTOR = 6
BASE_PWM = 1530

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 1
COLOR_RED = (0, 0, 255)
THICKNESS = 2
DEPTH_CENTER_COLOR = (255, 0, 0)
DEPTH_CENTER_RADIUS = 5
width = None  # Başlangıçta tanımlayın


GPS1_enlem = 0 # todo: SCRİPT ÇALIŞTIRILMADAN ÖNCE DÜZENLENİR.
GPS1_boylam = 0 #video için: dikdörtgenin 1.köşe koordinatları girilir

GPS2_enlem = 0
GPS2_boylam = 0 #video için: dikdörtgenin 2.köşe koordinatları girilir

GPS3_enlem = 0
GPS3_boylam = 0 #video için: dikdörtgenin 3.köşe koordinatları girilir

GPS4_enlem = 0
GPS4_boylam = 0 #video için: dikdörtgenin 4.köşe koordinatları girilir

GPS5_enlem = 0
GPS5_boylam = 0 #video için: İDA'nın eve dönüş,rıhtım, koordinatları girilir

current_goal = GPS1
adviced_course = 0
error = 0
hedefe_mesafe = 1000

# Boş bir gri tuval tanımla (örneğin, 720p için 1280×720)
blank = sl.Mat(720, 1280, sl.MAT_TYPE.U8_C3)  # 3 kanallı (RGB)
blank.set_to(sl.Vec3f(128, 128, 128))  # gri ton: (128,128,128)
frame = blank

def initialize_camera():
    # ZED kamera nesnesi oluştur
    zed = sl.Camera()
    # ZED başlatma parametreleri ayarla
    init_params = sl.InitParameters()
    runtime_params = sl.RuntimeParameters(enable_fill_mode=True)
    init_params.camera_resolution = sl.RESOLUTION.HD720  # 720p çözünürlük
    init_params.camera_fps = 60  # 30 FPS
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE  # todo: VİDEO İÇİN EN DÜŞÜĞE ALINACAK. depth mode best quality at neural_plus
    init_params.coordinate_units = sl.UNIT.METER  # using metric system
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP  # finale
    init_params.depth_minimum_distance = 0.20
    init_params.depth_maximum_distance = 20
    init_params.camera_disable_self_calib = False
    init_params.depth_stabilization = 50  # titreme azaltıcı
    init_params.sensors_required = False  # true yaparsan imu açılmadan kamera açılmaz
    init_params.enable_image_enhancement = True  # true was always the default
    init_params.async_grab_camera_recovery = False  # set true if u want to keep processing if cam gets shutdown
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        raise Exception("Failed to open ZED camera. Exiting.")
    return zed


# Initialize positional tracking
def initialize_positional_tracking(zed):
    # Enable positional tracking with default parameters
    py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
    tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
    tracking_parameters.enable_pose_smoothing = True  # set true to lower the loop_closures. effect Default False
    tracking_parameters.set_floor_as_origin = False  # Set the floor as the reference point Default False todo: True ile dene
    tracking_parameters.enable_area_memory = True  # Persistent memory for localization, Whether the camera can remember its surroundings. Default True
    tracking_parameters.enable_imu_fusion = True  # When set to False, only the optical odometry will be used. Default True
    tracking_parameters.set_as_static = False  # Whether to define the camera as static. Default False
    tracking_parameters.depth_min_range = 0.40  # It may be useful for example if any steady objects are in front of the camera and may perturb the positional tracking algorithm. Dfault no min range: -1
    tracking_parameters.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1  # gen_1 for performance, gen_2 for accuracy #todo: GEN_1 ve GEN_2 kıyaslamasını yap.

    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Enable positional tracking : " + repr(err) + ". Exit program.")
        zed.close()
        exit()


# Initialize spatial mapping
def initialize_spatial_mapping(zed):
    # Enable spatial mapping
    mapping_parameters = sl.SpatialMappingParameters(map_type=sl.SPATIAL_MAP_TYPE.MESH)  # .mesh or .fused_point_cloud
    mapping_parameters.resolution_meter = 0.10  # Define resolution (0.05m for fine mapping)
    mapping_parameters.range_meter = 20  # Maximum range for the mesh
    mapping_parameters.use_chunk_only = True  # Allow chunks of the map #true for better performance #false for accuracy
    mapping_parameters.max_memory_usage = 2048  # The maximum CPU memory (in MB) allocated for the meshing process. #default 2048
    # Whether to inverse the order of the vertices of the triangles.If your display process does not handle front and back face culling,
    mapping_parameters.reverse_vertex_order = False  # you can use this to correct it. Default: False. only for mesh

    error = zed.enable_spatial_mapping(mapping_parameters)
    if error != sl.ERROR_CODE.SUCCESS:
        raise Exception(f"Spatial mapping initialization failed: {error}")


# Render text over the frame
def render_text(frame, text, position):
    cv2.putText(frame, text, position, FONT, FONT_SCALE, COLOR_RED, THICKNESS)


# Basic class to handle the timestamp of the different sensors to know if it is a new sensors_data or an old one
class TimestampHandler:
    def __init__(self):
        self.t_imu = sl.Timestamp()

    # check if the new timestamp is higher than the reference one, and if yes, save the current as reference
    def is_new(self, sensor):
        if isinstance(sensor, sl.IMUData):
            new_ = (sensor.timestamp.get_microseconds() > self.t_imu.get_microseconds())
            if new_:
                self.t_imu = sensor.timestamp
            return new_


def draw_map_with_heading(current_x, current_y, magnetic_heading):
    """
    2D harita üzerinde aracın izini günceller ve o anki magnetic_heading yönünde ok çizer.
    Önceki ok silinir; ancak aracın izi kalıcı olarak haritada tutulur.
    """
    global map_image, map_center, scale

    # World koordinatlarını piksele dönüştür (başlangıç konumunu merkez kabul ederek)
    pixel_x = int(map_center[0] + current_x * scale)
    pixel_y = int(map_center[1] - current_y * scale)  # Görüntü koordinatlarında y ters yönde artar

    # Kalıcı haritaya aracın izini ekle (küçük daire)
    cv2.circle(map_image, (pixel_x, pixel_y), 2, (0, 0, 255), -1)

    # Haritadan kopya alarak geçici çizim yap; bu kopyaya ok çizilecek
    display_map = map_image.copy()

    # Ok çizimi için ayarlar:
    arrow_length = 40  # ok uzunluğu (piksel)
    # Manyetik başlığı radyana çevir (0° kuzeyi temsil edecek şekilde)
    rad = math.radians(magnetic_heading)
    # Okun ucunu hesapla:
    arrow_tip_x = pixel_x + int(arrow_length * math.sin(rad))
    arrow_tip_y = pixel_y - int(arrow_length * math.cos(rad))

    # O anki yönü gösteren ok çiz:
    cv2.arrowedLine(display_map, (pixel_x, pixel_y), (arrow_tip_x, arrow_tip_y), (0, 255, 0), 2, tipLength=0.3)

    # Haritayı göster:
    cv2.imshow("Vehicle Map", display_map)
    cv2.waitKey(1)

def signed_angle_difference(C, A): # C as current_heading(magnetic_heading) and A as adviced_course
    # Açılar 0-360 arasında normalize edilir
    C %= 360
    A %= 360

    # Saat yönünde fark (pozitif)
    clockwise_diff = (A - C) % 360

    # Eğer fark 180'den küçükse ya da eşitse, pozitif döndür
    if clockwise_diff <= 180:
        return clockwise_diff
    else:
        # Açı farkı saat yönünün tersindeyse, negatif döndür
        return clockwise_diff - 360

def haversine(lat1, lon1, lat2, lon2): #todo: add to git if success
        R = 6371000  # Dünya'nın yarıçapı (metre cinsinden)
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)

    x = math.sin(delta_lambda) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)

    bearing = math.atan2(x, y)
    bearing_degrees = (math.degrees(bearing) + 360) % 360  # 0–360 derece arasında

    return bearing_degrees

def main():
    global width, manual_mode, magnetic_heading
    print("Initializing Camera...")
    zed = initialize_camera()
    print("Camera initialized! Initializing positional tracking...")
    initialize_positional_tracking(zed)
    print("Tracking initialized! Initializing spatial mapping...")
    initialize_spatial_mapping(zed)
    print("Mapping initialized!")

    # Kamera çözünürlüğünü al
    camera_info = zed.get_camera_information()
    width = camera_info.camera_configuration.resolution.width
    print(width)
    height = camera_info.camera_configuration.resolution.height
    print(height)
    # Görüntüde merkez noktasını hesapla
    center_x = width // 2
    center_y = height // 2
    print("Kamera çözünürlüğü: ", width, "x", height)
    print("Görüntü orta noktası: ", (center_x, center_y))

    # Used to store the sensors timestamp to know if the sensors_data is a new one or not
    ts_handler = TimestampHandler()

    # Görüntü ve derinlik verilerini almak için Mat nesneleri oluştur
    image = sl.Mat()
    depth = sl.Mat()
    pose = sl.Pose()
    # mesh = sl.Mesh()

    # Sensör verisi al
    sensors_data = sl.SensorsData()

    manual_mode = False

    # Sonsuz bir döngüde görüntü akışı
    while True:
        # Kameradan bir yeni kare alın
        if zed.grab() == sl.ERROR_CODE.SUCCESS:

            ida_enlem, ida_boylam = get_current_position()

            # retrieve the current sensors sensors_data
            if zed.get_sensors_data(sensors_data,
                                    sl.TIME_REFERENCE.IMAGE):  # time_reference.image for synchorinzed timestamps
                # Check if the data has been updated since the last time
                # IMU is the sensor with the highest rate
                if ts_handler.is_new(sensors_data.get_imu_data()):
                    # Access the magnetometer data
                    magnetometer_data = sensors_data.get_magnetometer_data()
                    # Get the raw magnetic heading  # Apply low-pass filter
                    # magnetic_heading = magnetic_filter.update(sensors_data.get_magnetometer_data().magnetic_heading)
                    magnetic_heading = sensors_data.get_magnetometer_data().magnetic_heading

                    # Access the magnetic heading and state
                    magnetic_heading_info = (
                        f"Magnetic Heading: {magnetic_heading:.0f} "
                        f"({magnetometer_data.magnetic_heading_state}) "
                        f"[{magnetometer_data.magnetic_heading_accuracy:.1f}]"
                    )
                    render_text(frame, magnetic_heading_info, (frame.shape[1] - 1300, 30))

            # mevcut koordinatları al
            translation = pose.get_translation(sl.Translation()).get()  # [tx, ty, tz]
            current_x = translation[0]
            current_y = -(translation[1])
            draw_map_with_heading(current_x, current_y, magnetic_heading)



            if CURRENT_GOAL == GPS1:
                adviced_course = calculate_bearing(ida_enlem, ida_boylam, GPS1_enlem, GPS1_boylam)
                error = signed_angle_difference(magnetic_heading, adviced_course)
                # Calculate the error using: #negatif deger tavsiye rotanın iskelede, pozitif deger tavsiye rotanın sancakta kaldıgı anlamına gelir

                #MOCKED: controller.set_servo(SOL_MOTOR, BASE_PWM+error)
                #MOCKED: controller.set_servo(SAG_MOTOR, BASE_PWM-error)

                hedefe_mesafe = haversine(ida_enlem, ida_boylam, GPS1_enlem, GPS1_boylam)

                if hedefe_mesafe < 1:
                    current_goal = GPS2

            elif CURRENT_GOAL == GPS2:
                adviced_course = calculate_bearing(ida_enlem, ida_boylam, GPS2_enlem, GPS2_boylam)
                error = signed_angle_difference(magnetic_heading, adviced_course)

                controller.set_servo(SOL_MOTOR, BASE_PWM+error)
                controller.set_servo(SAG_MOTOR, BASE_PWM-error)

                hedefe_mesafe = haversine(ida_enlem, ida_boylam, GPS2_enlem, GPS2_boylam)
                if hedefe_mesafe < 1:
                    current_goal = GPS3

            elif CURRENT_GOAL == GPS3:
                adviced_course = calculate_bearing(ida_enlem, ida_boylam, GPS3_enlem, GPS3_boylam)
                error = signed_angle_difference(magnetic_heading, adviced_course)

                controller.set_servo(SOL_MOTOR, BASE_PWM+error)
                controller.set_servo(SAG_MOTOR, BASE_PWM-error)

                hedefe_mesafe = haversine(ida_enlem, ida_boylam, GPS3_enlem, GPS3_boylam)
                if hedefe_mesafe < 1:
                    current_goal = GPS4

            elif CURRENT_GOAL == GPS4:
                adviced_course = calculate_bearing(ida_enlem, ida_boylam, GPS4_enlem, GPS4_boylam)
                error = signed_angle_difference(magnetic_heading, adviced_course)

                controller.set_servo(SOL_MOTOR, BASE_PWM + error)
                controller.set_servo(SAG_MOTOR, BASE_PWM - error)

                hedefe_mesafe = haversine(ida_enlem, ida_boylam, GPS4_enlem, GPS4_boylam)
                if hedefe_mesafe < 1:
                    current_goal = GPS5

            elif CURRENT_GOAL == GPS5:
                adviced_course = calculate_bearing(ida_enlem, ida_boylam, GPS5_enlem, GPS5_boylam)
                error = signed_angle_difference(magnetic_heading, adviced_course)

                controller.set_servo(SOL_MOTOR, BASE_PWM + error)
                controller.set_servo(SAG_MOTOR, BASE_PWM - error)

                hedefe_mesafe = haversine(ida_enlem, ida_boylam, GPS5_enlem, GPS5_boylam)
                if hedefe_mesafe < 1:
                    controller.set_servo(SOL_MOTOR, 1500)
                    controller.set_servo(SAG_MOTOR, 1500)
                    current_goal = 3169


            # Tuş kontrolü: 'm' tuşu ile modlar arasında geçiş yapılır.
            key = cv2.waitKey(1) & 0xFF
            if key == ord('m'):
                manual_mode = not manual_mode
                if manual_mode is True:
                    print("Manuel mod aktif. Otomatik sürüş durdu.")
                else:
                    print("Otomatik mod aktif. Manuel kontrol devre dışı.")
                # Küçük bir gecikme, tuşun sürekli algılanmasını önlemek için
                # time.sleep(0.2)

            # Manuel mod aktifse, WASD tuşlarıyla kontrol yapılır.
            if manual_mode:
                cv2.putText(frame, "MANUEL MOD", (50, 50), FONT, 1, (0, 255, 255), 2)  # todo: ekran orta noktasına al
                if key == ord('w'):
                    # İleri hareket: her iki motor ileri
                    print("manual")
                    #MOCKED: controller.set_servo(SOL_MOTOR, 1600)
                    #MOCKED: controller.set_servo(SAG_MOTOR, 1600)
                elif key == ord('s'):
                    # Geri hareket: her iki motor geri
                    print("manual")
                    #MOCKED: controller.set_servo(SOL_MOTOR, 1300)
                    #MOCKED: controller.set_servo(SAG_MOTOR, 1300)
                elif key == ord('a'):
                    # Sola dönüş: sol motor yavaş, sağ motor hızlı
                    print("manual")
                    #MOCKED: controller.set_servo(SOL_MOTOR, 1420)
                    #MOCKED: controller.set_servo(SAG_MOTOR, 1580)
                elif key == ord('d'):
                    # Sağa dönüş: sol motor hızlı, sağ motor yavaş
                    print("manual")
                    #MOCKED: controller.set_servo(SOL_MOTOR, 1580)
                    #MOCKED: controller.set_servo(SAG_MOTOR, 1420)
                elif key == ord('l'):
                    # Tuşlara basılmadığında motorlar nötr konumda kalır
                    print("manual")
                    #MOCKED: controller.set_servo(SOL_MOTOR, 1500)
                    #MOCKED: controller.set_servo(SAG_MOTOR, 1500)


            cv2.putText(frame, f"FPS: {int(zed.get_current_fps())}", (10, 30), FONT, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"{str(zed.get_spatial_mapping_state())}", (10, 60), FONT, 0.5, (20, 220, 20), 1)
            cv2.putText(frame, f"POSITIONAL_TRACKING_STATE.{str(zed.get_position(pose, sl.REFERENCE_FRAME.WORLD))}",
                        (10, 90), FONT, 0.5, (20, 220, 20), 1)
            cv2.putText(frame, f"Coordinates X,Y: {current_x:.1f} {current_y:.1f}", (10, 120), FONT, 0.75,
                        (0, 150, 240), 1, )

            cv2.putText(frame, f"SOL İTİCİ HIZ İSTEĞİ: {int(get_servo_pwm(SOL_MOTOR))}", (10, 230), FONT, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"SAĞ İTİCİ HIZ İSTEĞİ: {int(get_servo_pwm(SAG_MOTOR))}", (10, 260), FONT, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"İDA GERÇEK HIZ: {int(get_servo_pwm(SAG_MOTOR))} m/s", (10, 290), FONT, 1, (0, 255, 0), 2)

            cv2.putText(frame, f"İDA GERÇEK HEADING: {int(magnetic_heading)}", (10, 320), FONT, 1, (0, 255, 0), 2)
            cv2.putText(frame, f"HEADING İSTEĞİ: {int(adviced_course)}", (10, 340), FONT, 1, (0, 255, 0), 2)

            cv2.putText(frame, f"KALAN MESAFE: {int(hedefe_mesafe)} metre", (10, 370), FONT, 1, (0, 255, 0), 2)

            # Görüntüyü göster
            frame_resized = cv2.resize(frame, (960, 540))  # Resize the frame to desired dimensions960, 540
            cv2.imshow("DHO KEMALREİS", frame_resized)

            if key % 256 == 27:
                print("Esc tuşuna basıldı.. Kapatılıyor..")
                #MOCKED: controller.set_servo(SOL_MOTOR,1500)
                #MOCKED: controller.set_servo(SAG_MOTOR,1500)
                break

    # Kaynakları serbest bırak ve kamerayı kapat
    cv2.destroyAllWindows()
    zed.close()


if __name__ == "__main__":
    main()