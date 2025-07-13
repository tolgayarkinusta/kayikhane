import math

def signed_angle_difference(C, A):  # C as current_heading(magnetic_heading) and A as adviced_course
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


def haversine(lat1, lon1, lat2, lon2):  # todo: add to git if success
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
