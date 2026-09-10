import math

wheel_base = 0.195  # m
def obs_pos(distance, angle_degrees):
    angle_radians = math.radians(angle_degrees)

    x = distance * math.cos(angle_radians)
    y = distance * math.sin(angle_radians)

    return x, y


def car_position(car_x, car_y, car_theta, deltaSL, deltaSR):
    """
    Car's position(cm) and orientation(degree)
    car_x, y: cm
    car_theta: degrees
    deltaSL, R: m
    """
    delta_s = (deltaSL + deltaSR) * 50
    delta_theta = (deltaSR - deltaSL) / 0.195
    
    car_x += delta_s * math.cos(math.radians(car_theta) + delta_theta / 2.0)
    car_y += delta_s * math.sin(math.radians(car_theta) + delta_theta / 2.0)

    car_theta += math.degrees(delta_theta)
    if car_theta > 180 :
       car_theta -=360
    if car_theta < -180:
       car_theta +=360

    return car_x, car_y, car_theta

def error_calculation(car_x, car_y, car_theta, goal_x, goal_y):
    dx = goal_x - car_x;
    dy = goal_y - car_y;
    if dx==0 and dy==0:
        errordistance = 0;
        errorAngle = 0;
    else:
        errordistance = math.hypot(dx, dy) / 100;
        targetAngle = -math.atan2(dy, dx) * 180 / math.pi;
        errorAngle = targetAngle - car_theta; 
        while (errorAngle > 180.0):  errorAngle -= 360.0;
        while (errorAngle < -180.0):  errorAngle += 360.0;
    
    return errordistance, errorAngle

def world_to_pixel(CENTER_X, CENTER_Y, x, y, GRID_SIZE):
    """
    Convert world coordinates (x, y) to pixel coordinates (px, py) for visualization.
    """
    px = int(CENTER_X + x * GRID_SIZE)
    py = int(CENTER_Y - y * GRID_SIZE)
    return px, py

def pixel_to_world(CENTER_X, CENTER_Y, x, y, GRID_SIZE):
    """將 Pygame 螢幕像素座標轉換為世界網格座標"""
    world_x = math.floor((x - CENTER_X) / GRID_SIZE)
    world_y = math.floor((CENTER_Y - y) / GRID_SIZE)
    return world_x, world_y