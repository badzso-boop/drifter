import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import linregress
import time
import random
import math
import logging

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU, Electrics, Sensor, PowertrainSensor

def cooling(vehicle):
    vehicle.control(steering=0, throttle=0.2, brake=0, parkingbrake=0,clutch=0, gear=3)
    time.sleep(10)
    electrics_data = vehicle.sensors["electrics"]
    while round(electrics_data["wheelspeed"],1) != 0:
        vehicle.sensors.poll()
        electrics_data = vehicle.sensors["electrics"]
        vehicle.control(steering=0, throttle=0, brake=1)
    vehicle.control(steering=0, throttle=1, brake=0)

def main():
    random.seed(1703)
    set_up_simple_logging()
    logging.basicConfig(level=logging.INFO, format='%(message)s')

    beamng = BeamNGpy("localhost", 25252, home="F:/BeamNG.tech.v0.31.3.0/BeamNG.tech.v0.31.3.0")
    bng = beamng.open(launch=True)

    scenario = Scenario("smallgrid","autonomous_drifting_demo",)
    vehicle = Vehicle("ego_vehicle", model="etk800", license="SPEED-007", color="Blue")

    scenario.add_vehicle(vehicle, pos=(0,0,0))
    scenario.make(bng)

    bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.scenario.start()
    
    options = vehicle.get_part_options()

    config = {
        "etk800_differential_R": "etk800_differential_R_active_LSD",
        "etk800_radiator": "etk800_radiator_high_performance",
        "etk800_steering_wide":"etk800_steering_wide_drift",
        "etk_engine":"etk_engine_v8_4.4_petrol",
        "etk_intake_v8_4.4_petrol":"etk_intake_v8_4.4_petrol_rennspecht"}
    vehicle.set_part_config({"parts":config})

    imu = AdvancedIMU("accel1", bng, vehicle, gfx_update_time=0.01)
    electrics = Electrics()
    vehicle.sensors.attach("electrics", electrics)
    vehicle.set_shift_mode("realistic_automatic")

    vehicle.control(steering=1, throttle=1, brake=0, parkingbrake=0,clutch=0, gear=3)
    time.sleep(2)

    # PID paraméterek (kísérleti értékek, finomhangolni kell)
    Kp = 0.5
    Ki = 0.1
    Kd = 0.05

    # A kívánt drift szöghez tartozó yaw_rate (3° rad/s)
    # Ha a drift például pozitív irányú, akkor:
    # desired_yaw_rate = math.radians(3)  # ~0.05236 rad/s
    desired_yaw_rate = -2  # ~0.05236 rad/s
    # Ha a drift negatív irányú kell legyen, akkor: desired_yaw_rate = -math.radians(3)

    error_integral = 0.0
    prev_error = 0.0
    prev_time = time.time()  # vagy az első adatból származó idő

    def control_loop():
        nonlocal error_integral, prev_error, prev_time

        vehicle.sensors.poll()
        # Lekérjük az adatok legfrissebb állapotát
        electrics_data = vehicle.sensors["electrics"]
        speed = electrics_data["virtualAirspeed"]

        data = imu.poll()[0.0]

        # Hőmérséklet figyelése
        if electrics_data["water_temperature"] > 115:
            cooling(vehicle)

        # Szenzoradatok: az angVel[2] a yaw_rate
        yaw_rate = data["angVel"][2]
        print("yaw_rate:",yaw_rate)
        # Az angAccel[2] a szöggyorsulás (opcionális felhasználás)
        yaw_accel = data["angAccel"][2]
        print("yaw_accel:", yaw_accel)

        # Számoljuk ki a vezérlési ciklus idejét (dt)
        current_time = time.time()
        dt = current_time - prev_time if current_time - prev_time > 0 else 0.01
        prev_time = current_time
        # print("dt:", dt)

        # Hiba kiszámítása: kívánt yaw_rate - mért yaw_rate
        error = desired_yaw_rate - yaw_rate
        # print("error:",error)
        

        # Integráljuk a hibát
        error_integral += error * dt

        # Derivált számítása
        error_derivative = (error - prev_error) / dt
        prev_error = error

        # PID kimenet a kormányzásra
        steering_command = Kp * error + Ki * error_integral + Kd * error_derivative
        # print("steering_command:",steering_command)
        print("------------------")

        # Gázpedál szabályozás: ha a drift hiba nagy, csökkentsük a throttle-t
        # Például: ha az abs(error) meghalad egy küszöböt, csökkentsünk
        error_threshold = 0.1  # ez egy kísérleti küszöb (rad/s-ben)
        if abs(error) > error_threshold:
            throttle = max(0.5, 1 - abs(error) * 5)  # egyszerű szabályozás
        else:
            throttle = 1.0

        # Küldjük el a vezérlési parancsokat
        # vehicle.control(steering=steering_command, throttle=throttle, brake=0)

    while True:
        control_loop()
        time.sleep(0.1)

    

if __name__ == "__main__":
    main()


