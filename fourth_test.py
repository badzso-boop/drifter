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
    electrics_data = vehicle.sensors["electrics"]
    while electrics_data["water_temperature"] > 91:
        vehicle.sensors.poll()
        electrics_data = vehicle.sensors["electrics"]
    while round(electrics_data["wheelspeed"],1) != 0:
        vehicle.sensors.poll()
        electrics_data = vehicle.sensors["electrics"]
        vehicle.control(steering=0, throttle=0, brake=1)
    time.sleep(5)
    vehicle.control(steering=-1, throttle=1, brake=0)
    time.sleep(2)

def left_circle(vehicle):
    vehicle.control(steering=-1, throttle=1, brake=0, parkingbrake=0,clutch=0, gear=3)
    time.sleep(2)

def right_circle(vehicle):
    vehicle.control(steering=1, throttle=1, brake=0, parkingbrake=0,clutch=0, gear=3)
    time.sleep(2)

def control_loop(vehicle, imu, yaw_rates, velocities):
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
    yaw_rates.append(yaw_rate)

    # Az angAccel[2] a szöggyorsulás (opcionális felhasználás)
    velocity = data["angAccel"][2]
    velocities.append(velocity)

    desired_yaw_rate = 2
    # Hiba kiszámítása: kívánt yaw_rate - mért yaw_rate
    error = desired_yaw_rate - abs(yaw_rate)
    print("yaw_rate:", yaw_rate)
    print("error:", error)

    # Gázpedál szabályozás: ha a drift hiba vagy a velocity nagy, csökkentsük a throttle-t
    # Például: ha az abs(error) meghalad egy küszöböt, csökkentsünk
    error_threshold = 0.1  # ez egy kísérleti küszöb (rad/s-ben)
    if abs(error) > error_threshold:
        throttle = max(0.3, 1 - abs(error)*2)  # egyszerű szabályozás
    else:
        throttle = 1.0
    print("Throttle:", throttle)

    # Melyik irányba halad az auto
    if abs(yaw_rate) > 0.2:  # Ha van számottevő szögsebesség
        turn_direction = "Right" if yaw_rate > 0 else "Left"
        print(turn_direction)

    #Vezérlés normalizálva -1 és 1 közé
    steering = 0
    if len(yaw_rates) > 1:
        normalized_yaw_rate = -1 + 2 * ((yaw_rate - min(yaw_rates)) / (max(yaw_rates) - min(yaw_rates)))
        normalized_velocity = -1 + 2 * ((velocity - min(velocities)) / (max(velocities) - min(velocities)))

        print("Normalized_yaw_rate: ", normalized_yaw_rate)
        print("Normalized_velocity: ", normalized_velocity)
        aggression = 2
        steering = (normalized_yaw_rate*aggression)+normalized_velocity
        print("Steering:", steering)

    if len(yaw_rates) > 20:
        yaw_rates.pop(0)
        velocities.pop(0)

    # Küldjük el a vezérlési parancsokat
    vehicle.control(steering=steering, throttle=throttle)
    print("--------------------------")


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

    
    left_circle(vehicle) if random.randint(0, 1) == 0 else right_circle(vehicle)

    yaw_rates = []
    velocities = []

    while True:
        control_loop(vehicle, imu, yaw_rates, velocities)
        time.sleep(0.1)

if __name__ == "__main__":
    main()


