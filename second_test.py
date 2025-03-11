import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import linregress
import random
from time import sleep
import math
import logging

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU, Electrics, Sensor, PowertrainSensor

def cooling(vehicle):
    vehicle.control(steering=0, throttle=0.2, brake=0, parkingbrake=0,clutch=0, gear=3)
    sleep(10)
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
    sleep(2)



    

    acc_historyX = []
    regressedX = []

    yaw_rates = []
    velocities = []

    log_data = []
    i = 0
    turn_direction=""
    normalized_yaw_rate = 0
    normalized_velocity = 0
    directions = []
    previous_directions = 0
    while True:
        sleep(0.1)  # Include a small delay between each reading.
        vehicle.sensors.poll()
        electrics_data = vehicle.sensors["electrics"]
        # print(electrics_data)
        speed = electrics_data["virtualAirspeed"]


        data = imu.poll()  # Fetch the latest readings from the sensor.
        data = data[0.0]

        if electrics_data["water_temperature"] > 115:
            cooling(vehicle)

        accSmooth = data["accSmooth"] # linearis gyorsulas

        velocity = data["angAccel"][2] # szoggyorsulas -> most kezdi-e a kanyarodast vagy mar kanyarodik
        velocities.append(velocity)
        yaw_rate = data["angVel"][2] # szogsebesseg
        yaw_rates.append(yaw_rate)

        if len(yaw_rates) > 1:
            normalized_yaw_rate = -1 + 2 * ((yaw_rate - min(yaw_rates)) / (max(yaw_rates) - min(yaw_rates)))
            normalized_velocity = -1 + 2 * ((velocity - min(velocities)) / (max(velocities) - min(velocities)))

            steering = normalized_yaw_rate+normalized_velocity

            # vehicle.control(steering=steering, throttle=1, brake=0)









        acc_historyX.append(accSmooth[0])
        if len(acc_historyX) > 10:
            acc_historyX.pop(0)

        # Lineáris regresszió számítása
        Xx = np.arange(len(acc_historyX))  # X tengely: indexek
        Xy = np.array(acc_historyX)  # Y tengely: gyorsulás adatok

        slope, intercept, _, _, _ = linregress(Xx, Xy)
        acc_regressedX = slope * (len(acc_historyX) - 1) + intercept  # Legfrissebb becsült érték
        regressedX.append(acc_regressedX)

        threshold = max(0.05, np.std(acc_historyX) * 1.2)

        if acc_regressedX > threshold:
            direction = "Forward"
            previous_directions = 1
        elif acc_regressedX < -threshold:
            direction = "Backward"
            previous_directions = -1
        else:
            if previous_directions != 0:
                if previous_directions > 0:
                    direction = "Forward"
                else: 
                    direction = "Backward"
            else:
                direction = "Stationary"


        if abs(yaw_rate) > 0.2:  # Ha van számottevő szögsebesség
            turn_direction = "Right" if yaw_rate > 0 else "Left"
            if direction != "Stationary":
                print(f"{turn_direction}-{direction}")
            else:
                print("Stationary")
        else:
            if direction != "Stationary":
                print(f"{direction}")
            else:
                print("Stationary")

        # print(f"AccSmooth: {accSmooth}, Regressed X: {acc_regressedX:.3f}, Threshold: {threshold:.3f}")

        # log_data.append(
        #         {
        #             "AccSmooth": accSmooth[0],
        #             "Regressed X":float(acc_regressedX),
        #             "Threshold":float(threshold),
        #             "Turn direction":turn_direction,
        #             "Direction":direction,
        #             "Yaw rate": yaw_rate,
        #             "Normalized Yaw rate": normalized_yaw_rate,
        #             "Velocity": velocity,
        #             "Normalized Velocity": normalized_velocity
        #         }
        #     )
        # i+=1

    # Adatok kiírása
    for i, entry in enumerate(log_data, start=1):
        print(f'--- Entry {i} ---')
        for key, value in entry.items():
            print(f'{key}: {value}')
        print('')

    print()
    print()

    print(yaw_rates)

    print()
    print()

    print(velocities)

if __name__ == "__main__":
    main()


