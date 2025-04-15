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
    left_circle(vehicle) if random.randint(0, 1) == 0 else right_circle(vehicle)





def left_circle(vehicle):
    vehicle.control(steering=-1, throttle=1, brake=0, parkingbrake=0,clutch=0, gear=3)
    time.sleep(2)

def right_circle(vehicle):
    vehicle.control(steering=1, throttle=1, brake=0, parkingbrake=0,clutch=0, gear=3)
    time.sleep(2)



def get_data(vehicle, imu, data_dict, yaw_rates, velocities):
    vehicle.sensors.poll()
    electrics_data = vehicle.sensors["electrics"]
    imu_data = imu.poll()[0.0]

    speed = electrics_data["virtualAirspeed"]
    yaw_rate = imu_data["angVel"][2]
    velocity = imu_data["angAccel"][2]
    water_temp = electrics_data["water_temperature"]
    wheelspeed = round(electrics_data["wheelspeed"], 1)

    turn_direction = "Straight"

    # Melyik irányba halad az auto
    if abs(yaw_rate) > 0.2:  # Ha van számottevő szögsebesség
        turn_direction = "Right" if yaw_rate > 0 else "Left"

    # Tároljuk a szenzoradatokat a szótárban
    data_dict["speed"] = speed
    data_dict["yaw_rate"] = yaw_rate
    data_dict["velocity"] = velocity
    data_dict["water_temp"] = water_temp
    data_dict["wheelspeed"] = wheelspeed
    data_dict["turn_direction"] = turn_direction

    # Logoláshoz és normalizáláshoz gyűjtjük őket külön is
    yaw_rates.append(yaw_rate)
    velocities.append(velocity)






def control_loop(vehicle, data_dict, yaw_rates, velocities):
    yaw_rate = data_dict["yaw_rate"]
    velocity = data_dict["velocity"]
    water_temp = data_dict["water_temp"]

    # Hőmérséklet figyelése
    if water_temp  > 115:
        cooling(vehicle)

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
    vehicle.control(steering=steering, throttle=throttle, brake=0)
    print("--------------------------")



def change_direction_calculate(yaw_rate):
    if abs(yaw_rate) > 1.2:
        return "Right" if yaw_rate > 0 else "Left"

def change_direction(vehicle, imu, data_dict, yaw_rates, velocities):
    current_direction = data_dict["turn_direction"]
    desired_direction = "Left" if current_direction == "Right" else "Right"

    # Állítsuk be a vezérlést az ellenkező irányba, 30% gázzal
    if desired_direction == "Left":
        vehicle.control(steering=1, throttle=0.3, brake=0.1)
    else:
        vehicle.control(steering=-1, throttle=0.3, brake=0.1)

    # Frissítsük folyamatosan az adatokat, amíg el nem érjük a kívánt irányt
    max_attempts = 200  # végtelen ciklus elkerülése végett
    attempts = 0
    while attempts < max_attempts:
        get_data(vehicle, imu, data_dict, yaw_rates, velocities)
        print("Current turning direction:", change_direction_calculate(data_dict["yaw_rate"]))
        if change_direction_calculate(data_dict["yaw_rate"]) == desired_direction:
            print("Direction change complete.")
            break
        time.sleep(0.1)
        attempts += 1

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
        "etk_intake_v8_4.4_petrol":"etk_intake_v8_4.4_petrol_rennspecht",
        "etk_DSE_drivemodes_default":"etk_DSE_drivemodes_default_off",
        }
    vehicle.set_part_config({"parts":config})

    imu = AdvancedIMU("accel1", bng, vehicle, gfx_update_time=0.01)
    electrics = Electrics()
    vehicle.sensors.attach("electrics", electrics)
    vehicle.set_shift_mode("realistic_automatic")

    
    left_circle(vehicle) if random.randint(0, 1) == 0 else right_circle(vehicle)

    yaw_rates = []
    velocities = []
    data_dict = {}


    i = 0

    while True:
        get_data(vehicle, imu, data_dict, yaw_rates, velocities)
        control_loop(vehicle, data_dict, yaw_rates, velocities)
        if i == 200:
            change_direction(vehicle, imu, data_dict, yaw_rates, velocities)
            i = 0
        i+=1
        time.sleep(0.1)

if __name__ == "__main__":
    main()


