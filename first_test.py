import random
from time import sleep
import math

from beamngpy import BeamNGpy, Scenario, Vehicle, set_up_simple_logging
from beamngpy.sensors import AdvancedIMU, Electrics, Sensor, PowertrainSensor


def main():
    random.seed(1703)
    set_up_simple_logging()

    beamng = BeamNGpy("localhost", 25252, home="F:/BeamNG.tech.v0.31.3.0/BeamNG.tech.v0.31.3.0")
    bng = beamng.open(launch=True)

    scenario = Scenario("smallgrid","autonomous_drifting_demo",)
    vehicle = Vehicle("ego_vehicle", model="etk800", license="SPEED-007", color="Blue")

    scenario.add_vehicle(vehicle, pos=(0,0,0))
    scenario.make(bng)

    bng.settings.set_deterministic(60)  # Set simulator to 60hz temporal resolution

    bng.scenario.load(scenario)
    bng.scenario.start()

    # NOTE: Create sensor after scenario has started.
    imu = AdvancedIMU("accel1", bng, vehicle, gfx_update_time=0.01)
    


    # sleep(5)
    vehicle.set_shift_mode("realistic_automatic")
    
    sleep(3)
    vehicle.control(steering=-1, throttle=1, brake=0, parkingbrake=0,clutch=0, gear=2)
    sleep(2.5)

    yaw_rates = []
    velocities = []
    while True:
        sleep(0.1)  # Include a small delay between each reading.
        data = imu.poll()  # Fetch the latest readings from the sensor.

        velocity = data[0.0]["angAccel"][2]
        velocities.append(velocity)

        yaw_rate = data[0.0]["angVel"][2]
        yaw_rates.append(yaw_rate)

        if len(yaw_rates) > 1:
            normalized_yaw_rate = -1 + 2 * ((yaw_rate - min(yaw_rates)) / (max(yaw_rates) - min(yaw_rates)))
            normalized_velocity = -1 + 2 * ((velocity - min(velocities)) / (max(velocities) - min(velocities)))

            aggression = 2
            steering = (normalized_yaw_rate*aggression)+normalized_velocity

            print(f"Steering: {steering}, Yaw Rate: {normalized_yaw_rate}, Velocity: {normalized_velocity}")
            vehicle.control(steering=steering, throttle=1, brake=0, parkingbrake=0, clutch=0, gear=2)

if __name__ == "__main__":
    main()
