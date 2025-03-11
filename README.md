# Autonomous Drifting Project

This research project focuses on developing an autonomous drifting algorithm that can control a vehicle in a continuously unstable state while also exploring its application in real-world stability control scenarios. By leveraging sensors such as cameras, LIDAR, and IMUs, the study aims to implement classical control methods (PID, MPC) to achieve precise vehicle maneuvering. The project will analyze different drivetrain configurations (FWD, AWD, RWD) to optimize drifting techniques and improve vehicle control. Ultimately, the knowledge gained from autonomous drifting will be applied to enhance vehicle stability in critical situations, such as hydroplaning or sudden traction loss, contributing to advancements in automotive safety and performance. Testing and validation will be conducted using BeamNG.drive simulations, with findings documented for potential real-world applications in emergency vehicle control, high-performance driving, and highway safety systems.

# Research plan

1. **Sensor selection and Data Acquisition**
    - __Camera__ – detects the road and track layout.
    - __LIDAR__ – provides depth perception, useful for recognizing track edges and obstacles.
    - __IMU__ (Inertial Measurement Unit: accelerometer + gyroscope) – measures angular velocity, tilt and accelaration.
2. **Algorithm Development**
    The objective is to create a control algorithm that maintains the vehicle in an unstable yet controlled drift.
    - Classical control methods (PID, MPC) – initial experiments could use PID control, followed by Model Predictive Control (MPC) for more advanced optimization.
3. **Drifting Mechanics and Drift controlling**
    - Throttle and braking optimization.
    - Steering angle adjustments.
    - Rear-wheel slip control.
4. **Stability Controll Research**
    Once the drifting algorithm is functional, the next step is to investigate how the collected data can be used for stability recovery in real-world situations. The system would:
    - Detect loss of control (using IMU, LIDAR, and camera data).
    - Automatically intervene (adjust steering, throttle, and braking).
    - React faster than a human driver to prevent accidents.

# Steps in the development

1. step - __2025.02.12__
    - Love drifting! :heart:
    - Make the plan in the `Autonomous Drifting and Stability Control Research Proposal.docx`
    - Request access for **Beamng.tech** (Thank you! :grinning: )
2. step - __2025.02.20__
    - Learn how to use `beamngpy`
    - Make basic scenarios
    - Analyze the data from the sensors
    - Draw some animation on paper to understand the basics of the project
3. step - __2025.03.04__
    - Make the `first_test.py` and the `second_test.py`
        - [first_test.py](https://github.com/badzso-boop/drifter/blob/main/first_test.py)
            - The basics of donut drifting with only the steering
            - Not perfect, because it goes in a spiral not in a circle
        - [second_test.py](https://github.com/badzso-boop/drifter/blob/main/second_test.py)
            - Recognize the directions the car is moving with some error treshold
4. step - __2025.03.11__
    - __Almost perfect__ donut drifting, just a little spiral, sometimes almost none
    - It works with manipulating the **steering** and the **throttle**
    - Make the `third_test.py` and the `fourth_test.py`
        - [third_test.py](https://github.com/badzso-boop/drifter/blob/main/third_test.py)
            - Basic PID controlls with error threshold and some __chatgpt__ code that is not working fine :grinning:
            - It helped to understand how to synchronize the steering and the throttle
        - [fourth_test.py](https://github.com/badzso-boop/drifter/blob/main/fourth_test.py)
            - Merge the knowledge of the three file before
            - Controll **steering** and **throttle** based on the __velocity__ and __yaw_rate__
            - Built in some cooling algorithm too. If the car reaches a certain water temperature it will drive forward to cool down.
            - The algorithm has a `desired_yaw_rate` which tries to match the `yaw_rate`
            - It has some threshold for the throttle too.
5. step
    - Make the car do some 8 not just donuts
> [!NOTE]
> Still under planning and construction :grinning: