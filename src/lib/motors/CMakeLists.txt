idf_component_register(
        SRCS src/DCMotor.cpp src/AbstractDriver.cpp src/AbstractMotors.cpp src/DRV8876.cpp src/I2CDCDriver.cpp src/WheelMotor.cpp
        INCLUDE_DIRS include
        REQUIRES pca9685 mathutils
)