package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    class Directions {
        public static final int FORWARD = 0;
        public static final int BACKWARD = 1;
        public static final int LEFT = 2;
        public static final int RIGHT = 3;
        public static final int CLOCKWISE = 0;
        public static final int COUNTER_CLOCKWISE = 1;
    }

    class AutonomousConstants {
        public static final double turningP = 0.0000015;
        public static final double strafeP = 1/10^10;
        public static final double lateralP = 1/10^10;
        public static final double breakThreshold = 0.01;
    }

    class HardwareConstants {
        public static final double odometerWheelCircumference = (23.04 * Math.PI);
    }
}
