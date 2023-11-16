package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GlobalFunctions {

    public static double proportional(double target, double state, double modifier) {
        return (target - state) / modifier;
    }

    public static boolean check(double value) {
        if (value >= -0.005 && value <= 0.005) {
            return true;
        } else {
            return false;
        }
    }
}
