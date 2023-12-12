package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class GlobalFunctions {

    public static double proportional(double target, double state, double modifier) {
        return (target - state) * modifier;
    }

    public static boolean check(double value) {
        if (value >= -0.005 && value <= 0.005) {
            return true;
        } else {
            return false;
        }
    }

    public static void wait(double currentSeconds, double waitSeconds) {
        while(true) {
            if(currentSeconds == waitSeconds) {
                break;
            }
        }
    }

    public static double slew(double input, double prev, double rate) {
//        if (rate < Math.abs(input - prev)) { // Can slew
//            if (input < prev) {
//                return prev - rate;
//            } else if (input > prev) {
//                return prev + rate;
//            }
//        }
        return input; // Close enough that you can just use input
    }
}
