package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants.Directions;

import java.util.Timer;

@Autonomous(name = "Odometer Movement Tests", group = "Strafer")
public class AutonomousTemplate extends LinearOpMode {

    //* Declaring Hardware Devices
    //Motors
    DcMotorEx leftBack = null;
    DcMotorEx leftFront = null;
    DcMotorEx rightBack = null;
    DcMotorEx rightFront = null;
    Servo clawPitch;
    Servo clawRoll;
    Servo clawActuate;

    //Odometer Pods
    DcMotorEx strafeOdometer = null;
    DcMotorEx leftLateralOdometer = null;
    DcMotorEx rightLateralOdometer = null;



    double runtime = getRuntime();

    //IMU (builtin 3-axis accelerometer)
    IMU gyroscope = null;

    //* Function Runs on Initialization
    @Override
    public void runOpMode() {

        //Left Back Motor Configuration
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        //Left Front Motor Configuration
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        //Right Back Motor Configuration
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        //Right Front Motor Configuration
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        clawActuate = hardwareMap.get(Servo.class, "claw");
        //clawPitch = hardwareMap.get(Servo.class, "clawPitch");
        //clawRoll = hardwareMap.get(Servo.class, "clawRoll");

        //Odometer Pod Configurations
        strafeOdometer = hardwareMap.get(DcMotorEx.class, "strafe_odometer");
        leftLateralOdometer = hardwareMap.get(DcMotorEx.class, "left_odometer");
        rightLateralOdometer = hardwareMap.get(DcMotorEx.class, "right_odometer");

        //IMU and IMU Parameter Configuration
        gyroscope = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters gyroParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        //Initializing IMU
        gyroscope.initialize(gyroParameters);

        //* Waiting for routine to start
        waitForStart();
        resetRuntime();

        //Resetting IMU
        gyroscope.resetYaw();

        //! COMMANDS
//        linearMovement(Directions.RIGHT, 100);
//        rotate90deg(Directions.COUNTER_CLOCKWISE);
//        rotate90deg(Directions.COUNTER_CLOCKWISE);
//        linearMovement(Directions.RIGHT, 100);
//        rotate90deg(Directions.COUNTER_CLOCKWISE);
//        rotate90deg(Directions.COUNTER_CLOCKWISE);

        linearMovement(Directions.FORWARD, 100);
        linearMovement(Directions.RIGHT, 100);
        linearMovement(Directions.BACKWARD, 100);
        linearMovement(Directions.LEFT, 100);


//        while(opModeIsActive()) {
//            clawActuate.getController().setServoPosition(clawActuate.getPortNumber(), 0.5);
//        }
    }

//! OUTSIDE ROUTINE CODE !

    //* Movement
    public void linearMovement(int direction, double distance) {
        //Reset Hardware Devices
        resetMotors();
        resetOdometers();

        //Storing odometer start position
        double lateralStartPosition = leftLateralOdometer.getCurrentPosition();
        double strafeStartPosition = strafeOdometer.getCurrentPosition();

        //Defines Power Values
        double lateralPower = 0;
        double strafePower = 0;

        //Creates a value based on the # of ticks in a given distance
        double tickDistance = distance * 2000 / Constants.HardwareConstants.odometerWheelCircumference;

        //Command loop
        while (opModeIsActive()) {

            resetRuntime();

            if (direction == Directions.FORWARD) {
                lateralPower = GlobalFunctions.proportional(
                        lateralStartPosition + tickDistance,
                        -rightLateralOdometer.getCurrentPosition(),
                        Constants.AutonomousConstants.lateralP
                );

                lateralMotorPower(Range.clip(lateralPower, -1, 1));

            } else if (direction == Directions.BACKWARD) {
                lateralPower = GlobalFunctions.proportional(
                        lateralStartPosition - tickDistance,
                        -rightLateralOdometer.getCurrentPosition(),
                        Constants.AutonomousConstants.lateralP
                );

                lateralMotorPower(Range.clip(lateralPower, -1, 1));

            } else if (direction == Directions.LEFT) {
                strafePower = GlobalFunctions.proportional(
                        strafeStartPosition - tickDistance,
                        -strafeOdometer.getCurrentPosition(),
                        Constants.AutonomousConstants.strafeP
                );

                strafeMotorPower(Range.clip(strafePower, -1, 1));

            } else if (direction == Directions.RIGHT) {
                strafePower = GlobalFunctions.proportional(
                        strafeStartPosition + tickDistance,
                        -strafeOdometer.getCurrentPosition(),
                        Constants.AutonomousConstants.strafeP
                );

                strafeMotorPower(Range.clip(strafePower, -1, 1));

            }

            if (lateralPower >= -0.01 && lateralPower <= 0.01
                    && strafePower >= -0.01 && strafePower <= 0.01) {
                break;
            }

            if(getRuntime() >= 3) {
                break;
            }
        }

        resetMotors();
        resetOdometers();
        gyroscope.resetYaw();
    }

    public void rotate90deg(int direction) {

        resetRuntime();

        //Reset Hardware Devices
        resetMotors();
        resetOdometers();
        gyroscope.resetYaw();

        //Defines Power value
        double power = 0;

        //Command Loop
        while(opModeIsActive()) {

            if (direction == Directions.CLOCKWISE) {
                power = GlobalFunctions.proportional(
                        -90,
                        gyroscope.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                        Constants.AutonomousConstants.turningP
                );
            } else if (direction == Directions.COUNTER_CLOCKWISE) {
                power = GlobalFunctions.proportional(
                        90,
                        gyroscope.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),
                        Constants.AutonomousConstants.turningP
                );
            } else {
                power = 0;
            }

            rotaryMotorPower(Range.clip(power, -1, 1));

            if(power >= -0.01 && power <= 0.01 || getRuntime() >= 2) {
                break;
            }
        }
    }

    //* Reset Functions
    public void resetMotors() {
        leftBack.setPower(0);
        leftFront.setPower(0);

        rightBack.setPower(0);
        rightFront.setPower(0);
    }

    public void resetOdometers() {
        strafeOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLateralOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLateralOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //* Control Functions
    public void lateralMotorPower(double power) {
        leftBack.setPower(power);
        leftFront.setPower(power);

        rightBack.setPower(power);
        rightFront.setPower(power);
    }

    public void strafeMotorPower(double power) {
        leftBack.setPower(power);
        leftFront.setPower(-power);

        rightBack.setPower(power);
        rightFront.setPower(-power);
    }

    public void rotaryMotorPower(double power) {
        leftBack.setPower(-power);
        leftFront.setPower(-power);

        rightBack.setPower(power);
        rightFront.setPower(power);
    }
}

//THIS HURTS MY BRAIN.