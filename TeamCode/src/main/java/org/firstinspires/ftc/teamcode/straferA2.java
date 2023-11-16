package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.GlobalFunctions;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "Strafer Blue A2", group = "Strafer Competition")
@Disabled
public class straferA2 extends LinearOpMode {

    //Declaring Left and Right Motors
    DcMotorEx leftBack = null;
    DcMotorEx leftFront = null;

    DcMotorEx rightBack = null;
    DcMotorEx rightFront = null;

    //Declaring IMU (builtin 3-Axis Accelerometer)
    IMU imu = null;

    DcMotor strafeEncoder;

    //Defining a modifier to be used later
    double proportionalModifier = 0.015;

    @Override
    public void runOpMode() {

        //Hardware Mapping left and right motors and IMU
        leftBack = hardwareMap.get(DcMotorEx.class, "blmotor");
        leftFront = hardwareMap.get(DcMotorEx.class, "flmotor");

        rightBack = hardwareMap.get(DcMotorEx.class, "brmotor");
        rightFront = hardwareMap.get(DcMotorEx.class, "frmotor");

        strafeEncoder = hardwareMap.get(DcMotor.class, "strafeEncoder");

        imu = hardwareMap.get(IMU.class, "imu");

        //Creating IMU configuration object
        IMU.Parameters imuParam = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
                )
        );

        //Initializing IMU and Start Actions
        imu.initialize(imuParam);
        waitForStart();
        strafeRobot(.5, -2600);
        lateralRobot(.5, -3850);
    }

    //Creating turnRobot method to turn to an angle using a Proportional Function [-180, 180]
    public void turnRobot(double angle) {
        imu.resetYaw();

        //Loop while the parameter angle is not met
        while (opModeIsActive()) {

            //Set mode to not use encoder
            leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            //Creating and storing Robot Yaw value in Degrees
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            //Assigning each side a separate power using the Proportional EQ and the target angle
            double leftPower = GlobalFunctions.proportional(angle, yaw, proportionalModifier);
            double rightPower = GlobalFunctions.proportional(angle, yaw, proportionalModifier);

            //Powers motors to previously defined powers
            leftBack.setPower(leftPower);
            leftFront.setPower(leftPower);

            rightBack.setPower(rightPower);
            rightFront.setPower(rightPower);

            if(GlobalFunctions.proportional(angle, yaw, proportionalModifier) >= -0.01 && GlobalFunctions.proportional(angle, yaw, proportionalModifier) <= 0.01) {
                break;
            }
        }

        leftBack.setPower(0);
        leftFront.setPower(0);

        rightBack.setPower(0);
        rightFront.setPower(0);

        runUsingEncoders();
    }

    public void strafeRobot(double power, int distance) {
        imu.resetYaw();

        resetMotors();

        while (opModeIsActive()) {

            leftBack.setTargetPosition(-distance);
            leftFront.setTargetPosition(distance);
            rightBack.setTargetPosition(-distance);
            rightFront.setTargetPosition(distance);

            runToPositionMode();

            leftBack.setPower(power);
            leftFront.setPower(power);
            rightBack.setPower(power);
            rightFront.setPower(power);

            if (!leftBack.isBusy() && !leftFront.isBusy()
                    && !rightBack.isBusy() && !rightFront.isBusy()) {
                break;
            }
        }

        runUsingEncoders();
    }

    public void lateralRobot(double power, int distance) {
        imu.resetYaw();

        resetMotors();

        int lbStart = leftBack.getCurrentPosition();
        int lfStart = leftFront.getCurrentPosition();

        int rbStart = rightBack.getCurrentPosition();
        int rfStart = rightFront.getCurrentPosition();

        while (opModeIsActive()) {

            leftBack.setTargetPosition(lbStart + distance);
            rightFront.setTargetPosition(-(rfStart + distance));
            leftFront.setTargetPosition(lfStart + distance);
            rightBack.setTargetPosition(-(rbStart + distance));


            runToPositionMode();

            leftBack.setPower(power);
            leftFront.setPower(power);
            rightBack.setPower(power);
            rightFront.setPower(power);

            if (!leftBack.isBusy() && !leftFront.isBusy()
                    && !rightBack.isBusy() && !rightFront.isBusy()) {
                break;
            }
        }

        runUsingEncoders();
    }

    public void resetMotors() {
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPositionMode() {
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void runUsingEncoders() {
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
