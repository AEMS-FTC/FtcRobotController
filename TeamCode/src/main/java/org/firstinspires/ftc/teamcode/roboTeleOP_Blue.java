package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="roboTeleOP (BLUE)", group = "Strafer Competition")
public class roboTeleOP_Blue extends OpMode {
    DcMotor blmotor;
    DcMotor flmotor;
    DcMotor brmotor;
    DcMotor frmotor;
    DcMotor lift;
    Servo launcher;
    Servo clawPitch;
    Servo clawRoll;
    Servo clawActuate;
    IMU gyro;
    DcMotor strafeEncoder;

    DcMotor leftOdometer;
    DcMotor rightOdometer;

    @Override
    public void init(){

        blmotor = hardwareMap.get(DcMotor.class,"blmotor");
        flmotor = hardwareMap.get(DcMotor.class,"flmotor");
        brmotor = hardwareMap.get(DcMotor.class,"brmotor");
        frmotor = hardwareMap.get(DcMotor.class,"frmotor");

        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.get(DcMotor.class, "hook");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher = hardwareMap.get(Servo.class, "railServo");
        clawActuate = hardwareMap.get(Servo.class, "claw");
        clawPitch = hardwareMap.get(Servo.class, "clawPitch");
        clawRoll = hardwareMap.get(Servo.class, "clawRoll");

        strafeEncoder = hardwareMap.get(DcMotor.class, "strafeEncoder");

        leftOdometer = hardwareMap.get(DcMotor.class, "leftOdometer");
        rightOdometer = hardwareMap.get(DcMotor.class, "rightOdometer");

        clawPitch.scaleRange(-1, 1);
        clawRoll.scaleRange(-1, 1);

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParam = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));

        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        gyro.initialize(imuParam);
        leftOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        strafeEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        gyro.resetYaw();
    }
    @Override
    public void loop() {

        double pModifier;

        if (gamepad1.left_bumper) {
            pModifier = 0.75;
        } else {
            pModifier = 1;
        }

        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

//EXPIREMENTAL:
        double gyroYawRad = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + 1.5708;
//        Field Orientated Drive (Relative to IMU Yaw)
        double temp = axial * Math.cos(gyroYawRad) - lateral * Math.sin(gyroYawRad);
        lateral = axial * Math.sin(gyroYawRad) + lateral * Math.cos(gyroYawRad);
        axial = temp;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial - lateral + yaw;
        double rightBackPower  = axial + lateral - yaw;

        if(gamepad1.start) {
            gyro.resetYaw();
        }

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower  /= max;
            rightFrontPower /= max;
            leftBackPower   /= max;
            rightBackPower  /= max;
        }

        // Send calculated power to wheels
        flmotor.setPower(leftFrontPower * pModifier);
        frmotor.setPower(rightFrontPower * pModifier);
        blmotor.setPower(leftBackPower * pModifier);
        brmotor.setPower(rightBackPower * pModifier);

        lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad2.a) {
            launcher.setPosition(0);
        } else if (gamepad2.b) {
            launcher.setPosition(0.4);
        } else {}

        if(gamepad2.left_bumper) {
            clawActuate.setPosition(0.3);
        } else if (gamepad2.right_bumper) {
            clawActuate.setPosition(0);
        }

        if (gamepad2.dpad_up) {
            clawPitch.setPosition(0.45);
        } else if (gamepad2.dpad_right) {
            clawPitch.setPosition(0.5);
        } if (gamepad2.dpad_down) {
            clawPitch.setPosition(0.03);
        }
        clawRoll.setPosition(Range.clip(gamepad2.right_stick_x, 0.03, 1));

        // Show the wheel power.
        telemetry.addData("Strafe Encoder", strafeEncoder.getCurrentPosition());
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Robot Yaw", Math.round(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        telemetry.addData("Left Odometer", Math.round(
                leftOdometer.getCurrentPosition() * 2000 / Constants.HardwareConstants.odometerWheelCircumference
        ));
        telemetry.addData("Right Odometer", Math.round(
                rightOdometer.getCurrentPosition() * 2000 / Constants.HardwareConstants.odometerWheelCircumference
        ));
        telemetry.addData("Strafe Odometer", Math.round(
                strafeEncoder.getCurrentPosition() * 2000 / Constants.HardwareConstants.odometerWheelCircumference
        ));
        telemetry.update();

    }
}
