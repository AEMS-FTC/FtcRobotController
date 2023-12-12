package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="roboTeleOP (RED)", group = "Strafer Competition")
public class roboTeleOP_Red extends OpMode {
    DcMotor blmotor;
    DcMotor flmotor;
    DcMotor brmotor;
    DcMotor frmotor;
    DcMotor lift;
    CRServo launcher;
    Servo clawPitch;
    Servo clawRoll;
    Servo clawActuate;
    IMU gyro;

    double clawPitchAngle;

    @Override
    public void init(){

        blmotor = hardwareMap.get(DcMotor.class,"left_back");
        flmotor = hardwareMap.get(DcMotor.class,"left_front");
        brmotor = hardwareMap.get(DcMotor.class,"right_back");
        frmotor = hardwareMap.get(DcMotor.class,"right_front");

        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift = hardwareMap.get(DcMotor.class, "hook");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcher = hardwareMap.get(CRServo.class, "railServo");
        clawActuate = hardwareMap.get(Servo.class, "claw");
        clawPitch = hardwareMap.get(Servo.class, "clawPitch");
        clawRoll = hardwareMap.get(Servo.class, "clawRoll");

        clawPitch.scaleRange(-1, 1);
        clawRoll.scaleRange(-1, 1);

        clawPitchAngle = 0.03;

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
        double gyroYawRad = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - 1.5708;
//        Field Orientated Drive (Relative to IMU Yaw)
        double temp = axial * Math.cos(gyroYawRad) - lateral * Math.sin(gyroYawRad);
        lateral = axial * Math.sin(gyroYawRad) + lateral * Math.cos(gyroYawRad);
        axial = temp;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower  = axial - lateral + yaw; //swapped
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower   = axial + lateral + yaw; //swapped
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
        flmotor.setPower(GlobalFunctions.slew((leftFrontPower * pModifier), flmotor.getPower(), 0.15));
        frmotor.setPower(GlobalFunctions.slew((rightFrontPower * pModifier), frmotor.getPower(), 0.15));
        blmotor.setPower(GlobalFunctions.slew((leftBackPower * pModifier), blmotor.getPower(), 0.15));
        brmotor.setPower(GlobalFunctions.slew((rightBackPower * pModifier), brmotor.getPower(), 0.15));

        lift.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

        if(gamepad2.a) {
            launcher.setPower(-1);
        } else if (gamepad2.b) {
            launcher.setPower(1);
        } else {}

        if(gamepad2.left_bumper) {
            clawActuate.setPosition(0.3);
        } else if (gamepad2.right_bumper) {
            clawActuate.setPosition(0);
        }

        clawPitchAngle = Range.clip(clawPitchAngle, 0.03, 0.45);
        clawPitchAngle += -gamepad2.left_stick_y*0.025;

//        if (gamepad2.dpad_up) {
//            clawPitch.setPosition(0.45);
//        } else if (gamepad2.dpad_right) {
//            clawPitch.setPosition(0.5);
//        } if (gamepad2.dpad_down) {
//            clawPitch.setPosition(0.03);
//        }
        clawPitch.setPosition(clawPitchAngle);

        clawRoll.setPosition(Range.clip(gamepad2.right_stick_x, 0.03, 1));

        // Show the wheel power.
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Robot Yaw", Math.round(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
        telemetry.update();

    }
}
