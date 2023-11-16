package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="testing", group = "EXPIREMENTAL")
@Disabled
public class testing extends OpMode {
    DcMotor blmotor;
    DcMotor flmotor;
    DcMotor brmotor;
    DcMotor frmotor;

    DcMotor lift;

    Double[] lateralValues = {};
    Double[] strafeValues = {};

    Double[] rotateValues = {};

    Servo launcher;

    IMU gyro;
    @Override
    public void init(){

        blmotor = hardwareMap.get(DcMotor.class,"blmotor");
        flmotor = hardwareMap.get(DcMotor.class,"flmotor");
        brmotor = hardwareMap.get(DcMotor.class,"brmotor");
        frmotor = hardwareMap.get(DcMotor.class,"frmotor");
        lift = hardwareMap.get(DcMotor.class, "hook");

        launcher = hardwareMap.get(Servo.class, "railServo");

        gyro = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters imuParam = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD
        ));


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
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
    public void loop(){
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial   = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral =  gamepad1.left_stick_x;
        double yaw     =  gamepad1.right_stick_x;

//        double gyroYawRad = gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //Field Orientated Drive (Relative to IMU Yaw)
//        double temp = axial * Math.cos(gyroYawRad) - lateral * Math.sin(gyroYawRad);
//        lateral = axial * Math.sin(gyroYawRad) + lateral * Math.cos(gyroYawRad);
//        axial = temp;

        //testing

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

        // This is test code:
        //
        // Uncomment the following code to test your motor directions.
        // Each button should make the corresponding motor run FORWARD.
        //   1) First get all the motors to take to correct positions on the robot
        //      by adjusting your Robot Configuration if necessary.
        //   2) Then make sure they run in the correct direction by modifying the
        //      the setDirection() calls above.
        // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        flmotor.setPower(leftFrontPower);
        frmotor.setPower(rightFrontPower);
        blmotor.setPower(leftBackPower);
        brmotor.setPower(rightBackPower);

        lift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

        if(gamepad2.a) {
            launcher.setPosition(0);
        } else if (gamepad2.b) {
            launcher.setPosition(0.4);
        } else {}

        // Show the elapsed game time and wheel power.
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.addData("Robot Yaw", Math.round(gyro.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)));
//        telemetry.addData("Control lateral/strafe/rotate", "%4.2f, %4.2f, %4.2f", lateral, axial, yaw);
        telemetry.update();


        /*
        if (gamepad1.atRest()) {
            blmotor.setPower(0);
            flmotor.setPower(0);
            brmotor.setPower(0);
            frmotor.setPower(0);
        }
        if (gamepad1.right_stick_y != 0) {
            blmotor.setPower(1 * gamepad1.right_stick_y);
            flmotor.setPower(1 * gamepad1.right_stick_y);
            brmotor.setPower(-1 * gamepad1.right_stick_y);
            frmotor.setPower(-1 * gamepad1.right_stick_y);
        }
        if (gamepad1.left_stick_x < 0) {
            blmotor.setPower(-1 * gamepad1.left_stick_x);
            flmotor.setPower(-1 * gamepad1.left_stick_x);
            brmotor.setPower(1);
            frmotor.setPower(1);
        }
        if (gamepad1.left_stick_x > 0) {
            blmotor.setPower(-1);
            flmotor.setPower(-1);
            brmotor.setPower(-1 * gamepad1.left_stick_x);
            frmotor.setPower(-1 * gamepad1.left_stick_x);
        }
        */
    }
}
