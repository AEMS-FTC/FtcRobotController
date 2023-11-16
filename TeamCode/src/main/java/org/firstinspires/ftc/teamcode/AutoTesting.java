package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="AutoTesting")
@Disabled
public class AutoTesting extends OpMode {
    DcMotor blmotor;
    DcMotor flmotor;
    DcMotor brmotor;
    DcMotor frmotor;

    @Override
    public void init() {

        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");

        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        /*
        // Send calculated power to wheels
        flmotor.setPower(leftFrontPower);
        frmotor.setPower(rightFrontPower);
        blmotor.setPower(leftBackPower);
        brmotor.setPower(rightBackPower);
        */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
        telemetry.update();

        //A2 starting location
        if (gamepad1.a) {
            forward(.58, 1801);
            left(.45,996);
            forward(.77, 2000);
        }

        // A4 starting location
        if (gamepad1.b) {
            /*
            right(.5, 800);
            left(0.5, 500);
            forward(.8, 700);
            reverse(.6, 400);
            */
        }

        // F2 starting location
        if (gamepad1.x) {
            /*
            right(.5, 800);
            left(0.5, 500);
            forward(.8, 700);
            reverse(.6, 400);
            */
        }

        // F4 starting location
        if (gamepad1.y) {
            /*
            right(.5, 800);
            left(0.5, 500);
            forward(.8, 700);
            reverse(.6, 400);
            */
        }
    }

    public void stop() {
        //Set the motor power
        flmotor.setPower(0);
        blmotor.setPower(0);
        frmotor.setPower(0);
        brmotor.setPower(0);
    }

    public void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    // Given:
    //      Percentage (in decimal form) of relative power
    //      Time in milliseconds
    public void forward(double power, long time) {
        // Set the motor power
        flmotor.setPower(power);
        blmotor.setPower(power);
        frmotor.setPower(power);
        brmotor.setPower(power);

        // Delay for a few seconds, then set all power back to 0
        sleep(time);
        stop();
    }

    // Given:
    //      Percentage (in decimal form) of relative power
    //      Time in milliseconds
    public void reverse(double power, long time) {
        // Set the motor power
        flmotor.setPower(-power);
        blmotor.setPower(-power);
        frmotor.setPower(-power);
        brmotor.setPower(-power);

        // Delay for a few seconds, then set all power back to 0
        sleep(time);
        stop();
    }

    // Given:
    //      Percentage (in decimal form) of relative power
    //      Time in milliseconds
    public void left(double power, long time) {
        // Set the motor power
        flmotor.setPower(-power);
        blmotor.setPower(-power);
        frmotor.setPower(power);
        brmotor.setPower(power);

        // Delay for a few seconds, then set all power back to 0
        sleep(time);
        stop();
    }

    // Given:
    //      Percentage (in decimal form) of relative power
    //      Time in milliseconds
    public void right(double power, long time) {
        // Set the motor power
        flmotor.setPower(power);
        blmotor.setPower(power);
        frmotor.setPower(-power);
        brmotor.setPower(-power);

        // Delay for a few seconds, then set all power back to 0
        sleep(time);
        stop();
    }

    //public   Servo  claw        =null
    //public final static double ARM_HOME = 0.0;  // Starting position for Servo claw
    // public final static double ARM_MIN_RANGE =0.0; // Smallest number value allowed for servo position
    //public final static double ARM_MAX_RANGE = 1.0;  //Largest number value allowed for servo position
}