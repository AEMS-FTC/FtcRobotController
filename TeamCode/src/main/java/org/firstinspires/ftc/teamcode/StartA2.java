/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous (name="Start A2", group="EXPERIMENTAL")
@Disabled
public class StartA2 extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    DcMotor blmotor;
    DcMotor flmotor;
    DcMotor brmotor;
    DcMotor frmotor;

    // According to GoBilda spec sheet
    double ppr = 537.7;
    // Wheels are 96mm in diameter
    double revDist = 96 * Math.PI;

    boolean done;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialization
        blmotor = hardwareMap.get(DcMotor.class,"blmotor");
        flmotor = hardwareMap.get(DcMotor.class,"flmotor");
        brmotor = hardwareMap.get(DcMotor.class,"brmotor");
        frmotor = hardwareMap.get(DcMotor.class,"frmotor");

        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // Starting at position A2
        // Facing ... against wall and A2/A1 tile line
        //moveForward(0.5, 126);
        turnLeft(0.5, 90);
        //moveForward(0.5, 247);
    }

    // Distance in centimeters
    public void moveForward(double power, int distance) {
        // The reason for the "if (opModeIsActive())" is to handle the condition where
        // the driver presses init, and then decides to abort the run, and presses stop.
        if (opModeIsActive()) {
            // GoBilda wheels are 96mm in diameter
            // GoBilda Yellow Jacket motor = 537.7 PPR [ppr]
            // 1 revolution is 96*pi mm = 301.59mm [revDist]
            // Multiply by 10 to get in terms of centimeters
            int cmDistance = (int) (distance * ppr / revDist * 10);

            stopAndResetEncoder();

            blmotor.setTargetPosition(cmDistance);
            flmotor.setTargetPosition(cmDistance);
            brmotor.setTargetPosition(cmDistance);
            frmotor.setTargetPosition(cmDistance);

            runToPosition();
            // Move forward
            setPower(power);

            while(blmotor.isBusy() && flmotor.isBusy() && brmotor.isBusy() && frmotor.isBusy()) {
                // Does nothing - waits - until all motors are finished
            }

            stopMotors();
            // Turn off RUN_TO_POSITION
            runUsingEncoder();
        }
    }

    // Distance in centimeters
    public void moveBackward(double power, int distance) {
        // The reason for the "if (opModeIsActive())" is to handle the condition where
        // the driver presses init, and then decides to abort the run, and presses stop.
        if (opModeIsActive()) {
            // GoBilda wheels are 96mm in diameter
            // GoBilda Yellow Jacket motor = 537.7 PPR [ppr]
            // 1 revolution is 96*pi mm = 301.59mm [revDist]
            // Multiply by 10 to get in terms of centimeters
            int cmDistance = (int) (distance * ppr / revDist * 10);

            stopAndResetEncoder();

            blmotor.setTargetPosition(-cmDistance);
            flmotor.setTargetPosition(-cmDistance);
            brmotor.setTargetPosition(-cmDistance);
            frmotor.setTargetPosition(-cmDistance);

            runToPosition();
            // Move forward
            setPower(power);

            while(blmotor.isBusy() && flmotor.isBusy() && brmotor.isBusy() && frmotor.isBusy()) {
                // Does nothing - waits - until all motors are finished
            }

            stopMotors();
            // Turn off RUN_TO_POSITION
            runUsingEncoder();
        }
    }

    public void turnRight(double power, int degrees) {
        // The reason for the "if (opModeIsActive())" is to handle the condition where
        // the driver presses init, and then decides to abort the run, and presses stop.
        if (opModeIsActive()) {
            // Using formula: motor rotation in revolutions * gear ratio * (wheel diameter * pi)
            int calcDegrees = (int) ((degrees / 360) * 19.2 * 96 * Math.PI);

            stopAndResetEncoder();

            blmotor.setTargetPosition(calcDegrees);
            flmotor.setTargetPosition(calcDegrees);
            brmotor.setTargetPosition(-calcDegrees);
            frmotor.setTargetPosition(-calcDegrees);

            runToPosition();
            // Move forward
            setPower(power);

            while(blmotor.isBusy() && flmotor.isBusy() && brmotor.isBusy() && frmotor.isBusy()) {
                // Does nothing - waits - until all motors are finished
            }

            stopMotors();
            // Turn off RUN_TO_POSITION
            runUsingEncoder();
        }
    }

    public void turnLeft(double power, int degrees) {
        // The reason for the "if (opModeIsActive())" is to handle the condition where
        // the driver presses init, and then decides to abort the run, and presses stop.
        if (opModeIsActive()) {
            // Using formula: motor rotation in revolutions * gear ratio * (wheel diameter * pi)
            int calcDegrees = (int) ((degrees / 360) * 19.2 * 96 * Math.PI);

            stopAndResetEncoder();

            blmotor.setTargetPosition(-calcDegrees);
            flmotor.setTargetPosition(-calcDegrees);
            brmotor.setTargetPosition(calcDegrees);
            frmotor.setTargetPosition(calcDegrees);

            runToPosition();
            // Move forward
            setPower(power);

            while(blmotor.isBusy() && flmotor.isBusy() && brmotor.isBusy() && frmotor.isBusy()) {
                // Does nothing - waits - until all motors are finished
            }

            stopMotors();
            // Turn off RUN_TO_POSITION
            runUsingEncoder();
        }
    }

    public void stopAndResetEncoder() {
        blmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runToPosition() {
        blmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        brmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setPower(double power) {
        blmotor.setPower(power);
        flmotor.setPower(power);
        brmotor.setPower(power);
        frmotor.setPower(power);
    }

    public void stopMotors() {
        // Set the motor power
        flmotor.setPower(0);
        blmotor.setPower(0);
        frmotor.setPower(0);
        brmotor.setPower(0);
    }

    public void runUsingEncoder() {
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
