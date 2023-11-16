package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous (name = "Universal Movement")
public class ExperimentalUniversalMovement extends LinearOpMode {

    //! HARDWARE DECLARATIONS BELOW
    //* Drive Motors
    private DcMotor left_front = null;
    private DcMotor left_back = null;
    private DcMotor right_front = null;
    private DcMotor right_back = null;

    //* Odometers
    private DcMotor left_odometer = null;
    private DcMotor right_odometer = null;
    private DcMotor strafe_odometer = null;

    @Override
    public void runOpMode() {

        //! HARDWARE MAP AND CONFIGURATION CODE BELOW

        //Mapping and Configuring the Left Front Motor
        left_front = hardwareMap.get(DcMotor.class, "left_front");
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);

        //Mapping and Configuring the Left Back Motor
        left_back = hardwareMap.get(DcMotor.class, "left_back");
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        //Mapping and Configuring the Right Front Motor
        right_front = hardwareMap.get(DcMotor.class, "right_front");
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setDirection(DcMotorSimple.Direction.FORWARD);

        //Mapping and Configuring the Right Back Motor
        right_back = hardwareMap.get(DcMotor.class, "right_back");
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);

        //Mapping the Strafe Odometer because it should not be bound to an existing motor
        strafe_odometer = hardwareMap.get(DcMotor.class, "strafe_odometer");

        //Binding side Odometers because the inherit side motion
        left_odometer = hardwareMap.get(DcMotor.class, "left_odometer");
        right_odometer = hardwareMap.get(DcMotor.class, "right_odometer");

        //! ROUTINE CODE BELOW

        waitForStart();
        resetOdometers();
        universalMovement(10 , 180);
    }

    //! MOVEMENT FUNCTIONS BELOW
    public void universalMovement(double distance_cm, double angle_degrees) {
        stopMotors();
        resetOdometers();

        //Storing Radian angle because JAVA uses Radians and not Degrees in trigonometric functions
        double angle_radians = Math.toRadians(angle_degrees);

        //Storing the distance in ticks for the odometer calculation to correctly interpret the distance
        double distance_ticks = distance_cm * 2000 / Constants.HardwareConstants.odometerWheelCircumference;

        //Storing Cosine and Sine values from the angle (reduce lag and risk)
        double angle_cos = Math.cos(angle_radians);
        double angle_sin = Math.sin(angle_radians);

        //* Command Loop
        while(opModeIsActive()) {

            //Creating the Strafe power, multiplies by the Cosine value so that movement remains linear
            double strafe_power = GlobalFunctions.proportional(
                    angle_cos * distance_ticks,
                    strafe_odometer.getCurrentPosition(),
                    Constants.AutonomousConstants.strafeP
            ) * angle_cos;

            //Creating the Lateral power, multiplies by the Sine value so that movement remains linear
            double lateral_power = GlobalFunctions.proportional(
                    angle_sin * distance_ticks,
                    left_odometer.getCurrentPosition(),
                    Constants.AutonomousConstants.lateralP
            ) * angle_sin;

            //Storing the power value for each wheel
            double left_front_power = lateral_power + strafe_power;
            double left_back_power = lateral_power - strafe_power;

            double right_front_power = lateral_power - strafe_power;
            double right_back_power = lateral_power + strafe_power;

            //Setting the motor power using each power value
            setMotorPower(
                    left_front_power,
                    left_back_power,
                    right_front_power,
                    right_back_power
            );

            //Checking for break condition to break the while loop
            if(
                    GlobalFunctions.check(left_front_power)
                    && GlobalFunctions.check(left_back_power)
                    && GlobalFunctions.check(right_front_power)
                    && GlobalFunctions.check(right_back_power)
            ) {
                break;
            }
        }
    }

    //! UTILITY FUNCTIONS BELOW
    public void stopMotors() {
        left_front.setPower(0);
        left_back.setPower(0);

        right_front.setPower(0);
        right_back.setPower(0);
    }

    public void resetOdometers() {
        strafe_odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left_odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_odometer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void setMotorPower(
            double left_front_power,
            double left_back_power,
            double right_front_power,
            double right_back_power
    ) {
        left_front.setPower(left_front_power);
        left_back.setPower(left_back_power);

        right_front.setPower(right_front_power);
        right_back.setPower(right_back_power);
    }
}
