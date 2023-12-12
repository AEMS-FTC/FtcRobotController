package org.firstinspires.ftc.teamcode.aems.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

//!THIS IS A MECANUM SAMPLE, ENSURE TO USE 'UNIFIED SAMPLE' CONFIG
public class MecanumSample extends OpMode {

    //* MOTORS
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;


    //* IMU
    private IMU gyroscope = null;

    //* OTHER VARIABLES
    private double rateLimit = 0.8;

    private boolean fieldCentric = true;
    
    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");

    }

    @Override
    public void loop() {
        
    }
}
