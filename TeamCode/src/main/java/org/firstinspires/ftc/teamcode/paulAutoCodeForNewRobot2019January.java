
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="paulAutoCodeForNewRobot2019January", group="Linear Opmode")
public class paulAutoCodeForNewRobot2019January extends LinearOpMode{

    //PUT VARIABLES INSIDE CLASSES
    private ElapsedTime timeRunning = new ElapsedTime();

    static final double slowSpeed = 0.5, fastSpeed = 1;

    static final int     COUNTS_PER_MOTOR_REV    = 1680 ;    // eg: TETRIX Motor Encoder
    static final int     DRIVE_GEAR_REDUCTION    = 1 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor leftMotorFront = null;
    private DcMotor rightMotorFront = null;
    private DcMotor rightMotorRear = null;
    private DcMotor leftMotorRear = null;

    //private ColorSensor colorSensorBob;

    //private Servo arm1;

    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        leftMotorFront  = hardwareMap.dcMotor.get("leftMotorFront");
        rightMotorFront = hardwareMap.dcMotor.get("rightMotorFront");
        rightMotorRear = hardwareMap.dcMotor.get("rightMotorRear");
        leftMotorRear = hardwareMap.dcMotor.get("leftMotorRear");


        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        leftMotorFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftMotorRear.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightMotorFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightMotorRear.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors


        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //jewelArm = hardwareMap.servo.get("jewelArm");

        //colorSensorBob = hardwareMap.colorSensor.get("colorSensor");
        //touchWall = hardwareMap.touchSensor.get("touchWall");

        timeRunning.reset();
        timeRunning.startTime();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        timeRunning.reset();


        moveStraightSlowly(10000);
    moveStraightQuickly(5000);



    }


    public void moveStraightSlowly (int sleepTime){
        leftMotorFront.setPower(slowSpeed);
        leftMotorRear.setPower(slowSpeed);
        rightMotorFront.setPower(slowSpeed);
        rightMotorRear.setPower(slowSpeed);
        sleep(sleepTime);
    }

    public void moveStraightQuickly (int sleepTime){
        leftMotorFront.setPower(fastSpeed);
        leftMotorRear.setPower(fastSpeed);
        rightMotorFront.setPower(fastSpeed);
        rightMotorRear.setPower(fastSpeed);
        sleep(sleepTime);
    }

    public void stopMotors (int sleepTime){
        leftMotorFront.setPower(0);
        leftMotorRear.setPower(0);
        rightMotorFront.setPower(0);
        rightMotorRear.setPower(0);
        sleep(sleepTime);
    }

}
