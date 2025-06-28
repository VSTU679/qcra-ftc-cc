package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="MechanumAuto", group="Robot")
public class MechanumAuto extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor frontLeft = null;
    public DcMotor backLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backRight = null;
    private ColorSensor colorSensor;
    public DcMotor armMotor = null;
    public ServoImplEx clawServo = null;
    private ColorSensor colorSensor;
    //    public DcMotor forearmMotor = null;
//    public CRServo intakeServo = null;
    private ElapsedTime runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV

    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 30.24;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 3.543;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.8;
    static final double TURN_SPEED = 0.7;
//    static final double ARMSPEED    = 0.75;
//    static final double FOREARMSPEED = 0.75;

    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        frontLeft = hardwareMap.get(DcMotor.class, "leftfrontwheels");
        backLeft = hardwareMap.get(DcMotor.class, "leftbackwheels");
        frontRight = hardwareMap.get(DcMotor.class, "rightfrontwheels");
        backRight = hardwareMap.get(DcMotor.class, "rightbackwheels");
        armMotor = hardwareMap.get(DcMotor.class, "arm");
        clawServo = hardwareMap.get(ServoImplEx.class, "claw");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawServo.scaleRange(0.9, 1);
//        forearmMotor = hardwareMap.get(DcMotor.class, "forearm");
//        intakeServo = hardwareMap.get(CRServo.class, "intake1");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        forearmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        forearmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at", "%7d :%7d",
                frontLeft.getCurrentPosition(),
                backLeft.getCurrentPosition());
//        armMotor.getCurrentPosition();
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        if (opModeIsActive()) {

         closeclaw();
         encoderDriveForArm(ARMSPEED, 2815, 2); //moving the arm up to 2815
// Step through each leg of the path,
            // Note: Reverse movement is obtained by setting a negative distance (not speed)
            encoderDrive(DRIVE_SPEED, 12.3, 12.3, 12.3, 12.3, 2);  // move forward for 12 inches
//            encoderDriveForArm(ARMSPEED, 2000, 2); // moving the arm down to 1682
//            openclaw(); // open the claw to move from the specimen
            encoderDrive(DRIVE_SPEED, -6, -6, -6, -6, 1); // move backward by 6 inches
            encoderDrive(DRIVE_SPEED, 19, -19, 19, -19, 1);
            encoderDrive(DRIVE_SPEED, 18.5, 18.5, 18.5, 18.5, 2); // move forward by 10.5 inches
            encoderDrive(DRIVE_SPEED, 4, -4, 4, -4, 1);
            encoderDrive(DRIVE_SPEED, -32, -32, -32, -32, 2);
            encoderDrive(DRIVE_SPEED, 32, 32, 32, 32, 2);
            encoderDrive(DRIVE_SPEED, 4, -4, 4, -4, 1);
            encoderDrive(DRIVE_SPEED, -32, -32, -32, -32, 2);
            encoderDrive(DRIVE_SPEED, 5, 5, 5, 5, 1);
            encoderDrive(DRIVE_SPEED, -5, 5, -5, 5, 1);
            encoderDrive(TURN_SPEED, 15, 15, -15, -15, 1);
//            openclaw();
//            encoderDriveForArm(ARMSPEED, 1200, 1);
//            closeclaw();
//            encoderDriveForArm(ARMSPEED, 2000, 1);
            encoderDrive(TURN_SPEED, 15, 15, -15, -15, 1);
            encoderDrive(DRIVE_SPEED, -19, 19, -19, 19, 2);
//            encoderDriveForArm(ARMSPEED, 2815,1);
            encoderDrive(DRIVE_SPEED, 9, 9, 9, 9, 1);
//            encoderDriveForArm(ARMSPEED, 2000, 1);
            //openclaw();
            encoderDrive(DRIVE_SPEED, -9, -9, -9, -9, 1);
            encoderDrive(DRIVE_SPEED, 19, -19, 19, -19, 2);
            runtime.reset();
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
    }
    // Step through each leg of the path,
    // Note: Reverse movement is obtained by setting a negative distance (not speed)

    //3) Driver stops the OpMode running.

    public void encoderDrive(double speed, double frontLeftInches, double backLeftInches,
                             double frontRightInches, double backRightInches, double timeoutS)
    {
        int newFLtarget;
        int newBLtarget;
        int newFRtarget;
        int newBRtarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFLtarget = frontLeft.getCurrentPosition() + (int) (frontLeftInches * COUNTS_PER_INCH);
            newBLtarget = backLeft.getCurrentPosition() + (int) (backLeftInches * COUNTS_PER_INCH);
            newFRtarget = frontRight.getCurrentPosition() + (int) (frontRightInches * COUNTS_PER_INCH);
            newBRtarget = backRight.getCurrentPosition() + (int) (backRightInches * COUNTS_PER_INCH);
            frontLeft.setTargetPosition(newFLtarget);
            backLeft.setTargetPosition(newBLtarget);
            frontRight.setTargetPosition(newFRtarget);
            backRight.setTargetPosition(newBRtarget);
            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            // sleep(250);   // optional pause after each move.
        }
    }
    public void openclaw()
    {
        clawServo.setPosition(0);
    }
    public void closeclaw()
    {
        clawServo.setPosition(1);
    }

}
