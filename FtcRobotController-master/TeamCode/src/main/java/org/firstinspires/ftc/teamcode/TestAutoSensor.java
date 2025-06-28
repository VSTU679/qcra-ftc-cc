package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.PwmControl;

@Autonomous(name = "TTTestAutoSensor", group = "Robot")



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



public class TestAutoSensor extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor frontLeft, frontRight, backRight, backLeft = null;
    public DcMotor armMotor = null;
    public ServoImplEx clawServo = null;
    public DcMotor forearmMotor = null;
    public CRServo intakeServo = null;
    private ElapsedTime runtime = new ElapsedTime();
    private ColorSensor colorSensor;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_MOTOR_REV = 28;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 19.2;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 4.09449;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.2;
    static final double TURN_SPEED = 0.5;
    static final double ARMSPEED = 0.6;
    static final double FOREARMSPEED = 0.75;

    @Override
    public void runOpMode() {
        // Initialize the drive system variables.
        frontRight= hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft" );
        backLeft = hardwareMap.get(DcMotor.class, "backLeft" );
        backRight = hardwareMap.get(DcMotor.class, "backRight" );


        armMotor = hardwareMap.get(DcMotor.class, "arm");
        clawServo = hardwareMap.get(ServoImplEx.class, "claw");
        clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        clawServo.scaleRange(0, 1);
        forearmMotor = hardwareMap.get(DcMotor.class, "forearm");
        intakeServo = hardwareMap.get(CRServo.class, "intake1");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);


        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        forearmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Send telemetry message to indicate successful Encoder reset

        // Wait for the game to start (driver presses START)
        waitForStart  ();
        if (opModeIsActive()) {

            closeclaw();
            encoderDriveForArm(ARMSPEED, -2815, 2);

            encoderDrive(0.65, 18.5,-18.5, 18.5, -18.5, 2);
            encoderDrive(0.65, -21,-21, -21,-21,2);
            encoderDrive(0.65, 4.5,-4.5, 4.5, -4.5, 2);
            //encoderDrive(0.6, -30,-30, -30,-30,2);
            encoderDriveForArm(ARMSPEED, -1850,2);
            openclaw();
            sleep(1000);
            encoderDrive(0.6,-8,8,-8, 8,2);
            closeclaw();
            encoderDriveForForeArm(FOREARMSPEED, 6, 2);
            encoderDriveForArm(ARMSPEED, 50,2);
            encoderDrive(0.6,31.5,31.5,31.5, 31.5,2);
            encoderDrive(0.6,31.5,-31.5, 31.5, -31.5, 2);
            encoderDrive(0.6, 11.5,11.5,11.5,11.5, 2);
            sleep(1000);
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();
            colorSensor.enableLed(true);
            telemetry.addLine("red :" + red + "blue: " + blue + " green: " + green);
            if (red > 200 && green > 200 && blue > 200) {
                telemetry.addLine("White Detected avoiding");
                //encoderDrive(0.6, 19, -19, 19, -19, 2);
                encoderDriveForForeArm(FOREARMSPEED, 6, 2);
                encoderDrive(0.6,10,-10,10,-10,2);
                encoderDrive(0.6, 8.5, 8.5, 8.5, 8.5, 2);
                encoderDrive(0.6, -58.5,58.5, -58.5, 58.5, 2);
                encoderDriveForForeArm(FOREARMSPEED, 6, 2);
                encoderDrive(0.6,50, -50 ,50, -50, 2);
                encoderDrive (0.6, -21.5, -21.5, 21.5, 21.5, 2);
                encoderDrive(0.6,-13,13,-13,13,2);
                encoderDriveForForeArm(FOREARMSPEED, 6, 2);
                encoderDrive(0.75, -43,-43,-43,-43,2);
                encoderDrive(0.6,100,-100,100,-100,2);
                encoderDrive(0.6,100,-100,100,-100,2);

            } else{
                telemetry.addLine("WHITE NOT DETECTED");
                encoderDrive(0.6, -47,47,-47,47,2);
                encoderDrive(0.6, 47, -47, 47, -47,2);
                encoderDrive(0.6,11,11,11,11,2);
                encoderDriveForForeArm(FOREARMSPEED, 6,2);
                sleep(500);
                if (red > 200 && green > 200 && blue > 200) {
                    telemetry.addLine("White Detected avoiding");
                    sleep(500);
                    encoderDrive(0.6,-17,-17,-17,-17,2);
                    encoderDrive(0.6,-20,20,-20,20,2);
                    encoderDrive(0.6,-48,-48,-48,-48,2);

                } else {
                    encoderDrive(0.6,-45,45,-45,45,2);
                    encoderDrive(0.6,7,-7,7,-7,2);
                    encoderDrive(0.6,-55,-55,-55,-55,2);

                }


            }
            encoderDrive(0.6, 7.5,-7.5, 7.5, -7.5, 2);
            encoderDriveForArm(0.6, -121,2);
            openclaw();

            encoderDrive(0.6, -7.5,7.5,7.5,-7.5,2);
            runtime.reset();

            sleep(1000);  // pause to display final telemetry message.
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double speed,
                             double leftFrontInches, double rightFrontInches, double rightBackInches, double leftBackInches,
                             double timeoutS) {

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            int  newLeftFrontTarget = frontLeft.getCurrentPosition() + (int) (leftFrontInches * COUNTS_PER_INCH);
            int    newRightFrontTarget = frontRight.getCurrentPosition() + (int) (rightFrontInches * COUNTS_PER_INCH);
            int   newRightBackTarget = backRight.getCurrentPosition() + (int) (rightBackInches * COUNTS_PER_INCH);
            int   newLeftBackTarget = backLeft.getCurrentPosition() + (int) (leftBackInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newLeftFrontTarget);
            frontRight.setTargetPosition(newRightFrontTarget);
            backLeft.setTargetPosition(newLeftBackTarget);
            backRight.setTargetPosition(newRightBackTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (backLeft.isBusy() && backRight.isBusy() && frontRight.isBusy () && frontLeft.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newRightBackTarget, newLeftFrontTarget, newLeftBackTarget, newRightFrontTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        frontLeft.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
    // put new code right here not in the curly brckets MAINLY FOR ANISH SPECIFICALLY

    public void encoderDriveForArm(double speed,
                                   int targetPosition,
                                   double timeoutS) {
        int newArmTarget;
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // newArmTarget = armMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            armMotor.setTargetPosition(targetPosition);

            // Turn On RUN_TO_POSITION
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            armMotor.setPower(Math.abs(0.75));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (armMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d", targetPosition);
                telemetry.addData("Currently at", " at %7d",
                        armMotor.getCurrentPosition());
                telemetry.update();
            }
            if (armMotor.getCurrentPosition() > 2815) {
                armMotor.setPower(0);
                armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            // Stop all motion;
            armMotor.setPower(0);
            //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            // armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void encoderDriveForForeArm(double speed, int targetPosition, double timeoutS) {
        int newArmTarget;
        telemetry.addData("Currently at", " at %7d",
                forearmMotor.getCurrentPosition());
        //    forearmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            // newArmTarget = armMotor.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            telemetry.addData("Currently at", " at %7d",
                    forearmMotor.getCurrentPosition());
            forearmMotor.setTargetPosition(targetPosition);
            telemetry.addData("Currently at", " at %7d",
                    forearmMotor.getCurrentPosition());
            // Turn On RUN_TO_POSITION
            forearmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            forearmMotor.setPower(Math.abs(0.75));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (forearmMotor.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d", targetPosition);
                telemetry.addData("Currently at", " at %7d",
                        forearmMotor.getCurrentPosition());
                telemetry.update();
            }
            if (forearmMotor.getCurrentPosition() > 150) {
                forearmMotor.setPower(0);
                forearmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            // Stop all motion;
            forearmMotor.setPower(0);
            //armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            // Turn off RUN_TO_POSITION
            // armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void openclaw() {
        clawServo.setPosition(0);
    }

    public void closeclaw() {
        clawServo.setPosition(1);
    }
}



