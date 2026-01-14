/* Copyright (c) 2025 FIRST. All rights reserved.
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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/*
 * Opmomde with "all" features
 * - Field relative and robot relative motion
 * - Intake and shooter
 */
@TeleOp(name = "Robot: Everything", group = "Robot")
public class RobotEverything extends OpMode {
    // This declares the four motors needed
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    Config config = new Config();

    // This declares the IMU needed to get the current direction the robot is facing
    IMU imu;

    private DcMotor shooterLeft, shooterRight, floorIntake;
    private CRServo indexerLeft, indexerRight;
    private Servo selector;

    private boolean shooterOn = true;

    private boolean selectorRight = true;

    private boolean fieldRelative = true;

    double shooterStartTime = 9e99;

    double elapsed = 0;

    private static class ShooterConstants {
        static float shooterOffPower = 0.0f;
        static float shooterShootPower = 0.65f;
        static float intakeOutPower = -1f;
        static float intakeInPower = 1f;
        static float indexerFeedPower = 1f;
        static float indexerEjectPower = -1f;
    }

    @Override
    public void init() {
        config.initDrive(hardwareMap);
        config.initShooter(hardwareMap);
        config.initIntake(hardwareMap);
        frontLeftDrive = config.frontLeftDrive;
        frontRightDrive = config.frontRightDrive;
        backLeftDrive = config.backLeftDrive;
        backRightDrive = config.backRightDrive;

        shooterLeft = config.shooterLeft;
        shooterRight = config.shooterRight;
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        floorIntake = config.intakeFront;

        indexerLeft = config.intakeLeft;
        indexerRight = config.intakeRight;

        selector = config.selector;

        // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
        // wires, you should remove these
//        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");
        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

    }

    @Override
    public void loop() {
        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.a) {
            imu.resetYaw();
        }

        if (gamepad1.dpadLeftWasPressed())
            fieldRelative = !fieldRelative;

        // If you press the left bumper, you get a drive from the point of view of the robot
        // (much like driving an RC vehicle)
        if (!fieldRelative) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        if (gamepad1.yWasPressed() && !shooterOn) {
            shooterOn = true;
            elapsed = 0;
        }

        // todo add safety switch to turn off shooter mode

        if (shooterOn) {
            final double firstBallDelay = 3;
            final double indexTime = 1; // time to feed indexer for each ball
            final double cooldownTime = 1; // time to let shooter rev back up after
            final double secondBallDelay = firstBallDelay + indexTime + cooldownTime;
            final double thirdBallDelay = secondBallDelay + indexTime + cooldownTime;

            shooterLeft.setPower(ShooterConstants.shooterShootPower);
            shooterRight.setPower(ShooterConstants.shooterShootPower);
            if (elapsed > firstBallDelay && elapsed < firstBallDelay + indexTime) {
                indexerRight.setPower(ShooterConstants.indexerFeedPower);
                indexerLeft.setPower(0);
                selector.setPosition(0.6);
            } else if (elapsed > secondBallDelay && elapsed < secondBallDelay + indexTime) {
                indexerLeft.setPower(ShooterConstants.indexerFeedPower);
                indexerRight.setPower(0);
                selector.setPosition(0.35);
            } else if (elapsed > thirdBallDelay && elapsed < thirdBallDelay + indexTime) {
                indexerLeft.setPower(ShooterConstants.indexerFeedPower);
                indexerRight.setPower(0);
                selector.setPosition(0.35);
            } else if (elapsed > thirdBallDelay + indexTime) {
                shooterOn = false;
            } else {
                indexerLeft.setPower(0);
                indexerRight.setPower(0);
            }
        } else {
            // intake - b
            // outtake - x
            floorIntake.setPower(gamepad1.x ? ShooterConstants.intakeOutPower : (gamepad1.b ? ShooterConstants.intakeInPower : 0));

            // indexer can only outtake when shooter is off
            double indexerPower = gamepad1.x ? ShooterConstants.indexerEjectPower : 0;
            indexerLeft.setPower(indexerPower);
            indexerRight.setPower(indexerPower);

            // shooter should never run in this mode
            shooterLeft.setPower(ShooterConstants.shooterOffPower);
            shooterRight.setPower(ShooterConstants.shooterOffPower);

            // set the selector for ball intake
            if (gamepad1.leftBumperWasPressed())
                selectorRight = false;
            else if (gamepad1.rightBumperWasPressed())
                selectorRight = true;

            selector.setPosition(selectorRight ? 0.6 : 0.35);
        }



        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        telemetry.addData("In field relative", fieldRelative);
        telemetry.addData("Shooter left power", shooterLeft.getPower());
        telemetry.addData("Shooter right power", shooterRight.getPower());
        telemetry.update();
    }


    // assumes two balls are in left, one in right, with selector covering right
    // shoots the right, then both left balls
    // elapsed: time since shooter started running
    private void runShooter(double elapsed) {

    }

    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        double max;

        // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
        double axial = forward;  // Note: pushing stick forward gives negative value
        double lateral = right;
        double yaw = rotate;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
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
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

        // Send calculated power to wheels
        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);


        telemetry.addData("Front left/Right", "%4.2f, %4.2f", frontLeftPower, frontRightPower);
        telemetry.addData("Back  left/Right", "%4.2f, %4.2f", backLeftPower, backRightPower);
        telemetry.addData("gamepad", gamepad1.toString());
//        telemetry.update();

//        // This calculates the power needed for each wheel based on the amount of forward,
//        // strafe right, and rotate
//        double frontLeftPower = forward + right + rotate;
//        double frontRightPower = forward - right - rotate;
//        double backRightPower = forward + right - rotate;
//        double backLeftPower = forward - right + rotate;
//
//        double maxPower = 1.0;
//        double maxSpeed = 1.0;  // make this slower for outreaches
//
//        // This is needed to make sure we don't pass > 1.0 to any wheel
//        // It allows us to keep all of the motors in proportion to what they should
//        // be and not get clipped
//        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
//        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backRightPower));
//        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
//
//        // We multiply by maxSpeed so that it can be set lower for outreaches
//        // When a young child is driving the robot, we may not want to allow full
//        // speed.
//        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
//        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
//        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
//        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
