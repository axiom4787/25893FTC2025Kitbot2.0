/* Copyright (c) 2021 FIRST. All rights reserved.
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

import androidx.lifecycle.LifecycleRegistry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.IncludedFirmwareFileInfo;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Run intake & shooter", group="Linear OpMode")
public class BasicShooterOpmode extends LinearOpMode {
    Config config = new Config();

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor shooterLeft, shooterRight, floorIntake;
    private CRServo indexerLeft, indexerRight;
    private Servo selector;

    private boolean shooterOn = true;

    private boolean selectorRight = true;

    double shooterStartTime = 9e99;

    private static class ShooterConstants
    {
        static float shooterOffPower = 0.0f;
        static float shooterShootPower = 0.6f;
        static float intakeOutPower = -0.5f;
        static float intakeInPower = 0.5f;
        static float indexerFeedPower = 1f;
        static float indexerEjectPower = -1f;
    }

    @Override
    public void runOpMode() {
        config.initShooter(hardwareMap);
        config.initIntake(hardwareMap);

        shooterLeft = config.shooterLeft;
        shooterRight = config.shooterRight;
        shooterLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooterRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        floorIntake = config.intakeFront;

        indexerLeft = config.intakeLeft;
        indexerRight = config.intakeRight;

        selector = config.selector;

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        /**
         * control scheme
         * shooter and intake on by default
         * y - run indexer (feeds into shooter)
         * left bumper - set selector to left shooter
         * right bumper - set selector to right shooter
         * a - reset gyro
         * b - reverse intake and indexer to eject balls]
         * up arrow - toggle shooter
         */

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.dpadUpWasPressed())
                shooterOn = !shooterOn;

            shooterLeft.setPower(shooterOn ? ShooterConstants.shooterShootPower : ShooterConstants.shooterOffPower);
            shooterRight.setPower(shooterOn ? ShooterConstants.shooterShootPower : ShooterConstants.shooterOffPower);

            indexerLeft.setPower(gamepad1.y ? ShooterConstants.indexerFeedPower : (gamepad1.b ? ShooterConstants.indexerEjectPower : 0));
            indexerRight.setPower(gamepad1.y ? ShooterConstants.indexerFeedPower : (gamepad1.b ? ShooterConstants.indexerEjectPower : 0));

            floorIntake.setPower(gamepad1.b ? ShooterConstants.intakeOutPower : ShooterConstants.intakeInPower);

            if (gamepad1.leftBumperWasPressed())
                selectorRight = false;
            else if (gamepad1.rightBumperWasPressed())
                selectorRight = true;
            selector.setPosition(selectorRight ? 0.6 : 0.3);

            telemetry.addData("Status", "Run Time: %s", runtime.toString());
            telemetry.addData("Intake left/Right", "%4.2f, %4.2f", indexerLeft.getPower(), indexerRight.getPower());
            telemetry.addData("Shooter", "%4.2f", shooterLeft.getPower());
            telemetry.addData("Selector position", selector.getPosition());
            telemetry.update();
        }
    }
}