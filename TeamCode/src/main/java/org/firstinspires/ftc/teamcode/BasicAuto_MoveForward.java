package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Basic autonomous move forward", group="Basic Auto")
public class BasicAuto_MoveForward extends LinearOpMode {
    Config config = new Config();

    DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        config.initDrive(hardwareMap);
        frontLeftDrive = config.frontLeftDrive;
        backLeftDrive = config.backLeftDrive;
        frontRightDrive = config.frontRightDrive;
        backRightDrive = config.backRightDrive;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Running");
        telemetry.update();

        move(0.5);

        sleep(500);

        stopRobot();

        telemetry.addData("Status", "Done/stopped");
        telemetry.update();
    }
    void sleep(int ms) {
        double startTime = runtime.milliseconds();
        while(opModeIsActive() && (runtime.milliseconds() - startTime) < ms);
        // Wait for `ms` milliseconds
    }

    void move(double speed) {
        if(!opModeIsActive()) {
            stopRobot();
            return;
        } // Stop the robot if it shouldn't me moving

        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(speed);
        backLeftDrive.setPower(speed);
        backRightDrive.setPower(speed);
    }

    void stopRobot() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
