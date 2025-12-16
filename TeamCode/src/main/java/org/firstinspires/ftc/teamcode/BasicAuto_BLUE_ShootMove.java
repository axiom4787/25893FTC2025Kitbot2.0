package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto BLUE: shoot & move", group="Basic Auto")
public class BasicAuto_BLUE_ShootMove extends LinearOpMode {

    Config config = new Config();

    ElapsedTime runtime = new ElapsedTime();

    DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    DcMotor shooter;
    CRServo intakeLeft, intakeRight;

    @Override
    public void runOpMode() {
        config.initDrive(hardwareMap);
        config.initIntake(hardwareMap);
        frontLeftDrive = config.frontLeftDrive;
        backLeftDrive = config.backLeftDrive;
        frontRightDrive = config.frontRightDrive;
        backRightDrive = config.backRightDrive;
        shooter = config.shooterLeft;
        intakeLeft = config.intakeLeft;
        intakeRight = config.intakeRight;

        telemetry.addData("Status", "Initialized");
        telemetry.addLine("AUTO - Blue alliance");
        telemetry.update();

        waitForStart();
        runtime.reset();

        telemetry.addData("Status", "Running shooter");
        telemetry.update();

        shooter.setPower(0.5);
        sleep(1000);
        intakeLeft.setPower(0.15);
        intakeRight.setPower(0.15);
        sleep(3000);
        shooter.setPower(0.0);
        intakeLeft.setPower(0.0);
        intakeRight.setPower(0.0);

        telemetry.addData("Status", "Moving");
        telemetry.update();

        move(-0.5);
        sleep(600);
        strafe(0.5); // Positive is right, negative is left
        sleep(600);
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

    void strafe(double speed) {
        if(!opModeIsActive()) {
            stopRobot();
            return;
        } // Stop the robot if it shouldn't me moving

        frontLeftDrive.setPower(speed);
        frontRightDrive.setPower(-speed);
        backLeftDrive.setPower(-speed);
        backRightDrive.setPower(speed);
    }

    void stopRobot() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
    }
}
