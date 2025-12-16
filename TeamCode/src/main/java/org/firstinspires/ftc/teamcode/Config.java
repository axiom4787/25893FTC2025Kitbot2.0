package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Config {
    DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive;
    DcMotor shooterLeft, shooterRight, intakeFront;
    CRServo intakeLeft, intakeRight;
    Servo selector;

    public void init(HardwareMap hardwareMap) {
        initDrive(hardwareMap);
        initIntake(hardwareMap);
        initShooter(hardwareMap);
    }
    public void initDrive(HardwareMap hardwareMap) {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }

    public void initIntake(HardwareMap hardwareMap) {
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRight = hardwareMap.get(CRServo.class, "intakeRight");
        selector = hardwareMap.get(Servo.class, "selector");
        intakeFront = hardwareMap.get(DcMotor.class, "intakeFront");

        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.REVERSE);
        intakeFront.setDirection(DcMotor.Direction.REVERSE);
    }

    public void initShooter(HardwareMap hardwareMap) {
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");

        shooterLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooterRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}
