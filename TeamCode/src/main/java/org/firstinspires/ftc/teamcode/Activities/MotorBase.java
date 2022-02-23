package org.firstinspires.ftc.teamcode.Activities;

import static java.lang.Math.abs;

import android.icu.text.Transliterator;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Sensors.RobotPosition;


public class MotorBase {

    private RobotPosition position = new RobotPosition();
    private LinearOpMode linearOpMode = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private static final double d = 9.8;
    private static final double encoderTicksPerRevolution = 1440 / 3.0;
    private static final double encoderTicksToCm = (Math.PI * d) / encoderTicksPerRevolution;
    private double correction = 1.0;

    /*public double getForwardDistance() {
        return (leftDrive.getCurrentPosition() + leftDrive1.getCurrentPosition() + rightDrive.getCurrentPosition()
                + rightDrive1.getCurrentPosition()) * 0.25 * encoderTicksToCm;
    }*/

    /*public double getSideDistance() {
        return (leftDrive.getCurrentPosition() - leftDrive1.getCurrentPosition() - rightDrive.getCurrentPosition()
                + rightDrive1.getCurrentPosition()) * 0.25 * encoderTicksToCm;
    }*/

    public void init(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        leftDrive = linearOpMode.hardwareMap.get(DcMotor.class, "L1");
        rightDrive = linearOpMode.hardwareMap.get(DcMotor.class, "R1");
        leftDrive1 = linearOpMode.hardwareMap.get(DcMotor.class, "L2");
        rightDrive1 = linearOpMode.hardwareMap.get(DcMotor.class, "R2");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
    }

    public void move(double drive, double side, double turn) {
        drive = drive * correction;
        double leftPower1;
        double rightPower1;
        double leftPower2;
        double rightPower2;

        leftPower1 = drive + turn + side;
        rightPower1 = drive - turn - side;
        leftPower2 = drive + turn - side;
        rightPower2 = drive - turn + side;

        setMotorPowers(leftPower1, rightPower1, leftPower2, rightPower2);
    }

    public void setMotorPowers(double powerFL, double powerFR, double powerRL, double powerRR){
        leftDrive1.setPower(powerRL);
        leftDrive.setPower(powerFL);
        rightDrive1.setPower(powerRR);
        rightDrive.setPower(powerFR);
    }

    public void goToPosition(double x, double y) {
        //resetEncoders();
        double kP = 0.1;
        double errorX = x - position.position_x();
        double errorY = y - position.position_y();

        while ((abs(errorX) > 5 || abs(errorY) > 5) && linearOpMode.opModeIsActive()) {
            errorX = x - position.position_x();
            errorY = y - position.position_y();
            move(errorY * kP, errorX * kP, 0);
        }
    }

    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
