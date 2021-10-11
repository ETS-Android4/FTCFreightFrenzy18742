package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorBase {
    private LinearOpMode linearOpMode = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private static final double d=9.8;
    private static final double encoderroteate=1440/3.0;
    private static final double encodertodistance=(Math.PI*d)/encoderroteate;
    public double getForwardDistants()
    {
        return (leftDrive.getCurrentPosition()+leftDrive1.getCurrentPosition()+rightDrive.getCurrentPosition()
                +rightDrive1.getCurrentPosition())*0.25*encodertodistance;
    }
    public void motor_init(LinearOpMode linearOpMode)
    {
        this.linearOpMode = linearOpMode;
        leftDrive = linearOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive =linearOpMode.hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive1 = linearOpMode.hardwareMap.get(DcMotor.class, "left_drive1");
        rightDrive1 = linearOpMode.hardwareMap.get(DcMotor.class, "right_drive1");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void move(double drive,double side,double turn)
    {
        double leftPower1;
        double rightPower1;
        double leftPower2;
        double rightPower2;
        leftPower1=drive+turn+side;
        rightPower1=drive-turn-side;
        leftPower2=drive+turn-side;
        rightPower2=drive-turn+side;
        leftDrive1.setPower(leftPower2);
        leftDrive.setPower(leftPower1);
        rightDrive1.setPower(rightPower2);
        rightDrive.setPower(rightPower1);
    }
}
