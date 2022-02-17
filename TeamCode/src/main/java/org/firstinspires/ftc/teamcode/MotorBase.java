package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import static java.lang.Math.abs;

public class MotorBase {
    //public DcMotor Elevator = null;
    private LinearOpMode linearOpMode = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private static final double d = 9.8;
    private static final double encoderTicksPerRevolution = 1440 / 3.0;
    private static final double encoderTicksToCm = (Math.PI * d) / encoderTicksPerRevolution;

    public double getForwardDistance() {
        return (leftDrive.getCurrentPosition() + leftDrive1.getCurrentPosition() + rightDrive.getCurrentPosition()
                + rightDrive1.getCurrentPosition()) * 0.25 * encoderTicksToCm;
    }

    public double getSideDistance() {
        return (leftDrive.getCurrentPosition() - leftDrive1.getCurrentPosition() - rightDrive.getCurrentPosition()
                + rightDrive1.getCurrentPosition()) * 0.25 * encoderTicksToCm;
    }

    public void init(LinearOpMode linearOpMode) {
        this.linearOpMode = linearOpMode;
        leftDrive = linearOpMode.hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = linearOpMode.hardwareMap.get(DcMotor.class, "right_drive");
        leftDrive1 = linearOpMode.hardwareMap.get(DcMotor.class, "left_drive1");
        rightDrive1 = linearOpMode.hardwareMap.get(DcMotor.class, "right_drive1");
        //Elevator = linearOpMode.hardwareMap.get(DcMotor.class,"Elevator");

        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        //Elevator.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
    }

    public void move(double drive, double side, double turn) {
        double leftPower1;
        double rightPower1;
        double leftPower2;
        double rightPower2;

        leftPower1 = drive + turn + side;
        rightPower1 = drive - turn - side;
        leftPower2 = drive + turn - side;
        rightPower2 = drive - turn + side;

        leftDrive1.setPower(leftPower2);
        leftDrive.setPower(leftPower1);
        rightDrive1.setPower(rightPower2);
        rightDrive.setPower(rightPower1);
    }

    /*public void elevator(double direction){
        Elevator.setPower(0);
        if(direction == 1){ Elevator.setPower(1);}
        else if(direction == -1){ Elevator.setPower(-1);}
    }*/

    public void goToPosition(double x, double y) {
        resetEncoders();
        double kP = 0.1;
        double errorX = x - getSideDistance();
        double errorY = y - getForwardDistance();

        while ((abs(errorX) > 5 || abs(errorY) > 5) && linearOpMode.opModeIsActive()) {
            errorX = x - getSideDistance();
            errorY = y - getForwardDistance();
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
