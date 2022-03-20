package org.firstinspires.ftc.teamcode.Activities;

import static com.qualcomm.hardware.lynx.LynxServoController.apiPositionFirst;

import static java.lang.Math.abs;
import static java.lang.Math.min;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryInternal;

import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Sensors.RobotPosition;


@Config
public class MotorBase {
    public static double GyroK = 0.01d;
    private static final double d = 9.8d;
    public static double kp = 0.4;
    public static double kpA = 0.4;
    private static final double encoderTicksPerRevolution = 480.0d;
    private static final double encoderTicksToCm = 0.06414085001079162d;
    Acceleration acceleration;
    private double correction = 1.0d;
    public BNO055IMU gyro;
    public DcMotor leftDrive = null;
    public DcMotor leftDrive1 = null;
    private LinearOpMode linearOpMode = null;
    Orientation orientation;
    private RobotPosition position = new RobotPosition();
    public DcMotor rightDrive = null;
    public DcMotor rightDrive1 = null;

    public double getForwardDistance() {
        return ((double) (this.leftDrive1.getCurrentPosition() + this.rightDrive.getCurrentPosition())) * 0.5d * encoderTicksToCm;
    }

    public double getSideDistance() {
        return ((double) (((this.leftDrive.getCurrentPosition() - this.leftDrive1.getCurrentPosition()) - this.rightDrive.getCurrentPosition()) + this.rightDrive1.getCurrentPosition())) * 0.25d * encoderTicksToCm;
    }
    private Telemetry telemetry = null;
    public void init(LinearOpMode linearOpMode2) {
        this.linearOpMode = linearOpMode2;
        telemetry = new TelemetryImpl(linearOpMode2);
        telemetry = FtcDashboard.getInstance().getTelemetry();
        this.leftDrive = (DcMotor) linearOpMode2.hardwareMap.get(DcMotor.class, "L1");
        this.rightDrive = (DcMotor) linearOpMode2.hardwareMap.get(DcMotor.class, "R1");
        this.leftDrive1 = (DcMotor) linearOpMode2.hardwareMap.get(DcMotor.class, "L2");
        this.rightDrive1 = (DcMotor) linearOpMode2.hardwareMap.get(DcMotor.class, "R2");
        this.leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        this.leftDrive1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightDrive1.setDirection(DcMotorSimple.Direction.REVERSE);
        this.leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightDrive1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        BNO055IMU bno055imu = (BNO055IMU) linearOpMode2.hardwareMap.get(BNO055IMU.class, "imu");
        this.gyro = bno055imu;
        bno055imu.initialize(new BNO055IMU.Parameters());
    }

    public void move(double drive, double side, double turn) {
        double drive2 = drive * this.correction;
        setMotorPowers(drive2 + turn + side, (drive2 - turn) - side, (drive2 + turn) - side, (drive2 - turn) + side);
    }

    public void setMotorPowers(double powerFL, double powerFR, double powerRL, double powerRR) {
        this.leftDrive1.setPower(powerRL);
        this.leftDrive.setPower(powerFL);
        this.rightDrive1.setPower(powerRR);
        this.rightDrive.setPower(powerFR);
    }

    public void TurnP(double angle) {
        double angle2 = -angle;
        double errorAngle = angle2 - ((double) this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while (abs(errorAngle) > 360.0d) {
            if (errorAngle > apiPositionFirst) {
                errorAngle -= 360.0d;
            } else {
                errorAngle += 360.0d;
            }
        }
        double errorAngle2 = errorAngle;
        while ((abs(errorAngle2) > 4.0d) && linearOpMode.opModeIsActive()) {
            errorAngle2 = angle2 - ((double) this.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            move(apiPositionFirst, apiPositionFirst, (-errorAngle2) * 0.06d);
        }
        move(apiPositionFirst, apiPositionFirst, apiPositionFirst);
    }
    public void forwardMove(double target, double targetAngle){
        double err = target - rightDrive.getCurrentPosition();
        double encoderCorrection = rightDrive.getCurrentPosition();
        double errA = targetAngle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double minErr = 20;

        while (linearOpMode.opModeIsActive() && abs(rightDrive.getCurrentPosition() - encoderCorrection) < abs(target) + minErr && abs(errA) < abs(targetAngle) + 3){
            err = target - rightDrive.getCurrentPosition();
            errA = targetAngle - gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            move(err * kp,0,errA * MotorBase.kpA);
            telemetry.addData("errA", errA);
            telemetry.addData("err", err);
            telemetry.addData("to Err", abs(rightDrive.getCurrentPosition() - encoderCorrection));
            telemetry.addData("to ErrA", abs(targetAngle) + 3);

            telemetry.update();
        }
        StopMotors();
    }




    public void GoToPosition(double x, double y) { }
    public void GoToPositionTrue(double x, double y) {
        MotorBase motorBase = this;
        double d2 = (double) motorBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double errorX = x - getSideDistance();
        double errorY = y - getForwardDistance();
        while (linearOpMode.opModeIsActive()) {
            if ((abs(errorX) > 5.0d || abs(errorY) > 5.0d) && motorBase.linearOpMode.opModeIsActive()) {
                errorX = x - getSideDistance();
                errorY = y - getForwardDistance();
                move(errorY * 0.1d, errorX * 0.1d, (0/*(double) motorBase.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle) * 0.05d*/));
                motorBase = this;
            }
        }
        StopMotors();
    }

    public void StopMotors(){ move(0,0,0); }

    public void resetEncoders() {
        this.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.leftDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
