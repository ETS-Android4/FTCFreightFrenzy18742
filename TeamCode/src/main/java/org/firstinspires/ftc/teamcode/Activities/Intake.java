package org.firstinspires.ftc.teamcode.Activities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {
    public static double reverseSpeed = 0.5d;
    public static double reverseTime = 1.0d;
    public static double voltageThreshold = 0.5d;
    private DigitalChannel digitalTouch_start;
    public AnalogInput freightSensor = null;
    public DcMotor intakeMotor = null;
    private double lastSpeed = LynxServoController.apiPositionFirst;
    private DcMotor ledStrip = null;
    private ElapsedTime reverseTimer = new ElapsedTime();

    public void init(LinearOpMode linearOpMode) {
        DcMotor dcMotor = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "brushMotor");
        this.intakeMotor = dcMotor;
        dcMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        DcMotor dcMotor2 = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "ledStrip");
        this.ledStrip = dcMotor2;
        dcMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        this.freightSensor = (AnalogInput) linearOpMode.hardwareMap.get(AnalogInput.class, "freightSensor");
        DigitalChannel digitalChannel = (DigitalChannel) linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_start");
        this.digitalTouch_start = digitalChannel;
        digitalChannel.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isFreightDetected() {
        return this.freightSensor.getVoltage() > voltageThreshold;
    }

    public void IntakeMotor(double speed) {
        this.ledStrip.setPower(isFreightDetected() ? 1.0d : 0.0d);
        if (this.digitalTouch_start.getState() || isFreightDetected()) {
            speed = LynxServoController.apiPositionFirst;
        }
        if (speed == LynxServoController.apiPositionFirst && this.lastSpeed > LynxServoController.apiPositionFirst) {
            this.reverseTimer.reset();
        }
        if (this.reverseTimer.seconds() < reverseTime) {
            speed = -reverseSpeed;
        }
        this.intakeMotor.setPower(speed);
        this.lastSpeed = speed;
    }
}
