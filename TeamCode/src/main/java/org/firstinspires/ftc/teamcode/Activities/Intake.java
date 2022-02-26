package org.firstinspires.ftc.teamcode.Activities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {
    public DcMotor intakeMotor = null;
    private DcMotor ledStrip = null;
    private AnalogInput freightSensor = null;
    private DigitalChannel digitalTouch_start;

    private double lastSpeed = 0;
    private ElapsedTime reverseTimer = new ElapsedTime();

    public static double reverseTime = 1.0;
    public static double reverseSpeed = 0.5;

    public void init(LinearOpMode linearOpMode) {
        intakeMotor = linearOpMode.hardwareMap.get(DcMotor.class, "brushMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        ledStrip = linearOpMode.hardwareMap.get(DcMotor.class, "ledStrip");
        ledStrip.setDirection(DcMotorSimple.Direction.FORWARD);
        freightSensor = linearOpMode.hardwareMap.get(AnalogInput.class, "freightSensor");
        digitalTouch_start = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_start");
        digitalTouch_start.setMode(DigitalChannel.Mode.INPUT);
    }

    public static double voltageThreshold = .5;

    public boolean isFreightDetected() {
        return freightSensor.getVoltage() > voltageThreshold;
    }

    public void IntakeMotor(double speed) {
        boolean freightDetected = isFreightDetected();
        ledStrip.setPower(freightDetected ? 1 : 0);
        if (digitalTouch_start.getState() || isFreightDetected())
            speed = 0;
        if (speed == 0 && lastSpeed > 0)
            reverseTimer.reset();
        if (reverseTimer.seconds() < reverseTime)
            speed = -reverseSpeed;
        intakeMotor.setPower(speed);
        lastSpeed = speed; //
    }
}
