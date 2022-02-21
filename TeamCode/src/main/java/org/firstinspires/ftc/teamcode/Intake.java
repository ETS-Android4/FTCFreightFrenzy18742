package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake {
    public DcMotor intakeMotor = null;
    private DigitalChannel digitalTouch_end;

    private double lastSpeed = 0;
    private ElapsedTime reverseTimer = new ElapsedTime();

    public static double reverseTime = 1.0;
    public static double reverseSpeed = 0.5;

    public void init(LinearOpMode linearOpMode) {
        intakeMotor = linearOpMode.hardwareMap.get(DcMotor.class, "Elevator");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        digitalTouch_end = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_end");
        digitalTouch_end.setMode(DigitalChannel.Mode.INPUT);
    }

    public void IntakeMotor(double speed) {
        if (!digitalTouch_end.getState())
            speed = 0;
        if (speed == 0 && lastSpeed > 0)
            reverseTimer.reset();
        if (reverseTimer.seconds() < reverseTime)
            speed = -reverseSpeed;
        intakeMotor.setPower(speed);
        lastSpeed = speed; //
    }
}
