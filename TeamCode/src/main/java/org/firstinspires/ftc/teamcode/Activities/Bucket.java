package org.firstinspires.ftc.teamcode.Activities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class Bucket {
    DigitalChannel digitalTouch_end;

    private Servo servo = null;
    private boolean servoLast = false;
    private ElapsedTime servoTimer = new ElapsedTime();
    public static double position1 = 0.15;
    public static double position2 = 0.50;

    public static double servoSpeedSeconds = 0.1;

    public void init(LinearOpMode linearOpMode) {
        servo = linearOpMode.hardwareMap.get(Servo.class, "frow");
        digitalTouch_end = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_end");
        digitalTouch_end.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setServoPosition(boolean servo) {
        servo = servo && !digitalTouch_end.getState();
        if (servo && !servoLast)
            servoTimer.reset();
        if (servo)
            this.servo.setPosition(Range.clip(position1 + servoTimer.seconds() * (Math.abs(position2 - position1) / servoSpeedSeconds), position1, position2));
        else
            this.servo.setPosition(position1);
        servoLast = servo;
    }
}
