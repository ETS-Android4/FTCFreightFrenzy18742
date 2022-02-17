package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Frow {
    DigitalChannel digitalTouch_start;
    DigitalChannel digitalTouch_end;

    private Servo frow = null;
    public double position1 = 0.2;
    public double position2 = 0.7;

    public void init(LinearOpMode linearOpMode) {
        frow = linearOpMode.hardwareMap.get(Servo.class, "frow");
        digitalTouch_start = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_start");
        digitalTouch_start.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch_end = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_end");
        digitalTouch_end.setMode(DigitalChannel.Mode.INPUT);
    }

    public void setServoPosition(boolean servo) {
        if (servo && digitalTouch_end.getState()) {frow.setPosition(position2);}
        else {frow.setPosition(position1);}
    }
}
