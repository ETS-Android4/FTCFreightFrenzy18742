package org.firstinspires.ftc.teamcode.Activities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Sensors.TouchSystem;

@Config
public class Slide {

    public TouchSystem touchSystem = new TouchSystem();
    public DcMotor slideMotor = null;
    private LinearOpMode linearOpMode = null;
    public static double backwardSpeed = 0.5;
    public static double forwardSpeed = 0.85;
    private boolean[] sensors = new boolean[3];
    /*public DigitalChannel digitalTouch_start;
    public DigitalChannel digitalTouch_end;*/


    public void init(LinearOpMode linearOpMode) {
        slideMotor = linearOpMode.hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        touchSystem.init(linearOpMode);
        /*digitalTouch_start = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_start");
        digitalTouch_start.setMode(DigitalChannel.Mode.INPUT);
        digitalTouch_end = linearOpMode.hardwareMap.get(DigitalChannel.class, "sensor_end");
        digitalTouch_end.setMode(DigitalChannel.Mode.INPUT);*/
    }

    public void setMotorTarget(int direction, boolean extra) {
        sensors[0] = touchSystem.getTouch(0);
        sensors[2] = touchSystem.getTouch(1);
        if ((!sensors[1 + direction]) || (extra)) {
            slideMotor.setPower(direction == 1 ? forwardSpeed : backwardSpeed * (double) direction);
            if (extra){ slideMotor.setPower(1.0); }
        }
        else{ slideMotor.setPower(0); }
    }
}
