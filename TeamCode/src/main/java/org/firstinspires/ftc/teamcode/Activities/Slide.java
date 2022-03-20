package org.firstinspires.ftc.teamcode.Activities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.Sensors.TouchSystem;

@Config
public class Slide {
    public static double backwardSpeed = 0.5d;
    public static double forwardSpeed = 0.85d;
    private LinearOpMode linearOpMode = null;
    private boolean[] sensors = new boolean[3];
    public DcMotor slideMotor = null;
    public TouchSystem touchSystem = new TouchSystem();

    public void init(LinearOpMode linearOpMode2) {
        DcMotor dcMotor = (DcMotor) linearOpMode2.hardwareMap.get(DcMotor.class, "slideMotor");
        this.slideMotor = dcMotor;
        dcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.touchSystem.init(linearOpMode2);
    }

    public void setMotorTarget(int direction, boolean extra) {
        this.sensors[0] = this.touchSystem.getTouch(0);
        this.sensors[2] = this.touchSystem.getTouch(1);
        if (!this.sensors[direction + 1] || extra) {
            this.slideMotor.setPower(direction == 1 ? forwardSpeed : backwardSpeed * ((double) direction));
            if (extra) {
                this.slideMotor.setPower(1.0d);
                return;
            }
            return;
        }
        this.slideMotor.setPower(LynxServoController.apiPositionFirst);
    }
}
