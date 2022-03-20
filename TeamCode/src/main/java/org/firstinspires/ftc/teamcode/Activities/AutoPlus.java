package org.firstinspires.ftc.teamcode.Activities;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class AutoPlus {
    public static double positionA1 = 0.15d;
    public static double positionA2 = 0.65d;
    public static double positionB1 = 0.6d;
    public static double positionB2 = 0.9d;
    public boolean M = false;
    public DcMotor ledPlus = null;
    public Servo servoA = null;
    public Servo servoB = null;
    public ElapsedTime servoTimer = new ElapsedTime();
    public double timer = LynxServoController.apiPositionFirst;

    public void init(LinearOpMode linearOpMode) {
        this.servoA = (Servo) linearOpMode.hardwareMap.get(Servo.class, "ServoA");
        this.servoB = (Servo) linearOpMode.hardwareMap.get(Servo.class, "ServoB");
        DcMotor dcMotor = (DcMotor) linearOpMode.hardwareMap.get(DcMotor.class, "LedPlus");
        this.ledPlus = dcMotor;
        dcMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setServoAPosition(boolean servo) {
        if (!servo) {
            this.servoA.setPosition(positionA2);
        } else {
            this.servoA.setPosition(positionA1);
        }
    }

    public void setServoBPosition(boolean servo) {
        if (servo) {
            this.servoB.setPosition(positionB2);
        } else {
            this.servoB.setPosition(positionB1);
        }
    }
}
