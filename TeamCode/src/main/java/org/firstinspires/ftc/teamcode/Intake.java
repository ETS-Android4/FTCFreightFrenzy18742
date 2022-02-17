package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotor intakeMotor = null;
    public void init(LinearOpMode linearOpMode){
        intakeMotor = linearOpMode.hardwareMap.get(DcMotor.class, "Elevator");
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }
    public void IntakeMotor(double speed){intakeMotor.setPower(speed);}
}
