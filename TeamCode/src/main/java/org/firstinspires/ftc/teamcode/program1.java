package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Basic: Linear OpMode", group = "Linear Opmode")

public class program1 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();
    private MotorBase motorBase = new MotorBase();

    @Override
    public void runOpMode() {
        motorBase.motor_init(this);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double turn = gamepad1.left_stick_x;
            double drive = gamepad1.right_trigger - gamepad1.left_trigger;
            double side = gamepad1.right_stick_x;
            motorBase.move(drive, side, turn);
            telemetry.addData("Distanse", motorBase.getForwardDistants());
            telemetry.update();

        }
        motorBase.move(0, 0, 0);
    }
}
