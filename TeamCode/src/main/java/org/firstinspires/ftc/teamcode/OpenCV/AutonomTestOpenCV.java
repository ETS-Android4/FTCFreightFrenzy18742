/*package org.firstinspires.ftc.teamcode.OpenCV;

import static org.firstinspires.ftc.teamcode.opencv.ArucoDetect.centreOfDuck;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class AutonomTestOpenCV extends BaseDetectionAutonomous{

    Runnable[] test = {
            ()->{
                robot.telemetryNode.getTelemetry().addData("forceGetPosition",robot.arucoDetect.forceGetPosition()  );
                robot.telemetryNode.getTelemetry().addData("timePosition",robot.arucoDetect.timePosition);
                robot.telemetryNode.getTelemetry().addData("offset", centreOfDuck);
                },
    };

    @Override
    public void main() {
        while (opModeIsActive())
            execute(test);
    }

}*/
