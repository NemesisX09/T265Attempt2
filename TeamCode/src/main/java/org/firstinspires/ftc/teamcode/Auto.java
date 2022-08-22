package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.RoadRunner.util.Localizers.T265Loc;

@Autonomous(name="Auto T265", group="Iterative Opmode")

public class Auto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(hardwareMap, telemetry, "autonomous");

        while (!T265Loc.is_fin) {
            telemetry.addData("Waiting for Localizer", "");
            telemetry.update();
        }

        robot.setPoseEstimate(new Pose2d(-60,60, Math.toRadians(90)));

        waitForStart();

        telemetry.addData("Robot",robot.getPoseEstimate());
        telemetry.addData("Localizer",robot.t265Localizer.getPoseEstimate());
        telemetry.update();
        sleep(1000);
        if (robot.getPoseEstimate().getX() == 0 && robot.getPoseEstimate().getY() == 0) {
            return;
        }
        TrajectorySequence traj1 = robot.trajectorySequenceBuilder(robot.getPoseEstimate())
                .turn(Math.toRadians(90))
                .forward(10)
                .build();

//        robot.followTrajectorySequence(traj1);
        while (opModeIsActive()) {
            telemetry.addData("Robot",robot.getPoseEstimate());
            telemetry.addData("Localizer",robot.t265Localizer.getPoseEstimate());
            telemetry.update();
            robot.update();
        }
    }
}
