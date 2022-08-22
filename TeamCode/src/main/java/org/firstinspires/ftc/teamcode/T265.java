package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Robot;

import java.util.Objects;

@TeleOp(name="Test T265", group="Iterative Opmode")
public class T265 extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private Robot robot = null;

    private Pose2d st = new Pose2d(0,0);

    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 1.0, hardwareMap.appContext);
        }
        robot = new Robot(hardwareMap, telemetry, "teleop");
        robot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        robot.update();
        double leftPower = 2 * (-gamepad1.left_stick_y + gamepad1.left_stick_x / 2) / 3;
        double rightPower = 2 * (-gamepad1.left_stick_y - gamepad1.left_stick_x / 2) / 3;

        final int robotRadius = 9; // inches2

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();

        T265Camera.CameraUpdate up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;


        // We divide by 0.0254 to convert meters to inches
        Translation2d translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Rotation2d rotation = up.pose.getRotation();

        if (gamepad1.right_bumper) {
//            robot.getPoseEstimate()
            st = new Pose2d(translation.getX(), translation.getY());
        }

        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = -rotation.getCos() * robotRadius, arrowY = -rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        field.strokeCircle(st.getX(), st.getY(), robotRadius);

        dashboard.sendTelemetryPacket(packet);

        if (!robot.isBusy()) {
            robot.setMotorPowers(leftPower, rightPower);
        }

        telemetry.addData("Rotation", rotation.getDegrees());
        telemetry.addData("Robot Pose Estimate - T265", robot.getPoseEstimate().minus(new Pose2d(translation.getX(), translation.getY(), rotation.getRadians())));
        telemetry.addData("T265 Confidence", slamra.getLastReceivedCameraUpdate().confidence.toString());
        telemetry.addData("Distance Travelled", Math.pow(Math.pow(st.getX() - translation.getX(), 2) + Math.pow(st.getY() - translation.getY(), 2), 0.5));
        telemetry.update();
        robot.update();
    }

    @Override
    public void stop() {
        slamra.stop();
    }
}