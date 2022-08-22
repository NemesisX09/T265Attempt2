package org.firstinspires.ftc.teamcode.RoadRunner.util.Localizers;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import com.spartronics4915.lib.T265Camera;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

/**
 * a Road Runner localizer that uses the Intel T265 Realsense
 */
@Config
public class T265Loc implements Localizer {
    public static T265Camera slamra;
    public T265Camera.CameraUpdate up;
    public static T265Camera.PoseConfidence conf;
    public static boolean is_fin = false;
    private boolean poseinited = false;

    public T265Loc(HardwareMap hardwareMap) {
        try {
            stopRealsense();
        } catch (Exception ignore) {

        }
        is_fin = true;
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 1.0, hardwareMap.appContext);
        }

        try {
            startRealsense();
        } catch (Exception ignored) {
            RobotLog.d("T265: RealSense was started already, crying...");
//            stopRealsense();
//            RobotLog.d("T265: Stopping...");
//            startRealsense();
//            RobotLog.d("T265: Starting...");
        }
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(0, 0, new Rotation2d(0)));
        is_fin = true;
    }

    @NotNull
    @Override
    public Pose2d getPoseEstimate() {
        if (up != null) {
            return new Pose2d(up.pose.getX() / .0254, up.pose.getY() / .0254, up.pose.getHeading());
        } else {
            update();
            return new Pose2d(up.pose.getX() / .0254, up.pose.getY() / .0254, up.pose.getHeading());
        }
    }

    @Override
    public void setPoseEstimate(@NotNull Pose2d pose2d) {
        slamra.setPose(new com.arcrobotics.ftclib.geometry.Pose2d(pose2d.getX() * .0254, pose2d.getY() * .0254, new Rotation2d(pose2d.getHeading())));
        poseinited = true;
    }

    public T265Camera.PoseConfidence getConfidence() {
        return up.confidence;
    }

    public double getHeading() {
        return norma(up.pose.getHeading());
    }

    /**
     * updates the camera.  Used in
     * @see org.firstinspires.ftc.teamcode in update()
     */
    @Override
    public void update() {
        if (poseinited) {
            up = slamra.getLastReceivedCameraUpdate();
            if (up != null) {
                conf = up.confidence;
            }
        }
    }

    /**
     No idea what the purpose getPoseVelocity.  Everything works fine by just using getPoseEstimate()
     That said, the code to get the velocity is comment out below.  Haven't testing it much
     and I don't know how well getting the velocity work or if use the velocity has any effect
     at all.
     */
    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        ChassisSpeeds vel = up.velocity;
        return new Pose2d(vel.vxMetersPerSecond / .0254, vel.vyMetersPerSecond / .0254, vel.omegaRadiansPerSecond);
    }

    /**
     * @param angle angle in radians
     * @return normiazled angle between ranges 0 to 2Pi
     */
    private double norm(double angle)
    {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }
    private static double norma(double angle)
    {
        while (angle>Math.toRadians(360)) angle-=Math.toRadians(360);
        while (angle<=0) angle+=Math.toRadians(360);
        return angle;
    }

    /**
     * starts realsense
     * (Called automatically when a program using this starts)
     */
    /*
    Unused methods.  Here just in case they may be needed.
     */
    public static void startRealsense() {
        slamra.start();
        RobotLog.d("T265: Slamra started " + slamra.isStarted());
    }

    /**
     * stops the realsense
     * (called automatically when a program stops)
     */
    public static void stopRealsense() {
        slamra.stop();
        RobotLog.d("T265: Slamra is at: " + slamra.isStarted());
    }
}