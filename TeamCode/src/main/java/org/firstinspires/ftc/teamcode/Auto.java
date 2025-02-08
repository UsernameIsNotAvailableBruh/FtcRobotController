package org.firstinspires.ftc.teamcode;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;


@Autonomous(name = "Auto", group = "PEDRO")
public class Auto extends OpMode {
    private Follower follower = null;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private PathChain a;

    /**
     * User-defined init method
     * <p>
     * This method will be called once, when the INIT button is pressed.
     */
    @Override
    public void init() {
        pathTimer = new Timer();
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        Pose startPose = new Pose(10, 68);
        Pose pose2 = new Pose(20, 80);
        Pose pose3 = new Pose(20, 120, 135);
        follower.setStartingPose(startPose);

        PathBuilder builder = new PathBuilder();
        builder.addPath(
                new BezierLine(
                        new Point(startPose),
                        new Point(pose2)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .addPath(
                        new BezierLine(
                                new Point(pose2),
                                new Point(pose3)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(135));
        a = builder.build();
    }

    public void setPathState(int pState) {
        pathTimer.resetTimer();
    }

    /**
     * User-defined start method
     * <p>
     * This method will be called once, when the play button is pressed.
     * <p>
     * This method is optional. By default, this method takes no action.
     * <p>
     * Example usage: Starting another thread.
     */
    @Override
    public void start() {};

    /**
     * User-defined stop method
     * <p>
     * This method will be called once, when this OpMode is stopped.
     * <p>
     * Your ability to control hardware from this method will be limited.
     * <p>
     * This method is optional. By default, this method takes no action.
     */
    public void stop() {};

    /**
     * User-defined loop method
     * <p>
     * This method will be called repeatedly during the period between when
     * the play button is pressed and when the OpMode is stopped.
     */
    @Override
    public void loop() {
        follower.update();
        follower.followPath(a, true);
    }
}
