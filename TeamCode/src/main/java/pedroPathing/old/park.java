package pedroPathing.old;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled
@Autonomous(name = "park", group = "Auton")
public class park extends OpMode {

    private Follower follower;
    private PathChain path;

    private final Pose startPose = new Pose(9, 130, Math.toRadians(90));
    private final Pose endPose = new Pose(14, 118, Math.toRadians(90));

    @Override
    public void init() {

        follower = new Follower(
                hardwareMap,
                FConstants.class,
                LConstants.class
        );

        follower.setStartingPose(startPose);

        path = follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .build();
    }

    @Override
    public void start() {
        follower.followPath(path);
    }

    @Override
    public void loop() {

        follower.update();

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.addData("Status", follower.isBusy() ? "Moving" : "Done");

        telemetry.update();
    }
}