
package pedroPathing.old;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

@Disabled
@Config
@Autonomous(name = "tip_park", group = "Auton")
public class tip_park extends OpMode {

    public static double kP = 0.000038;
    public static double kD = 0.000035;
    public static double LOOP_TIME = 0.025;

    public static double transferLow = 0.6;
    public static double transferHigh = 1.0;
    public static double high = 0.9;
    public static double closeZone = 250;

    public static double gateHigh = 0.64;
    public static double gateLow = 0.5;

    public static double tongueUp = 0.4;
    public static double tongueDown = 0.95;

    public static double pathPower = 0.7;

    private Follower follower;

    private DcMotorEx leftMotor;
    private DcMotorEx rightMotor;

    private DcMotor frontMotor;
    private DcMotor backMotor;

    private Servo gate;
    private Servo left;

    private ElapsedTime autonTimer = new ElapsedTime();
    private ElapsedTime pidTimer = new ElapsedTime();
    private ElapsedTime actionTimer = new ElapsedTime();

    private final Pose startPose = new Pose(56, 9, Math.toRadians(270));
    private final Pose middlePose = new Pose(24, 48, Math.toRadians(270));
    private final Pose endPose = new Pose(11, 97, Math.toRadians(270));

    private PathChain path1;
    private PathChain path2;

    private int pathState = 0;

    private double launcherVelocity = 1550;
    private double currentVelocity = 0;

    private double error = 0;
    private double lastError = 0;
    private double pidOutput = 0;

    private boolean gateOpened = false;
    private boolean transferOn = false;
    private boolean transferHighOn = false;
    private boolean shootingFinished = false;

    private boolean tongueDeployed = false;

    public void buildPaths() {

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(startPose),
                        new Point(middlePose)
                ))
                .setLinearHeadingInterpolation(
                        startPose.getHeading(),
                        middlePose.getHeading()
                )
                .setZeroPowerAccelerationMultiplier(2)
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(middlePose),
                        new Point(endPose)
                ))
                .setLinearHeadingInterpolation(
                        middlePose.getHeading(),
                        endPose.getHeading()
                )
                .setZeroPowerAccelerationMultiplier(2)
                .build();
    }

    private double runPD(double target, double current, double currentPower) {

        error = target - current;

        double derivative = (error - lastError) / LOOP_TIME;

        double pdCorrection = (kP * error) + (kD * derivative);

        lastError = error;

        return Math.max(0, Math.min(1, currentPower + pdCorrection));
    }

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        gate = hardwareMap.get(Servo.class, "gate");
        left = hardwareMap.get(Servo.class, "left");

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftF");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightF");

        frontMotor = hardwareMap.get(DcMotor.class, "front");
        backMotor = hardwareMap.get(DcMotor.class, "back");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate.setPosition(gateHigh);
        left.setPosition(tongueUp);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        frontMotor.setPower(0);
        backMotor.setPower(0);

        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

        follower.setStartingPose(startPose);

        buildPaths();

        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);
    }

    @Override
    public void start() {

        autonTimer.reset();
        pidTimer.reset();

        lastError = 0;
        error = 0;
        pidOutput = 0;

        leftMotor.setPower(high);
        rightMotor.setPower(high);
    }

    @Override
    public void loop() {

        follower.update();

        currentVelocity = leftMotor.getVelocity();

        error = launcherVelocity - currentVelocity;

        if (!shootingFinished && pidTimer.seconds() >= LOOP_TIME) {

            if (error > closeZone && currentVelocity < launcherVelocity) {

                pidOutput = high;
                lastError = error;

            } else {

                pidOutput = runPD(
                        launcherVelocity,
                        currentVelocity,
                        leftMotor.getPower()
                );
            }

            leftMotor.setPower(pidOutput);
            rightMotor.setPower(pidOutput);

            pidTimer.reset();
        }

        if (!shootingFinished) {

            if (autonTimer.seconds() >= 5 && !gateOpened) {
                gate.setPosition(gateLow);
                gateOpened = true;
            }

            if (autonTimer.seconds() >= 6 && !transferOn) {
                frontMotor.setPower(transferLow);
                backMotor.setPower(transferLow);
                transferOn = true;
            }

            if (autonTimer.seconds() >= 9 && !transferHighOn) {
                frontMotor.setPower(transferHigh);
                backMotor.setPower(transferHigh);
                transferHighOn = true;
            }

            if (autonTimer.seconds() >= 11) {
                leftMotor.setPower(0);
                rightMotor.setPower(0);

                frontMotor.setPower(0);
                backMotor.setPower(0);

                gate.setPosition(gateHigh);

                shootingFinished = true;

                follower.setMaxPower(pathPower);
                follower.followPath(path1, true);

                pathState = 1;
            }
        }

        if (shootingFinished) {

            switch (pathState) {

                case 1:

                    if (!follower.isBusy()) {

                        follower.holdPoint(middlePose);

                        left.setPosition(tongueDown);

                        actionTimer.reset();

                        tongueDeployed = true;
                        pathState = 2;
                    }

                    break;

                case 2:

                    if (actionTimer.seconds() >= 0.5) {

                        left.setPosition(tongueUp);

                        follower.setMaxPower(pathPower);
                        follower.followPath(path2, true);

                        pathState = 3;
                    }

                    break;

                case 3:

                    if (!follower.isBusy()) {

                        follower.holdPoint(endPose);

                        pathState = 4;
                    }

                    break;
            }
        }

        telemetry.addData("Auton Time", autonTimer.seconds());

        telemetry.addData("Target Velocity", launcherVelocity);
        telemetry.addData("Actual Velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("PID Output", pidOutput);

        telemetry.addData("Gate Opened", gateOpened);
        telemetry.addData("Transfer Low", transferOn);
        telemetry.addData("Transfer High", transferHighOn);

        telemetry.addData("Path State", pathState);

        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.addData("Tongue Deployed", tongueDeployed);

        telemetry.update();
    }

    @Override
    public void stop() {

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        frontMotor.setPower(0);
        backMotor.setPower(0);
    }
}
