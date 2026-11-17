package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "newTeleop", group = "Adon")
public class newTeleop extends OpMode {

    DcMotor LF;
    DcMotor LR;
    DcMotor RF;
    DcMotor RR;

    Servo left;
    Servo gate;

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;
    DcMotor frontMotor;
    DcMotor backMotor;

    ElapsedTime pidTimer = new ElapsedTime();

    double tonguePosition = ConfigValues.tongueHigh;
    double gatePosition = ConfigValues.gateOpen;

    double launcherVelocity = 1520;
    double currentVelocity = 0;

    double error = 0;
    double lastError = 0;
    double pidOutput = 0;

    double feedforwardVelocity1 = 1400;
    double feedforwardPower1 = 0.59;

    double feedforwardVelocity2 = 1650;
    double feedforwardPower2 = 0.69;

    double spinUpPower = 0.95;

    double feedforwardPower = 0;
    double pCorrection = 0;
    double dCorrection = 0;

    int intakeState = 0;

    boolean motorsOn = false;

    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastLeft = false;
    boolean lastRight = false;

    boolean lastB = false;
    boolean lastA = false;
    boolean lastY = false;
    boolean lastX = false;

    boolean lastLeftStick = false;
    boolean lastRightStick = false;

    boolean lastGamepad2LeftStick = false;
    boolean lastGamepad2RightStick = false;

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        LF = hardwareMap.get(DcMotor.class, "leftFront");
        LR = hardwareMap.get(DcMotor.class, "leftRear");
        RF = hardwareMap.get(DcMotor.class, "rightFront");
        RR = hardwareMap.get(DcMotor.class, "rightRear");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LR.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left = hardwareMap.get(Servo.class, "left");
        gate = hardwareMap.get(Servo.class, "gate");

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftF");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightF");

        frontMotor = hardwareMap.get(DcMotor.class, "front");
        backMotor = hardwareMap.get(DcMotor.class, "back");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);

        pidTimer.reset();
    }

    double getFeedforwardPower(double target) {

        if (feedforwardVelocity2 <= feedforwardVelocity1) {
            return feedforwardPower1;
        }

        double fraction =
                (target - feedforwardVelocity1) /
                        (feedforwardVelocity2 - feedforwardVelocity1);

        fraction = Math.max(0, Math.min(1, fraction));

        return feedforwardPower1 +
                fraction * (feedforwardPower2 - feedforwardPower1);
    }

    double runPIDF(double target, double current, double dt) {

        error = target - current;

        double derivative = (error - lastError) / dt;

        feedforwardPower = getFeedforwardPower(target);

        pCorrection = ConfigValues.pN * error;
        dCorrection = ConfigValues.dN * derivative;

        lastError = error;

        double output = feedforwardPower +
                pCorrection +
                dCorrection;

        return Math.max(0, Math.min(1, output));
    }

    @Override
    public void loop() {

        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(
                Math.abs(y) + Math.abs(x) + Math.abs(rx), 1
        );

        double lfPower = (y + x + rx) / denominator;
        double lrPower = (y - x + rx) / denominator;
        double rfPower = (y - x - rx) / denominator;
        double rrPower = (y + x - rx) / denominator;

        LF.setPower(lfPower);
        LR.setPower(lrPower);
        RF.setPower(rfPower);
        RR.setPower(rrPower);

        if (gamepad1.dpad_up && !lastUp) {
            tonguePosition = ConfigValues.tongueHigh;
        }

        if (gamepad1.dpad_down && !lastDown) {
            tonguePosition = ConfigValues.tongueLow;
        }

        if (gamepad1.dpad_left && !lastLeft) {
            tonguePosition += 0.02;
        }

        if (gamepad1.dpad_right && !lastRight) {
            tonguePosition -= 0.02;
        }

        if (gamepad1.left_bumper) {
            gatePosition = ConfigValues.gateOpen;
        } else {
            gatePosition = ConfigValues.gateClosed;
        }

        if (gamepad1.b && !lastB) {
            motorsOn = !motorsOn;

            lastError = 0;
            error = 0;
            pidOutput = 0;

            feedforwardPower = 0;
            pCorrection = 0;
            dCorrection = 0;

            pidTimer.reset();
        }

        if (gamepad1.left_stick_button && !lastLeftStick) {
            launcherVelocity -= 10;
        }

        if (gamepad1.right_stick_button && !lastRightStick) {
            launcherVelocity += 10;
        }

        if (gamepad2.left_stick_button && !lastGamepad2LeftStick) {
            launcherVelocity -= 10;
        }

        if (gamepad2.right_stick_button && !lastGamepad2RightStick) {
            launcherVelocity += 10;
        }

        if (gamepad1.a && !lastA) {
            intakeState = 1;
        }

        if (gamepad1.y && !lastY) {
            intakeState = -1;
        }

        if (gamepad1.x && !lastX) {
            intakeState = 0;
        }

        launcherVelocity = Math.max(0, Math.min(2800, launcherVelocity));

        currentVelocity = leftMotor.getVelocity();

        error = launcherVelocity - currentVelocity;

        if (motorsOn) {

            if (pidTimer.seconds() >= ConfigValues.LOOP_TIME) {

                double dt = pidTimer.seconds();

                if (error > ConfigValues.closeZone &&
                        currentVelocity < launcherVelocity) {

                    pidOutput = spinUpPower;

                    feedforwardPower = getFeedforwardPower(launcherVelocity);

                    pCorrection = 0;
                    dCorrection = 0;

                    lastError = error;

                } else {

                    pidOutput = runPIDF(
                            launcherVelocity,
                            currentVelocity,
                            dt
                    );
                }

                leftMotor.setPower(pidOutput);
                rightMotor.setPower(pidOutput);

                pidTimer.reset();
            }

        } else {

            leftMotor.setPower(0);
            rightMotor.setPower(0);

            currentVelocity = 0;

            pidOutput = 0;
            error = 0;
            lastError = 0;

            feedforwardPower = 0;
            pCorrection = 0;
            dCorrection = 0;

            pidTimer.reset();
        }

        double transferSpeed = gatePosition == ConfigValues.gateOpen
                ? ConfigValues.transferLow
                : ConfigValues.transferHigh;

        if (gamepad2.y) {
            frontMotor.setPower(transferSpeed);
            backMotor.setPower(-transferSpeed);
        } else {
            frontMotor.setPower(intakeState * transferSpeed);
            backMotor.setPower(intakeState * transferSpeed);
        }

        tonguePosition = Math.max(0.0, Math.min(1.0, tonguePosition));
        gatePosition = Math.max(0.0, Math.min(1.0, gatePosition));

        left.setPosition(tonguePosition);
        gate.setPosition(gatePosition);

        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
        lastLeft = gamepad1.dpad_left;
        lastRight = gamepad1.dpad_right;

        lastB = gamepad1.b;
        lastA = gamepad1.a;
        lastY = gamepad1.y;
        lastX = gamepad1.x;

        lastLeftStick = gamepad1.left_stick_button;
        lastRightStick = gamepad1.right_stick_button;

        lastGamepad2LeftStick = gamepad2.left_stick_button;
        lastGamepad2RightStick = gamepad2.right_stick_button;

        telemetry.addData("Tongue Position", tonguePosition);
        telemetry.addData("Gate Position", gatePosition);

        telemetry.addData("Launcher On", motorsOn);

        telemetry.addData("Intake", intakeState == 1 ? "Forward" :
                intakeState == -1 ? "Reverse" : "Off");

        telemetry.addData("Transfer Speed", transferSpeed);

        telemetry.addData("Target Velocity", launcherVelocity);
        telemetry.addData("Actual Velocity", currentVelocity);

        telemetry.addData("Error", error);
        telemetry.addData("PIDF Output", pidOutput);

        telemetry.addData("Feedforward Power", feedforwardPower);
        telemetry.addData("P Correction", pCorrection);
        telemetry.addData("D Correction", dCorrection);

        telemetry.update();
    }
}