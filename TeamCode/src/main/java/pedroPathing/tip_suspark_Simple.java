
package pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "tip_leave_suspark_Simple", group = "Auton")
public class tip_suspark_Simple extends OpMode {

    DcMotorEx leftMotor;
    DcMotorEx rightMotor;

    DcMotor frontMotor;
    DcMotor backMotor;

    Servo gate;
    Servo left;

    AutonMovement movement;

    ElapsedTime autonTimer = new ElapsedTime();
    ElapsedTime pidTimer = new ElapsedTime();

    double launcherVelocity = 1560;
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

    boolean sequenceFinished = false;

    int movementStep = 0;

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
    public void init() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        movement = new AutonMovement(hardwareMap);

        gate = hardwareMap.get(Servo.class, "gate");
        left = hardwareMap.get(Servo.class, "left");

        leftMotor = hardwareMap.get(DcMotorEx.class, "leftF");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightF");

        frontMotor = hardwareMap.get(DcMotor.class, "front");
        backMotor = hardwareMap.get(DcMotor.class, "back");

        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gate.setPosition(ConfigValues.gateClosed);
        left.setPosition(ConfigValues.tongueHigh);

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        frontMotor.setPower(0);
        backMotor.setPower(0);

        FtcDashboard.getInstance().setTelemetryTransmissionInterval(25);
    }

    @Override
    public void start() {

        autonTimer.reset();
        pidTimer.reset();

        lastError = 0;
        error = 0;
        pidOutput = 0;

        feedforwardPower = 0;
        pCorrection = 0;
        dCorrection = 0;

        sequenceFinished = false;
        movementStep = 0;

        gate.setPosition(ConfigValues.gateClosed);
        left.setPosition(ConfigValues.tongueHigh);

        leftMotor.setPower(ConfigValues.high);
        rightMotor.setPower(ConfigValues.high);
    }

    @Override
    public void loop() {

        movement.update();

        double time = autonTimer.seconds();

        if (!sequenceFinished || movementStep < 7) {

            if (movementStep == 0) {

                currentVelocity = leftMotor.getVelocity();
                error = launcherVelocity - currentVelocity;

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

                if (time >= 8) {
                    gate.setPosition(ConfigValues.gateOpen);
                }

                if (time >= 11) {
                    frontMotor.setPower(ConfigValues.transferLow);
                    backMotor.setPower(ConfigValues.transferLow);
                }

                if (time >= 13) {
                    frontMotor.setPower(ConfigValues.transferHigh);
                    backMotor.setPower(ConfigValues.transferHigh);
                }

                if (time >= 15) {

                    leftMotor.setPower(0);
                    rightMotor.setPower(0);

                    frontMotor.setPower(0);
                    backMotor.setPower(0);

                    gate.setPosition(ConfigValues.gateClosed);
                    left.setPosition(ConfigValues.tongueHigh);

                    movement.moveBack(0.5, 500);

                    movementStep = 1;
                }
            }

            if (movementStep == 1 && !movement.isBusy()) {
                movement.moveRight(0.6, 1100);
                movementStep = 2;
            }

            if (movementStep == 2 && !movement.isBusy()) {
                movement.moveBack(0.4, 1300);
                movementStep = 3;
            }

            if (movementStep == 3 && !movement.isBusy()) {
                movement.moveRight(0.5, 2000);
                movementStep = 4;
            }

            if (movementStep == 4 && !movement.isBusy()) {
                movement.moveLeft(0.5, 450);
                movementStep = 5;
            }

            if (movementStep == 5 && !movement.isBusy()) {
                movement.moveBack(0.3, 2600);
                movementStep = 6;
            }

            if (movementStep == 6 && !movement.isBusy()) {
                movementStep = 7;
                sequenceFinished = true;
            }
        }

        telemetry.addData("Auton Time", time);
        telemetry.addData("Target Velocity", launcherVelocity);
        telemetry.addData("Actual Velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("PIDF Output", pidOutput);

        telemetry.addData("Feedforward Power", feedforwardPower);
        telemetry.addData("P Correction", pCorrection);
        telemetry.addData("D Correction", dCorrection);

        telemetry.addData("Movement Step", movementStep);
        telemetry.addData("Sequence Finished", sequenceFinished);
        telemetry.addData("Driving", movement.isBusy());

        telemetry.update();
    }

    @Override
    public void stop() {

        leftMotor.setPower(0);
        rightMotor.setPower(0);

        frontMotor.setPower(0);
        backMotor.setPower(0);

        movement.stopMotors();
    }
}
