
package pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "parkSimple", group = "Auton")
public class park_leave_Simple extends OpMode {

    AutonMovement movement;

    Servo left;
    Servo gate;

    int step = 0;

    @Override
    public void init() {

        movement = new AutonMovement(hardwareMap);

        left = hardwareMap.get(Servo.class, "left");
        gate = hardwareMap.get(Servo.class, "gate");

        left.setPosition(ConfigValues.tongueHigh);
        gate.setPosition(ConfigValues.gateClosed);
    }

    @Override
    public void start() {
        step = 1;
    }

    @Override
    public void loop() {

        movement.update();

        switch (step) {

            case 1:
                if (!movement.isBusy()) {
                    movement.moveBack(0.4, 400);
                    step = 2;
                }
                break;

            case 2:
                if (!movement.isBusy()) {
                    movement.moveRight(0.5, 400);
                    step = 999;
                }
                break;

            case 999:
                movement.stopMotors();
                break;
        }

        telemetry.addData("Step", step);
        telemetry.addData("Busy", movement.isBusy());
        telemetry.update();
    }

    @Override
    public void stop() {
        movement.stopMotors();
    }
}
