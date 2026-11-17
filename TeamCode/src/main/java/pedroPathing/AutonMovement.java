
package pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutonMovement {

    DcMotor LF;
    DcMotor LR;
    DcMotor RF;
    DcMotor RR;

    long moveStart;
    long moveTime;

    boolean busy = false;

    public AutonMovement(HardwareMap hardwareMap) {

        LF = hardwareMap.get(DcMotor.class, "leftFront");
        LR = hardwareMap.get(DcMotor.class, "leftRear");
        RF = hardwareMap.get(DcMotor.class, "rightFront");
        RR = hardwareMap.get(DcMotor.class, "rightRear");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LR.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveLeft(double power, long time) {
        LF.setPower(-power);
        LR.setPower(power);
        RF.setPower(power);
        RR.setPower(-power);
        beginMove(time);
    }

    public void moveRight(double power, long time) {
        LF.setPower(power);
        LR.setPower(-power);
        RF.setPower(-power);
        RR.setPower(power);
        beginMove(time);
    }

    public void moveForward(double power, long time) {
        LF.setPower(power);
        LR.setPower(power);
        RF.setPower(power);
        RR.setPower(power);
        beginMove(time);
    }

    public void moveBack(double power, long time) {
        LF.setPower(-power);
        LR.setPower(-power);
        RF.setPower(-power);
        RR.setPower(-power);
        beginMove(time);
    }

    private void beginMove(long time) {
        moveTime = time;
        moveStart = System.currentTimeMillis();
        busy = true;
    }

    public void update() {
        if (busy && System.currentTimeMillis() - moveStart >= moveTime) {
            stopMotors();
            busy = false;
        }
    }

    public boolean isBusy() {
        return busy;
    }

    public void stopMotors() {
        LF.setPower(0);
        LR.setPower(0);
        RF.setPower(0);
        RR.setPower(0);
        busy = false;
    }
}
