
package pedroPathing.old;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Drive Motor Testing", group = "Adon")
public class DriveMotorTesting extends OpMode {

    DcMotor LF;
    DcMotor LR;
    DcMotor RF;
    DcMotor RR;

    @Override
    public void init() {

        LF = hardwareMap.get(DcMotor.class, "LF");
        LR = hardwareMap.get(DcMotor.class, "LR");
        RF = hardwareMap.get(DcMotor.class, "RF");
        RR = hardwareMap.get(DcMotor.class, "RR");

        LF.setDirection(DcMotorSimple.Direction.FORWARD);
        LR.setDirection(DcMotorSimple.Direction.FORWARD);
        RF.setDirection(DcMotorSimple.Direction.REVERSE);
        RR.setDirection(DcMotorSimple.Direction.REVERSE);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {

        LF.setPower(gamepad1.dpad_up ? 0.2 : 0); // up= Left Front 2
        LR.setPower(gamepad1.dpad_down ? 0.2 : 0); // down= Left Rear 3
        RF.setPower(gamepad1.dpad_right ? 0.2 : 0); // right= Right Front 1
        RR.setPower(gamepad1.dpad_left ? 0.2 : 0); // left= Right Rear 0
    }
}
