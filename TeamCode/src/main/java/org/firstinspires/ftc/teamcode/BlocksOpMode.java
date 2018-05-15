package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Blocks", group = "java")
public class BlocksOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor rightFrontMotor0 = hardwareMap.get(DcMotor.class, "rightFrontMotor0");
        DcMotor leftFrontMotor1 = hardwareMap.get(DcMotor.class, "leftFrontMotor1");;
        DcMotor rightBackMotor2 = hardwareMap.get(DcMotor.class, "rightBackMotor2");;
        DcMotor leftBackMotor3 = hardwareMap.get(DcMotor.class, "leftBackMotor3");;

        Servo servo1 = hardwareMap.get(Servo.class, "servo1");
        Servo servo2 = hardwareMap.get(Servo.class, "servo2");
        Servo servo3 = hardwareMap.get(Servo.class, "servo3");
        Servo servo4 = hardwareMap.get(Servo.class, "servo4");
        CRServo servo5 = hardwareMap.get(CRServo.class, "servo5");

        double leftStickY = 0;
        double leftStickY2 = 0;

        waitForStart();


        while(opModeIsActive()) {
            // Drive logic
            // LT/RT Move sideways
            double lt = gamepad1.left_trigger;
            double rt = gamepad1.right_trigger;

            telemetry.addData("LT", lt);
            telemetry.addData("RT", rt);

            telemetry.update();

            if(lt > 0.01) {
                // Sideways left
                rightFrontMotor0.setPower(gamepad1.left_trigger);
                leftFrontMotor1.setPower(gamepad1.left_trigger);
                rightBackMotor2.setPower(-gamepad1.left_trigger);
                leftBackMotor3.setPower(-gamepad1.left_trigger);
            } else if(rt > 0.01) {
                // Sideways right
                rightFrontMotor0.setPower(-gamepad1.right_trigger);
                leftFrontMotor1.setPower(-gamepad1.right_trigger);
                rightBackMotor2.setPower(gamepad1.right_trigger);
                leftBackMotor3.setPower(gamepad1.right_trigger);
            } else {
                rightFrontMotor0.setPower((-gamepad1.right_stick_x - gamepad1.right_stick_y)/2.0);
                leftFrontMotor1.setPower((-gamepad1.right_stick_x - gamepad1.right_stick_y)/2.0);
                rightBackMotor2.setPower((-gamepad1.right_stick_x + gamepad1.right_stick_y)/2.0);
                leftBackMotor3.setPower((-gamepad1.right_stick_x + gamepad1.right_stick_y)/2.0);
            }

            if((gamepad2.right_stick_y > 0.1) || (gamepad2.right_stick_y < -0.1)) {
                servo3.setPosition(leftStickY);
                servo4.setPosition(leftStickY2);

                leftStickY += gamepad2.right_stick_y * 0.05;
                if(leftStickY < 0) {
                    leftStickY = 0;
                }
                if(leftStickY > 1) {
                    leftStickY =  1;
                }

                leftStickY2 -= gamepad2.right_stick_y * 0.05;
                if(leftStickY2 < 0) {
                    leftStickY2 = 0;
                }
                if(leftStickY2 > 1) {
                    leftStickY2 =  1;
                }

                telemetry.addData("var", leftStickY);
                telemetry.addData("var2", leftStickY2);

                telemetry.addData("ser0", servo3.getPosition());
                telemetry.addData("ser1",servo4.getPosition());
                telemetry.update();
            }

            if(gamepad2.x) {
                servo1.setPosition(0.35);
                servo2.setPosition(0.66);
            } else if(gamepad2.y) {
                servo1.setPosition(0.6);
                servo2.setPosition(0.4);

            } else {

            }

            if(gamepad2.left_bumper) {
                servo5.setDirection(DcMotorSimple.Direction.REVERSE);
                servo5.setPower(0.5);
            } else if(gamepad2.right_bumper) {
                servo5.setDirection(DcMotorSimple.Direction.FORWARD);
                servo5.setPower(0.5);
            } else {
                servo5.setPower(0);
            }

            telemetry.update();
        }

    }

}
