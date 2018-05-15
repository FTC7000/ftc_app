package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Teleop", group = "java")
public class TeleOpMode extends OpMode {

    private CRServo armMoveServo_;
    private Servo armTiltLeftServo_;
    private Servo armTltRightServo_;

    private double armTiltPos_ = 0;

    private DcMotor leftFrontMotor_;
    private DcMotor leftBackMotor_;
    private DcMotor rightFrontMotor_;
    private DcMotor rightBackMotor_;

    private Servo clawLeftServo_;
    private Servo clawRightServo_;


    @Override
    public void init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        // Arm
        armMoveServo_ = hardwareMap.get(CRServo.class, "servo5");
        armTiltLeftServo_ = hardwareMap.get(Servo.class, "servo1");
        armTltRightServo_ = hardwareMap.get(Servo.class, "servo2");

        // Set the direction of the motor, such that a positive value moves the arm forward
        armMoveServo_.setDirection(DcMotorSimple.Direction.FORWARD);

        // One of the tilting servo's is inverted to the other
        armTiltLeftServo_.setDirection(Servo.Direction.FORWARD);
        armTltRightServo_.setDirection(Servo.Direction.REVERSE);

        // Drive
        leftFrontMotor_ = hardwareMap.get(DcMotor.class, "leftFrontMotor1");
        leftBackMotor_ = hardwareMap.get(DcMotor.class, "leftBackMotor3");
        rightFrontMotor_ = hardwareMap.get(DcMotor.class, "rightFrontMotor0");
        rightBackMotor_ = hardwareMap.get(DcMotor.class, "rightBackMotor2");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFrontMotor_.setDirection(DcMotor.Direction.FORWARD);
        leftBackMotor_.setDirection(DcMotor.Direction.FORWARD);
        rightFrontMotor_.setDirection(DcMotor.Direction.REVERSE);
        rightBackMotor_.setDirection(DcMotor.Direction.REVERSE);

        // Claw
        clawLeftServo_ = hardwareMap.get(Servo.class, "servo3");
        clawRightServo_ = hardwareMap.get(Servo.class, "servo4");

        // One side should be inverted because each side is the mirror of the other
        clawLeftServo_.setDirection(Servo.Direction.FORWARD);
        clawRightServo_.setDirection(Servo.Direction.FORWARD);

    }

    @Override
    public void loop() {
        // Update drive subsystem
        double strafe = 0;
        if(gamepad1.left_trigger > 0) {
            strafe = gamepad1.left_trigger;
        } else if(gamepad1.right_trigger > 0) {
            strafe = -gamepad1.right_trigger;
        }
        driveCartesian(gamepad1.left_stick_y, strafe, gamepad1.left_stick_x);

        // Update claw subsystem
        if(gamepad2.x) {
            closeClaw();
        } else if(gamepad2.y) {
            openClaw();
        }

        // Update the Arm Subsystem
        // Move arm back and forward
        if(gamepad2.left_bumper) {
            moveArm(0.5);
        } else if(gamepad2.right_bumper) {
            moveArm(-0.5);
        } else {
            // Stop movement, when no button is pressed
            moveArm(0.0);
        }

        // Tilt the arm up and down
        if(Math.abs(gamepad2.right_stick_y) > 0.1) {
            armTiltPos_ += gamepad2.right_stick_y * 0.05;

            armTiltPos_ = Range.clip(armTiltPos_, 0, 1);

            setArmTiltPosition(armTiltPos_);
        }
    }

    /**
     * Set the move speed of the arm.
     * This moves it back and forward
     * @param speed speed > 0 Move forward, speed < 0 move backward
     */
    public void moveArm(double speed) {
        armMoveServo_.setPower(speed);
    }

    /**
     * Set the tilt servo position
     * @param pos position
     */
    private void setArmTiltPosition(double pos) {
        armTiltLeftServo_.setPosition(pos);
        armTltRightServo_.setPosition(pos);
    }


    public void driveCartesian(double ySpeed, double xSpeed, double zRotation) {
        ySpeed = Range.clip(ySpeed, -1.0, 1.0);

        xSpeed = Range.clip(ySpeed, -1.0, 1.0);

        double lf =  xSpeed + ySpeed + zRotation;
        double rf =  xSpeed - ySpeed + zRotation;
        double lb = -xSpeed + ySpeed + zRotation;
        double rb = -xSpeed - ySpeed + zRotation;

        leftFrontMotor_.setPower(lf);
        rightFrontMotor_.setPower(rf);
        leftBackMotor_.setPower(lb);
        rightBackMotor_.setPower(rb);
    }

    /**
     * Move the claw to the open position
     */
    public void openClaw() {
        clawLeftServo_.setPosition(0.6);
        clawRightServo_.setPosition(0.6);
    }

    /**
     * Move the claw to the close position
     */
    public void closeClaw() {
        clawLeftServo_.setPosition(0.4);
        clawRightServo_.setPosition(0.4);
    }

}
