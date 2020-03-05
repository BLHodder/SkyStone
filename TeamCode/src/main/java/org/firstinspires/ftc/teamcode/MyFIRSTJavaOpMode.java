package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.android.AndroidAccelerometer;
import org.firstinspires.ftc.robotcore.external.android.AndroidGyroscope;
import org.firstinspires.ftc.robotcore.external.android.AndroidOrientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 *  Created by ben on 27/02/202
 */

@TeleOp
public class MyFIRSTJavaOpMode extends LinearOpMode {
    private AndroidGyroscope gyroSensor;
    private AndroidOrientation orientSensor;
    private AndroidAccelerometer accelSensor;
    private DcMotor motor1;
    private DcMotor motor2;
    private DcMotor motor3;
    private DcMotor motor4;
    private Servo servoTest;

    @Override
    public void runOpMode () {

        this.gyroSensor = hardwareMap.get(AndroidGyroscope.class, "gyroSensor");
        orientSensor = hardwareMap.get(AndroidOrientation.class, "orientSensor");
        accelSensor = hardwareMap.get(AndroidAccelerometer.class, "accelSensor");

        motor1 = hardwareMap.get(DcMotor.class, "motorTest");
        motor2 = hardwareMap.get(DcMotor.class, "motorTest");
        motor3 = hardwareMap.get(DcMotor.class, "motorTest");
        motor4 = hardwareMap.get(DcMotor.class, "motorTest");
        Servo servoTest = hardwareMap.get(Servo.class, "servoTest");

        this.telemetry.addData("Status", "Running");
        this.telemetry.update();

        float joystickDeadzone = 0.2f;

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // Wait for the game to start (drive presses PLAY)
        waitForStart();


        while (opModeIsActive())  {
            telemetry.addData("Status", "Running");

            double tgtPower1 = this.gamepad1.left_stick_y;
            double tgtPower2 = this.gamepad1.left_stick_x;
            motor1.setPower(tgtPower1);
            telemetry.addData("Motor Power", "Motor1 set to " + tgtPower1);
            motor2.setPower(tgtPower2);
            telemetry.addData("Motor Power", "Motor2 set to " +tgtPower2);

            if (this.gamepad1.a || this.gamepad2.a) {
                telemetry.addData("Motor Power", "Motor3 set to 0");
                motor4.setPower(-1);
            }else if (this.gamepad1.left_bumper) {
                   motor4.setPower(0.5);
                }else {
                motor4.setPower(0);
            }
            telemetry.addData("Motor Power", "Motor4 set to " + motor4.getPower());

            if (gamepad1.y) {
                servoTest.setPosition(0d);
            } else if (gamepad1.x || gamepad1.b) {
                servoTest.setPosition(0.5);
            } else if (gamepad1.a) {
                 servoTest.setPosition(1);
            }
            telemetry.addData("Servo Position", servoTest.getPosition());

            AngleUnit unit = gyroSensor.getAngleUnit();
            float gyroX = unit.toDegrees(this.gyroSensor.getX());
            float gyroY = unit.toDegrees(this.gyroSensor.getY());
            float gyroZ = unit.toDegrees(this.gyroSensor.getZ());

            telemetry.addData("X velocity", gyroX);
            telemetry.addData("Y velocity", gyroY);
            telemetry.addData("Z velocity", gyroZ);




        telemetry.update();

        }
    }
}