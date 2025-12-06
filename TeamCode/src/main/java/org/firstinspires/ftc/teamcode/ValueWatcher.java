package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class ValueWatcher extends OpMode {
    ConfigureIMU IMU = new ConfigureIMU();
    DcMotorEx m_fl;
    DcMotorEx m_fr;
    DcMotorEx m_leftshooter;
    DcMotorEx m_rightshooter;
    CRServo s_servo;
    @Override
    public void init() {
        IMU.init(hardwareMap);
        m_leftshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorA");
        m_rightshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorB");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        s_servo = hardwareMap.get(CRServo.class, "Servo");

        s_servo.setDirection(CRServo.Direction.REVERSE);
        m_rightshooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        m_fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        m_fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        m_rightshooter.setVelocityPIDFCoefficients(60,0.0001,30,.0005);
    }

    @Override
    public void loop() {
        telemetry.addData("Orientation:", IMU.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Shooter Velocity:", m_rightshooter.getVelocity());
        telemetry.addData("FR Position:", m_fr.getCurrentPosition());
        telemetry.addData("FL Position:", m_fl.getCurrentPosition());

        if (gamepad1.back)
            IMU.resetImu();
        if (gamepad1.a){
            s_servo.setPower(1);
        }
        else {
            s_servo.setPower(0);
        }
    }
}
