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
    DcMotorEx m_bl;
    DcMotorEx m_br;
    DcMotorEx m_leftshooter;
    DcMotorEx m_rightshooter;
    Servo s_servo;
    ConfigureColor distance = new ConfigureColor();
    @Override
    public void init() {
        IMU.init(hardwareMap);
        m_leftshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorA");
        m_rightshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorB");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        m_br = hardwareMap.get(DcMotorEx.class, "BRMotor");
        m_bl = hardwareMap.get(DcMotorEx.class, "BLMotor");
        s_servo = hardwareMap.get(Servo.class, "Servo");
        distance.init(hardwareMap);

        m_rightshooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        m_fr.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_fl.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fr.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_fl.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        m_rightshooter.setVelocityPIDFCoefficients(60,0.0001,30,.0005);
    }

    @Override
    public void loop() {
        telemetry.addData("Orientation:", IMU.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Shooter Velocity:", m_rightshooter.getVelocity());
        telemetry.addData("Distance:", distance.getDistance());
        telemetry.addData("FL Position:", m_fl.getCurrentPosition());
        telemetry.addData("BL Position:", m_bl.getCurrentPosition());
        telemetry.addData("FR Position:", m_fr.getCurrentPosition());
        telemetry.addData("BR Position:", m_br.getCurrentPosition());
        telemetry.update();


        if (gamepad1.back)
            IMU.resetImu();
        if (distance.getDistance() <=1){
            s_servo.setPosition(0.7);
        }
        else {
            s_servo.setPosition(0.3);
        }
    }
}
