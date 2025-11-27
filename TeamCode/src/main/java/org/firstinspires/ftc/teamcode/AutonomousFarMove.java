package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class AutonomousFarMove extends LinearOpMode {

    DcMotor m_fr, m_fl, m_br, m_bl;

    @Override
    public void runOpMode() throws InterruptedException {

        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_bl = hardwareMap.get(DcMotorEx.class, "BLMotor");
        m_br = hardwareMap.get(DcMotorEx.class, "BRMotor");

        m_fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_bl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_br.setDirection(DcMotorSimple.Direction.REVERSE);
        m_fl.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        if (opModeIsActive()) {

            ElapsedTime time = new ElapsedTime();


            // -------------------- straightFowardLA --------------------

            time = new ElapsedTime();
            time.reset();
            while (opModeIsActive() && time.seconds() <= 0.8) {
                telemetry.addData("Time:", time.seconds());
                telemetry.update();

                m_bl.setPower(-.6);
                m_br.setPower(-.6);
                m_fl.setPower(-.6);
                m_fr.setPower(-.6);

                idle();
            }

            stopMotors();


        }
    }

    public void stopMotors() {
        m_fl.setPower(0);
        m_fr.setPower(0);
        m_bl.setPower(0);
        m_br.setPower(0);
    }
}