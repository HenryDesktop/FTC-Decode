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
public class AutonomousFar1Line extends LinearOpMode {

    DcMotor m_fr, m_fl, m_br, m_bl;
    DcMotorEx m_intake, m_leftshooter, m_rightshooter;
    CRServo s_midintake;
    ConfigureDistance distancesensor = new ConfigureDistance();
    ConfigureIMU imu = new ConfigureIMU();

    @Override
    public void runOpMode() throws InterruptedException {

        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_bl = hardwareMap.get(DcMotorEx.class, "BLMotor");
        m_br = hardwareMap.get(DcMotorEx.class, "BRMotor");
        m_intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        m_leftshooter= hardwareMap.get(DcMotorEx.class, "ShooterMotorA");
        m_rightshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorB");
        s_midintake = hardwareMap.get(CRServo.class, "Servo");
        imu.init(hardwareMap);
        distancesensor.init(hardwareMap);

        m_fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_bl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_br.setDirection(DcMotorSimple.Direction.REVERSE);
        m_fl.setDirection(DcMotorSimple.Direction.FORWARD);

        m_rightshooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        m_rightshooter.setVelocityPIDFCoefficients(60,0.0001,30,.0005);

        waitForStart();

        if (opModeIsActive()) {

            // -------------------- turnHeadingtoscore --------------------

            double angle = imu.getHeading(AngleUnit.DEGREES);
            double distance = distancesensor.getDistance();

            telemetry.addData("Current Orientation:", angle);
            telemetry.addData("Distance:", distance);
            telemetry.update();

            while (opModeIsActive() && angle >= -14) {

                s_midintake.setPower(0);
                m_bl.setPower(-.5);
                m_br.setPower(.5);
                m_fl.setPower(-.5);
                m_fr.setPower(.5);

                idle();
            }

            stopMotors();

            // -------------------- shootArtifact --------------------

            double DesearedRPMLong = 1140;

            ElapsedTime time = new ElapsedTime();
            time.reset();
            while (opModeIsActive() && time.seconds() <= 5) {
                telemetry.addData("Actual time:", time.seconds());
                m_leftshooter.setVelocity(-DesearedRPMLong);
                m_rightshooter.setVelocity(DesearedRPMLong);
                m_intake.setPower(1);
                s_midintake.setPower(1);
                idle();
            }
            stopMotors();

            // -------------------- turnHeadingCentral --------------------

            while (opModeIsActive() && angle <= -2) {

                s_midintake.setPower(0);
                m_bl.setPower(.5);
                m_br.setPower(-.5);
                m_fl.setPower(.5);
                m_fr.setPower(-.5);

                idle();
            }

            stopMotors();

            // -------------------- straightFowardLA --------------------

            time = new ElapsedTime();
            time.reset();
            while (opModeIsActive() && time.seconds() <= 2) {
                telemetry.addData("Time:", time.seconds());
                telemetry.update();

                m_bl.setPower(-.3);
                m_br.setPower(-.3);
                m_fl.setPower(-.3);
                m_fr.setPower(-.3);
                m_intake.setPower(1);

                idle();
            }

            stopMotors();

            // -------------------- turnHeadingtoright --------------------

            while (opModeIsActive() && angle >= -87) {

                s_midintake.setPower(0);
                m_bl.setPower(-.5);
                m_br.setPower(.5);
                m_fl.setPower(-.5);
                m_fr.setPower(.5);

                idle();
            }

            stopMotors();

            // -------------------- straightFowardtorecolect --------------------

            while (opModeIsActive() && distance >= 20) {
                distance = distancesensor.getDistance();
                telemetry.addData("Distance:", distance);
                telemetry.update();

                m_bl.setPower(-.2);
                m_br.setPower(-.2);
                m_fl.setPower(-.2);
                m_fr.setPower(-.2);
                m_intake.setPower(1);

                idle();
            }

            stopMotors();

            // -------------------- reverseToShootA --------------------

            time = new ElapsedTime();
            time.reset();

            while (opModeIsActive() && time.seconds() <= 2.0)  {
                distance = distancesensor.getDistance();
                m_bl.setPower(.5);
                m_br.setPower(.5);
                m_fl.setPower(.5);
                m_fr.setPower(.5);
                m_leftshooter.setVelocity(-DesearedRPMLong);
                m_rightshooter.setVelocity(DesearedRPMLong);

                idle();
            }

            // -------------------- turnHeadingCentralAgain --------------------


            while (opModeIsActive() && angle <= -2) {

                s_midintake.setPower(0);
                m_bl.setPower(.5);
                m_br.setPower(-.5);
                m_fl.setPower(.5);
                m_fr.setPower(-.5);

                idle();
            }

            stopMotors();

            // -------------------- straightReverseL --------------------

            time = new ElapsedTime();
            time.reset();
            while (opModeIsActive() && time.seconds() <= 1.0) {
                telemetry.addData("Time:", time.seconds());
                telemetry.update();

                m_bl.setPower(.6);
                m_br.setPower(.6);
                m_fl.setPower(.6);
                m_fr.setPower(.6);

                idle();
            }

            stopMotors();

            // -------------------- turnHeadingtoscore --------------------


            while (opModeIsActive() && angle >= -14) {

                s_midintake.setPower(0);
                m_bl.setPower(-.5);
                m_br.setPower(.5);
                m_fl.setPower(-.5);
                m_fr.setPower(.5);

                idle();
            }

            stopMotors();

            //------------------------------------
            // shootArtifactF(1)
            //------------------------------------

            time = new ElapsedTime();
            time.reset();
            while (opModeIsActive() && time.seconds() <= 5) {
                telemetry.addData("Actual time:", time.seconds());
                m_leftshooter.setVelocity(-DesearedRPMLong);
                m_rightshooter.setVelocity(DesearedRPMLong);
                m_intake.setPower(1);
                s_midintake.setPower(1);
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
        m_leftshooter.setPower(0);
        m_rightshooter.setPower(0);
        m_intake.setPower(0);
    }
}
