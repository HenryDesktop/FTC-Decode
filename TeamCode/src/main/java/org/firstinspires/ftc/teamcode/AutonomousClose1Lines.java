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
public class AutonomousClose1Lines extends LinearOpMode {

    //---------------------------A-U-T-O-C-L-O-S-E--------------------------


    //---------------------------C-h-a-s-i-s--------------------------

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

            runAutonomous();

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
    public void runAutonomous() throws InterruptedException {

        // -------------------- straightReverseA1 --------------------
        double DesearedRPMshort = 900;
        double distance = distancesensor.getDistance();
        double angle = imu.getHeading(AngleUnit.DEGREES);
        ElapsedTime time = new ElapsedTime();
        imu.resetImu();



        while (opModeIsActive() && time.seconds() <= 1) { //0.95
            telemetry.addData("Time:", time.seconds());
            telemetry.update();

            m_bl.setPower(-.8);
            m_br.setPower(-.8);
            m_fl.setPower(-.8);
            m_fr.setPower(-.8);


            idle();
        }

        stopMotors();


        // -------------------- turnHeadingtoscore --------------------
        while (opModeIsActive() && angle <= 164) {
            s_midintake.setPower(0);
            m_bl.setPower(.8);
            m_br.setPower(-.8);
            m_fl.setPower(.8);
            m_fr.setPower(-.8);
            m_leftshooter.setVelocity(-DesearedRPMshort);
            m_rightshooter.setVelocity(DesearedRPMshort);

            angle = imu.getHeading(AngleUnit.DEGREES);
            telemetry.addData("Current Orientation:", angle);
            telemetry.update();

            idle();
        }
        stopMotors();

        // -------------------- shootArtifactA1 --------------------
        time = new ElapsedTime();
        time.reset();

        while (opModeIsActive() && time.seconds() <= 5) {
            telemetry.addData("Actual time:", time.seconds());
            m_leftshooter.setVelocity(-DesearedRPMshort);
            m_rightshooter.setVelocity(DesearedRPMshort);
            m_intake.setPower(1);
            s_midintake.setPower(1);
            telemetry.addData("Intake Velocity:",m_intake.getVelocity());
            idle();
        }
        imu.resetImu();
        stopMotors();

        // -------------------- turnHeadingNegA1 --------------------
        while (opModeIsActive() && angle >= -40) {
            s_midintake.setPower(0);
            m_bl.setPower(-.8);
            m_br.setPower(.8);
            m_fl.setPower(-.8);
            m_fr.setPower(.8);

            angle = imu.getHeading(AngleUnit.DEGREES);
            telemetry.addData("Current Orientation:", angle);
            telemetry.update();

            idle();
        }
        stopMotors();

        // -------------------- straightFowardA1 --------------------
        time = new ElapsedTime();
        time.reset();
        //distance = distancesensor.getDistance();
        while (opModeIsActive() && time.seconds() <= 3.9) {
            /* distance = distancesensor.getDistance();
            telemetry.addData("Distance:", distance);
            telemetry.update(); */

            m_bl.setPower(-.2);
            m_br.setPower(-.2);
            m_fl.setPower(-.2);
            m_fr.setPower(-.2);
            m_intake.setPower(1);
            telemetry.addData("Intake Velocity:",m_intake.getVelocity());

            idle();
        }
        stopMotors();

        // -------------------- straightReverseA2 --------------------
        time = new ElapsedTime();
        time.reset();

        while (opModeIsActive() && time.seconds() <= 0.85) {
            m_bl.setPower(.8);
            m_br.setPower(.8);
            m_fl.setPower(.8);
            m_fr.setPower(.8);
            idle();
        }

        stopMotors();
        //imu.resetImu();

        // -------------------- turnHeadingPosA --------------------
        angle = imu.getHeading(AngleUnit.DEGREES);
        while (opModeIsActive() && angle <= -20) {
            angle = imu.getHeading(AngleUnit.DEGREES);
            telemetry.addData("Current Orientation:", angle);
            telemetry.update();

            m_bl.setPower(.4);
            m_br.setPower(-.4);
            m_fl.setPower(.4);
            m_fr.setPower(-.4);

            idle();
        }
        stopMotors();

        // -------------------- shootArtifactA2 --------------------
        time = new ElapsedTime();
        time.reset();
        while (opModeIsActive() && time.seconds() <= 5) {
            telemetry.addData("Actual time:", time.seconds());

            m_leftshooter.setVelocity(-DesearedRPMshort);
            m_rightshooter.setVelocity(DesearedRPMshort);
            m_intake.setPower(1);
            s_midintake.setPower(1);
            idle();
        }
        stopMotors();

    }


}
