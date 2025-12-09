
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous
public class AutonomousClose2LinesTEST extends LinearOpMode {
    DcMotorEx m_fl, m_fr, m_bl, m_br;
    DcMotorEx m_intake;
    DcMotorEx m_leftshooter;
    DcMotorEx m_rightshooter;
    Servo s_midintake;
    ConfigureIMU imu = new ConfigureIMU();
    ConfigureDistance distance = new ConfigureDistance();
    ConfigureColor shootdistance = new ConfigureColor();
    double TICKSRPM = 42.8;
    double DesearedRPMshort = 1000;


    @Override
    public void runOpMode() throws InterruptedException {

        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_bl = hardwareMap.get(DcMotorEx.class, "BLMotor");
        m_br = hardwareMap.get(DcMotorEx.class, "BRMotor");

        m_intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        m_leftshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorA");
        m_rightshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorB");

        s_midintake = hardwareMap.get(Servo.class, "Servo");
        imu.init(hardwareMap);
        distance.init(hardwareMap);
        shootdistance.init(hardwareMap);

        m_fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_bl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_br.setDirection(DcMotorSimple.Direction.REVERSE);
        m_fl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_intake.setDirection(DcMotorSimple.Direction.REVERSE);

        m_fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        m_br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        m_fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_rightshooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        m_rightshooter.setVelocityPIDFCoefficients(60, 0.0001, 30, 0.0005);

        waitForStart();

        if (opModeIsActive()) {
            resetIMU();
            goToShootB(20, 0.2);

        }
    }

    public void goToShootB(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        m_fr.setTargetPosition(-TICKSDISTANCE);
        m_bl.setTargetPosition(TICKSDISTANCE);

        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m_fl.setPower(0);
        m_fr.setPower(power);
        m_br.setPower(0);
        m_bl.setPower(power);

        while (opModeIsActive() && (m_fr.isBusy() || m_bl.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.addData("BL Position:", m_bl.getCurrentPosition());
            telemetry.addData("BR Position:", m_br.getCurrentPosition());
            telemetry.update();
            idle();
        }
        stopMotors();

    }

    public void resetIMU(){
        imu.resetImu();
    }

    public void stopMotors() {
        m_fl.setPower(0);
        m_fr.setPower(0);
        m_bl.setPower(0);
        m_br.setPower(0);
        m_leftshooter.setPower(0);
        m_rightshooter.setPower(0);
        m_intake.setPower(0);
        s_midintake.setPosition(.3);

        m_fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
