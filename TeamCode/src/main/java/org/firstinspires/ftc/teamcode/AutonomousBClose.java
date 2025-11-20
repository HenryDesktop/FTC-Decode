package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "AutonomousBClose")
public class AutonomousBClose extends LinearOpMode {

    //---------------------------A-U-T-O-B--------------------------





    //---------------------------C-h-a-s-i-s--------------------------

    DcMotor m_fr, m_fl, m_br, m_bl;
    DcMotorEx m_intake, m_shooter1, m_shooter2;
    ConfigureIMU bench = new ConfigureIMU();
    double angle = bench.getHeading(AngleUnit.DEGREES);





    //---------------------------G-l-o-b-a-l--------------------------

    double TICKS_PER_INCH = 42.8;
    double DesearedRPMshort = 530;



    @Override
    public void runOpMode() throws InterruptedException {

        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_bl = hardwareMap.get(DcMotorEx.class, "BLMotor");
        m_br = hardwareMap.get(DcMotorEx.class, "BRMotor");
        m_intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");
        m_shooter1 = hardwareMap.get(DcMotorEx.class, "ShooterMotorA");
        m_shooter2 = hardwareMap.get(DcMotorEx.class, "ShooterMotorB");
        bench.init(hardwareMap);

        m_fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_bl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_br.setDirection(DcMotorSimple.Direction.REVERSE);
        m_fl.setDirection(DcMotorSimple.Direction.FORWARD);

        m_fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        m_shooter2.setVelocityPIDFCoefficients(20,0,5,13.5);

        waitForStart();

        if (opModeIsActive()) {
            straightFowardA(20, 1);
            turnHeadingFoward(1);
            shootArtifact(1);
        }
    }

    public void straightFowardA(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKS_PER_INCH);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(TICKSDISTANCE);
        m_fr.setTargetPosition(TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m_fl.setPower(-power);
        m_fr.setPower(-power);
        m_bl.setPower(-power);
        m_br.setPower(-power);

        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.update();
            idle();
        }

        stopMotors();
        sleep(500);
    }

    public void turnHeadingFoward(double power) {
        while (opModeIsActive() && angle <=130) {
            m_fl.setPower(power);
            m_fr.setPower(-power);
            m_br.setPower(-power);
            m_bl.setPower(power);

            telemetry.addData("Current Orientation:", angle);
            telemetry.update();
            idle();
        }

        stopMotors();
        sleep(500);
    }

    public void shootArtifact(double power) {
        telemetry.addData("Actual time:", time);

        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (opModeIsActive() && time.seconds() <= 5) {
            m_shooter1.setVelocity(-DesearedRPMshort);
            m_shooter2.setVelocity(DesearedRPMshort);
            m_intake.setPower(power);
            idle();
        }
        stopMotors();
    }

    public void stopMotors() {
        m_fl.setPower(0);
        m_fr.setPower(0);
        m_bl.setPower(0);
        m_br.setPower(0);
        m_shooter1.setPower(0);
        m_shooter2.setPower(0);
        m_intake.setPower(0);
    }

}
