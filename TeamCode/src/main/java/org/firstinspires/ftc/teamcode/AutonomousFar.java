
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
public class AutonomousFar extends LinearOpMode {
    DcMotorEx m_fl, m_fr, m_bl, m_br;
    DcMotorEx m_intake;
    DcMotorEx m_leftshooter;
    DcMotorEx m_rightshooter;
    CRServo s_midintake;
    ConfigureIMU imu = new ConfigureIMU();
    double TICKSRPM = 42.8;

    @Override
    public void runOpMode() throws InterruptedException {

        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_bl = hardwareMap.get(DcMotorEx.class, "BLMotor");
        m_br = hardwareMap.get(DcMotorEx.class, "BRMotor");

        m_intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        m_leftshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorA");
        m_rightshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorB");

        s_midintake = hardwareMap.get(CRServo.class, "Servo");
        imu.init(hardwareMap);

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
        m_rightshooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        m_rightshooter.setVelocityPIDFCoefficients(60,0.0001,30,.0005);

        waitForStart();

        if (opModeIsActive()) {

            resetIMU();
            miniFoward(6, 0.4);
            degreesToClassify();
            shootClassifier();
            degreesCentral();
            fowardFirstLine(23, 0.4);
            degreesFirstLine();
            fowardFirstRecolect(43, 0.4);
            reverseFirstRecolect(35, 0.4);
            degreesStartCentral();
            reverseFirstLine(24, 0.4);
            degreesToClassify();
            shootClassifier();

        }
    }
    public void miniFoward(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(-TICKSDISTANCE);
        m_fr.setTargetPosition(-TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        m_fl.setPower(power);
        m_fr.setPower(power);
        m_bl.setPower(-power*.45);
        m_br.setPower(-power*.45);
        m_intake.setPower(1);

        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.update();
            idle();
        }
        //Micro frenado
        double brakePower;
        if (inches > 40) {
            brakePower = -0.15;
        } else {
            brakePower = -0.10;
        }
        m_fl.setPower(0);
        m_fr.setPower(0);
        m_bl.setPower(brakePower);
        m_br.setPower(brakePower);

        stopMotors();
    }

    public void shootClassifier(){
        double DesearedRPMshort = 1140;
        ElapsedTime time = new ElapsedTime();
        time.reset();

        while (opModeIsActive() && time.seconds() <= 9) {
            telemetry.addData("Actual time:", time.seconds());
            m_leftshooter.setVelocity(-DesearedRPMshort);
            m_rightshooter.setVelocity(DesearedRPMshort);

            while(opModeIsActive() && m_rightshooter.getVelocity() >=DesearedRPMshort){
                m_intake.setPower(.6);
                s_midintake.setPower(1);

            }
            m_intake.setPower(0);
            s_midintake.setPower(0);
        }
        stopMotors();
    }

    public void degreesToClassify() {

        while (opModeIsActive() && imu.getHeading(AngleUnit.DEGREES) >= -15) {
            s_midintake.setPower(0);
            m_bl.setPower(-.7);
            m_br.setPower(.8);
            m_fl.setPower(-.8);
            m_fr.setPower(.8);

            telemetry.addData("Current Orientation:", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            idle();
        }

        stopMotors();
    }
    public void degreesFirstLine() {
        while (opModeIsActive() && imu.getHeading(AngleUnit.DEGREES) >= -83) {
            s_midintake.setPower(0);
            m_bl.setPower(-.7);
            m_br.setPower(.8);
            m_fl.setPower(-.8);
            m_fr.setPower(.8);

            telemetry.addData("Current Orientation:", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            idle();
        }

        stopMotors();
    }

    public void fowardFirstRecolect(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(-TICKSDISTANCE);
        m_fr.setTargetPosition(-TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        m_fl.setPower(power);
        m_fr.setPower(power);
        m_bl.setPower(-power*.45);
        m_br.setPower(-power*.45);
        m_intake.setPower(1);

        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.update();
            idle();
        }
        //Micro frenado
        double brakePower;
        if (inches > 40) {
            brakePower = -0.15;
        } else {
            brakePower = -0.10;
        }
        m_fl.setPower(0);
        m_fr.setPower(0);
        m_bl.setPower(brakePower);
        m_br.setPower(brakePower);

        stopMotors();
    }
    public void fowardFirstLine(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(-TICKSDISTANCE);
        m_fr.setTargetPosition(-TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        m_fl.setPower(power);
        m_fr.setPower(power);
        m_bl.setPower(-power*.45);
        m_br.setPower(-power*.45);
        m_intake.setPower(1);

        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.update();
            idle();
        }
        //Micro frenado
        double brakePower;
        if (inches > 40) {
            brakePower = -0.15;
        } else {
            brakePower = -0.10;
        }
        m_fl.setPower(0);
        m_fr.setPower(0);
        m_bl.setPower(brakePower);
        m_br.setPower(brakePower);

        stopMotors();
    }
    public void reverseFirstRecolect(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(TICKSDISTANCE);
        m_fr.setTargetPosition(TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        m_fl.setPower(-power);
        m_fr.setPower(-power);
        m_bl.setPower(power * .45);
        m_br.setPower(power * .45);

        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }

    public void reverseFirstLine(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(TICKSDISTANCE);
        m_fr.setTargetPosition(TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        m_fl.setPower(-power);
        m_fr.setPower(-power);
        m_bl.setPower(power * .45);
        m_br.setPower(power * .45);

        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
    public void degreesCentral() {

        while (opModeIsActive() && imu.getHeading(AngleUnit.DEGREES) <=0) {
            s_midintake.setPower(0);
            m_bl.setPower(.7);
            m_br.setPower(-.8);
            m_fl.setPower(.8);
            m_fr.setPower(-.8);

            telemetry.addData("Current Orientation:", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            idle();
        }

        stopMotors();
    }
    public void degreesStartCentral() {
        m_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive() && imu.getHeading(AngleUnit.DEGREES) <=0) {
            s_midintake.setPower(0);
            m_bl.setPower(.7);
            m_br.setPower(-.8);
            m_fl.setPower(.8);
            m_fr.setPower(-.8);

            telemetry.addData("Current Orientation:", imu.getHeading(AngleUnit.DEGREES));
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

        m_fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


}
