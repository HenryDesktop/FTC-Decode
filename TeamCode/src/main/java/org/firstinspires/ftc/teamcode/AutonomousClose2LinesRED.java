
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
public class AutonomousClose2LinesRED extends LinearOpMode {
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
            reverseClassifier(46, 1);
            CorrectPosition();
            shootClassifier();
            degreesFirstLine();
            fowardFirstLine(32, 0.4);
            reverseFirstLine(30, 1);
            GyroClassify1();
            CorrectPosition();
            shootClassifier();
            degreesFirstLine();
            secondLineGo(28, 1);
            degreesSecondLine();
            fowardSecondLine(42, 0.4);
            goToShootB(20, 0.5);
            GyroClassify1();
            shootClassifier();
            stopMotors();

        }
    }

    public void reverseClassifier(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(TICKSDISTANCE);
        m_fr.setTargetPosition(TICKSDISTANCE);
        m_bl.setTargetPosition(TICKSDISTANCE);
        m_br.setTargetPosition(TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        m_leftshooter.setVelocity(-DesearedRPMshort);
        m_rightshooter.setVelocity(DesearedRPMshort);
        m_fl.setPower(power);
        m_fr.setPower(power);
        m_bl.setPower(power);
        m_br.setPower(power);


        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy() || m_br.isBusy() || m_bl.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.addData("BL Position:", m_bl.getCurrentPosition());
            telemetry.addData("BR Position:", m_br.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }

    public void CorrectPosition() {
        double heading = imu.getHeading(AngleUnit.DEGREES);
        double power = 0.4;
        double deadband = 1;

        m_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && (heading = imu.getHeading(AngleUnit.DEGREES)) > deadband) {
            // Girar hacia la IZQUIERDA
            m_fl.setPower(-power);
            m_fr.setPower(power);
            m_bl.setPower(-power);
            m_br.setPower(power);

            idle();
        }

        while (opModeIsActive() && (heading = imu.getHeading(AngleUnit.DEGREES)) < -deadband) {
            // Girar hacia la DERECHA
            m_fl.setPower(power);
            m_fr.setPower(-power);
            m_bl.setPower(power);
            m_br.setPower(-power);

            idle();
        }

        stopMotors();

    }

    public void shootClassifier(){
        ElapsedTime time = new ElapsedTime();
        time.reset();
        while (opModeIsActive() && time.seconds() <= 5.8) {
            telemetry.addData("Actual time:", time.seconds());
            m_leftshooter.setVelocity(-DesearedRPMshort);
            m_rightshooter.setVelocity(DesearedRPMshort);

            //----------Intake----------
            if (m_rightshooter.getVelocity() >= DesearedRPMshort){
                m_intake.setPower(.7);
            }
            else {
                m_intake.setPower(0);
            }

            //----------Servo----------
            if (m_rightshooter.getVelocity() >=DesearedRPMshort && shootdistance.getDistance()>=6.5){
                s_midintake.setPosition(0.7);
            }
            else{
                s_midintake.setPosition(.3);
            }

        }

        stopMotors();
    }

    public void degreesFirstLine() {

        while (opModeIsActive() && imu.getHeading(AngleUnit.DEGREES) >= -39) {
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

    public void fowardFirstLine(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(-TICKSDISTANCE);
        m_fr.setTargetPosition(-TICKSDISTANCE);
        m_bl.setTargetPosition(-TICKSDISTANCE);
        m_br.setTargetPosition(-TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        m_fl.setPower(power);
        m_fr.setPower(power);
        m_bl.setPower(power);
        m_br.setPower(power);
        m_intake.setPower(.8);

        if (m_intake.getPower() >0){
            s_midintake.setPosition(0.3);
        }


        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy() || m_br.isBusy() || m_bl.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.addData("BL Position:", m_bl.getCurrentPosition());
            telemetry.addData("BR Position:", m_br.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
    public void reverseFirstLine(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);


        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(TICKSDISTANCE);
        m_fr.setTargetPosition(TICKSDISTANCE);
        m_bl.setTargetPosition(TICKSDISTANCE);
        m_br.setTargetPosition(TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        m_fl.setPower(power);
        m_fr.setPower(power);
        m_bl.setPower(power);
        m_br.setPower(power);


        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy() || m_br.isBusy() || m_bl.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.addData("BL Position:", m_bl.getCurrentPosition());
            telemetry.addData("BR Position:", m_br.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
    public void GyroClassify1() {


        m_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive() && imu.getHeading(AngleUnit.DEGREES) <= 0) {
            m_leftshooter.setVelocity(-DesearedRPMshort);
            m_rightshooter.setVelocity(DesearedRPMshort);

            m_bl.setPower(.7);
            m_br.setPower(-.8);
            m_fl.setPower(.7);
            m_fr.setPower(-.8);

            telemetry.addData("Current Orientation:", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            idle();
        }

        stopMotors();
    }
    public void GyroClassifyReverse() {

        m_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (opModeIsActive() && imu.getHeading(AngleUnit.DEGREES) >=5) {
            m_bl.setPower(-.8);
            m_br.setPower(.8);
            m_fl.setPower(-.8);
            m_fr.setPower(.8);

            telemetry.addData("Current Orientation:", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            idle();
        }

        stopMotors();
    }

    public void secondLineGo(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(TICKSDISTANCE);
        m_fr.setTargetPosition(-TICKSDISTANCE);
        m_bl.setTargetPosition(-TICKSDISTANCE);
        m_br.setTargetPosition(TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        m_fl.setPower(-power);
        m_fr.setPower(power);
        m_bl.setPower(power);
        m_br.setPower(-power);


        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy() || m_br.isBusy() || m_bl.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.addData("BL Position:", m_bl.getCurrentPosition());
            telemetry.addData("BR Position:", m_br.getCurrentPosition());
            telemetry.update();
            idle();
        }
    }
    public void degreesSecondLine() {


        m_fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m_br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && imu.getHeading(AngleUnit.DEGREES) >= -42) {
            m_bl.setPower(-.7);
            m_br.setPower(.8);
            m_fl.setPower(-.7);
            m_fr.setPower(.8);

            telemetry.addData("Current Orientation:", imu.getHeading(AngleUnit.DEGREES));
            telemetry.update();
            idle();
        }

        stopMotors();
    }

    public void fowardSecondLine(double inches, double power) {
        int TICKSDISTANCE = (int) (inches * TICKSRPM);

        m_fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m_bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        m_fl.setTargetPosition(-TICKSDISTANCE);
        m_fr.setTargetPosition(-TICKSDISTANCE);
        m_bl.setTargetPosition(-TICKSDISTANCE);
        m_br.setTargetPosition(-TICKSDISTANCE);

        m_fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        m_bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        m_fl.setPower(power);
        m_fr.setPower(power);
        m_bl.setPower(power);
        m_br.setPower(power);
        m_intake.setPower(.8);

        while (opModeIsActive() && (m_fl.isBusy() || m_fr.isBusy() || m_br.isBusy() || m_bl.isBusy())) {
            telemetry.addData("Target:", TICKSDISTANCE);
            telemetry.addData("FL Position:", m_fl.getCurrentPosition());
            telemetry.addData("FR Position:", m_fr.getCurrentPosition());
            telemetry.addData("BL Position:", m_bl.getCurrentPosition());
            telemetry.addData("BR Position:", m_br.getCurrentPosition());
            telemetry.update();
            idle();
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

        m_fr.setTargetPosition(TICKSDISTANCE);
        m_bl.setTargetPosition(-TICKSDISTANCE);

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

    public void servoActive(){
        s_midintake.setPosition(.3);
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
