package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "RobotCode2PostureCorrector")
public class RobotCode_20252PostureCorrector extends OpMode {
    //---------------------------C-h-a-s-i-s--------------------------
    DcMotorEx m_fl, m_fr, m_bl, m_br;
    DcMotorEx m_intake;
    DcMotorEx m_leftshooter;
    DcMotorEx m_rightshooter;
    Servo s_midintake;
    ConfigureIMU IMUHeading = new ConfigureIMU();
    ConfigureColor colorsensor = new ConfigureColor();
    ConfigureColorAlt poscorrector = new ConfigureColorAlt();



    //---------------------------G-l-o-b-a-l--------------------------

    double DesearedRPMlong = 1200 ;

    double actualVelocity;
    final double DesearedRPMshort = 1040; //960
    boolean isCorrector = false;


    @Override
    public void init() {

        //---------------------------C-h-a-s-i-s---I-n-i-t-i-a-l-i-z-e-----------------------

        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_bl = hardwareMap.get(DcMotorEx.class, "BLMotor");
        m_br = hardwareMap.get(DcMotorEx.class, "BRMotor");

        m_intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        m_leftshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorA");
        m_rightshooter = hardwareMap.get(DcMotorEx.class, "ShooterMotorB");

        s_midintake = hardwareMap.get(Servo.class, "Servo");

        IMUHeading.init(hardwareMap);
        colorsensor.init(hardwareMap);
        poscorrector.init(hardwareMap);

        m_rightshooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        m_rightshooter.setVelocityPIDFCoefficients(60,0.0001,30,.0005);

        m_fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_bl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_br.setDirection(DcMotorSimple.Direction.REVERSE);
        m_fl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_intake.setDirection(DcMotorSimple.Direction.REVERSE);
        s_midintake.setDirection(Servo.Direction.REVERSE);


    }


    @Override
    public void loop() {
        telemetry.addLine("Back & Start - Resetear IMU");
        telemetry.addLine("Cruceta Arriba - RPM 1160");
        telemetry.addLine("Cruceta Abajo - RPM 1080");
        telemetry.addLine("Cruceta Derecha - RPM 1140");
        telemetry.addLine("Cruceta Izquierda - RPM 1120");
        telemetry.addLine();
        telemetry.addData("Orientación:", IMUHeading.getHeading(AngleUnit.DEGREES));
        telemetry.addData("RPM del disparador:", DesearedRPMlong);
        telemetry.addData("Velocidad del disparador:", m_rightshooter.getVelocity());
        telemetry.addData("Velocidad del recolector:",m_intake.getVelocity());
        telemetry.addData("Posición del servo:", s_midintake.getPosition());
        telemetry.addData("Distancia de color:", colorsensor.getDistance());
        telemetry.addData("Correción de postura:", poscorrector.getDistance());
        telemetry.addData("Corrector activo:", isCorrector);
        telemetry.update();




        chasisMethod();
        shootMechanism();
        intakeMechanism();
        servoMechanism();
        resetGyro();

    }
    public void chasisMethod() {

        double y = gamepad1.left_stick_y * 2;
        double x = -gamepad1.left_stick_x * 2;
        double rx = gamepad1.right_stick_x * 3;

        if (gamepad1.right_bumper){
            y = gamepad1.left_stick_y * .5;
            x = -gamepad1.left_stick_x * .5;
            rx = gamepad1.right_stick_x * .5;
        }

        double botHeading = Math.toRadians(IMUHeading.getHeading(AngleUnit.DEGREES));

        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double FLpower = - rotY + rotX - rx;
        double BLpower = rotY - rotX - rx;
        double FRpower = - rotY - rotX + rx;
        double BRpower = rotY + rotX + rx;

        double MaxPower = Math.max(1.0, Math.max(Math.abs(FLpower),
                Math.max(Math.abs(FRpower),
                        Math.max(Math.abs(BLpower),
                                Math.abs(BRpower)))));

        if (gamepad1.left_bumper){
            isCorrector = true;
            if (poscorrector.getDistance() <= 70){
                if (y >= 0.1){
                    FLpower = 0;
                    BLpower = 0;
                    FRpower = 0;
                    BRpower = 0;
                }
            }
        }

        m_fl.setPower(FLpower / MaxPower);
        m_fr.setPower(FRpower / MaxPower);
        m_bl.setPower(BLpower / MaxPower);
        m_br.setPower(BRpower / MaxPower);


    }

    public void shootMechanism() {

        actualVelocity = DesearedRPMshort;
        double joystickVel = gamepad2.left_stick_x;

        // Selección de RPM
        if (gamepad2.dpad_down)  DesearedRPMlong = 1120;
        if (gamepad2.dpad_left)  DesearedRPMlong = 1140;
        if (gamepad2.dpad_right) DesearedRPMlong = 1170;
        if (gamepad2.dpad_up)    DesearedRPMlong = 1190;

        if (gamepad2.right_bumper) {
            actualVelocity = DesearedRPMlong;
        }


        if (gamepad2.right_trigger >= 0.15) {
            m_leftshooter.setVelocity(-actualVelocity );
            m_rightshooter.setVelocity( actualVelocity );

        }
        else if (gamepad2.left_stick_x >=0.15 || gamepad2.left_stick_x <=-0.15){
            m_leftshooter.setPower(-joystickVel*.3 );
            m_rightshooter.setPower( joystickVel*.3 );
        }
        else {
            m_leftshooter.setPower(0);
             m_rightshooter.setPower(0);
        }





        if (colorsensor.getDistance() <=8 && m_rightshooter.getVelocity() >= actualVelocity){
            s_midintake.setPosition(0.7);
        }
        else {
            s_midintake.setPosition(0.3);
        }


    }

    public void intakeMechanism(){
        double velTrigger = gamepad2.left_trigger * 1.5;
        double velJoystick = gamepad2.right_stick_x *.5;

        if (gamepad2.left_trigger >=0.15){
            m_intake.setPower(velTrigger);
        }
        else if (gamepad2.right_stick_x >= 0.15 || gamepad2.right_stick_x <= -0.15){
            m_intake.setPower(velJoystick);

        }
        else if (m_rightshooter.getVelocity() >= actualVelocity){
            m_intake.setPower(.7);
        }
        else {
            m_intake.setPower(0);
        }

    }
    public void servoMechanism(){
        if (gamepad2.left_bumper) {
            s_midintake.setPosition(0.3);
        }

        if (gamepad2.back){
            s_midintake.setPosition(0.7);
        }
        if (gamepad2.start){
            s_midintake.setPosition(0.3);
        }
    }

    public void resetGyro(){
        if (gamepad1.start){
            IMUHeading.resetImu();
        }
    }

}