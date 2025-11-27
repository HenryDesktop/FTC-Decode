package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name = "RobotCode1P")
public class RobotCode_20251P extends OpMode {
    //---------------------------C-h-a-s-i-s--------------------------
    DcMotorEx m_fl;
    DcMotorEx m_fr;
    DcMotorEx m_bl;
    DcMotorEx m_br;
    DcMotorEx m_intake;
    DcMotorEx m_shooter1;
    DcMotorEx m_shooter2;
    ConfigureIMU bench = new ConfigureIMU();
    boolean mechaMode = false;


    //---------------------------G-l-o-b-a-l--------------------------

    double DesearedRPMlong = 650 ;
    final double DesearedRPMshort = 530;


    @Override
    public void init() {

        //---------------------------C-h-a-s-i-s---I-n-i-t-i-a-l-i-z-e-----------------------

        m_fl = hardwareMap.get(DcMotorEx.class, "FLMotor");
        m_fr = hardwareMap.get(DcMotorEx.class, "FRMotor");
        m_bl = hardwareMap.get(DcMotorEx.class, "BLMotor");
        m_br = hardwareMap.get(DcMotorEx.class, "BRMotor");

        m_intake = hardwareMap.get(DcMotorEx.class, "IntakeMotor");

        m_shooter1 = hardwareMap.get(DcMotorEx.class, "ShooterMotorA");
        m_shooter2 = hardwareMap.get(DcMotorEx.class, "ShooterMotorB");

        bench.init(hardwareMap);

        m_shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        m_intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        m_shooter2.setVelocityPIDFCoefficients(20,0,5,13.5);

        m_fr.setDirection(DcMotorSimple.Direction.REVERSE);
        m_bl.setDirection(DcMotorSimple.Direction.FORWARD);
        m_br.setDirection(DcMotorSimple.Direction.REVERSE);
        m_fl.setDirection(DcMotorSimple.Direction.FORWARD);
    }


    @Override
    public void loop() {
        telemetry.addLine("Tutorial (Chasis):");
        telemetry.addLine("El movimiento del chasis es libre");
        telemetry.addLine("X - Restablecer giroscopio");
        telemetry.addLine();
        telemetry.addLine("Start - Cambio de modo");
        telemetry.addLine();
        telemetry.addLine("Tutorial (Mecanismos):");
        telemetry.addLine("RT - Disparar");
        telemetry.addLine("LT - Recolectar");
        telemetry.addLine("Cruceta Arriba - RPM 650");
        telemetry.addLine("Cruceta Abajo - RPM 570");
        telemetry.addLine("Cruceta Derecha - RPM 630");
        telemetry.addLine("Cruceta Izquierda - RPM 600");
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine("Datos del robot:");
        telemetry.addData("Orientación:", bench.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocidad del disparador:", m_shooter2.getVelocity());
        telemetry.addData("Velocidad del recolector:",m_intake.getVelocity());
        telemetry.addData("Modo mecanico:", mechaMode);
        telemetry.update();

        chasisMethod();
        shootMechanism();
        intakeMechanism();
        resetGyro();
        changeMode();

    }
    public void chasisMethod(){

        if (mechaMode == false) {
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            double botHeading = Math.toRadians(bench.getHeading(AngleUnit.DEGREES));

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double FLpower = rotY + rotX - rx;
            double BLpower = rotY - rotX - rx;
            double FRpower = rotY - rotX + rx;
            double BRpower = rotY + rotX + rx;

            double MaxPower = Math.max(1.0, Math.max(Math.abs(FLpower),
                    Math.max(Math.abs(FRpower),
                            Math.max(Math.abs(BLpower),
                                    Math.abs(BRpower)))));

            m_fl.setPower(FLpower / MaxPower);
            m_fr.setPower(FRpower / MaxPower);
            m_bl.setPower(BLpower / MaxPower);
            m_br.setPower(BRpower / MaxPower);
        }
        else{
            return;
        }
    }
    public void shootMechanism(){

        if (mechaMode == true) {

            if (gamepad1.dpad_down) {
                DesearedRPMlong = 570;
            }
            if (gamepad1.dpad_left) {
                DesearedRPMlong = 600;
            }
            if (gamepad1.dpad_right) {
                DesearedRPMlong = 630;

            }
            if (gamepad1.dpad_up) {
                DesearedRPMlong = 650;
            }

            if (gamepad1.right_trigger >= 0.1) {

                telemetry.addData("RPM del disparador:", DesearedRPMlong);
                m_shooter1.setPower(-gamepad1.right_trigger * .4);
                m_shooter2.setPower(gamepad1.right_trigger * .4);
            } else {
                if (gamepad1.a) {
                    m_shooter1.setVelocity(-DesearedRPMlong);
                    m_shooter2.setVelocity(DesearedRPMlong);
                } else if (gamepad1.b) {
                    m_shooter1.setVelocity(-DesearedRPMshort);
                    m_shooter2.setVelocity(DesearedRPMshort);
                } else {
                    m_shooter1.setPower(-gamepad1.left_stick_x * .7);
                    m_shooter2.setPower(gamepad1.left_stick_x * .7);
                }
            }
        }
        else{
            return;
        }

    }
    public void intakeMechanism(){
      if (mechaMode == true){
          m_intake.setPower(gamepad1.left_trigger);
      }
      else {
          m_intake.setPower(0);
      }

    }
    public void resetGyro(){
        if (gamepad1.back){
            bench.resetImu();
        }
    }
    public void changeMode(){
        if (gamepad1.startWasPressed()){
            mechaMode =  !mechaMode;
        }

    }

}