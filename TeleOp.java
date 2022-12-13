package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ScaraBigData.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//Nom du programe.
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ControllerCode", group = "ScaraBetter")
public class TeleOp extends LinearOpMode {
    boolean handIsHolding = false;

    //Appel aux classes utilisées.
    //******************************************************************
    DcMotor rightMotor = null, leftMotor = null, elevatorMotor = null;
    Servo   handServo  = null;

    Controller.Joystick JoyR     = new Controller.Joystick(),
                        JoyL     = new Controller.Joystick();
    Controller.Trigger  TriggerR = new Controller.Trigger(),
                        TriggerL = new Controller.Trigger();
    Controller.Bumper   BumperR  = new Controller.Bumper(),
                        BumperL  = new Controller.Bumper();
    Controller.Button   aButton  = new Controller.Button(),
                        bButton  = new Controller.Button(),
                        xButton  = new Controller.Button(),
                        yButton  = new Controller.Button();
    //********************************************************************

    private void readController(){
        //Lecture des entrés en temps réel de la première manette.
        JoyR.x            = gamepad1.right_stick_x;
        JoyR.y            = gamepad1.right_stick_y;
        JoyL.x            = gamepad1.left_stick_x;
        JoyL.y            = gamepad1.left_stick_y;
        TriggerR.state    = gamepad1.right_trigger;
        TriggerL.state    = gamepad1.left_trigger;
        BumperR.isPressed = gamepad1.right_bumper;
        BumperL.isPressed = gamepad1.left_bumper;
        aButton.isPressed = gamepad1.a;
        bButton.isPressed = gamepad1.b;
        xButton.isPressed = gamepad1.x;
        yButton.isPressed = gamepad1.y;
    }
    private void setMotorPower(){
        //Calcule de la puissance des moteurs selon la direction
        double rm = Range.clip(JoyL.y - JoyL.x, -1, 1);
        double lm = Range.clip(JoyL.y + JoyL.x, -1, 1);

        //Ralentisseur de vitesse.
        if(BumperL.isPressed){
            rm = rm/2;
            lm = lm/2;
        }

        //Exécuter le calcule.
        rightMotor.setPower(rm);
        leftMotor. setPower(lm);
    }
    private void runElevator(int level){
        //Vérification de la position actuelle du bras.
        if(elevatorMotor.getCurrentPosition() > ENCODER_ARM_LEVEL[level]){
            //Si la position du bras est plus haute que la position désiré,
            //Définir la position voulue du bras, puis accéder à cette position en le descendant vers le bas.
            elevatorMotor.setTargetPosition(ENCODER_ARM_LEVEL[level]);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor.setPower(-POWER_ARM);
        }else{
            //Si la position du bras est plus basse que la position désiré,
            //Définir la position voulue du bras, puis accéder à cette position en le montant vers le haut.
            elevatorMotor.setTargetPosition(ENCODER_ARM_LEVEL[level]);
            elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotor.setPower(POWER_ARM);
        }
    }
    private void setArmPosition(){
        int level = 0;

        //Execution du bras selon la hauteur désiré à partir des bouttons de la manette.
        if(xButton.isPressed){
            runElevator(0);
        }
        else if(aButton.isPressed){
            runElevator(1);
            level = 1;
        }else if(bButton.isPressed){
            runElevator(2);
            level = 2;
        }else if(yButton.isPressed){
            runElevator(3);
            level = 3;
        }

        //Affichage des information concerant le bras en temps réel.
        while(elevatorMotor.isBusy()) {
            telemetry.addData("Niveau de hauteur ", level);
        }elevatorMotor.setPower(0);

        //Execution du bras manuellement avec les gachètes.
        elevatorMotor.setPower(TriggerR.state/2);
        elevatorMotor.setPower(-TriggerL.state/2);

        //Vérification de l'excès de hauteur du bras.
        if(elevatorMotor.getCurrentPosition() >= ARM_MAX_LIMIT || elevatorMotor.getCurrentPosition() < ARM_MIN_LIMIT){
            elevatorMotor.setPower(0);
        }
    }
    private void setHandState(){
        //Vérification de l'état du bouton dans la manette.
        if(BumperR.isPressed){
            //Si la main est fermé, ouvrir la main puis définir sa position en tant que ouverte.
            //Si la main est ouverte, fermer la main puis définir sa position en tant que fermé.
            if(handIsHolding){
                handIsHolding = false;
                handServo.setPosition(HAND_OFF);
            }else{
                handIsHolding = true;
                handServo.setPosition(HAND_ON);
            }
        }
    }
    @Override
    public void runOpMode(){
        //Appel aux composants du robot.
        rightMotor    = hardwareMap.get(DcMotor.class, "rm");
        leftMotor     = hardwareMap.get(DcMotor.class, "lm");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elm");
        handServo     = hardwareMap.get(Servo.class, "hs");

        //Prévenir toute forme d'accident.
        rightMotor.   setPower(0.0);
        leftMotor.    setPower(0.0);
        elevatorMotor.setPower(0.0);

        //Définir la direction des moteurs.
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotor .setDirection(DcMotorSimple.Direction.FORWARD);
        handServo. setDirection(Servo.Direction.FORWARD);

        //Définir le fonctionnement du moteur.
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotor. setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Attendre la confirmation du pilote.
        waitForStart();

        //Lire le programme en boucle, une fois activé.
        while(opModeIsActive()){
            readController();
            setMotorPower();
            setArmPosition();
            setHandState();
        }
    }
}