package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ScaraBigData.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.*;

//Nom du programe.
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "AutonomousCode", group = "ScaraBetter")
public class Autonomous extends LinearOpMode {
    //Appel au classes utilisées.
    //******************************************************************************
    DcMotor     rightMotor = null,       leftMotor = null, elevatorMotor = null;
    Orientation gyroscope  = null; Servo handServo = null; IMU       imu = null;
    //******************************************************************************

    private void drive(int distance){
        //Fonction permettant le mouvement translationel bidirectionel du robot.

        //Remettre les encodeurs à 0;
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Définir la valeur des encodeurs une fois le robot arrivé à sa destination.
        rightMotor.setTargetPosition((int) (distance * ENCODER_PER_CM));
        leftMotor. setTargetPosition((int) (distance * ENCODER_PER_CM));

        //Execution de la tache demandée.
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor. setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //Vérfier si le robot devra rouler vers l'avant ou vers l'arrière.
        if(distance > 0){
            rightMotor.setPower(POWER_DRIVE);
            leftMotor. setPower(POWER_DRIVE);
        }else if(distance < 0){
            rightMotor.setPower(-POWER_DRIVE);
            leftMotor. setPower(-POWER_DRIVE);
        }

        //Affichage du trajet parcouru en temps réel.
        while(rightMotor.isBusy() && leftMotor.isBusy()){
            int currentDist = (int)(rightMotor.getCurrentPosition()/ENCODER_PER_CM);
            telemetry.addData("Progression du trajet ",  currentDist + "/" + distance + " (cm)");
            telemetry.update();
        }

        //Arrêt du robot une fois le niveau d'encodeurs
        rightMotor.setPower(0);
        leftMotor. setPower(0);
    }
    private void turn_Gyro(int angle){
        //Fonction en cours de développement... Aucune information de plus.

        this.imu.resetYaw();
        String direction = null;

        while(angle < gyroscope.secondAngle){
            if(angle > 0){
                direction = "Gauche";
                rightMotor.setPower(POWER_TURN);
                leftMotor .setPower(-POWER_TURN);
            }else if(angle < 0){
                direction = "Droite";
                rightMotor.setPower(-POWER_TURN);
                leftMotor. setPower(POWER_TURN);
            }
            telemetry.addData("Direction ", direction);
            telemetry.update();
        }

        rightMotor.setPower(0);
        leftMotor. setPower(0);
    }
    private void turn_Encoder(int angle){
        //Fonction permettant le mouvement rotationel du robot selon le degré demandé.

        //Remettre les encodeurs à 0;
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Calcule de la valeur de l'encodeur final une fois la rotation effectuée.
        // 1 deg = x valeur de l'encodeur. (Voir ScaraBigData)
        // n deg = y valeur de l'encodeur. (Voir ScaraBigData)
        rightMotor.setTargetPosition((int) (angle * ENCODER_PER_DEG));
        leftMotor. setTargetPosition((int) (-angle * ENCODER_PER_DEG));

        //Execution de la tache demandée.
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotor. setMode(DcMotor.RunMode.RUN_TO_POSITION);

        String direction = null;
        //Vérification de la direction à prendre.
        // Si l'angle demandé est en dessous de 0, tourner à droit.
        // Si l'angle demandé est au dessus de 0, tourner à gauche.
        if(angle > 0){
            direction = "Gauche";
            rightMotor.setPower(POWER_TURN);
            leftMotor .setPower(-POWER_TURN);
        }else if(angle < 0){
            direction = "Droite";
            rightMotor.setPower(-POWER_TURN);
            leftMotor. setPower(POWER_TURN);
        }

        //Affichage de l'angle parcouru en temps réel.
        while(rightMotor.isBusy() && leftMotor.isBusy()){
            int currentAngle = (int)(rightMotor.getCurrentPosition()/ENCODER_PER_DEG);
            telemetry.addData("Tourner à ", currentAngle + "/" + angle + " deg. (" + direction + ")");
            telemetry.update();
        }

        //Arrêt des moteurs une fois le moteur tourné.
        rightMotor.setPower(0);
        leftMotor. setPower(0);
    }
    private void useElevator(int level){
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

        //Affichage des information concerant le bras en temps réel.
        while(elevatorMotor.isBusy()){
            telemetry.addData("Niveau de hauteur ", level);
        }elevatorMotor.setPower(0);
    }
    private void useHand(boolean state){
        //Fonction appelant un argument vrai ou faux.
        //true = main fermée. false = main ouverte.
        if(state){
            //Main ouverte.
            handServo.setPosition(HAND_ON);
        }else{
            //Main fermé.
            handServo.setPosition(HAND_OFF);
        }
    }
    @Override
    public void runOpMode() {
        //Appel aux composants du robot.
        rightMotor    = hardwareMap.get(DcMotor.class, "rm");
        leftMotor     = hardwareMap.get(DcMotor.class, "lm");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elm");
        handServo     = hardwareMap.get(Servo.class, "hs");
        imu           = hardwareMap.get(IMU.class, "imu");

        //En cours de développement...
        //----------------------------------------------------
        gyroscope = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        //----------------------------------------------------

        //Prévenir tout danger au démarrage.
        rightMotor.   setPower(0.0);
        leftMotor.    setPower(0.0);
        elevatorMotor.setPower(0.0);

        //Définir le fonctionnement du moteur.
        rightMotor.    setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.     setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotor. setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Définir la direction des moteurs.
        rightMotor.   setDirection(DcMotor.Direction.FORWARD);
        leftMotor.    setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        handServo.    setDirection(Servo.Direction.FORWARD);

        //Attendre confirmation au contrôle.
        waitForStart();
        //LA PARTIE LA PLUS IMPORTANTE DU CODE COMMENCE ICI!

        useHand(false);
        sleep(2000);
    }
}
