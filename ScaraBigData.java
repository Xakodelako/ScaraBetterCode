package org.firstinspires.ftc.teamcode;

public class ScaraBigData {
    // Niveau du servo lorsqu'il est ouvert.
    public static final double HAND_ON = 1;
    // Niveau du servo lorsqu'il est fermé.
    public static final double HAND_OFF = 0.1;
    // Limite maximal du bras en terme de hauteur.
    public static final int ARM_MIN_LIMIT = 0;
    // Limite minimal du bras en terme de hauteur.
    public static final int ARM_MAX_LIMIT = -0;
    // Données d'encodeur des 3 niveau de hauteur.
    public static final int[] ENCODER_ARM_LEVEL = {0, 1000, 2000, 3000};
    // Proportion du niveau d'encodeur par centimètre.
    public static final double ENCODER_PER_CM = 40;
    // Proportion du niveau d'encoder par degrées. (Selon la distance entre les deux roues)
    public static final double ENCODER_PER_DEG = 0.24 * ENCODER_PER_CM;
    // Puissance/Vitesse du robot lorsqu'il se déplace.
    public static final double POWER_DRIVE = 1;
    // Puissance/Vitesse du robot lorsqu'il tourne.
    public static final double POWER_TURN = 0.5;
    // Puissance/Vitesse du Bras.
    public static final double POWER_ARM = 0.6;
}
