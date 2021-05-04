package org.firstinspires.ftc.teamcode;

/**
 * Carry over variables from autonomous to teleop.
 * @author TM
 */
public class SharedVariables {
    static Transform transform = Constants.STARTING_T[0];

    public static void startingTransform(int i) {
        transform = Constants.STARTING_T[i];
    }
}
