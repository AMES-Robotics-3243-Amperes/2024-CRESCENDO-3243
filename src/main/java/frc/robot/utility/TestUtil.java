// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import javax.swing.JFrame;
import javax.swing.JOptionPane;

/** Add your docs here. */
public class TestUtil {


    /** 
     * Some weird object to allow us to put off execution to another thread and give 
     * a {@link Future} back instead of a real value.
     * 
     * @author H!
     */
    private static ExecutorService executor = Executors.newSingleThreadExecutor();

    /**
     * A method for asking the user questions as part of integrated testing. This
     * is something you might want to do because you can't be sure encoders are always
     * acurate - those need to be checked too.
     * 
     * @param question  The question to ask the user
     * @param optionYes The text displayed on the button returning true
     * @param optionNo  The text displayed on the button returning false
     * @return          A {@link Future} which will eventually contain the user's decision.
     *                  True means the "yes" option was picked. False means the "No" option was picked.
     *                  Null means the user closed the window or it ended in another manner.
     * 
     * @author H!
     */
    public static Future<Boolean> askUserBool(String question, String optionYes, String optionNo) {
        return executor.submit(() -> {
            Object[] options = {
                optionYes,
                optionNo
            };

            switch (JOptionPane.showOptionDialog(new JFrame(), question, "Test Input", 2, JOptionPane.QUESTION_MESSAGE, null, options, options[0])) {
                case (JOptionPane.CLOSED_OPTION):
                    return null;
                
                case 0:
                    return true;
                
                case 1:
                    return false;
                
                default:
                    return null;
            }
        });
    }

    /**An overload where {@code optionYes} and {@code optionNo} are presumed to be "Yes" and "No"
     * @see TestUtil#askUserBool(String, String, String)
     */
    public static Future<Boolean> askUserBool(String question) {
        return askUserBool(question, "Yes", "No");
    }


}
