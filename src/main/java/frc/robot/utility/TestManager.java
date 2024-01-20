// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import javax.swing.JOptionPane;
import javax.swing.JScrollPane;
import javax.swing.JTextPane;
import javax.swing.plaf.DimensionUIResource;
import javax.swing.text.html.HTMLDocument;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/** A system which will run all tests queued to it and display their results. @author H! */
public class TestManager {
    /* TODO:
     * Make the test runner behave nicely if the testing is cut off halfway through
     * Add "Require enabled" method/abstract class
     * Other Utils
     * 
     * Paralalizable (<- spelled wrong):
     * Test test implementation
     * 
     * Test the test test
     */

    public enum TestSuccess {
        NOTRUN,
        FAIL,
        SUCCESS;
    }

    public TestSuccess testSuccessFromBool(Boolean bool) {
        if (bool == true) {
            return TestSuccess.SUCCESS;
        } else if (bool == false) {
            return TestSuccess.FAIL;
        } else {
            return TestSuccess.NOTRUN;
        }
    }

    /**
     * Stores the results of a test, that being whether it succeeded and any other message
     * it may have provided.
     * 
     * @author H!
     */
    public static class TestResults {
        public TestSuccess m_succeessResult;
        public String m_message;

        public TestResults(TestSuccess succeessResult, String message) {
            m_succeessResult = succeessResult;
            m_message = message;
        }

        public TestResults(TestSuccess succeessResult) {
            this(succeessResult, "");
        }
    }

    /**
     * Used for storing the progression of a test.
     * 
     * @author H!
     */
    protected static enum TestState {
        SETUP,
        RUNNING,
        CLOSEDOWN
    }



    public static Map<String, Map<String, TestResults>> results = new HashMap<String, Map<String, TestResults>>();
    protected static Map<Test, TestSuccess> testsRun = new HashMap<Test, TestSuccess>();

    protected static List<TestGroup> groupsToTest = new ArrayList<TestGroup>();
    protected static List<Test> testsToTest = new ArrayList<Test>();


    protected static int testIndex = 0;
    protected static TestState testState = TestState.SETUP;

    public static boolean testsFinished = false;
    public static boolean testStarted = false;
    private static boolean inInitialPause = true;

    protected static int cyclesRun = 0;






    /**
     * Used to add a test group to be tested to the queue. This is an external access point.
     * 
     * @param toTest The {@link TestGroup} to test
     * 
     * @author H!
     */
    public static void queueGroupToTest(TestGroup toTest) {
        System.out.println("test Queued");
        groupsToTest.add(toTest);
    }

    /**
     * Should be run when test mode is started by {@link Robot#testInit()}. Resets everything and clears the test queue.
     * 
     * @author H!
     */
    public static void init() {
        System.out.println("init!");
        groupsToTest.clear();
        testIndex = 0;
        testState = TestState.SETUP;
        inInitialPause = true;
        testsFinished = false;
    }

    /**
     * Should be run periodically by {@link Robot#testPeriodic()}. Runs queued tests.
     * 
     * @author H!
     */
    public static void periodic() {
        if (Robot.managerFirst == null) {
            Robot.managerFirst = true;
        }

        if (inInitialPause) {
            inInitialPause = false;
            return;
        }
        
        cyclesRun++;
        SmartDashboard.putNumber("CyclesRun", cyclesRun);
        //System.out.println("#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#\n#");
        String[] testGroupNames = new String[groupsToTest.size()];
        for (int i = 0; i < groupsToTest.size(); i++) {
            testGroupNames[i] = groupsToTest.get(i).getName();
        }

        SmartDashboard.putBoolean("TestManagerAlive", true);

        SmartDashboard.putStringArray("TestGroupsQueued", testGroupNames);
        try {
            SmartDashboard.putString("FirstTestGroup", groupsToTest.get(0).getName());
        } catch (Exception e) {
            //System.out.println("No test groups");
        }
        

        if (groupsToTest.size() > 0) {
            results.putIfAbsent(groupsToTest.get(0).getName(), new HashMap<String, TestResults>());
            runTests(groupsToTest.get(0));

        } else {
            if (!testsFinished) {
                displayTestResults();;
                testsFinished = true;
            }
        }
    }

    /**
     * Runs one cycle of the current test on the given test group. Should be run periodically.
     * When all tests are done, removes this element from {@link #groupsToTest} and resets {@link #testIndex}
     * 
     * @param testGroup The test group to run the tests of
     * 
     * @author H!
     */
    protected static void runTests(TestGroup testGroup) {
        if (testsToTest.size() == 0) {
            testsToTest = new LinkedList<Test>(Arrays.asList(testGroup.getTests()));
        }

        if (testsToTest.get(0).getName() == "Example Test") {
            System.out.println("it's time");
        }

        if (!testStarted) {
            /* DEPENDENCY LOGIC:
             * 
             * All done, all correct         -> Run test
             * All done, not all correct     -> Mark test as not run, and remove it from the queue
             * Not all done, all correct     -> Buffer with dependencies, move test to the back of the queue
             * Not all done, not all correct -> Mark test as not run, and remove it from the queue
             * 
             * A dependency being "correct" means that its result (success/failure) matched the result the main test required of the dependency.
             */
            // Check if dependencies are already done:
            boolean allDependenciesDone = true;
            boolean allDependenciesCorrect = true;
            for (int i = 0; i < testsToTest.get(0).getDependencies().length; i++) {
                Test test = testsToTest.get(0).getDependencies()[i];
                TestSuccess isRun = testsRun.get(test);
                if (isRun == null) {
                    allDependenciesDone = false;
                } else if (isRun == TestSuccess.FAIL && testsToTest.get(0).getDependencySuccessRequirements()[i] == true) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                } else if (isRun == TestSuccess.SUCCESS && testsToTest.get(0).getDependencySuccessRequirements()[i] == false) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                } else if (isRun == TestSuccess.NOTRUN) {
                    allDependenciesCorrect = false;
                    break; // We can stop the loop if any are wrong, because no matter what else happens, if one dependency is wrong, we have to cancel the test
                }
            }

            if (!allDependenciesCorrect) {
                results.get(groupsToTest.get(0).getName()).put(testsToTest.get(0).getName(), new TestResults(TestSuccess.NOTRUN, "Dependencies Not Correct"));
                testsToTest.remove(0);
                if (testsToTest.size() == 0) {
                    groupsToTest.remove(0);
                }
                testStarted = false;
                return;
            } else if (!allDependenciesDone) {
                for (Test test : testsToTest.get(0).getDependencies()) {
                    if (!testsRun.containsKey(test) && !testsToTest.contains(test)) {
                        testsToTest.add(1, test);
                    }
                }

                testsToTest.add(testsToTest.remove(0));
                return; // Return to this method next cycle now that the test list has been updated
            }
        }
        
        runTest(testsToTest.get(0));
    }

    /** Runs all logic that must run when a tests finishes.
     * This invloves managing reseting counters and preparing the next tests.
     */
    public static void onTestDone() {
        testState = TestState.SETUP;
        testsToTest.remove(0);
        if (testsToTest.size() == 0) {
            groupsToTest.remove(0);
        }
        testStarted = false;
    }

    /**
     * Logic to run one cycle of a test. Should be run periodically to perform the test.
     * When done, increments to the next test.
     * 
     * @param test The test to run
     * 
     * @author H!
     */
    protected static void runTest(Test test) {
        try {
            switch (testState) {
                case SETUP:
                    test.setupPeriodic();
                    if (test.setupIsDone()) {
                        testState = TestState.RUNNING;
                    }
                    break;
            
                case RUNNING:
                    test.testPeriodic();
                    if (test.testIsDone()) {
                        testState = TestState.CLOSEDOWN;
                    }
                    break;
                
                case CLOSEDOWN:
                    test.closedownPeriodic();
                    if (test.closedownIsDone()) {
                        results.get(groupsToTest.get(0).getName()).put(test.getName(), new TestResults(TestSuccess.SUCCESS));
                        testsRun.put(test, TestSuccess.SUCCESS);
                        onTestDone();
                    }
                    break;
                
                default:
                    break;
            }
        } catch (AssertionError e) {
            System.out.println("\n\n\n\n\n\nFAILURE\n\n\n\n\n\n\n");
            results.get(groupsToTest.get(0).getName()).put(test.getName(), new TestResults(TestSuccess.FAIL, e.getMessage()));
            testsRun.put(test, TestSuccess.FAIL);
            onTestDone();
        }
    }

    /** The format used to generate the HTML group headers in the test results @author H! */
    protected static final String groupFormat = "<li><h2 class='%2$s'>%1$s | %3$s %4$s </h2> <p><em class='success'>%5$d/%7$d/%8$d Succeess</em> | <em class='fail'>%6$d/%7$d/%8$d Fails</em></p><ul></ul></li>";
    /**A utility method for getting the proper HTML to make a subsytem test result wrapper be displayed
     * 
     * @param resultEntry One entry of the map corresponding to the test group to display
     * @return A {@link String} with the HTML in plaintext repersenting the group header
     * 
     * @author H!
     */
    protected static String getGroupHTMLElement(Entry<String, Map<String, TestResults>> resultEntry) {
        int successCount = 0;
        int performedCount = 0;
        int totalCount = resultEntry.getValue().size();

        for (TestResults testResult : resultEntry.getValue().values()) {
            if (testResult.m_succeessResult == TestSuccess.SUCCESS) {
                successCount++;
                performedCount++;
            } else if (testResult.m_succeessResult != TestSuccess.NOTRUN) {
                performedCount++;
            }
        }

        return String.format(
            groupFormat, 
            resultEntry.getKey(), 
            successCount == performedCount ? "success" : "fail",
            successCount == performedCount ? "✔" : "✗",
            performedCount == totalCount ? "" : "*",
            successCount,
            performedCount - successCount,
            performedCount,
            totalCount
        );
    }

    /** The format used to generate the HTML for individual tests in the test results @author H! */
    protected static final String testFormat = "<li><h3 class='%2$s'>%1$s | %3$s</h3>%4$s</li>";
    /**A utility method for getting the proper HTML to make a test result be displayed
     * 
     * @param testEntry One entry of the map corresponding to the test to display
     * @return A {@link String} with the HTML in plaintext repersenting the test result
     * 
     * @author H!
     */
    protected static String getTestHTMLFormat(Entry<String, TestResults> testEntry) {
        String cssClass = "";
        String resultIcon = "";
        if        (testEntry.getValue().m_succeessResult == TestSuccess.SUCCESS) {
            cssClass = "success";
            resultIcon = "✔";
        } else if (testEntry.getValue().m_succeessResult == TestSuccess.FAIL) {
            cssClass = "fail";
            resultIcon = "✗";
        } else if (testEntry.getValue().m_succeessResult == TestSuccess.NOTRUN) {
            cssClass = "notRun";
            resultIcon = "-";
        }

        return String.format(
            testFormat, 
            testEntry.getKey(), 
            cssClass,
            resultIcon,
            testEntry.getValue().m_message.equals("") ? "" : "<p>" + testEntry.getValue().m_message + "</p>"
        );
    }

    /**Displays the latest results of the integrated tests in a Swing dialog
     * @author H!
     */
    public static void displayTestResults() {
        JTextPane textPane = new JTextPane();
        textPane.setContentType("text/html");
        textPane.setText("<!DOCTYPE html><html><head><style>.fail{color:red}.success{color:green}.notRun{color:grey}</style></head><body><h1>Integrated Test Results</h1><ul id=testGroupList></ul></body></html>");
        HTMLDocument doc = (HTMLDocument) textPane.getDocument();

        for (Entry<String, Map<String, TestResults>> groupEntry : results.entrySet()) {
            try {
                doc.insertAfterStart(doc.getElement("testGroupList"), getGroupHTMLElement(groupEntry));
                
                for (Entry<String, TestResults> testResultEntry : groupEntry.getValue().entrySet()) {
                    doc.insertAfterStart(doc.getElement("testGroupList").getElement(0).getElement(2), getTestHTMLFormat(testResultEntry));
                }


            } catch (Exception e) {
                System.out.println("Test result display generation failed:");
                e.printStackTrace();
            }
        }
        
        JScrollPane scrollPane = new JScrollPane(textPane);

        
        scrollPane.setPreferredSize(new DimensionUIResource(
            600, 
            500
        ));
        
        JOptionPane.showMessageDialog(
            new JTextPane(), 
            scrollPane,
            "Integrated Test Results",
            JOptionPane.PLAIN_MESSAGE
        );
    }
}
