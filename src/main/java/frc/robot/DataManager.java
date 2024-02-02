package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.PowerManager;
import edu.wpi.first.wpilibj.I2C;

/** <b>Stores all of the data that is shared between systems, especially positions.</b>
 * 
 * <p>Note that the entries in this class should be responsible for getting the correct information 
 * at the correct time. The classes from which this takes information should never interact 
 * directly with classes which use their information. Most processing of the information provided
 * by systems should be done in this class.</p>
 * 
 * <p>Also, it's possible you'll have to write entries that don't fit the {@link Entry} interface;
 * this is ok.</p>
 * 
 * @implNote This class has method calls in {@link Robot}
 * 
 * @author H!
 */
public class DataManager {
    //#########################################################
    //                    ENTRY FRAMEWORK
    //#########################################################


    /** Stores some kind of data of type {@link T} to be interchanged between systems
     *  @author H!
     */
    public static interface Entry<T> {
        /** Returns the data stored in this entry
         *  @return The data stored in this entry
         */
        T get();
    }


    /** An {@link Entry} that can be set using a set method
     * @author H!
     */
    public static interface SettableEntry<T> extends Entry<T> {
        /**
         * Sets the value of this entry
         * @param newValue The new value for the entry
         */
        void set(T newValue);
    }

    /** An entry that does not change throughout time
     * @author H!
     */
    public static class StaticEntry<T> implements Entry<T> {
        protected final T m_value;
        /** 
         * 
         * @param value
         */
        public StaticEntry(T value) {
            m_value = value;
        }

        @Override
        public T get() {
            return m_value;
        }
    }


    /** An entry for a position and orientation in 3D @author H! */
    public static interface FieldPose extends Entry<Pose3d> {}
    /** An entry for a position in 3D @author H! */
    public static interface FieldLocation extends Entry<Translation3d> {}


    //#########################################################
    //                      ENTRY TYPES
    //#########################################################
    

    /** The entry for the positionn and orientation of the robot
     * 
     */
    /** The entry for the positionn and orientation of the robot
     * 
     */
    public static class CurrentRobotPose implements FieldPose {
        /** The pose the robot's located at @author :3 */
        protected Pose3d m_robotPose = new Pose3d();

        protected boolean m_robotPoseIsCurrent = false;
        /** The previous pose odometry read @author :3 */
        protected Pose3d m_previousOdometryPose = new Pose3d();


        protected Pose3d m_latestOdometryPose = new Pose3d();

        protected Pose3d m_latestPhotonPose = new Pose3d();

        protected double m_latestAmbiguity = 0.0;

        /**
         * Creates a new {@link CurrentRobotPose} object
         */
        public CurrentRobotPose() {
            // TODO add anything that is needed here
        }

        @Override
        public Pose3d get() {
            if (!m_robotPoseIsCurrent) {
                combineUpdateData();
                m_robotPoseIsCurrent = true;
            }

            return new Pose3d(m_robotPose.getTranslation(), m_robotPose.getRotation().rotateBy(new Rotation3d(0, 0, Math.PI / 2)));
        }

        /*
         * Find the new robot pose using the odometry and vision data
         * @author H!
         */
        protected void combineUpdateData() {
            // This seems to behave a little weird and alternate bettween using vision and not, but it's fine-ish for now (I hope) H!
            SmartDashboard.putNumber("photonAmbiguity", m_latestAmbiguity);
            if (m_latestAmbiguity > 0.15 || m_latestPhotonPose == null) {
                Transform3d transformSinceLastUpdate = new Transform3d(m_previousOdometryPose, m_latestOdometryPose);

                // transform the robot pose and update the previous odometry
                m_robotPose = m_robotPose.transformBy(transformSinceLastUpdate);

                m_previousOdometryPose = m_latestOdometryPose;
                SmartDashboard.putBoolean("usingVision", false);
            } else {
                SmartDashboard.putNumber("photonPoseX", m_latestPhotonPose.getX());
                SmartDashboard.putNumber("photonPoseY", m_latestPhotonPose.getY());
                SmartDashboard.putNumber("photonPoseRotZ", m_latestPhotonPose.getRotation().getZ());
                m_robotPose = m_latestPhotonPose;
                SmartDashboard.putBoolean("usingVision", true);
            }
        }

        /**
         * Updates the robot's position using odometry.
         * Should be run periodically.
         * 
         * @param odometryReading the current Pose2d reported by odometry
         */
        public void updateWithOdometry(Pose2d odometryReading) {
            // get the Transform3d from the last odometry update
            m_latestOdometryPose = new Pose3d(odometryReading);

            m_robotPoseIsCurrent = false;
        }

        public void updateWithVision(Pose3d visionEstimate, double ambiguity) {
            m_latestPhotonPose = visionEstimate;
            m_latestAmbiguity = ambiguity;

            m_robotPoseIsCurrent = false;
        }
        
    }
    public static class AccelerationConstant implements Entry<Double> {
        public Double get() {
            return (PowerManager.getDriveAccelerationDampener());
        }
    }
    public static class VelocityConstant implements Entry<Double> {
        public Double get() {
            return (PowerManager.getDriveSpeedDamper());
        }
    }

    public static class NoteStorageSensor implements Entry<Boolean> {
        ColorSensorV3 colorSensor;
        boolean hasNote = false;

        public NoteStorageSensor() {
            colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        }

        @Override
        public Boolean get() {
            int proximity = colorSensor.getProximity();

            if (proximity < Constants.ColorSensor.emptyDistance) {
                hasNote = false;
            }

            if (proximity > Constants.ColorSensor.filledDistance) {
                hasNote = true;
            }

            return hasNote;
        }
    }

    //#########################################################
    //                        ENTRIES
    //#########################################################

    public static CurrentRobotPose currentRobotPose = new CurrentRobotPose();
    public static AccelerationConstant currentAccelerationConstant = new AccelerationConstant();
    public static VelocityConstant currentVelocityConstant = new VelocityConstant();
    //#########################################################
    //               INITIALIZATION AND RUNTIME
    //#########################################################

    /**Called after robot container is done being constructed. This happens
     * when the robot code is deployed.
     * 
     * Put all construction of entries here, instead of in the definition. This
     * ensures we have control over when those entries aare constructed. You may want
     * to put some further initialization in the folowing methods, called at different times.
     */
    public static void onRobotInit() {
        currentRobotPose = new CurrentRobotPose();
    }

    /** Called when the robot is enabled. */
    public static void onEnable() {}

    /** Called when the robot is disabled, from {@link Robot#disabledInit()} */
    public static void onDisable() {}

    /** Called when the robot is enabled in teleop, from {@link Robot#teleopInit()} */
    public static void onTeleopInit() {
        onEnable();
    }

    /** Called when the robot is enabled in autonomous, from {@link Robot#autonomousInit()} */
    public static void onAutoInit() {
        onEnable();
    }

    /** Called when the robot is enabled in test mode, from {@link Robot#testInit()} */
    public static void onTestInit() {
        onEnable();
    }

    /** Called when the robot is created during a simulation, from {@link Robot#simulationInit()} */
    public static void onSimulationInit() {}

    /**Called every 20 milliseconds, from {@link Robot#robotPeriodic()}. Try to avoid using this method
     * if you can, there's a decent chance if you're using it this is really something that should
     * be done by a subsystem or command.
     */
    public static void periodic() {}
}
