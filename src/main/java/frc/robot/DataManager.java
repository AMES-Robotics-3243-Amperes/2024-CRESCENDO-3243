package frc.robot;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utility.PowerManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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

    public static class FieldPoses {
        // :> From 0-7, field left to field right
        public static Pose2d getNotePositions(int arrayPosition) {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    return Constants.FieldConstants.noteRedPositions[arrayPosition];
                }
                return Constants.FieldConstants.noteBluePositions[arrayPosition];
            }
            // :> Please make something to catch this at the other end
            return null;
        }
        
        public static Pose2d getAmpPosition() {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    return Constants.FieldConstants.redAmp;
                }
                return Constants.FieldConstants.blueAmp;
            }
            // :> Please catchi this on the other side
            return null;
        }

        public static Pose2d getSpeakerPosition() {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    return Constants.FieldConstants.redSpeakerCenterReference;
                }
                return Constants.FieldConstants.blueSpeakerCenterReference;
            }
            // :> Please catch this on the other side
            return null;
        }
        public static Pose2d getStagePositions(int arrayPosition) {
            if (DriverStation.getAlliance().isPresent()) {
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    return Constants.FieldConstants.stageRedPositions[arrayPosition];
                }
                return Constants.FieldConstants.stageBluePositions[arrayPosition];
            }
            // :> Please make something to catch this at the other end
            return null;
        }
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
        protected Field2d field2d = new Field2d();

        protected boolean m_robotPoseIsCurrent = false;
        /** The previous pose odometry read @author :3 */
        protected Pose3d m_previousOdometryPose = new Pose3d();


        protected Pose3d m_latestOdometryPose = new Pose3d();

        // :3 keep this as null, or no photonvision will cause datamanager
        // to permanently return whatever this is
        protected Pose3d m_latestPhotonPose = null;

        protected double m_latestAmbiguity = 0.0;

        /**
         * Creates a new {@link CurrentRobotPose} object
         */
        public CurrentRobotPose() {
            // todo add anything that is needed here //H! is this a real todo?
        }

        @Override
        public Pose3d get() {
            if (!m_robotPoseIsCurrent) {
                combineUpdateData();
                m_robotPoseIsCurrent = true;
            }

            return new Pose3d(m_robotPose.getTranslation(), m_robotPose.getRotation());
        }

        /*
         * Find the new robot pose using the odometry and vision data
         * @author H!
         */
        protected void combineUpdateData() {
            // This seems to behave a little weird and alternate bettween using vision and not, but it's fine-ish for now (I hope) H!
            SmartDashboard.putNumber("photonAmbiguity", m_latestAmbiguity);
            if (m_latestAmbiguity > 0.15 || m_latestPhotonPose == null) {
                Translation3d translationChange = m_latestOdometryPose.getTranslation().minus(m_previousOdometryPose.getTranslation());
                Rotation3d rotationChange = m_latestOdometryPose.getRotation().minus(m_previousOdometryPose.getRotation());

                // transform the robot pose and update the previous odometry
                m_robotPose = new Pose3d(m_robotPose.getTranslation().plus(translationChange),
                    m_robotPose.getRotation().plus(rotationChange));

                // :> Testing data for debugging photonvision please ignore
                // :> Also worth noting this  was the first place I was able  to find a pose3D though I may be blind
                field2d.setRobotPose(m_robotPose.toPose2d());
                SmartDashboard.putData(field2d);
                SmartDashboard.putBoolean("usingVision", false);
            } else {
                SmartDashboard.putNumber("photonPoseX", m_latestPhotonPose.getX());
                SmartDashboard.putNumber("photonPoseY", m_latestPhotonPose.getY());
                SmartDashboard.putNumber("photonPoseRotZ", m_latestPhotonPose.getRotation().getZ());
                m_robotPose = m_latestPhotonPose;
                SmartDashboard.putBoolean("usingVision", true);
            }
            m_previousOdometryPose = m_latestOdometryPose;
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
      

        public NoteStorageSensor() {
            colorSensor = new ColorSensorV3(I2C.Port.kMXP);
        }

        @Override
        public Boolean get() {
     
            return colorSensor.getProximity() > Constants.ColorSensor.filledDistance;

        }
    }

    //#########################################################
    //                        ENTRIES
    //#########################################################

    public static CurrentRobotPose currentRobotPose = new CurrentRobotPose();
    public static AccelerationConstant currentAccelerationConstant = new AccelerationConstant();
    public static VelocityConstant currentVelocityConstant = new VelocityConstant();
    public static NoteStorageSensor currentNoteStorageSensor = new NoteStorageSensor();
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
    public static void periodic() {
        SmartDashboard.putNumber("noteProximity", currentNoteStorageSensor.colorSensor.getProximity());
    }
}
