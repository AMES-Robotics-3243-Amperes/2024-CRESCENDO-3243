package frc.robot.utility;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutomaticsConstants;
import frc.robot.Constants.DriveTrain.DriveConstants.AutoConstants;
import frc.robot.DataManager;

/**
 * Helper functions for making auto routines
 * 
 * @author :3
 */
public final class UtilFunctions {
    /**
     * Helper class for managing notes and angles to approach from
     * 
     * @author :3
     */
    public class NoteAndRotation {
        public int noteIndex;
        public Rotation2d rotation;

        /**
         * @param noteIndex the index passed into {@link DataManager}
         * @param rotation the rotation to pick the note up from
         * 
         * @author :3
         */
        public NoteAndRotation(int noteIndex, Rotation2d rotation) {
            this.noteIndex = noteIndex;
            this.rotation = rotation;
        }

        /**
         * Copy
         */
        public NoteAndRotation(NoteAndRotation other) {
            this.noteIndex = other.noteIndex;
            this.rotation = other.rotation;
          }

        /**
         * @return the translation of the note
         * 
         * @author :3
         */
        public Translation2d getLocation() {
            return DataManager.FieldPoses.getNotePositions(noteIndex).getTranslation();
        }

        /**
         * Used to find the positions on the field from which to approach
         * and pick a note up from.
         * 
         * Assumes (1, 0) is 0 degrees
         * 
         * @return a pair containing first the approach {@link Pose2d} and then the pickup one
         * 
         * @author :3
         */
        public final Pair<Pose2d, Pose2d> getTrajectoryPositions() {
            Translation2d unitVector = new Translation2d(1, 0);
            Translation2d offsetVector = unitVector.rotateBy(this.rotation);
            Translation2d approachOffset = offsetVector.times(AutomaticsConstants.noteApproachDistance);
            Translation2d pickupOffset = offsetVector.times(AutomaticsConstants.notePickupDistance);

            Pose2d approachPose = new Pose2d(this.getLocation().plus(approachOffset), this.rotation.times(-1));
            Pose2d pickupPose = new Pose2d(this.getLocation().plus(pickupOffset), this.rotation.times(-1));
            return new Pair<Pose2d, Pose2d>(approachPose, pickupPose);
        }
    }

    /**
     * STUB
     * 
     * Used to generate a {@link Trajectory} that picks up
     * each of the given notes with the given angle.
     * 
     * Assumes (1, 0) is 0 degrees
     * 
     * @author :3
     */
    public static final Trajectory getPickupTrajectory(List<NoteAndRotation> notes) {
        ArrayList<Translation2d> waypoints = new ArrayList<>();
        for (NoteAndRotation note : notes) {
            Pair<Pose2d, Pose2d> positions = note.getTrajectoryPositions();
            waypoints.add(positions.getFirst().getTranslation());
            waypoints.add(positions.getSecond().getTranslation());
        }

        Pose2d start = DataManager.currentRobotPose.get().toPose2d();
        if (notes.size() == 0) {
            return TrajectoryGenerator.generateTrajectory(start, new ArrayList<>(), start, AutoConstants.kTrajectoryConfig);
        }
        
        NoteAndRotation finalNote = notes.get(notes.size() - 1);
        Pose2d end = new Pose2d(finalNote.getLocation(), finalNote.rotation.times(-1));
        waypoints.remove(waypoints.size() - 1);

        return TrajectoryGenerator.generateTrajectory(start, waypoints, end, AutoConstants.kTrajectoryConfig);
    }
}
