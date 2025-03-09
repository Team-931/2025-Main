package frc.robot.utils;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Subsystems.Drivetrain;

public class AutoAlignUtil {
    public static enum CoralScoreDirection {
        LEFT, RIGHT;
    }

    private static Distance CORAL_SCORE_OFFSET = Units.Inches.of(6);
    
    private static Distance ROBOT_WIDTH = Units.Centimeters.of(73.5);
    private static Distance BUMPER_THICKNESS = Units.Centimeters.of(8.5);
    private static Distance FUDGE_FACTOR = Units.Centimeters.of(3);

    public static Pose2d getClosestAprilTagPose(Pose2d robotPose) {
        Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        
        List<Pose2d> coralAprilTagPoses = AprilTagDataUtil.get().getCoralAprilTagPoses(
            alliance.isPresent() ? alliance.get() : DriverStation.Alliance.Blue
        );

        if (coralAprilTagPoses.size() == 0) {
            return null;
        }

        return robotPose.nearest(coralAprilTagPoses);
    }

    public static Pose2d offsetAprilTagPose(Pose2d aprilTagPose, CoralScoreDirection coralScoreDirection) {
        // An AprilTag with 0 rotation faces the Red alliance wall.
        // This means that scoring "Right" is actually "Up", relative to the field.

        // ------------------------ Top of Field
        //
        //                                      ^ Positive Y             |
        //                                      |                        |
        //    T____T                            |---> Positive X         |
        //   /      \                                                    |
        //  /        \                                                   |
        // T          T ---->   0 rotation                               | Red Alliance Wall
        //  \        /                                                   |
        //   \______/                                                    |
        //   T      T                                                    |
        // 
        // ------------------------ Bottom of Field

        // Left is the opposite direction or Right
        Distance directionalOffset = coralScoreDirection == CoralScoreDirection.LEFT ? CORAL_SCORE_OFFSET.times(-1) : CORAL_SCORE_OFFSET;

        Pose2d poseAdjustment = new Pose2d(
            ROBOT_WIDTH.div(2).plus(BUMPER_THICKNESS).plus(FUDGE_FACTOR), 
            directionalOffset,
            new Rotation2d()
        );

        // Account for AprilTag Angle
        poseAdjustment = poseAdjustment.rotateBy(aprilTagPose.getRotation());

        return new Pose2d(
            aprilTagPose.getTranslation().plus(poseAdjustment.getTranslation()), 
            aprilTagPose.getRotation()
        );
    }

    // To aid in visualizing how the field, AprilTags, and AutoAlignment work, we have a website:
    // https://team-6615-bellevillains.github.io/AprilTagVisualizer/
    // It does not work on mobile.
    public static Pose2d calculateTargetPose(Pose2d robotPose, CoralScoreDirection coralScoreDirection) {
        Pose2d poseOfTargettedTag = getClosestAprilTagPose(robotPose);

        if (poseOfTargettedTag == null) {
            return null;
        }
        System.out.println(poseOfTargettedTag);
        
        Pose2d finalPose = offsetAprilTagPose(poseOfTargettedTag, coralScoreDirection);

        System.out.println(finalPose);

        return finalPose;
    }

    // Credit to FRC3136 ORCA (Official Robot Constructors of Ashland)
    private static Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target, Drivetrain Drivetrain) {
        if (Drivetrain.getVelocityMagnitude().in(MetersPerSecond) < 0.25) {
            var diff = target.minus(Drivetrain.getPose()).getTranslation();
            return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();// .rotateBy(Rotation2d.k180deg);
        }
        return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
    }

    // Credit to FRC3136 ORCA (Official Robot Constructors of Ashland)
    private static Command goToTargetPoseOrcaMethod(Pose2d targetPose, Drivetrain Drivetrain) {
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(Drivetrain.getPose().getTranslation(), getPathVelocityHeading(Drivetrain.getFieldVelocity(), targetPose, Drivetrain)),
            targetPose
        );

        if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
            return Commands.print("Auto alignment too close to desired position to continue");
        }

        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            new PathConstraints(
                Units.FeetPerSecond.of(2), 
                Units.FeetPerSecondPerSecond.of(4), 
                Units.RotationsPerSecond.of(100), 
                Units.RotationsPerSecondPerSecond.of(720)
            ),
            new IdealStartingState(Drivetrain.getVelocityMagnitude(), Drivetrain.getHeading()), 
            new GoalEndState(0.0, targetPose.getRotation())
        );

        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
    }

    private static Command buildAutoAlign(Drivetrain Drivetrain, CoralScoreDirection coralScoreDirection) {
        Pose2d targetPose = calculateTargetPose(Drivetrain.getPose(), coralScoreDirection);
        if (targetPose == null) {
            return Commands.print("Failed to load field data when calculating target pose!");
        }

        // return Commands.print("NOOP");
        //8.5
        
        return Commands.print("Auto Aligning to " + targetPose)
            .andThen(Commands.runOnce(() -> {
                SmartDashboard.putBoolean("Auto Align Status", false);
            }))
            .alongWith(
                // AutoBuilder.pathfindToPose(
                //     targetPose, 
                //     new PathConstraints(
                //         Units.FeetPerSecond.of(1), 
                //         Units.FeetPerSecondPerSecond.of(4), 
                //         Units.RotationsPerSecond.of(360), 
                //         Units.RotationsPerSecondPerSecond.of(300)
                //     )
                // )
                goToTargetPoseOrcaMethod(targetPose, Drivetrain)
                // Cancel pathfinding if Driver wants to take over
                .until(Drivetrain::isBeingControlledByHuman) 
            )
            .finallyDo((interrupted) -> {
                SmartDashboard.putBoolean("Auto Align Status", !interrupted);
            });
    }

    // This command is structured with a Supplier and Proxy
    // because Commands are generated when the code is first ran,
    // but we want to be able to generate the alignment command
    // using the current pose.
    //
    // If we weren't using PathPlanner generated commands,
    // it would probably be better to have a regular Command
    // and then pass the "starting robot pose" in with a 
    // Supplier<Pose2d>
    public static Command autoAlign(Drivetrain Drivetrain, CoralScoreDirection coralScoreDirection) {
        Supplier<Command> autoAlignSupplier = () -> buildAutoAlign(Drivetrain, coralScoreDirection);
        return Commands.deferredProxy(autoAlignSupplier);
    }
    
}