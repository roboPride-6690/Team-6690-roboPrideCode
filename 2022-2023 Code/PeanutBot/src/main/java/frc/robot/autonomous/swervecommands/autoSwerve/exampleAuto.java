package frc.robot.autonomous.swervecommands.autoSwerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveBase;
import java.util.List; 


public class exampleAuto extends SequentialCommandGroup {
  public exampleAuto(SwerveBase s_Swerve, double sx, double sy, double rx, double x, double y, double r) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.Drivebase.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.Drivebase.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.Drivebase.KINEMATICS);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(x/2, y/2)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(x, y, new Rotation2d(r)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.Drivebase.AutoConstants.kPThetaController,
            0,
            0,
            Constants.Drivebase.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    //SwerveControllerCommand swerveControllerCommand =
    //    new SwerveControllerCommand(
    //        exampleTrajectory,
    //        s_Swerve::getPose,
    //        Constants.Drivebase.KINEMATICS,
    //        new PIDController(Constants.Drivebase.AutoConstants.kPXController, 0, 0),
    //        new PIDController(Constants.Drivebase.AutoConstants.kPYController, 0, 0),
    //        thetaController,
    //        s_Swerve::setModuleStates,
    //        s_Swerve);

    //addCommands(
    //    new InstantCommand(() -> s_Swerve.resetOdometry(exampleTrajectory.getInitialPose())),
    //   swerveControllerCommand);
  }  

} 
