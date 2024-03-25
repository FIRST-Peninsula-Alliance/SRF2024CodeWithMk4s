// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.*;

import java.util.List;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class JustExitCmd extends SequentialCommandGroup {
  private SwerveSubsystem   m_swerveDrive;

  /** Creates a new ScoreAndMove. */
  public JustExitCmd(SwerveSubsystem swerveDrive) {
    m_swerveDrive = swerveDrive;

    TrajectoryConfig configExit =
            new TrajectoryConfig(AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                    AutoC.AUTO_SPEED_FACTOR_GENERIC,
                                AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                    AutoC.AUTO_ACCEL_FACTOR_GENERIC)
                .setKinematics(SDC.SWERVE_KINEMATICS);
                // .addConstraint(AutoConstants.autoVoltageConstraint);
        configExit.setReversed(false);

    TrajectoryConfig configReturn =
            new TrajectoryConfig(AutoC.AUTO_MAX_SPEED_M_PER_SEC *
                                    AutoC.AUTO_SPEED_FACTOR_GENERIC,
                                AutoC.AUTO_MAX_ACCEL_M_PER_SEC2 *
                                    AutoC.AUTO_ACCEL_FACTOR_GENERIC)
                .setKinematics(SDC.SWERVE_KINEMATICS);
                // .addConstraint(AutoConstants.autoVoltageConstraint);
        configReturn.setReversed(true);

        // An example trajectory to follow, one path.  All units in meters.

    Trajectory exitTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the -X direction, away from the
            // speaker goal. Position bumpers up against the subwoofer 
            // front, even if offset to one side (i.e. no angle shot with
            // this auto). Because the subwoofer extends ~1 m into the field,
            // the origin X is not really 0, but for now, consider it good.
            new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(300.0)),
            List.of(new Translation2d(.25, 0.0),
                    new Translation2d(0.5, 0.0),
                    new Translation2d(0.75, 0.0),
                    new Translation2d(1.0, 0.0),
                    new Translation2d(1.25, 0.0)),
            new Pose2d(1.5, 0.0, Rotation2d.fromDegrees(0.0)),     //Math.PI/2)
            configExit);

    Trajectory returnTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the end of the exit trajectory, and return to 0, 0
            new Pose2d(1.5, 0.0, Rotation2d.fromDegrees(0.0)),
            List.of(new Translation2d(1.25, 0.0),
                    new Translation2d(1.0, 0.0)),
            new Pose2d(0.75, 0.0, Rotation2d.fromDegrees(0.0)),     //Math.PI/2)
            configReturn);

    ProfiledPIDController thetaController =
          new ProfiledPIDController(AutoC.KP_THETA_CONTROLLER,
                                    AutoC.KI_THETA_CONTROLLER,
                                    0,
                                    AutoC.K_THETA_CONTROLLER_CONSTRAINTS);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerExitCmd =
          new SwerveControllerCommand(
              exitTrajectory,
              m_swerveDrive::getPose,
              SDC.SWERVE_KINEMATICS,
              new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, 0),
              new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
              thetaController,
              m_swerveDrive::setModuleStates,
              m_swerveDrive);

    SwerveControllerCommand swerveControllerReturnCmd =
          new SwerveControllerCommand(
              returnTrajectory,
              m_swerveDrive::getPose,
              SDC.SWERVE_KINEMATICS,
              new PIDController(AutoC.KP_X_CONTROLLER, AutoC.KI_X_CONTROLLER, 0),
              new PIDController(AutoC.KP_Y_CONTROLLER, 0, 0),
              thetaController,
              m_swerveDrive::setModuleStates,
              m_swerveDrive);

    addCommands(
                new InstantCommand(() -> m_swerveDrive.resetOdometry(exitTrajectory.getInitialPose())),
                swerveControllerExitCmd,
                new WaitCommand(1.0),
                swerveControllerReturnCmd,
                new InstantCommand(()-> m_swerveDrive.stop())
               );
    }
}
