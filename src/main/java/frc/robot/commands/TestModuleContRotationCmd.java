// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class TestModuleContRotationCmd extends Command {
  private SwerveSubsystem m_swerve;
  private double m_dir;
  private double m_testHeadingDeg;


  /** Creates a new TestModuleRotationCmd. */
  public TestModuleContRotationCmd(SwerveSubsystem swerve, double dir) {
    m_swerve = swerve;
    m_dir = dir;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_testHeadingDeg = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_swerve.rotateModulesToAngle(m_testHeadingDeg);
    m_testHeadingDeg += (1.25 * m_dir);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.rotateModulesToAngle(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(m_testHeadingDeg) >= 360.0);
  }
}
