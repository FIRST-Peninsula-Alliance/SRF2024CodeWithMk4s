// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Score2NotesAuto extends SequentialCommandGroup {
  private SwerveSubsystem m_swerveDrive;

  /** Creates a new Score2NotesAuto. */
  public Score2NotesAuto(SwerveSubsystem swerveDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
                // new ScoreIndexedSpeakerCmd(m_noteConductor),
                // new Goto2ndNoteCmd(m_swerveDrive).alongwith(new AcquireNoteCmd()),
                // new ScoreDistantSpeakerCmd(m_noteConductor),
                new JustExitCmd(m_swerveDrive)
               );
  }
}