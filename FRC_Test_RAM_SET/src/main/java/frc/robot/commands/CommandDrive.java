// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TrajectoryBuilder;

public class CommandDrive extends CommandBase {
  /** Creates a new CommandDrive. */

  private final DriveSubsystem m_drive = new DriveSubsystem();
  public final TrajectoryBuilder m_builder = new TrajectoryBuilder(m_drive);

  public CommandDrive() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
    addRequirements(m_builder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("command initialized");

    Trajectory cmdDrive = m_builder.Create(AutoConstants.startPos, List.of(new Translation2d(4, 0)),
        new Pose2d(4, 0, new Rotation2d(0)));

    m_builder.Drive(cmdDrive);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
