// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class Teleop extends CommandBase {
  private DriveSubsystem drive;
  private DoubleSupplier leftJS;
  private DoubleSupplier rightJS;

  /** Creates a new Telop. */
  public Teleop(DriveSubsystem drive, DoubleSupplier leftJS, DoubleSupplier rightJS) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.drive = drive;
    this.leftJS = leftJS;
    this.rightJS = rightJS;

    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.TankDriveDiff(leftJS, rightJS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("[INTERRUPTED] " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
