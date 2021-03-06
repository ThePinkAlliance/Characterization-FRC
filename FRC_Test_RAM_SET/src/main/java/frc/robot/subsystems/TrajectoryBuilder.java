// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.robot.subsystems.TrajectoryBuilder;

public class TrajectoryBuilder extends SubsystemBase {
  /** Creates a new TrajectoryBuilder. */

  public DriveSubsystem m_drive;

  public DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter),
      DriveConstants.kDriveKinematics, 5);

  // Create config for trajectory
  public TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          // Add kinematics to ensure max speed is actually obeyed
          .setKinematics(DriveConstants.kDriveKinematics)
          // Apply the voltage constraint
          .addConstraint(autoVoltageConstraint);

  public TrajectoryBuilder(DriveSubsystem m_drive) {
    this.m_drive = m_drive;
  }

  public Trajectory Create(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end) {
    // An example trajectory to follow. All units in meters.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        start,
        // Pass through these two interior waypoints, making an 's' curve path
        interiorWaypoints, end,
        // Pass config
        this.config);

    return trajectory;
  }

  public Trajectory ReadTrajectorys(String path) {
    Trajectory trajectory = new Trajectory();
    Path resolvedPath = Filesystem.getDeployDirectory().toPath().resolve(path);

    try {
      trajectory = TrajectoryUtil.fromPathweaverJson(resolvedPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + resolvedPath, ex.getStackTrace());
    }

    return trajectory;
  }

  // if there is an issue it should be in here
  public Command Drive(Trajectory trajectory) {
    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_drive::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drive::tankDriveVolts, m_drive);

    // Reset odometry to the starting pose of the trajectory.
    m_drive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_drive.tankDriveVolts(0, 0));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
