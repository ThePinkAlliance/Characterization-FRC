// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.

  private final WPI_TalonFX m_left = setupWPI_TalonFX(DriveConstants.kLeftMotor1Port, Sides.LEFT, false);
  private final WPI_TalonFX m_leftFollower = setupWPI_TalonFX(DriveConstants.kLeftMotor2Port, Sides.FOLLOWER, false);
  private final WPI_TalonFX m_right = setupWPI_TalonFX(DriveConstants.kRightMotor1Port, Sides.RIGHT, false);
  private final WPI_TalonFX m_rightFollower = setupWPI_TalonFX(DriveConstants.kRightMotor2Port, Sides.FOLLOWER, false);

  private final SpeedControllerGroup m_rightControllerGroup = new SpeedControllerGroup(m_right, m_rightFollower);
  private final SpeedControllerGroup m_leftControllerGroup = new SpeedControllerGroup(m_left, m_leftFollower);

  private final int PIDIDX = 0;

  private Supplier<Double> leftEncoderPosition;
  private Supplier<Double> leftEncoderRate;
  private Supplier<Double> rightEncoderPosition;
  private Supplier<Double> rightEncoderRate;
  private Supplier<Double> gyroAngleRadians;

  public enum Sides {
    LEFT, RIGHT, FOLLOWER
  }

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftControllerGroup, m_rightControllerGroup);

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_drive.setSafetyEnabled(false);
    gyroAngleRadians = () -> -1 * Math.toRadians(m_gyro.getAngle());

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    m_rightFollower.follow(m_right);
    m_leftFollower.follow(m_left);

    m_left.setInverted(false);
    m_leftFollower.setInverted(false);

    m_right.setInverted(false);
    m_rightFollower.setInverted(false);
  }

  // methods to create and setup motors (reduce redundancy)
  public WPI_TalonFX setupWPI_TalonFX(int port, Sides side, boolean inverted) {
    // create new motor and set neutral modes (if needed)
    WPI_TalonFX motor = new WPI_TalonFX(port);
    // setup talon
    motor.configFactoryDefault();
    motor.setNeutralMode(NeutralMode.Brake);

    // setup encoder if motor isn't a follower
    if (side != Sides.FOLLOWER) {

      motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDIDX, 10);

      switch (side) {
        // setup encoder and data collecting methods

        case RIGHT:
          // set right side methods = encoder methods

          motor.setSensorPhase(false);
          rightEncoderPosition = () -> (motor.getSelectedSensorPosition(PIDIDX) * DriveConstants.encoderConstant);// -
                                                                                                                  // 1.0;
          rightEncoderRate = () -> (motor.getSelectedSensorVelocity(PIDIDX) * DriveConstants.encoderConstant * 10); // -
                                                                                                                    // 1.0;

          break;
        case LEFT:
          motor.setSensorPhase(false);

          leftEncoderPosition = () -> (motor.getSelectedSensorPosition(PIDIDX) * DriveConstants.encoderConstant);
          leftEncoderRate = () -> (motor.getSelectedSensorVelocity(PIDIDX) * DriveConstants.encoderConstant * 10);

          break;
        default:
          // probably do nothing
          break;
      }

    }

    return motor;
  }

  public double GetEncoderPos(WPI_TalonFX motor, Sides side) {
    switch (side) {
      // setup encoder and data collecting methods

      case RIGHT:
        // set right side methods = encoder methods

        double RightPos = (motor.getSelectedSensorPosition(PIDIDX) * DriveConstants.encoderConstant); // - 1.0;

        return RightPos;
      case LEFT:
        double LeftPos = (motor.getSelectedSensorPosition(PIDIDX) * DriveConstants.encoderConstant);

        return LeftPos;
      default:
        // probably do nothing
        return 0;

    }
  }

  public double GetEncoderRate(WPI_TalonFX motor, Sides side) {
    switch (side) {
      case RIGHT:
        double RightRate = (motor.getSelectedSensorVelocity(PIDIDX) * DriveConstants.encoderConstant * 10);
        return RightRate;

      case LEFT:
        double LeftRate = (motor.getSelectedSensorVelocity(PIDIDX) * DriveConstants.encoderConstant * 10);
        return LeftRate;

      default:
        return 0.0;
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    SmartDashboard.putNumber("encoder_left", GetEncoderPos(m_left, Sides.LEFT));
    SmartDashboard.putNumber("encoder_right", GetEncoderPos(m_right, Sides.RIGHT));

    SmartDashboard.putNumber("encoder_right_rate", rightEncoderRate.get());
    SmartDashboard.putNumber("encoder_left_rate", leftEncoderRate.get());
    SmartDashboard.putNumber("navx angle", m_gyro.getAngle());

    m_odometry.update(m_gyro.getRotation2d(), GetEncoderPos(m_left, Sides.LEFT), GetEncoderPos(m_right, Sides.RIGHT));
  }

  public void TankDriveDiff(DoubleSupplier l, DoubleSupplier r) {
    m_drive.tankDrive(l.getAsDouble(), r.getAsDouble());
    m_drive.feed();
  }

  public void TankDrive(DoubleSupplier l, DoubleSupplier r) {
    m_right.set(r.getAsDouble());
    m_right.feed();

    m_left.set(l.getAsDouble());
    m_left.feed();
  }

  public void ResetAll() {
    resetEncoders();
    resetOdometry(new Pose2d());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(GetEncoderRate(m_left, Sides.LEFT), GetEncoderRate(m_right, Sides.RIGHT));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_left.setVoltage(-leftVolts);
    m_right.setVoltage(rightVolts);
    m_drive.feed();
  }

  public void tankAutoDiff(double left, double right) {
    double left_power = left / 12;
    double right_power = right / 12;

    System.out.println("left " + left);
    System.out.println("right " + right);

    System.out.println("left_power " + left_power);
    System.out.println("right_power " + right_power);

    System.out.println("gyro " + m_gyro.getRotation2d());

    m_right.set(ControlMode.PercentOutput, right_power * -1.0);
    m_right.feed();

    m_left.set(ControlMode.PercentOutput, left_power);
    m_left.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_gyro.reset();

    m_left.setSelectedSensorPosition(0);
    m_right.setSelectedSensorPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (leftEncoderPosition.get() + rightEncoderPosition.get()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public Encoder getLeftEncoder() {
    return null;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public Encoder getRightEncoder() {
    return null;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }
}
