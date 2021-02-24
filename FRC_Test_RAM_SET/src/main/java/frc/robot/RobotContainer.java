// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.commands.CommandDrive;
import frc.robot.commands.Teleop;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TrajectoryBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems

    private DriveSubsystem drive = new DriveSubsystem();
    private TrajectoryBuilder builder = new TrajectoryBuilder(drive);
    // private final CommandDrive commandDrive = new CommandDrive();
    private Joystick baseJS = new Joystick(0);

    // The driver's controller

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */

    public RobotContainer() {
        // Configure the button bindings

        // Configure default commands
        // Set the default drive command to split-stick arcade drive

        drive.setDefaultCommand(
                new Teleop(this.drive, () -> this.baseJS.getRawAxis(1), () -> this.baseJS.getRawAxis(3)));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return builder.getAutonomousCommand(drive);
    }
}
