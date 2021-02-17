// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.*;
import frc.robot.custom_triggers.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID.Hand;

// Imported Commands
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FlagWave;
import frc.robot.commands.FlagStop;
import frc.robot.commands.StraightDrive;
import frc.robot.commands.FaceAngle;
import frc.robot.commands.AlignToTarget;
import frc.robot.commands.BellSpeedThroughTarget;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final FlagWaver m_flagwaver = new FlagWaver();
  private final Vision m_vision = new Vision();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private static final XboxController gamepad = new XboxController(Constants.gamepadPort);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(gamepad, XboxController.Button.kA.value).whenPressed(new FlagWave(m_flagwaver));
    new JoystickButton(gamepad, XboxController.Button.kB.value).whenPressed(new FlagStop(m_flagwaver));
    new JoystickButton(gamepad, XboxController.Button.kY.value).whileHeld(new StraightDrive(m_drivetrain));
    new JoystickButton(gamepad, XboxController.Button.kX.value).whenPressed(new AlignToTarget(m_vision, m_drivetrain));
    new JoystickButton(gamepad, XboxController.Button.kBumperRight.value).whileHeld(new BellSpeedThroughTarget(m_flagwaver, m_vision));

    new GetDpadUp().whenPressed(new FaceAngle(0, m_drivetrain));
    new GetDpadRight().whenPressed(new FaceAngle(90, m_drivetrain));
    new GetDpadDown().whenPressed(new FaceAngle(180, m_drivetrain));
    new GetDpadLeft().whenPressed(new FaceAngle(270, m_drivetrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }

  public static double getLeftStickY()
  {
    return RobotContainer.gamepad.getY(Hand.kLeft);
  }

  public static double getRightStickY()
  {
    return RobotContainer.gamepad.getY(Hand.kRight);
  }

  public static boolean getDpadUp()
  {
    return gamepad.getPOV()==0;
  }

  public static boolean getDpadLeft()
  {
    return gamepad.getPOV()==90;
  }

  public static boolean getDpadDown()
  {
    return gamepad.getPOV()==180;
  }

  public static boolean getDpadRight()
  {
    return gamepad.getPOV()==270;
  }
}
