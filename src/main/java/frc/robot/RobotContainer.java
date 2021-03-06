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
import frc.robot.commands.AlignToTargetLimelight;
import frc.robot.commands.AlignToTargetPhoton;
import frc.robot.commands.BellSpeedThroughTarget;
import frc.robot.commands.DriveToTargetPhoton;
//Pathweaver stuff
import frc.robot.PathweaverConstants;

import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import java.util.List;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import java.io.IOException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.PIDController;

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
  private final Limelight m_limelight = new Limelight();
  private final PhotonVision m_photonvision = new PhotonVision();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  //Pathwever
  public String trajectoryJSON;
  private Trajectory test1Trajectory;
  private Path trajectoryPath;
  private RamseteCommand ramseteCommand;

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
    //new JoystickButton(gamepad, XboxController.Button.kX.value).whenPressed(new AlignToTargetLimelight(m_limelight, m_drivetrain));
    new JoystickButton(gamepad, XboxController.Button.kX.value).whenPressed(new AlignToTargetPhoton(m_photonvision, m_drivetrain));
    new JoystickButton(gamepad, XboxController.Button.kBumperLeft.value).whenPressed(new DriveToTargetPhoton(m_photonvision, m_drivetrain));
    new JoystickButton(gamepad, XboxController.Button.kBumperRight.value).whileHeld(new BellSpeedThroughTarget(m_flagwaver, m_limelight));

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
    // return m_autoCommand;

    //Pathweaver stuff
    trajectoryJSON = "paths/Test1.wpilib.json";
    trajectoryPath =  Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    try {
      test1Trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    }
    catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    ramseteCommand = new RamseteCommand(
        test1Trajectory,
        m_drivetrain::getPose,
        new RamseteController(PathweaverConstants.kRamseteB, PathweaverConstants.kRamseteZeta),
        new SimpleMotorFeedforward(PathweaverConstants.ksVolts,
                                   PathweaverConstants.kvVoltSecondsPerMeter,
                                   PathweaverConstants.kaVoltSecondsSquaredPerMeter),
        PathweaverConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
        new PIDController(PathweaverConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::driveByVolts,
        m_drivetrain
    );

    m_drivetrain.resetOdometry(test1Trajectory.getInitialPose());
    return ramseteCommand.andThen(() -> m_drivetrain.driveByVolts(0, 0));
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
