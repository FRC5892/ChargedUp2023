// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Intake.RetractIntake;
import frc.robot.commands.Intake.RunIntakeRollars;
import frc.robot.commands.Intake.ScoreLow;
import frc.robot.commands.Intake.ScoreMid;
import frc.robot.subsystems.*;

/* 
Summary:
This code is for the robot container and has a joy stick, joystick buttons, swerve subsystem, a sendable chooser for autonomous modes, autonomous modes, and methods for configuring button bindings and smart dashboard options. 
*/

public class RobotContainer {
  /* Controllers */
  public static final XboxController driver = new XboxController(0);

  /* Compressor */
  private Compressor compressor;


  /* Drive Controls */
  private static final int translationAxis = XboxController.Axis.kLeftY.value;
  private static final int strafeAxis = XboxController.Axis.kLeftX.value;
  private static final int rotationAxis = XboxController.Axis.kRightX.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton rotation0 = new JoystickButton(driver, XboxController.Button.kA.value);

  private final JoystickButton retractIntakeButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton scoreMidButton = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton scoreLowButton = new JoystickButton(driver, XboxController.Button.kBack.value);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();

  /* Commands */
  private final RunIntakeRollars runIntakeRollars = new RunIntakeRollars(intake, arm);
  private final RetractIntake retractIntake = new RetractIntake(arm);
  private final ScoreMid scoreMid = new ScoreMid(intake, arm);
  private final ScoreLow scoreLow = new ScoreLow(intake, arm);

  /* Autonomous Mode Chooser */
  private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

  /* Autonomous Modes */
  PathPlannerTrajectory moveForward = PathPlanner.loadPath("New",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory sCurve = PathPlanner.loadPath("S Curve",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory sussy = PathPlanner.loadPath("sussy",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    compressor = new Compressor(1, PneumaticsModuleType.REVPH);
    compressor.enableDigital();

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve, 
            () -> -driver.getRawAxis(translationAxis), 
            () -> -driver.getRawAxis(strafeAxis), 
            () -> -driver.getRawAxis(rotationAxis), 
            () -> robotCentric.getAsBoolean()
        )
    );

    // Configure the button bindings
    configureButtonBindings();

    // Configure Smart Dashboard options
    configureSmartDashboard();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    retractIntakeButton.onTrue(retractIntake);
    scoreMidButton.onTrue(scoreMid);
    scoreLowButton.onTrue(scoreLow);
}

  private void configureSmartDashboard() {
    autoChooser.setDefaultOption("Move forward", moveForward);
    autoChooser.addOption("S curve", sCurve);
    autoChooser.addOption("SUSSY - CADEN", sussy);

    SmartDashboard.putData(autoChooser);
  }

  public void disabledInit() {
    s_Swerve.resetToAbsolute();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Executes the autonomous command chosen in smart dashboard
    return new executeTrajectory(s_Swerve, autoChooser.getSelected());
  }
}
