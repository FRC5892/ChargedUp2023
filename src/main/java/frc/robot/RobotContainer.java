// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Balance.ActiveBalance;
import frc.robot.commands.Balance.PassiveBalance;
import frc.robot.commands.Intake.intake;
import frc.robot.commands.Intake.retract;
import frc.robot.commands.Scoring.scoreFull;
import frc.robot.commands.Scoring.scoreShort;
import frc.robot.subsystems.*;

/* 
Summary:
This code is for the robot container and has a joy stick, joystick buttons, swerve subsystem, a sendable chooser for autonomous modes, autonomous modes, and methods for configuring button bindings and smart dashboard options. 
*/

public class RobotContainer {
  /* Controllers */
  public final static Joystick driver = new Joystick(0);
  private final Joystick codriver = new Joystick(1);

  /* Compressor */
  private Compressor compressor;

  /* LED Strip */
  public static AddressableLED m_led;
  public static AddressableLEDBuffer m_ledBuffer;

  // Gyro Sensor
  private Pigeon2 gyro = new Pigeon2(Constants.Swerve.pigeonID);

  /* Drive Controls */
  private static final int translationAxis = XboxController.Axis.kLeftY.value;
  private static final int strafeAxis = XboxController.Axis.kLeftX.value;
  private static final int rotationAxis = XboxController.Axis.kRightX.value;
  private double SPEED_MULTIPLIER = 1.0;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton outtakeButton = new JoystickButton(codriver, XboxController.Button.kB.value);
  private final JoystickButton retractButton = new JoystickButton(codriver, XboxController.Button.kA.value);
  private final JoystickButton intakeButton = new JoystickButton(codriver, XboxController.Button.kX.value);
  public final static JoystickButton activeBalanceButton = new JoystickButton(driver,
      XboxController.Button.kRightBumper.value);
  private final JoystickButton passiveBalanceButton = new JoystickButton(driver,
      XboxController.Button.kLeftBumper.value);
  private final JoystickButton intakeFullButton = new JoystickButton(codriver, XboxController.Button.kY.value);
  private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

  /* Subsystems */
  public final static VisionSubsystem s_visionSubsystem = new VisionSubsystem();

  /* Commands */
  private final Swerve s_Swerve = new Swerve(gyro);
  private final Ground_Intake ground_intake = new Ground_Intake();
  // private final ActiveBalanceDavis autobalance = new
  // ActiveBalanceDavis(s_Swerve, gyro);
  private final PassiveBalance passiveBalance = new PassiveBalance(s_Swerve);
  private final ActiveBalance activeBalance = new ActiveBalance(s_Swerve, gyro);

  /* Pneumatics Commands */
  public final Command intake = new intake(ground_intake);
  public final Command outtake = new scoreShort(ground_intake);
  public final Command retract = new retract(ground_intake);
  public final Command outtakeFull = new scoreFull(ground_intake);

  /* Autonomous Mode Chooser */
  private final SendableChooser<PathPlannerTrajectory> autoChooser = new SendableChooser<>();

  /* Autonomous Modes */
  PathPlannerTrajectory Score1 = PathPlanner.loadPath("1 Score",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory Score1andLineCrossShort = PathPlanner.loadPath("1 Score + Line Cross Short",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory Score1andLineCross = PathPlanner.loadPath("1 Score + Line Cross",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory Score1ChargeStation = PathPlanner.loadPath("1 Score + Charge Station",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory Score1CSLC = PathPlanner.loadPath("1 Score + Charge Station + Line Cross",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
  PathPlannerTrajectory Score2 = PathPlanner.loadPath("2 Score",
      2, 1);
  PathPlannerTrajectory Score1LCC = PathPlanner.loadPath("1 Score + Line Cross Cable",
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
    compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    compressor.enableDigital();

    m_led = new AddressableLED(Constants.LEDConstants.LED_PORT);
    m_ledBuffer = new AddressableLEDBuffer(1);
    m_led.setData(m_ledBuffer);
    m_led.setLength(Constants.LEDConstants.LED_LENGTH);
    m_led.start();

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,

            () -> -driver.getRawAxis(translationAxis) * SPEED_MULTIPLIER,
            () -> -driver.getRawAxis(strafeAxis) * SPEED_MULTIPLIER,
            () -> -driver.getRawAxis(rotationAxis) * SPEED_MULTIPLIER,
            () -> robotCentric.getAsBoolean()));
    SmartDashboard.putNumber("Speed Multipler", SPEED_MULTIPLIER);

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

    /* Pneumatics Buttons */
    intakeButton.onTrue(intake);
    outtakeButton.onTrue(outtake);
    retractButton.onTrue(retract);
    activeBalanceButton.onTrue(activeBalance);
    passiveBalanceButton.onTrue(passiveBalance);
    intakeFullButton.onTrue(outtakeFull);

  }

  private void configureSmartDashboard() {
    autoChooser.setDefaultOption("1 Score", Score1);
    autoChooser.addOption("1 Score + Line Cross", Score1andLineCross);
    autoChooser.addOption("1 Score + Line Cross Short", Score1andLineCrossShort);
    autoChooser.addOption("1 Score + Charge Station", Score1ChargeStation);
    autoChooser.addOption("2 Score Left", Score2);
    autoChooser.addOption("1 Score + Charge Station + Line Cross", Score1CSLC);
    autoChooser.addOption("1 Score + Line Cross Cable", Score1LCC);

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
    return new executeTrajectory(s_Swerve, autoChooser.getSelected(), outtake, retract, intake, activeBalance,
        outtakeFull);
  }
}
