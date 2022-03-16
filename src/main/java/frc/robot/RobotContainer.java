// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  String trajectoryJSON1 = "Path/first.wpilib.json";
  Trajectory trajectory1 = new Trajectory();
  
  String trajectoryJSON2 = "Path/second.wpilib.json";
  Trajectory trajectory2 = new Trajectory();

  String trajectoryJSON3 = "Path/third.wpilib.json";
  Trajectory trajectory3 = new Trajectory();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getY(), m_driverController.getX()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    new JoystickButton(m_driverController, 1)
        .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
        .whenReleased(() -> m_robotDrive.setMaxOutput(1));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
  
    // An example trajectory to follow.  All units in meters.


  try {
      Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory1: " + trajectoryJSON1, ex.getStackTrace());
   }

   try {
    Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
    trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);
 } catch (IOException ex) {
    DriverStation.reportError("Unable to open trajectory2: " + trajectoryJSON2, ex.getStackTrace());
 }

 try {
  Path trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON3);
  trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);
} catch (IOException ex) {
  DriverStation.reportError("Unable to open trajectory3: " + trajectoryJSON3, ex.getStackTrace());
}
/*
var autoVoltageConstraint =
new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(
        DriveConstants.ksVolts,
        DriveConstants.kvVoltSecondsPerMeter,
        DriveConstants.kaVoltSecondsSquaredPerMeter),
    DriveConstants.kDriveKinematics,
    10);



TrajectoryConfig config =
new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
    // Add kinematics to ensure max speed is actually obeyed
    .setKinematics(DriveConstants.kDriveKinematics)
    .addConstraint(autoVoltageConstraint);

// An example trajectory to follow.  All units in meters.
Trajectory sTrajectory =
TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(3, 0, new Rotation2d(0)),
    // Pass config
    config);

*/

  
/*new Pose2d(1.418903,1.474303, Rotation2d.fromDegrees(0)),
      List.of(new Translation2d(5.034577, 2.099259)),
      new Pose2d(7.416237, 0.586639, Rotation2d.fromDegrees(0)),  // x , y
      new TrajectoryConfig(Units.feetToMeters(3.0), Units.feetToMeters(3.0)));
*/


// Create and push Field2d to SmartDashboard.
/*Field2d m_field = new Field2d();
Field2d m_field2 = new Field2d();


SmartDashboard.putData(m_field);
SmartDashboard.putData(m_field2);


// Push the trajectory to Field2d.
m_field.getObject("Path").setTrajectory(trajectory);
//m_field.getObject("Enco-Gyro").setTrajectory(trajectory2);
*/

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            trajectory1,
            m_robotDrive::getPose,
            new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ksVolts,
                DriveConstants.kvVoltSecondsPerMeter,
                DriveConstants.kaVoltSecondsSquaredPerMeter),
            DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0.5, 0),
            new PIDController(DriveConstants.kPDriveVel, 0.5, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);
            
            RamseteCommand ramseteCommand2 =
            new RamseteCommand(
                trajectory2,
                m_robotDrive::getPose,
                new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    DriveConstants.ksVolts,
                    DriveConstants.kvVoltSecondsPerMeter,
                    DriveConstants.kaVoltSecondsSquaredPerMeter),
                DriveConstants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(DriveConstants.kPDriveVel, 0.5, 0),
                new PIDController(DriveConstants.kPDriveVel, 0.5, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts,
                m_robotDrive);

                RamseteCommand ramseteCommand3 =
                new RamseteCommand(
                    trajectory3,
                    m_robotDrive::getPose,
                    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(
                        DriveConstants.ksVolts,
                        DriveConstants.kvVoltSecondsPerMeter,
                        DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics,
                    m_robotDrive::getWheelSpeeds,
                    new PIDController(DriveConstants.kPDriveVel, 0.5, 0),
                    new PIDController(DriveConstants.kPDriveVel, 0.5, 0),
                    // RamseteCommand passes volts to the callback
                    m_robotDrive::tankDriveVolts,
                    m_robotDrive);

                    /*
                    RamseteCommand SramseteCommand =
                    new RamseteCommand(
                        sTrajectory,
                        m_robotDrive::getPose,
                        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                        new SimpleMotorFeedforward(
                            DriveConstants.ksVolts,
                            DriveConstants.kvVoltSecondsPerMeter,
                            DriveConstants.kaVoltSecondsSquaredPerMeter),
                        DriveConstants.kDriveKinematics,
                        m_robotDrive::getWheelSpeeds,
                        new PIDController(DriveConstants.kPDriveVel, 0.5, 0),
                        new PIDController(DriveConstants.kPDriveVel, 0.5, 0),
                        // RamseteCommand passes volts to the callback
                        m_robotDrive::tankDriveVolts,
                        m_robotDrive);
*/



    
    // Reset odometry to the starting pose of the trajectory.
    //m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    
    
    return ramseteCommand.andThen(() -> ramseteCommand2.andThen(() -> ramseteCommand3.andThen(() -> m_robotDrive.tankDriveVolts(0, 0))));

    //return SramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));


  }
}
