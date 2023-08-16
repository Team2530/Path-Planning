// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;

import java.io.File;
import java.io.IOException;
import java.nio.file.*;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // ---------- Controllers ---------- \\
  public static final XboxController xboxController = new XboxController(ControllerConstants.XBOX_CONTROLLER_PORT);
  public static final Joystick joystick = new Joystick(ControllerConstants.JOYSTICK_PORT);

  // --------- Subsystems ---------- \\
  private static final DriveTrain driveTrain = new DriveTrain();

  // ---------- Commands ---------- \\
  private static final JoystickDrive normalJoystickDrive = new JoystickDrive(driveTrain);
  private static final SlowJoystickDrive slowJoystickDrive = new SlowJoystickDrive(driveTrain);

  PathPlannerTrajectory trajectory = PathPlanner.loadPath("A", new PathConstraints(4, 4));

  private SendableChooser<Command> autoChooser = new SendableChooser<Command>() {
    {
      // ? Example 1 Piece Auto
      setDefaultOption("1 Piece Auto", new SequentialCommandGroup(
          setInitialAutoPose("1PieceStart"),
          new PrintCommand("Auto Start"),
          new PrintCommand("Placing Piece"),
          new WaitCommand(3), // Grabber Place
          new PrintCommand("Piece Placed"),
          loadPathPlannerTrajectory("1PieceStart", true), // Go to grab piece
          new InstantCommand(() -> driveTrain.stop()),
          new PrintCommand("Arm Down"),
          new WaitCommand(2), // Arm down
          new PrintCommand("Piece Grabbed"),
          new WaitCommand(2), // Pick arm back up
          new PrintCommand("Arm Back Up"),
          loadPathPlannerTrajectory("1PieceBack", false), // Go back to place piece
          new InstantCommand(() -> driveTrain.stop()),
          new WaitCommand(3), // Place piece
          new PrintCommand("Piece Placed"),
          new WaitCommand(2), // Whatever else
          new PrintCommand("End Path")));
    }
  };

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    // Init sendable chooser options with path options from pathplanner
    // ! Use with caution, only for standalone paths for testing purposes
    // setPathOptions();

    SmartDashboard.putString("Drive Style", "Normal Drive");
    driveTrain.setDefaultCommand(normalJoystickDrive);
    Shuffleboard.getTab("Autonomous").add(autoChooser).withPosition(0, 0).withSize(2, 3);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    // ! Change button on real Robot
    // ? Changes drive mode based on button 2
    new JoystickButton(joystick, ControllerConstants.J_SLOW_MODE_BUTTON).whileTrue((new InstantCommand(() -> {
      driveTrain.getDefaultCommand().cancel();
      driveTrain.setDefaultCommand(slowJoystickDrive);
      SmartDashboard.putString("Drive Style", "Slow Drive");
    }))).whileFalse(new InstantCommand(() -> {
      driveTrain.getDefaultCommand().cancel();
      driveTrain.setDefaultCommand(normalJoystickDrive);
      SmartDashboard.putString("Drive Style", "Normal Drive");
    }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Call to stop everything on robot
   */
  public void stopRobot() {
    // TODO: Add other stops
    driveTrain.stop();
  }

  /**
   * Loads and returns path following command
   * 
   * @param file          file extension to PathPlanner JSON
   * @param resetOdometry if true, odometry will be set to initial pose of
   *                      trajectory
   * @return Ramsete command for path following
   */
  private Command loadPathPlannerTrajectory(String file, boolean resetOdometry) {
    file = pathToJSON(file) + ".wpilib.json";
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(file);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      System.out.println(String.format("Cannot find path %s", file) + " " + e.getStackTrace().toString());
      return new PrintCommand("Trajectory unable to be loaded, Auto is running...");
    }

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, driveTrain::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kKinematics, driveTrain::getWheelSpeeds, new PIDController(DriveConstants.kPDriveVel, 0, 0),
        new PIDController(DriveConstants.kPDriveVel, 0, 0), driveTrain::tankDriveVolts, driveTrain);

    if (resetOdometry) {
      return new SequentialCommandGroup(new InstantCommand(() -> driveTrain.resetOdometry(trajectory.getInitialPose())),
          ramseteCommand);
    }

    return ramseteCommand;

  }

  /**
   * Sets initial auto pose to whatever is initial pose of path is
   * 
   * @param file path to "path" file
   * @return Instant command to reset odometry, and if file not found, print
   *         command
   *         statin that the file couldn't be found
   */
  private Command setInitialAutoPose(String file) {
    file = pathToJSON(file) + ".wpilib.json";
    Trajectory trajectory;
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(file);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return new InstantCommand(() -> {
        driveTrain.resetOdometry(trajectory.getInitialPose());
      });
    } catch (IOException e) {
      System.out.println(String.format("Cannot find path %s", file) + " " + e.getStackTrace().toString());
      return new PrintCommand("Trajectory unable to be loaded, initial pose couldn't be reset");
    }
  }

  /**
   * Puts file extension in front of provided file, so all computers should be
   * able to test paths
   * <p>
   * Note: May need to be changed if deploy directory is somewhere else...
   * 
   * @param pathName File name of JSON path
   * @return String to path of file
   */
  private String pathToJSON(String pathName) {
    Path currentPath = Paths.get("");
    String path = currentPath.toAbsolutePath().toString()
        // Check to see if path is correct
        + "/src/main/java/frc/robot/deploy/deploy/pathplanner/generatedJSON/"
        + pathName;
    return path;
  }

  /**
   * Finds all paths in pathplanner directory and sets them as options in the
   * autochooser
   * <p>
   * <b>ONLY USE IF THERE ARE SINGLE PATHS FOR ALL AUTO PATHS (NOT NEEDED TO
   * CONTATENATE OTHER THINGS)</b>
   * </p>
   * 
   */
  private void setPathOptions() {
    String absolutePath = Paths.get("").toAbsolutePath().toString();
    File[] autoPaths = new File(absolutePath + "/src/main/java/frc/robot/deploy/deploy/pathplanner/generatedJSON/")
        .listFiles();

    for (int i = 0; i < autoPaths.length; i++) {
      // Set first one to default one
      String name = autoPaths[i].getName().substring(0, autoPaths[i].getName().indexOf("."));
      if (i == 0) {
        autoChooser.setDefaultOption(name, loadPathPlannerTrajectory(pathToJSON(autoPaths[i].getName()), true));
      } else {
        autoChooser.addOption(name, loadPathPlannerTrajectory(pathToJSON(autoPaths[i].getName()), true));
      }
    }
  }
}
