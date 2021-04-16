/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import static edu.wpi.first.wpilibj.XboxController.Axis.*;
import static edu.wpi.first.wpilibj.XboxController.Button.*;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

import frc.robot.shooter.ChangePosition;
import frc.robot.shooter.Conveyor;
import frc.robot.vision.AimTarget;
import frc.robot.vision.Limelight;
import frc.robot.climber.Lift;
import frc.robot.drive.Controller;
import frc.robot.drive.Gears;
import frc.robot.drive.RevDrivetrain;
import frc.robot.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import static frc.robot.Constants.*;
import static frc.robot.Gains.Ramsete.*;

import java.util.Arrays;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Drive Controller
  private XboxController xbox = new XboxController(kXboxPort);

  // Drive Subsystem
  private final RevDrivetrain rDrive = new RevDrivetrain();

  // Limelight Subsystem
  private final Limelight limelight = new Limelight();

  private final ChangePosition goalMover = new ChangePosition();

  private final Lift lift = new Lift();

  private final Shooter shooter = new Shooter(goalMover, limelight);

  private final Conveyor conveyor = new Conveyor(goalMover, shooter);

  private final Gears gears = new Gears();

  private final Controller xboxController = new Controller(xbox);

  // Update PID values
  private final Update update = new Update(shooter, goalMover);

  //  --- Default Commands ---

  // Drive with Controller 
  private Command manualDrive = new RunCommand(
    () -> rDrive.getDifferentialDrive().tankDrive(
      drivePercentLimit * rDrive.deadband(xbox.getRawAxis(kLeftY.value), percentDeadband), 
      drivePercentLimit * rDrive.deadband(xbox.getRawAxis(kRightY.value), percentDeadband),
      false
      ),
    rDrive
  );
  
  private Command moveLift = new RunCommand(
    () -> lift.move(xbox.getRawAxis(kRightTrigger.value) - xbox.getRawAxis(kLeftTrigger.value)), lift);

  // --- Command Groups ---

  private SequentialCommandGroup waitUntilVelocity = new SequentialCommandGroup(
    new WaitUntilCommand(() -> shooter.atSpeed()),
    new InstantCommand(() -> conveyor.setSpeed(conveyorVolts), conveyor)
  );

  private SequentialCommandGroup stopFeeders = new SequentialCommandGroup(
    new InstantCommand(() -> conveyor.stop(), conveyor));

  /**
   * Autonomous code
   */
  private SequentialCommandGroup shootThenGo = new SequentialCommandGroup(
    new InstantCommand(() -> goalMover.collectPose(), goalMover),
    
    new WaitCommand(.75),

    new InstantCommand(() -> goalMover.shootPose(), goalMover),
    new InstantCommand(() -> shooter.setSpeedSpark(), shooter),

    new WaitCommand(shooterRampUpTime).withInterrupt(goalMover::isPosOut),

    new InstantCommand(() -> conveyor.setSpeed(conveyorVolts), conveyor),

    new WaitCommand(2 + shooterRampUpTime),

    new InstantCommand(() -> conveyor.stop(), conveyor),
    new InstantCommand(() -> shooter.stop(), shooter),
    new RunCommand(() -> rDrive.getDifferentialDrive().tankDrive(0.4, 0.4), rDrive).withTimeout(2)
  );

  /**
   * Shakes the robot back and forth to dislodge balls
   */
  private SequentialCommandGroup robotShaker = new SequentialCommandGroup(     
    new RunCommand(() -> rDrive.getDifferentialDrive().tankDrive(0.3, -0.3), rDrive).withTimeout(1),
    new RunCommand(() -> rDrive.getDifferentialDrive().tankDrive(-0.3, 0.3), rDrive).withTimeout(1)
  );

/**
 * Tests each robot component individually. This command group runs in test mode.
 */
public SequentialCommandGroup testRobot = new SequentialCommandGroup(
  new RunCommand(() -> rDrive.getDifferentialDrive().tankDrive(0.4, 0.4), rDrive).withTimeout(2), //checks drivetrain
  new WaitCommand(.25),

  new InstantCommand(() -> gears.switchGears()), //checks gear switching solenoids
  new WaitCommand(.25),
  new InstantCommand(() -> gears.switchGears()),
  new WaitCommand(.25),

  new InstantCommand(() -> lift.move(.5)), //checks lift
  new WaitCommand(.5),
  new InstantCommand(() -> lift.stop()),
  new WaitCommand(.25),

  new InstantCommand(() -> shooter.toggleSpeedSpark()), //checks shooter
  new WaitCommand(.5), 
  new InstantCommand(() -> shooter.stop()),
  new WaitCommand(.25),

  new InstantCommand(() -> conveyor.toggleSpeed(conveyorVolts)), //checks conveyor
  new WaitCommand(.5), 
  new InstantCommand(() -> conveyor.stop()),
  new WaitCommand(.25),

  new InstantCommand(() -> goalMover.posSwitch()), //checks shooter solenoids
  new WaitCommand(.5),
  new InstantCommand(() -> goalMover.posSwitch())
);

  private RamseteCommand rBase = new RamseteCommand(
    getMovingTrajectory(), 
    rDrive::getPose, 
    new RamseteController(kBeta, kZeta), 
    rDrive.getFeedForwardDrive(), 
    rDrive.getKinematics(), 
    rDrive::getSpeeds, 
    rDrive.getLeftDrivePID(), 
    rDrive.getRightDrivePID(), 
    rDrive::setOutputVolts, 
    rDrive);

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() { 
    // Configure the button bindings
    configureButtonBindings();

    rDrive.setDefaultCommand(manualDrive);
    lift.setDefaultCommand(moveLift);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
    // Switch position between shooting and intake
    new JoystickButton(xbox, kA.value)
    .whenPressed(new InstantCommand(() -> goalMover.posSwitch(), goalMover));

    // Shoot or intake with voltage
    new JoystickButton(xbox, kBumperLeft.value)
    .whenPressed(new InstantCommand(() -> shooter.toggleSpeedVolts(), shooter))
    .whenPressed(new InstantCommand(() -> conveyor.toggleSpeed(conveyorVolts), shooter));
    
    // Shoot or intake with set RPM
    new JoystickButton(xbox, kB.value)
    .whenPressed(new InstantCommand(() -> shooter.toggleSpeedSpark()))
    .whenPressed(new ConditionalCommand(waitUntilVelocity, stopFeeders, shooter::isEngaged));

    // Vision correction
    new JoystickButton(xbox, kX.value)
    .whileHeld(new AimTarget(limelight, rDrive));

    // Switch Gears
    new JoystickButton(xbox, kBumperRight.value)
    .whenPressed(() -> gears.switchGears(), gears);

    //shakes the robot to dislodge balls
    new JoystickButton(xbox, kStart.value)
      .whileHeld(robotShaker)
      .whileHeld(new InstantCommand(() -> xboxController.startRumbleCalm()))
      .whenReleased(new InstantCommand(() -> xboxController.stopRumble()))
      .whenReleased(new RunCommand(() -> rDrive.getDifferentialDrive().tankDrive(0, 0), rDrive).withTimeout(0.001));
    
  }

  public void init(){
    limelight.driverMode();
    limelight.lightOff();
    limelight.PiPSecondaryStream();

    shooter.stop();
    conveyor.stop();
  } 

  public void periodic() {
    update.periodic();

    if (Timer.getMatchTime() < 30 && Timer.getMatchTime() > 28) {
      xboxController.startRumble();

    } else {
      xboxController.stopRumble();
    }
  }

  
  private Trajectory getMovingTrajectory() {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
      Arrays.asList(Update.getStartingPose(), new Pose2d(1.0, 0, new Rotation2d()),
        new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))), 
      new TrajectoryConfig(MaxSafeVelocityMeters, MaxSafeAccelerationMeters)
    );
    
    return trajectory;
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return shootThenGo;
  }
}
