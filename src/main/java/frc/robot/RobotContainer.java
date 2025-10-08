/* This class is where the bulk of the robot should be declared. Since Command-based is a
"declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
subsystems, Commands, and button mappings) should be declared here */

package frc.robot;
import java.time.Instant;
import java.util.function.BooleanSupplier;
import java.util.jar.Attributes.Name;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TeleopSwerve;
import frc.robot.Commands.toggleSpeed;
import frc.robot.Commands.toggleSpeedExtra;

// Subsystem imports
import frc.robot.Subsystems.AlgaePickup;
import frc.robot.Subsystems.AlgaePivot;
import frc.robot.Subsystems.Climber;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.ElevatorPivot;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.IntakePivot;
import frc.robot.Subsystems.Swerve;
import frc.robot.Subsystems.SwerveModule; // Not sure if we need this but im declaring it anyway
import frc.robot.Subsystems.Vision;

public class RobotContainer {
  // Sets the xBox Controllers
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController second = new CommandXboxController(1);
  private final CommandXboxController backup = new CommandXboxController(2);
 
  // Drive Control Declarations
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value ;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  // Driver Buttons
  // private final JoystickButton zeroGyro =
  //   new JoystickButton(driver, XboxController.Button.kY.value);
  // private final JoystickButton robotCentric =
  //   new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

  // Subsystems object declarations
  //private final AlgaePickup algaePickup = new AlgaePickup();
 // private final AlgaePivot algaePivot = new AlgaePivot();
  private final Climber climber = new Climber();
  private final Elevator elevator = new Elevator();
  private final ElevatorPivot elevatorPivot = new ElevatorPivot();
  private final Intake intake = new Intake();
  private final IntakePivot intakePivot = new IntakePivot();
  private final Vision vision = new Vision();
  private final Swerve s_Swerve = new Swerve(vision);




  private final SendableChooser<Command> chooser;

  // The container for the robot. Contains subsystems, OI devices, and Commands
  
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -driver.getRawAxis(translationAxis),
            () -> -driver.getRawAxis(strafeAxis),
            () -> -driver.getRawAxis(rotationAxis)/2));

    // Puts Sendable Chooser on SmartDashboard
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto:", chooser);

    // Configure the button bindings
    configureButtonBindings();  

    NamedCommands.registerCommand("elevator pivot score L1 command", elevatorPivot.elevatorPivotScoreL1Command());
    NamedCommands.registerCommand("elevator pivot score L4 command", elevatorPivot.elevatorPivotScoreL4Command());
    NamedCommands.registerCommand("elevator score L1 command", elevator.elevatorScoreL1Command());
    NamedCommands.registerCommand("elevator score L4 command", elevator.elevatorScoreL4Command());
    NamedCommands.registerCommand("Intake score L2 and L3 command", intake.intakeScoreCoralL2and3Command());
    NamedCommands.registerCommand("Intake pivot score L1 command", intakePivot.intakePivotScoreL1Command());
    NamedCommands.registerCommand("Intake pivot score L4 command", intakePivot.intakePivotScoreL4Command());
    NamedCommands.registerCommand("Intake auto score L4", intake.intakeAutoScoreL4Command());
    NamedCommands.registerCommand("Intake auto pullback", intake.intakeAutoPullBackCommand());
    NamedCommands.registerCommand("Climber drive position", climber.climbDrivePositionCommand());
    //NamedCommands.registerCommand("Algae pivot score", algaePivot.algaePivotScoreCommand());
    NamedCommands.registerCommand("Elevator Pivot Drive Position Command", elevatorPivot.elevatorPivotDrivePositionCommand());
    NamedCommands.registerCommand("Elevator Coral Pickup Position Command", elevator.elevatorCoralPickupPositionCommand());
    NamedCommands.registerCommand("Intake Pivot Coral Pickup Position Command", intakePivot.intakePivotCoralPickupPositionCommand());
    NamedCommands.registerCommand("Intake Stop Command", intake.intakeStopCommand());
   // NamedCommands.registerCommand("Algae Pivot Drive Setting Command", algaePivot.algaePivotDriveSettingCommand());

  }
  
  //Create Automated Commands here that make use of multiple subsystems [can be used in autonomous or teleop]
  //(ex. auto coral station pickup: moves elevator and elevator pivot)
  //See Crescendo's code for examples

  // Stops all moters except the drive system
  public Command stopMotorsAll() {
    return new ParallelCommandGroup(
      //algaePickup.algaePickupStopCommand(),
      //algaePivot.algaePivotStopCommand(),
      climber.climberStopCommand(),
      elevator.elevatorStopCommand(),
      elevatorPivot.elevatorPivotStopCommand(),
      intake.intakeStopCommand(),
      intakePivot.intakePivotStopCommand()
    );
  }
  
  // Sets all the moving parts on the elevator into position to score on L1
  public Command scoreL1() {
    return new SequentialCommandGroup(
      //algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorScoreL1Command(),
        elevatorPivot.elevatorPivotScoreL1Command(),
        intakePivot.intakePivotScoreL1Command()
      )
    );
  }

  // Sets all the moving parts on the elevator into position to score on L2
  public Command scoreL2() {
    return new SequentialCommandGroup(
      //algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorScoreL2Command(),
        elevatorPivot.elevatorPivotScoreL2Command(),
        intakePivot.intakePivotScoreL2Command()
      )
    );
  }

  // Sets all the moving parts on the elevator into position to score on L3
  public Command scoreL3() {
    return new SequentialCommandGroup(
      //algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorScoreL3Command(),
        elevatorPivot.elevatorPivotScoreL3Command(),
        intakePivot.intakePivotScoreL3Command()
      )
    );
  }

  // Sets all the moving parts on the elevator into position to score on L4
  public Command scoreL4() {
    return new SequentialCommandGroup(
      //algaePivot.algaePivotScoreCommand(),
     
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command()
    );
  }

  // Sets all the moving parts on the elevator into position to intake coral
  public Command coralPickupSetPositions() {
    return new SequentialCommandGroup(
      //algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorCoralPickupPositionCommand(),
        elevatorPivot.elevatorPivotCoralPickupPositionCommand(),
        intakePivot.intakePivotCoralPickupPositionCommand(),
        intake.intakeResetCommand()
      )
    );
  }

  // Not sure what this does so it is not mapped to anything on the controller
  public Command driveCommand() {
    return new SequentialCommandGroup(
      //algaePivot.algaePivotScoreCommand(),
      elevatorPivot.elevatorPivotDrivePositionCommand(),
      new ParallelCommandGroup(
        elevator.elevatorCoralPickupPositionCommand(),
        intakePivot.intakePivotDrivePositiCommand()
        //intake.intakeStopCommand()
      )
       //,algaePivot.algaePivotDriveSettingCommand()
    );
    
  }

  // Gets the robot into clibing position and begins to climb
  public Command climbPositionCommand() {
    return new SequentialCommandGroup( 
    //algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorClimbPositionCommand(),
        elevatorPivot.elevatorPivotClimbPositionCommand(),
        intakePivot.intakePivotClimbPositionCommand()
        //intake.intakeStopCommand()
        //,algaePickup.algaePickupStopCommand()
      ),
    //algaePivot.algaePivotDriveSettingCommand(),
    climber.climbDrivePositionCommand()

    );
  }

  // Goes to algae high
  public Command algaeHighCommand() {
    return new SequentialCommandGroup(
      //algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorAlgaeHighCommand(),
        elevatorPivot.elevatorPivotAlgaeHighCommand(),
        intakePivot.intakePivotAlgaeHighCommand(),
        intake.intakeStopCommand()
      )//,
      //algaePickup.algaePickupInCommand()
    );
  }
  
  // Goes to algae low
  public Command algaeLowCommand() {
    return new SequentialCommandGroup(
     // algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorAlgaeLowCommand(),
        elevatorPivot.elevatorPivotAlgaeLowCommand(),
        intakePivot.intakePivotAlgaeLowCommand(),
        intake.intakeStopCommand()
      )//,
     // algaePickup.algaePickupInCommand()
    );
  }
 
  // Sets subsystems in position to score algae in Proccessor
  public Command algaeScoreProcessorPositionsCommand() {
    return new SequentialCommandGroup(
     // algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorClimbPositionCommand(),
        elevatorPivot.elevatorPivotClimbPositionCommand(),
        intakePivot.intakePivotClimbPositionCommand(),
        intake.intakeStopCommand()
      )
    );
  }

  // Pathplanner autos for TELEOP

  public Command teleopTo1rightCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 1 right")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo1leftCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 1 left")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo2rightCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 2 right")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo2leftCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 2 left")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo3rightCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 3 right")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo3leftCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 3 left")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo4rightCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 4 right")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo4leftCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 4 left")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo5rightCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 5 right")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo5leftCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 5 left")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }
  
  public Command teleopTo6rightCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 6 right")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  public Command teleopTo6leftCommand() {
    return new ParallelCommandGroup(
      new PathPlannerAuto("teleop to 6 left")/*,
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)*/);
    
  }

  //WEEK OF 3/10 TELEOP COMMANDS FOR COMP

  public Command centerTo1RightAutoCommand() {
    return new SequentialCommandGroup(
        driveCommand(),
        teleopTo1rightCommand(),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command(),
      driveCommand()
    ); 
  }

  public Command centerTo1RightL1AutoCommand() {
    return new SequentialCommandGroup(
        driveCommand(),
        teleopTo1rightCommand(),
      new ParallelCommandGroup(
        elevator.elevatorScoreL1Command(),
        intakePivot.intakePivotScoreL1Command()
      ),
      elevatorPivot.elevatorPivotScoreL1Command(),
      intake.intakeAutoScoreL1Command(),
      driveCommand()
    ); 
  }

  public Command centerTo1LeftAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("teleop to 1 left"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command(),
      driveCommand()
    ); 
  }

  public Command rightTo2RightAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("right to 2 right Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command(),
      driveCommand()
    ); 
  }

  public Command rightTo2LeftAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("right to 2 left Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command(),
      driveCommand()
    ); 
  }

//outline of code for two piece
 /*  public Command twoLeftToRightCoralIntakeAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("2 left to right coral intake Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command(),
      driveCommand()
    ); 
  }
//more parts of the code for the two piece
  public Command rightCoralIntakeTo4LeftCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("right coral intake to 4 left Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command(),
      driveCommand()
    ); 
  } */

  public Command leftTo3RightAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("left to 3 right Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command()
    ); 
  }

  public Command leftTo3LeftAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("left to 3 left Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command()
    ); 
  }

  public Command rightTo4RightAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("right to 4 right Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command()
    ); 
  }

  public Command rightTo4LeftAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("right to 4 left Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command()
    ); 
  }

  public Command leftTo5RightAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("left to 5 right Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command()
    ); 
  }

  public Command leftTo5LeftAutoCommand() {
    return new SequentialCommandGroup(
      driveCommand(),
      new PathPlannerAuto("left to 5 left Auto"),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      ),
      elevatorPivot.elevatorPivotScoreL4Command(),
      intake.intakeAutoScoreL4Command()
    ); 
  }

  //Create Autonomous Routines here (sequences for first 15s of match)
  //See Crescendo's code for examples
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  //Add options for autonomous routines so that they appear in sendable chooser on SmartDashboard
  //https://docs.wpilib.org/en/stable/docs/software/dashboards/smartdashboard/choosing-an-autonomous-program-from-smartdashboard.html 
  
  private void configureButtonBindings() {
    chooser.setDefaultOption("nothing", null);
    
    chooser.addOption("C to 1 R", centerTo1RightAutoCommand());
    chooser.addOption("L1 C to 1 R", centerTo1RightL1AutoCommand());
    chooser.addOption("C to 1 L ", centerTo1LeftAutoCommand());
    chooser.addOption("R to 2 R", rightTo2RightAutoCommand());
    chooser.addOption("R to 2 L", rightTo2LeftAutoCommand());
    chooser.addOption("L to 3 R", leftTo3RightAutoCommand());
    chooser.addOption("L to 3 L", leftTo3LeftAutoCommand());
    chooser.addOption("R to 4 R", rightTo4RightAutoCommand());
    chooser.addOption("R to 4 L", rightTo4LeftAutoCommand());
    chooser.addOption("L to 5 R", leftTo5RightAutoCommand());
    chooser.addOption("L to 5 L", leftTo5LeftAutoCommand());

    //chooser.addOption("L to 3 R; CS to 5 R", new PathPlannerAuto("left to 3 right and coral intake to 5 right Auto"));
    //chooser.addOption("R to 2 L; CS to 4 L", new PathPlannerAuto("right to 2 left and coral intake to 4 left Auto"));
    chooser.addOption("Drive Forward Center", new PathPlannerAuto("Drive Forward Center"));
    chooser.addOption("Drive Forward Right", new PathPlannerAuto("Drive Forward Right"));
    chooser.addOption("Drive Forward Left", new PathPlannerAuto("Drive Forward Left"));
    //chooser.addOption("Test Named Command", new PathPlannerAuto("Test Named Command"));

    //Driver Open House
    driver.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)));

    driver.start().whileTrue(stopMotorsAll());

    //Buttons for Open House
 
    driver.a().onTrue(scoreL1());
    driver.x().onTrue(scoreL2());
    driver.b().onTrue(scoreL3());
    driver.y().onTrue(scoreL4());

    driver.leftBumper().onTrue(driveCommand());
    driver.rightBumper().onTrue(intake.intakeAutoPullBackCommand());

    driver.rightTrigger().whileTrue(intake.intakeScoreCoralL4Command());
    driver.rightTrigger().whileFalse(intake.intakeStopCommand());
    driver.leftTrigger().whileTrue(intake.intakeScoreCoralL2and3Command());
    driver.leftTrigger().whileFalse(intake.intakeStopCommand());

    driver.povUp().and(driver.povDown().negate()).and(driver.povRight().negate()).and(driver.povLeft().negate()).whileTrue(intakePivot.intakePivotUpCommand());
    driver.povUp().whileFalse(intakePivot.intakePivotStopCommand());
    driver.povRight().and(driver.povUp().negate()).and(driver.povLeft().negate()).and(driver.povDown().negate()).whileTrue(elevatorPivot.elevatorPivotUpCommand());
    driver.povRight().whileFalse(elevatorPivot.elevatorPivotStopCommand());
    driver.povLeft().and(driver.povUp().negate()).and(driver.povDown().negate()).and(driver.povRight().negate()).whileTrue(elevatorPivot.elevatorPivotDownCommand()); 
    driver.povLeft().whileFalse(elevatorPivot.elevatorPivotStopCommand()); 
  }
  
  public Swerve getSwerve(){
    return s_Swerve;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  

  public Command getAutonomousCommand() {

    s_Swerve.resetModulesToAbsolute();
    return chooser.getSelected();
    //return null;
  }
}
