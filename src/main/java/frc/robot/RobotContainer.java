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
  private final AlgaePickup algaePickup = new AlgaePickup();
  private final AlgaePivot algaePivot = new AlgaePivot();
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
            () -> -driver.getRawAxis(rotationAxis)));

    // Puts Sendable Chooser on SmartDashboard
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto:", chooser);

    // Configure the button bindings
    configureButtonBindings();  

    NamedCommands.registerCommand("elevator pivot score L1 command", elevatorPivot.elevatorPivotScoreL1Command());
    NamedCommands.registerCommand("elevator pivot score L4 command", elevatorPivot.elevatorPivotScoreL4Command());
    NamedCommands.registerCommand("elevator score L1 command", elevator.elevatorScoreL1Command());
    NamedCommands.registerCommand("elevator score L4 command", elevator.elevatorScoreL4Command());
    NamedCommands.registerCommand("Intake score L1 command", intake.intakeScoreCoralL1Command());
    NamedCommands.registerCommand("Intake pivot score L1 command", intakePivot.intakePivotScoreL1Command());
    NamedCommands.registerCommand("Intake pivot score L4 command", intakePivot.intakePivotScoreL4Command());
    NamedCommands.registerCommand("Intake auto score", intake.intakeAutoScoreCommand());
    NamedCommands.registerCommand("Intake auto pullback", intake.intakeAutoPullBackCommand());
    NamedCommands.registerCommand("Climber drive position", climber.climbDrivePositionCommand());
    NamedCommands.registerCommand("Algae pivot score", algaePivot.algaePivotScoreCommand());

  }

  //Create Automated Commands here that make use of multiple subsystems [can be used in autonomous or teleop]
  //(ex. auto coral station pickup: moves elevator and elevator pivot)
  //See Crescendo's code for examples

  // Stops all moters except the drive system
  public Command stopMotorsAll() {
    return new ParallelCommandGroup(
      algaePickup.algaePickupStopCommand(),
      algaePivot.algaePivotStopCommand(),
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
      algaePivot.algaePivotScoreCommand(),
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
      algaePivot.algaePivotScoreCommand(),
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
      algaePivot.algaePivotScoreCommand(),
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
      algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorScoreL4Command(),
        elevatorPivot.elevatorPivotScoreL4Command(),
        intakePivot.intakePivotScoreL4Command()
      )
    );
  }

  // Sets all the moving parts on the elevator into position to intake coral
  public Command coralPickup() {
    return new SequentialCommandGroup(
      algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorCoralPickupPositionCommand(),
        elevatorPivot.elevatorPivotCoralPickupPositionCommand(),
        intakePivot.intakePivotCoralPickupPositionCommand(),
        intake.intakePickupCoralCommand()
      )
    );
  }

  // Not sure what this does so it is not mapped to anything on the controller
  public Command driveCommand() {
    return new SequentialCommandGroup(
      algaePivot.algaePivotScoreCommand(),
      elevatorPivot.elevatorPivotDrivePositionCommand(),
      new ParallelCommandGroup(
        elevator.elevatorCoralPickupPositionCommand(),
        intakePivot.intakePivotCoralPickupPositionCommand(),
        intake.intakeStopCommand()
      ), 
       algaePivot.algaePivotDriveSettingCommand()
    );
    
  }

  // Gets the robot into clibing position and begins to climb
  public Command climbPositionCommand() {
    return new SequentialCommandGroup( 
    algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorClimbPositionCommand(),
        elevatorPivot.elevatorPivotClimbPositionCommand(),
        intakePivot.intakePivotClimbPositionCommand(),
        intake.intakeStopCommand()
      ),
    algaePivot.algaePivotDriveSettingCommand(),
    climber.climbDrivePositionCommand()

    );
  }

  // Goes to algae high
  public Command algaeHighCommand() {
    return new SequentialCommandGroup(
      algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorAlgaeHighCommand(),
        elevatorPivot.elevatorPivotAlgaeHighCommand(),
        intakePivot.intakePivotAlgaeHighCommand(),
        intake.intakeStopCommand()
      ),
      algaePickup.algaePickupInCommand()
    );
  }
  
  // Goes to algae low
  public Command algaeLowCommand() {
    return new SequentialCommandGroup(
      algaePivot.algaePivotScoreCommand(),
      new ParallelCommandGroup(
        elevator.elevatorAlgaeLowCommand(),
        elevatorPivot.elevatorPivotAlgaeLowCommand(),
        intakePivot.intakePivotAlgaeLowCommand(),
        intake.intakeStopCommand()
      ),
      algaePickup.algaePickupInCommand()
    );
  }
 
  // Sets subsystems in position to score algae in Proccessor
  public Command algaeScoreProcessorPositionsCommand() {
    return new SequentialCommandGroup(
      algaePivot.algaePivotScoreCommand(),
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
    return new PathPlannerAuto("teleop to 1 right");
  }

  public Command teleopTo1leftCommand() {
    return new PathPlannerAuto("teleop to 1 left");
  }

  public Command teleopTo2rightCommand() {
    return new PathPlannerAuto("teleop to 2 right");
  }

  public Command teleopTo2leftCommand() {
    return new PathPlannerAuto("teleop to 2 left");
  }

  public Command teleopTo3rightCommand() {
    return new PathPlannerAuto("teleop to 3 right");
  }

  public Command teleopTo3leftCommand() {
    return new PathPlannerAuto("teleop to 3 left");
  }

  public Command teleopTo4rightCommand() {
    return new PathPlannerAuto("teleop to 4 right");
  }

  public Command teleopTo4leftCommand() {
    return new PathPlannerAuto("teleop to 4 left");
  }

  public Command teleopTo5rightCommand() {
    return new PathPlannerAuto("teleop to 5 right");
  }

  public Command teleopTo5leftCommand() {
    return new PathPlannerAuto("teleop to 5 left");
  }
  
  public Command teleopTo6rightCommand() {
    return new PathPlannerAuto("teleop to 6 right");
  }

  public Command teleopTo6leftCommand() {
    return new PathPlannerAuto("teleop to 6 left");
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
    
    chooser.addOption("Center to 1 Right Auto", new PathPlannerAuto("Center to 1 right Auto"));
    chooser.addOption("Center to 1 Left Auto", new PathPlannerAuto("Center to 1 left Auto"));
   
    //add rest of autonomous routines here
  
  
  
    //Create Driver Button mapping here

    //Driver #1
    /* DON'T PUT ON CONTROLLER WITH back AS SHIFTER BUTTON
    driver.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -driver.getRawAxis(translationAxis),
        () -> -driver.getRawAxis(strafeAxis),
        () -> -driver.getRawAxis(rotationAxis)));*/
    
    driver.start().whileTrue(stopMotorsAll());

    
    // Testing buttons for driver #1, manual commands
    driver.rightTrigger().whileTrue(elevator.elevatorUpCommand());
    driver.rightTrigger().whileFalse(elevator.elevatorStopCommand());

    driver.leftTrigger().whileTrue(elevator.elevatorDownCommand());
    driver.leftTrigger().whileFalse(elevator.elevatorStopCommand());
    
    driver.rightBumper().whileTrue(elevatorPivot.elevatorPivotUpCommand());
    driver.rightBumper().whileFalse(elevatorPivot.elevatorPivotStopCommand());

    driver.leftBumper().whileTrue(elevatorPivot.elevatorPivotDownCommand());
    driver.leftBumper().whileFalse(elevatorPivot.elevatorPivotStopCommand());

    driver.a().whileTrue(intake.intakePickupCoralCommand());
    driver.a().whileFalse(intake.intakeStopCommand());

    driver.x().whileTrue(intake.intakeScoreCoralL1Command());
    driver.x().whileFalse(intake.intakeStopCommand());

    driver.b().whileTrue(intakePivot.intakePivotDownCommand());
    driver.b().whileFalse(intakePivot.intakePivotStopCommand());

    driver.y().whileTrue(intakePivot.intakePivotUpCommand());
    driver.y().whileFalse(intakePivot.intakePivotStopCommand());

    driver.povUp().whileTrue(algaePivot.algaePivotUpCommand());
    driver.povUp().whileFalse(algaePivot.algaePivotStopCommand());
    
    driver.povDown().whileTrue(algaePivot.algaePivotDownCommand());
    driver.povDown().whileFalse(algaePivot.algaePivotStopCommand());

    driver.povRight().whileTrue(algaePickup.algaePickupInCommand());
    driver.povRight().whileFalse(algaePickup.algaePickupStopCommand());

    driver.povLeft().whileTrue(algaePickup.algaePickupOutCommand());
    driver.povLeft().whileFalse(algaePickup.algaePickupStopCommand());


 //Testing buttons for driver #1, set position commands

    //Elevator
    /*driver.a().onTrue(elevator.elevatorScoreL1Command());
    driver.b().onTrue(elevator.elevatorScoreL2Command());
    driver.x().onTrue(elevator.elevatorScoreL3Command());
    driver.y().onTrue(elevator.elevatorScoreL4Command());
    driver.rightTrigger().onTrue(elevator.elevatorAlgaeHighCommand());
    driver.leftTrigger().onTrue(elevator.elevatorAlgaeLowCommand());*/

    //Elevator Pivot
    /*driver.a().onTrue(elevatorPivot.elevatorPivotScoreL1Command());
    driver.b().onTrue(elevatorPivot.elevatorPivotScoreL2Command());
    driver.x().onTrue(elevatorPivot.elevatorPivotScoreL3Command());
    driver.y().onTrue(elevatorPivot.elevatorPivotScoreL4Command());
    driver.rightTrigger().onTrue(elevatorPivot.elevatorPivotAlgaeHighCommand());
    driver.leftTrigger().onTrue(elevatorPivot.elevatorPivotAlgaeLowCommand());*/

    //Intake Pivot
   /*  driver.a().onTrue(intakePivot.intakePivotScoreL1Command());
    driver.b().onTrue(intakePivot.intakePivotScoreL2Command());
    driver.x().onTrue(intakePivot.intakePivotScoreL3Command());
    driver.y().onTrue(intakePivot.intakePivotScoreL4Command());
    driver.rightTrigger().onTrue(intakePivot.intakePivotAlgaeHighCommand());
    driver.leftTrigger().onTrue(intakePivot.intakePivotAlgaeLowCommand());*/

    //Algae Pivot
    /*driver.a().onTrue(algaePivot.algaePivotDriveSettingCommand());
    driver.b().onTrue(algaePivot.algaePivotScoreCommand());*/

    //Testing negate buttons
   /* driver.a().and(driver.rightBumper().negate()).whileTrue(intake.intakeScoreCoralL2toL4Command());
    driver.a().and(driver.rightBumper()).whileTrue(algaePickup.algaePickupInCommand());
    driver.start().whileTrue(stopMotorsAll());*/
  
    
   // Competition buttons for driver #1
   /*  driver.y().and(driver.back().negate()).whileTrue(teleopTo1rightCommand());
    driver.y().and(driver.back()).whileTrue(teleopTo1leftCommand());

    driver.x().and(driver.back().negate()).whileTrue(teleopTo2rightCommand());
    driver.x().and(driver.back()).whileTrue(teleopTo2leftCommand());

    driver.b().and(driver.back().negate()).whileTrue(teleopTo3rightCommand());
    driver.b().and(driver.back()).whileTrue(teleopTo3leftCommand());

    driver.leftBumper().and(driver.back().negate()).whileTrue(teleopTo4rightCommand());
    driver.leftBumper().and(driver.back()).whileTrue(teleopTo4leftCommand());

    driver.rightBumper().and(driver.back().negate()).whileTrue(teleopTo5rightCommand());
    driver.rightBumper().and(driver.back()).whileTrue(teleopTo5leftCommand());

    driver.a().and(driver.back().negate()).whileTrue(teleopTo6rightCommand());
    driver.a().and(driver.back()).whileTrue(teleopTo6leftCommand());

    driver.rightTrigger().onTrue(climbPositionCommand());
    driver.leftTrigger().onTrue(climber.ClimbDeepCageCommand());
*/

    //Driver #2
    second.back().toggleOnTrue(
      new toggleSpeed(
        s_Swerve,
        () -> -second.getRawAxis(translationAxis),
        () -> -second.getRawAxis(strafeAxis),
        () -> -second.getRawAxis(rotationAxis)));

    second.start().whileTrue(stopMotorsAll());

   // Testing buttons for driver #2, manual commands
    second.rightTrigger().whileTrue(climber.climberUpCommand());
    second.rightTrigger().whileFalse(climber.climberStopCommand());
    second.leftTrigger().whileTrue(climber.climberDownCommand());
    second.leftTrigger().whileFalse(climber.climberStopCommand());

   /* only need if DPad buttons don't work on driver #1 controller  
    second.rightBumper().whileTrue(algaePivot.algaePivotUpCommand());
    second.rightBumper().whileFalse(algaePivot.algaePivotStopCommand());
    second.leftBumper().whileTrue(algaePivot.algaePivotDownCommand());
    second.leftBumper().whileFalse(algaePivot.algaePivotStopCommand());

    second.x().whileTrue(algaePivot.algaePivotDownCommand());
    second.x().whileFalse(algaePivot.algaePivotStopCommand());
    second.b().whileTrue(algaePivot.algaePivotDownCommand());
    second.b().whileFalse(algaePivot.algaePivotStopCommand()); */

    // Competition Buttons for Driver #2
/* 
    second.a().onTrue(scoreL1());
    second.x().onTrue(scoreL2());
    second.b().onTrue(scoreL3());
    second.y().onTrue(scoreL4());

    second.leftBumper().onTrue(driveCommand());
    second.rightBumper().onTrue(intake.intakeAutoPullBackCommand());

    second.rightTrigger().whileTrue(intake.intakeScoreCoralL2toL4Command());
    second.rightTrigger().whileFalse(intake.intakeStopCommand());
    second.leftTrigger().whileTrue(intake.intakeScoreCoralL1Command());
    second.leftTrigger().whileFalse(intake.intakeStopCommand());

    second.povDown().onTrue(algaeHighCommand());
    second.povDown().onTrue(algaeLowCommand());
    second.povLeft().onTrue(algaeScoreProcessorPositionsCommand());
    second.povRight().whileTrue(algaePickup.algaePickupOutCommand());
    second.povRight().whileFalse(algaePickup.algaePickupStopCommand());
    */
    



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
