package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import frc.robot.Constants.*;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Subsystem local object handles */
    private final SwerveSubsystem          m_swerveSubsystem;

    private final SwerveParkCmd     m_parkCmd;

    // Create SmartDashboard chooser for autonomous routines
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // Xbox Controllers
    private static CommandXboxController m_xbox;

    //  Constructor for the robot container. Contains subsystems, OI devices, and commands.
    public RobotContainer() {
        m_xbox = new CommandXboxController(0);

        m_swerveSubsystem = new SwerveSubsystem();
        m_swerveSubsystem.setDefaultCommand(
                new DefaultDriveCmd(
                    m_swerveSubsystem, 
                    () -> -m_xbox.getLeftY(),            // translate: - fore / + laft
                    () -> -m_xbox.getLeftX(),            // strafe: - left / + right
                    () -> -m_xbox.getRightX()));

        m_parkCmd = new SwerveParkCmd(m_swerveSubsystem,
                                          () -> -m_xbox.getLeftY(),     // translate
                                          () -> -m_xbox.getLeftX(),     // strafe
                                          () -> -m_xbox.getRightX());   // rotate
        DoNothingCmd m_doNothingAuto = new DoNothingCmd();
        TestAutoCCWCmd m_testAuto = new TestAutoCCWCmd(m_swerveSubsystem);
        TestSquareAuto m_testSquareAuto = new TestSquareAuto(m_swerveSubsystem);
        JustExitCmd m_justExitAuto = new JustExitCmd(m_swerveSubsystem);
        
        //m_chooser.addOption("Do nothing: stage out of the way", m_doNothingAuto);
        m_chooser.setDefaultOption("Do Nothing", m_doNothingAuto);
        m_chooser.addOption("Test Wheel Directions", m_testAuto);
        m_chooser.addOption("Auto Square patterns", m_testSquareAuto);
        m_chooser.addOption("Exit 1 m", m_justExitAuto);
        SmartDashboard.putData("Autonomous Selection: ", m_chooser);

        configureButtonBindings();
    }
    
    /***********************************************
     * Button Bindings defines the operator UI
     ***********************************************/
    
    private void configureButtonBindings() {
        // Driver Buttons
        // The following govern the driving UI:
        //      POV() == UP     =>  avail 
        //      POV() == DOWN   =>  avail
        //      POV() == LEFT   =>  avail
        //      POV() == RIGHT  =>  avail
        //      Back()          =>  Reset all module absolute wheel angles
        //      Start()         =>  Zero the Gyro
        //      LeftTrigger     =>  avail
        //      RightTrigger    =>  avail
        //      Left Bumbper    =>  Alternate mode selections when held
        //      Right Bumper    =>  Slow mode when held
        //      Y()             =>  avail
        //      A()             =>  avail
        //      B()             =>  avail
        //      X()             =>  Park (crossed wheel angles)
        //      L Joystick Y    =>  Translate (move fore/aft)
        //      L Joystick X    =>  Strafe (move side to side)
        //      L Joystk Button =>  Set Field Oriented
        //      R Joystick Y    =>  avail
        //      R Joystick X    =>  Rotate (left = CCW, right = CW)
        //      R Joystk Button =>  Set Robot Oriented
        m_xbox.leftStick().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(true)));
        m_xbox.rightStick().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFieldOriented(false)));
        /*
        prior assignments used to trigger rotation about any given corner. Most useful when
        playing defense. Retained here in case a quick change to defense at competition
        is needed, and these button controls are no longer needed for offense. Or perhaps
        they could be re-mapped to POV buttons?
        m_xbox.leftTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFLCenOfRotation()));
        m_xbox.leftTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightTrigger().onTrue(new InstantCommand(()-> m_swerveSubsystem.setFRCenOfRotation()));                                        
        m_xbox.rightTrigger().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));  
        m_xbox.leftBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBLCenOfRotation()));
        m_xbox.leftBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        m_xbox.rightBumper().onTrue(new InstantCommand(()-> m_swerveSubsystem.setBRCenOfRotation()));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.resetCenOfRotation()));
        */
        m_xbox.rightBumper().and(m_xbox.leftBumper().negate()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.25)));
        m_xbox.rightBumper().and(m_xbox.leftBumper()).onTrue(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(.05)));
        m_xbox.rightBumper().onFalse(new InstantCommand(()-> m_swerveSubsystem.setVarMaxOutputFactor(1.0)));
       
        m_xbox.x().and(m_xbox.leftBumper().negate()).onTrue(m_parkCmd);
        m_xbox.start().onTrue(new InstantCommand(() -> m_swerveSubsystem.zeroGyro()));
        m_xbox.back().onTrue(new InstantCommand(() -> m_swerveSubsystem.resetModulesToAbsolute()));

        m_xbox.povUp().and(m_xbox.leftBumper().negate()).onTrue(new InstantCommand(()->m_swerveSubsystem.testSteerMotorsRotation()));
        m_xbox.povUp().and(m_xbox.leftBumper().negate()).onFalse(new InstantCommand(()->m_swerveSubsystem.stop()));
        
        m_xbox.leftBumper().and(m_xbox.povUp()).onTrue(new InstantCommand(()->m_swerveSubsystem.testDriveMotorsRotation()));
        m_xbox.leftBumper().and(m_xbox.povUp()).onFalse(new InstantCommand(()->m_swerveSubsystem.stop()));
 
        m_xbox.povDown().and(m_xbox.leftBumper().negate()).onTrue(new TestAutoCCWCmd(m_swerveSubsystem));
        m_xbox.leftBumper().and(m_xbox.povDown()).onTrue(new TestAutoCWCmd(m_swerveSubsystem));

        m_xbox.leftBumper().and(m_xbox.a()).onTrue(new TestModuleContRotationCmd(m_swerveSubsystem, -1.0));
        m_xbox.a().and(m_xbox.leftBumper().negate()).onTrue(new TestModuleContRotationCmd(m_swerveSubsystem, 1.0));
     }
    
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    }
} 
