## What is this "parting gift" & Why it Exists
I'm graduating and while I'm excited for the future, I would like to let something I've noticed be known. I haven't gotten a chance to properly **teach**, I've really only been able to **handhold** through problems. 

Whilst this allowed us to have an unimaginably successful 2024 season, I feel that if I can teach more, aid more, and get more people to do what I love doing for this team and for this program, you (we?) will become so much more successful, maybe even making it to playoffs, or **beyond**!

I wanted to leave the team (and myself) an example of some robot code that ***properly balances both function and form***. Right now, the code isn't in the prettiest state, nor is it the most functional. While I may not be able to test a lot of the functions considering I don't have a complete robot in my workspace, the code provided should hopefully show what a good working and good looking robot project looks like, at least in my opinion.

Below my little letter will be an explaination of every change, and coming seperately will be my thoughts on *Better Utilization of Robot Vision*, since that doesn't fit with this letter. Hopefully this allows for better code and more success to be had in the future!

Thanks for a great three years,\
Signing out *(as a student)*\
Julien

## Changes
### General Minor Changes
- `@SupressWarnings('unused')` in ***every*** file. Unused imports aren't a source of robot issues, these warnings just serve as an annoyance
- `TODO` comments left around for additional potential minor QoL changes. 


### Splitting Constants into Multiple Files
Mananging the many different classes in [Constants](src\main\java\frc\robot\Constants.java) has been a source of about 30 different headaches of ours, from finding random values to solving merge conflicts. I saw (somewhere) that some teams split their constants into seperate files, as you would most classses. My approach is similar, with grouping each subsystems' constants into one file, such as `ModuleConstants` and `DriveConstants` being in `DrivetrainConstants` This should allow editing of multiple subsystems' constants on ***the same branch*** without merge conflicts.

Various things, such as `ConfigConstants`, which contains info on what motors or gyroscope is used, and `MotorConstants` which contains various stats for the various motors don't fit into a subsystem specifically, so they'll go in `RobotConstants`, 

### Moving Various Commands into Command Factories in One File
This one is purely to aid anyone reading this who is new to coding. The keyword `new` is a bit confusing, especially in its use in Java. Here's an example
```
driverController.rightBumper().onTrue(new TurnToSource(m_drivetrain, m_leds, driverController, m_alliance));
```

Looking at this, one would think that when `rightBumper()` returns true, a *new* `TurnToSource` is created, when in fact every time this button is pressed, its doing the same action.

Compare this to what it could look like using a Factory in a `driveCommands` file.

```
example goes here
```

### SparkMAXs
Where do I even begin?\
We create these motor controllers in many ways, sometimes we were boring and used this, and created the object in one line
```
private static CANSparkMax shootMotor = new CANSparkMax(ShooterConstants.kShootMotorID, MotorType.kBrushless);
```
Other times we created them in the constructor of the subsystem, like this
```
public RollerClawReal() {
        rollerClawMotor = new CANSparkMax(RollerClawConstants.kRollerClawMotorID, MotorType.kBrushless);
        rollerClawEncoder = rollerClawMotor.getEncoder();
        rollerClawMotor.restoreFactoryDefaults();
        rollerClawMotor.setIdleMode(IdleMode.kBrake);
        rollerClawMotor.setSmartCurrentLimit(MotorConstants.kNEO550CurrentLimit);
        rollerClawMotor.burnFlash();
    }
```
And only *once*, we use the (in my opinion) best way to create these motors, which enforces a current limit. `MotorUtil.createSparkMAX`
