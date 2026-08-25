# FTC Team 5126 DECODE Robot Code

This repository contains FTC robot code for Team 5126's DECODE season work. It is based on the FTC SDK and Road Runner quickstart structure, with team-specific code under `TeamCode`.

## What Is In This Repo

- FTC SDK / Robot Controller project structure
- Road Runner-related drive/localization files
- Team-specific Java code in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode`
- Subsystem-style classes for robot mechanisms and helpers
- TeleOp and autonomous OpModes
- AprilTag/Limelight-related alignment code
- MeepMeep testing project files

## Areas Worth Reviewing

These paths are the best starting points if you are trying to understand the custom team code:

```text
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystem/
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/common/
MeepMeepTesting/
```

## Setup

For Road Runner tuning and quickstart details, use the official docs:

https://rr.brott.dev/docs/v1-0/tuning/

General FTC workflow:

1. Open the project in Android Studio.
2. Sync Gradle.
3. Confirm Robot Controller configuration names match the constants used in `TeamCode`.
4. Build and deploy to the Robot Controller phone/hub.
5. Test mechanisms one at a time before running full TeleOp or autonomous routines.

## Resume / Attribution Note

This is an organization/team repository. From repository contents alone, I can describe what the codebase contains, but individual contribution is not confidently established here. If this repo is used on a resume, add a short note or PR/commit references describing the specific files or systems you personally worked on.