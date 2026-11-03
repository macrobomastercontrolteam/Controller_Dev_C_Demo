# Common Infantry Code for MacRM
Robot configs:
- `HERO_2025_MECANUM` (5)
- `SENTRY_2026_OMNI` (6)
- `INFANTRY_2026_OMNI` (9, default)

Select one of these three configurations with `ROBOT_TYPE` in
[Inc/global_inc.h](Inc/global_inc.h). Other robot types are no longer supported.
The sentry configuration requires `CV_INTERFACE` to be enabled.

Developed based on "RoboMaster competition robot 2020 self-assembly version A", which was for Type A Dev Board. Refer to user manuals posted on
> "File download: [RoboMaster Competition Robot 2020 Self-Assembled Version A Type Related Documents](https://rm-static.djicdn.com/documents/26898/57bcb7163d7bd1575980335867998835.7z)"

on 
> RM's Official website: https://www.robomaster.com/zh-CN/resource/pages/announcement/1081.

# Firmware Environment
- Type C Development Board
- FreeRTOS

# Firmware Architecture
WIP on branch misc/Electrical_Assembly_Diagram

# Special Branches
- test_no_ref: test all other features without referee system by ignoring ref errrors
