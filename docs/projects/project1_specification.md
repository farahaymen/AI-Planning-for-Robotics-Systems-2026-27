---
title: "Project 1: Autonomous Mapping and Navigation System"
subtitle: "Autonomous Robotics with ROS 2 | 25 percent of the module mark"
author: "British University in Egypt"
date: "Issued week 3 | Demonstrated week 6"
---

# Project 1: Autonomous Mapping and Navigation System

**Weight** 25 percent of the module mark, multiplied by your individual peer contribution factor
**Teams** 3 or 4 students
**Issued** Week 3, immediately after Lab 3
**Demonstration** Week 6, in your timetabled laboratory slot
**Report deadline** End of week 6

---

## 1. What you are building

A robot that is placed in an environment it has never seen, builds a map of it,
saves that map, and then, after a restart, works out where it is and visits a
sequence of goals without being told where it started.

This is not a toy version of a real task. It is the commissioning workflow for a
commercial autonomous mobile robot, reduced to the parts that fit in three weeks.

The project is deliberately placed before Lab 6. You have mapping, localisation
and the ability to command the robot; what you do not have is a tuned navigation
framework. Assembling a working system from components rather than configuring a
finished one is the point.

## 2. Required capabilities

Your system must do all of the following, autonomously, in one continuous run
after the initial mapping phase.

1. **Map an environment.** Drive the robot through the provided environment and
   produce an occupancy map with SLAM Toolbox. Teleoperation during this phase is
   permitted and expected.
2. **Save and reload the map.** The map must persist to disk in the standard ROS
   format and be loadable by `nav2_map_server`.
3. **Localise from an unknown pose.** After the restart, the robot is placed
   somewhere in the mapped environment. It must determine its own pose. You may
   not publish an initial pose estimate by hand during the scored run.
4. **Visit at least four goals in sequence.** The goal list is supplied as a YAML
   file at run time. Your mission node reads it; goals are not hard coded.
5. **Handle one unannounced obstacle.** An obstacle will be placed on the path
   between two goals after the run has started. The robot must reach the goal
   anyway.
6. **Report mission status.** The system must publish or log, in a form a person
   can read, which goal it is pursuing, whether the previous goal succeeded, and
   what it did if a goal failed.

You may use the Nav2 `navigate_to_pose` action for the driving. Nav2 is
introduced properly in Lab 6, and a working configuration is supplied in
`arc_nav`. Using it is not a shortcut; the mapping, localisation, mission logic,
failure handling and system integration are still yours, and they are what is
assessed.

## 3. Engineering requirements

These are assessed separately from whether the robot works, and they are worth
almost as much.

**Package structure.** One or more properly formed ROS 2 packages with a correct
`package.xml`, declared dependencies, and a launch file that brings up the whole
system with a single command. A system that only starts when six terminals are
opened in the right order is not finished.

**Configuration, not constants.** Goal lists, map paths, topic names and tuning
parameters live in YAML or launch arguments. A magic number in a Python file
costs marks.

**Git history.** A repository whose history shows the work being done, by more
than one person, over more than one day. A single commit on the deadline is
evidence of something, and it is not evidence of teamwork.

**Tests.** At least three meaningful tests. Pure Python components should be
covered by `pytest`. At least one `launch_testing` test that brings up part of
the system and asserts something about it. Tests that assert `True == True` are
worse than none.

**Reproducibility.** A `README.md` that lets a demonstrator who has never seen
your repository clone it, build it, and run the system. It will be checked by
doing exactly that.

## 4. Evaluation over seeds

Your report must contain results from a seeded evaluation, not a description of
one successful run.

Run the full mission through `arc_eval` over **at least 30 seeds** and report the
standard metrics: success rate with its confidence interval, collision free rate,
time to goal, path length, minimum clearance, recovery events and interventions.

```
python3 -m arc_eval.runner --config arc_eval/configs/project1_<team>.yaml
```

Two rules apply, and they are worth marks rather than being formalities.

If a difference between two configurations is smaller than the standard deviation
of either, report it as inconclusive. Doing so correctly earns full method marks.
Presenting it as an improvement does not.

Your report must include **one honest failure analysis**. A team reporting 30
successes out of 30 has either not run 30 seeds or has not looked at them
carefully. Find a case where the system behaved badly, explain the mechanism, and
say what you would change. This section is worth more than a marginally higher
success rate.

## 5. Deliverables

**Repository**, pushed to the course organisation by the deadline, containing:

- Your packages, launch files and configuration
- `README.md` with build and run instructions
- `AI_USE.md` declaring any use of language models, which tool, what was
  generated and what you changed afterwards. Declared use is permitted and is not
  penalised. Undeclared use is an academic integrity matter.
- Your saved map and the goal list format you support
- Your evaluation results JSON and the script that produced it
- Tests

**Report**, four pages maximum excluding references and appendices, as PDF.
Length is a limit rather than a target. Structure it as:

1. System architecture, with a diagram, and one paragraph justifying the shape
2. What you implemented yourselves, and what you configured, stated plainly
3. Localisation and mapping approach, with the parameters you changed and why
4. Mission logic and failure handling
5. Evaluation over 30 seeds, with the results table
6. Failure analysis
7. What you would do differently

**Demonstration**, in the week 6 laboratory slot. Twelve minutes per team.

- Your demonstrator will clone your repository and follow your README
- You run the mission once, live, from the single launch command
- The unannounced obstacle is placed by the demonstrator during the run
- Each team member is asked questions individually, as described below

## 6. Individual viva

Five minutes per student, during the demonstration. Three questions:

1. One from the published question bank, which is released in week 4.
2. One justification of a parameter in your own submission. For example: your
   AMCL `alpha3` is 0.4, the default is 0.2, explain the change.
3. One live fault. The demonstrator will break something in front of you and ask
   you to diagnose it aloud using the standard workflow. You are not expected to
   fix it in five minutes. You are expected to know where to look and why.

The third question is the one that decides most of the viva mark. It assesses the
skill the course actually claims to teach, and it takes about ninety seconds to
find out whether you built the system or watched it being built.

## 7. Marking

| Component | Marks | What earns them |
|-----------|-------|-----------------|
| System function | 30 | Maps, saves, relocalises, visits all goals, handles the obstacle, runs from one command |
| ROS engineering | 20 | Package structure, launch files, configuration discipline, tests, README |
| Experimental method | 20 | The 30 seed evaluation, correct reporting of uncertainty, the failure analysis |
| Report and communication | 20 | Clarity, justified choices, a diagram that helps, honest limitations |
| Individual viva | 10 | Understanding of your own system and of the diagnostic workflow |

The total is then multiplied by your **individual peer contribution factor**, in
the range 0.8 to 1.1, determined from an anonymous peer form, the Git history
over the project window, and the viva.

### Notes on marking that are worth reading

The system function marks are not all or nothing. A team whose robot relocalises
correctly but fails on the fourth goal, and whose report explains exactly why,
will score better overall than a team whose robot succeeds and whose report
describes what the code does.

Configuring Nav2 rather than writing your own planner does not cost marks. Not
understanding what you configured does.

Clean failure is worth more than lucky success. A robot that detects it cannot
reach a goal, reports it, and moves to the next one is better engineered than one
that happens not to encounter the problem.

## 8. Practicalities

**Environment.** The mapping environment is `arc_warehouse`, provided in
`arc_gazebo`. The relocalisation start pose and the obstacle position are chosen
by the demonstrator on the day and are not published in advance.

**Graphics tiers.** Every part of this project must be completable in Tier B,
headless. If your workflow depends on the Gazebo GUI you have a problem that will
surface on the demonstration day.

**Infrastructure failures.** If `course-check` fails on the demonstration day,
record the incident ID and tell your demonstrator immediately. No marks are lost
for failures of the official environment, but the incident must be recorded at
the time rather than claimed afterwards.

**Work persistence.** Nothing inside the virtual machine survives. Push at the
end of every session. A team that loses work to a machine reset in week 5 will
not receive an extension for it, because the policy is stated here and was stated
in Lab 1.

**Getting stuck.** Demonstrator support is available in the week 4 and week 5
laboratory slots. Bring a specific question and evidence: what you observed,
what you expected, and which commands you have already run. "It does not work" is
not a question anyone can answer.

## 9. Suggested schedule

This is advice rather than a requirement, and it is the schedule that has the
fewest bad weeks.

**Week 3.** Read this document. Agree how you will split the work and write it
down. Create the repository, agree the package structure, and get an empty
package building and launching. Decide who owns which component.

**Week 4.** Mapping working end to end: drive, build, save, reload, verify in
RViz. This is the part with the fewest unknowns, so finishing it early buys
options later.

**Week 5.** Relocalisation and the mission node. Then the seeded evaluation, which
takes longer than teams expect: 30 runs at up to two minutes each is an hour of
wall clock before anything goes wrong, and something will.

**Week 6.** Failure handling, the report, and a full rehearsal of the
demonstration including a clone from a clean directory. Teams that skip the clean
clone rehearsal discover a missing dependency in front of the demonstrator.

The single most common failure on this project is leaving the evaluation until
the last day. Run it in week 5 even if the system is imperfect, because a partial
evaluation of a working-ish system tells you what to fix, and a perfect system
with no evaluation loses a fifth of the marks.
