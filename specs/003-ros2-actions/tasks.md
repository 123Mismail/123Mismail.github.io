# Task Breakdown: Chapter 3 - ROS 2 Actions and Services

**Feature**: `003-ros2-actions`
**Branch**: `003-ros2-actions`

## Summary

Generate Chapter 3 (ROS 2 Actions and Services) for the Physical AI & Humanoid Robotics textbook using the established workflow. Chapter uses context7 MCP for technical accuracy, follows mandatory template, and undergoes 11-item self-review checklist.

**Total Tasks**: 9 tasks
**Scope**: Single chapter (C3) generation and validation

## Chapter 3: ROS 2 Actions and Services

**Story Goal**: Enable students to understand ROS 2's goal-oriented communication (Actions) and request-response patterns (Services), implementing action servers/clients and service servers/clients.

### Chapter Generation Tasks

- [ ] T001 Invoke context7 for rclpy Action Server/Client API and .action file format
- [ ] T002 Draft C3 Learning Outcomes section (3 objectives: Actions vs Services, Action Server implementation, Service implementation)
- [ ] T003 Draft C3 Overview section (goal-oriented communication for humanoid robots)
- [ ] T004 Draft C3 Key Concepts section (Action, Service, Goal, Feedback, Result, Request, Response - bold on first use)
- [ ] T005 Draft C3 Main Content sections (Actions fundamentals, Services fundamentals, comparison)
- [ ] T006 Generate C3 code example 1: Action Server (Fibonacci example, Python rclpy, >=20 lines, inline comments)
- [ ] T007 Generate C3 code example 2: Action Client with feedback callback (Python rclpy, >=15 lines)
- [ ] T008 Draft C3 Summary section (ROS 2 goal-based control and request-response patterns)
- [ ] T009 Apply 11-item self-review checklist and save to chapters/c3-ros2-actions.md

**Acceptance Criteria**:
- C3 file exists at chapters/c3-ros2-actions.md
- All 11 self-review checklist items pass
- C3 renders correctly in Docusaurus
- Action and Service code examples verified via context7
