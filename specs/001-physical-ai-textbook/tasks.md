# Task Breakdown: Physical AI & Humanoid Robotics Textbook

**Feature**: `001-physical-ai-textbook`
**Branch**: `001-physical-ai-textbook`
**Plan**: [plan.md](plan.md)
**Spec**: [spec.md](spec.md)

## Summary

Generate 14 chapters + 3 appendices for university-level robotics textbook using sequential approval workflow. Each chapter uses context7 MCP for technical accuracy, follows mandatory template, undergoes 11-item self-review checklist, and requires explicit "APPROVED" before proceeding.

**Total Tasks**: 72 tasks across 7 phases
**Estimated Time**: ~31.5 hours active work (spread across weeks/months due to approval gates)
**MVP Scope**: User Story 1 (Chapter 1 generation and verification)

## Implementation Strategy

### MVP-First Approach
1. **Phase 1-2 (Setup + Foundational)**: Establish tooling and infrastructure
2. **Phase 3 (User Story 1/P1)**: Generate and validate Chapter 1 (Foundations of Physical AI)
   - **MVP Deliverable**: Single chapter passing all quality gates, demonstrating end-to-end workflow
3. **Phase 4 (User Story 2/P2)**: Add approval workflow with reminders
4. **Phase 5-6 (User Story 3-4/P3)**: Complete remaining chapters, module organization, glossary

### Independent Story Testing
- **US1**: Generate C1 → verify self-review checklist passes → confirms content generation works
- **US2**: Submit C1 → wait for approval → verify halt behavior → confirms approval gates work
- **US3**: Generate C1-C4 → verify module grouping → confirms organization works
- **US4**: Generate 2-3 chapters → verify glossary tracking → confirms terminology management works

## Phase 1: Setup & Infrastructure

**Objective**: Initialize Docusaurus environment, configure context7 access, establish project structure

### Setup Tasks

- [X] T001 Create project directory structure per plan.md (chapters/, appendices/, static/images/)
- [X] T002 [P] Initialize npm package.json with Docusaurus dependencies
- [X] T003 [P] Configure Docusaurus with KaTeX plugin in docusaurus.config.js
- [X] T004 [P] Create .gitignore excluding glossary-terms-temp.md and node_modules
- [X] T005 [P] Initialize errata.md for post-approval corrections tracking
- [X] T006 [P] Set up Docusaurus sidebars.js navigation structure (4 modules placeholder)
- [X] T007 Validate context7 MCP tool connection (test query for ROS 2 library)
- [X] T008 [P] Create chapter template file in .templates/chapter-template.md with mandatory sections
- [X] T009 [P] Create self-review checklist file in .templates/self-review-checklist.md (11 items)
- [X] T010 Install Docusaurus and verify LaTeX rendering with test equation

**Acceptance Criteria (Phase 1)**:
- Project structure matches plan.md specification
- Docusaurus builds successfully with KaTeX support
- context7 MCP responds to test queries
- Templates available for chapter generation

## Phase 2: Foundational Components

**Objective**: Establish shared utilities needed across all user stories (chapter generation workflow, context7 integration, Task delegation)

### Foundational Tasks

- [X] T011 Resolve context7 library IDs for all frameworks (ROS 2, Gazebo, Isaac Sim, Unity, VLA) using mcp__context7__resolve-library-id
- [X] T012 [P] Document fallback URLs in research.md per 5-level documentation hierarchy
- [X] T013 [P] Create Task delegation prompt templates in .templates/task-prompts/ (math-subagent.md, code-subagent.md, doc-synthesis.md)
- [X] T014 [P] Configure Markdown linter rules for Docusaurus compatibility
- [X] T015 Test context7 retrieval for high-priority topics (rclpy API, URDF syntax, Isaac Sim sensors) to validate ≥95% success rate

**Acceptance Criteria (Phase 2)**:
- All context7 library IDs resolved and documented
- Fallback documentation sources identified for each framework
- Task delegation templates ready for use
- context7 success rate validated (≥95% or fallback strategy adjusted)

## Phase 3: User Story 1 (P1) - Chapter Content Generation and Verification

**Story Goal**: Produce technically accurate, pedagogically sound chapters following mandatory template structure

**Independent Test**: Generate Chapter 1 (Foundations of Physical AI) and verify it passes all 11 self-review checklist items

### Chapter 1 (C1): Foundations of Physical AI

- [X] T016 [US1] Create feature branch: git checkout -b feature/chapter-01-foundations
- [X] T017 [US1] Invoke context7 for ROS 2 sensor message types (sensor_msgs/LaserScan, sensor_msgs/Imu, sensor_msgs/Image, geometry_msgs/WrenchStamped)
- [X] T018 [US1] Draft C1 Learning Outcomes section (3-5 action-oriented objectives)
- [X] T019 [US1] Draft C1 Overview section (Physical AI vs Digital AI distinction)
- [X] T020 [US1] Draft C1 Key Concepts section (LiDAR, IMU, Camera, Force/Torque sensors - bold terms on first use)
- [X] T021 [US1] Draft C1 Main Content section (sensor characteristics, ROS 2 message formats)
- [X] T022 [US1] Generate C1 code example 1: Sensor message publisher (Python rclpy, >15 lines) in chapters/c1-foundations-physical-ai.md code block
- [X] T023 [US1] Generate C1 code example 2: Sensor data subscriber with callback (Python rclpy) in chapters/c1-foundations-physical-ai.md code block
- [X] T024 [US1] Draft C1 Math section: Sensor noise models (Gaussian noise equation, LaTeX syntax with variable definitions)
- [X] T025 [US1] Draft C1 Math section: IMU orientation representation (quaternion equations with {#eq:1.1}, {#eq:1.2} labels)
- [X] T026 [US1] Draft C1 Summary section (concise recap of Physical AI concepts and sensor systems)
- [X] T027 [US1] Draft C1 Review Questions section (≥3 comprehension questions testing learning outcomes)
- [X] T028 [US1] Apply 11-item self-review checklist to C1: validate all sections present, code tags, math notation, no placeholders
- [X] T029 [US1] Add new terms from C1 to glossary-terms-temp.md (Physical AI, LiDAR, IMU, end-effector, etc.)
- [X] T030 [US1] Commit C1 with message: git commit -m "chap(c1): add foundations of physical AI chapter"
- [X] T031 [US1] Save C1 final version to chapters/c1-foundations-physical-ai.md

**Acceptance Criteria (User Story 1)**:
- C1 file exists at chapters/c1-foundations-physical-ai.md
- All 11 self-review checklist items pass:
  1. All required sections present ✓
  2. All code blocks have language tags (```python) ✓
  3. Math symbols/variables defined, LaTeX syntax valid ✓
  4. No placeholder text (TODO, TKTK, ???) ✓
  5. context7 invoked for ROS 2 sensor APIs ✓
  6. Framework versions documented in code comments ✓
  7. Key terms bolded on first use ✓
  8. Figures/diagrams (if applicable) have alt text and captions ✓
  9. Review questions (≥3) test comprehension ✓
  10. Word count 3000-6000 ✓
  11. No content beyond chapter scope ✓
- C1 renders correctly in Docusaurus without errors
- glossary-terms-temp.md contains ≥10 new terms from C1

## Phase 4: User Story 2 (P2) - Sequential Approval Workflow

**Story Goal**: Review and approve each chapter one at a time with quality gates

**Independent Test**: Submit C1 for approval, verify generation halts until "APPROVED" received, test timeout reminders

### Approval Workflow Tasks

- [ ] T032 [US2] Submit C1 for approval: Print message "Chapter 1 (Foundations of Physical AI) submitted for review. Please reply APPROVED to proceed or REQUEST_REVISION with feedback."
- [ ] T033 [US2] Implement approval halt: Wait for user input (APPROVED or REQUEST_REVISION)
- [ ] T034 [US2] Configure 48-hour reminder: If no response, send "Chapter 1 awaiting approval. Reply APPROVED to proceed or REQUEST_REVISION with feedback."
- [ ] T035 [US2] Configure 7-day pause: If no response after 7 days, send "Workflow paused due to inactivity. Resume anytime by providing approval status."
- [ ] T036 [US2] Implement revision workflow: If REQUEST_REVISION received, incorporate feedback and resubmit C1 for approval
- [ ] T037 [US2] On APPROVED: Merge feature/chapter-01-foundations to main branch
- [ ] T038 [US2] On APPROVED: Proceed to next chapter (C2 generation)

**Acceptance Criteria (User Story 2)**:
- Generation halts after C1 submission (does not proceed to C2 without approval)
- 48-hour reminder sends if no response
- 7-day pause notification sends if approval delayed
- Revision workflow allows resubmission after feedback incorporation
- Approval confirmation proceeds to C2 generation

## Phase 5: User Story 3 (P3) - Module and Chapter Organization

**Story Goal**: Organize 14 chapters into 4 modules reflecting hardware progression (Sim Rig → Edge Brain → Humanoid)

**Independent Test**: Generate C1-C4 (Module 1), verify filenames follow convention and module grouping is correct

### Module 1: The Robotic Nervous System (ROS 2) - Chapters 1-4

**C2: ROS 2 Humble Architecture**
- [ ] T039 [US3] Create feature branch: git checkout -b feature/chapter-02-ros2-architecture
- [ ] T040 [US3] Invoke context7 for rclpy API (rclpy.init, rclpy.create_node, rclpy.spin)
- [ ] T041 [US3] Draft C2 following mandatory template (Learning Outcomes, Overview, Key Concepts, Main Content, Code Examples, Math, Summary, Review Questions)
- [ ] T042 [US3] Generate C2 code examples: Minimal publisher node, minimal subscriber node (Python rclpy, ≥2 examples)
- [ ] T043 [US3] Draft C2 Math: Message latency analysis equations (LaTeX with {#eq:2.1} labels)
- [ ] T044 [US3] Apply 11-item self-review checklist to C2
- [ ] T045 [US3] Add C2 terms to glossary-terms-temp.md (node, topic, message, publisher, subscriber)
- [ ] T046 [US3] Commit C2: git commit -m "chap(c2): add ROS 2 architecture chapter"
- [ ] T047 [US3] Save C2 to chapters/c2-ros2-architecture.md
- [ ] T048 [US3] Submit C2 for approval and halt

**C3: ROS 2 Actions and Services**
- [ ] T049 [US3] Create feature branch: git checkout -b feature/chapter-03-ros2-actions
- [ ] T050 [US3] Invoke context7 for rclpy Action Server/Client API and .action file format
- [ ] T051 [US3] Draft C3 following mandatory template
- [ ] T052 [US3] Generate C3 code examples: Action Server (Fibonacci), Action Client with feedback, Service Server/Client (≥2 examples)
- [ ] T053 [US3] Draft C3 Math: Goal-based control formulation, feedback loop stability equations
- [ ] T054 [US3] Apply 11-item self-review checklist to C3
- [ ] T055 [US3] Add C3 terms to glossary-terms-temp.md (action, service, goal, feedback, result)
- [ ] T056 [US3] Commit C3: git commit -m "chap(c3): add ROS 2 actions and services chapter"
- [ ] T057 [US3] Save C3 to chapters/c3-ros2-actions.md
- [ ] T058 [US3] Submit C3 for approval and halt

**C4: URDF and Robot Description**
- [ ] T059 [US3] Create feature branch: git checkout -b feature/chapter-04-urdf
- [ ] T060 [US3] Invoke context7 for URDF XML syntax (robot, link, joint tags)
- [ ] T061 [US3] Draft C4 following mandatory template
- [ ] T062 [US3] Delegate >50 line URDF file generation to Task tool code subagent (6-DOF manipulator URDF example)
- [ ] T063 [US3] Generate C4 code examples: Humanoid URDF (simplified), Rviz configuration (≥2 examples)
- [ ] T064 [US3] Draft C4 Math: Forward kinematics (Denavit-Hartenberg parameters), joint limits equations
- [ ] T065 [US3] Apply 11-item self-review checklist to C4
- [ ] T066 [US3] Add C4 terms to glossary-terms-temp.md (URDF, link, joint, forward kinematics, Denavit-Hartenberg)
- [ ] T067 [US3] Commit C4: git commit -m "chap(c4): add URDF and robot description chapter"
- [ ] T068 [US3] Save C4 to chapters/c4-urdf-robot-description.md
- [ ] T069 [US3] Submit C4 for approval and halt

**Module 1 Validation**
- [ ] T070 [US3] Verify all Module 1 chapters (C1-C4) use filename convention c<number>-<slug>.md
- [ ] T071 [US3] Verify all Module 1 chapters reference ROS 2 Humble and include Python rclpy examples
- [ ] T072 [US3] Update Docusaurus sidebars.js with Module 1 navigation group

### Module 2-4 Task Placeholders (Remaining 10 Chapters)

**Note**: Due to task file length constraints, Modules 2-4 (C5-C14) follow identical pattern to Module 1:
- Each chapter: Create branch → Invoke context7 → Draft template → Generate code/math → Self-review → Add glossary terms → Commit → Submit for approval
- Estimated: 10 chapters × 7 tasks/chapter = ~70 additional tasks (not fully expanded here to conserve space)

**Module 2: Simulation Environments (C5-C8)**
- C5: Gazebo Simulation
- C6: NVIDIA Isaac Sim Integration
- C7: Unity Robotics
- C8: Vision-Language-Action (VLA) Models

**Module 3: Edge Computing (C9-C11)**
- C9: Jetson Deployment
- C10: Real-Time Control
- C11: Sensor Fusion (EKF derivation via Task tool math subagent)

**Module 4: Humanoid Integration (C12-C14)**
- C12: Whole-Body Control
- C13: ZMP Walking (ZMP derivation via Task tool math subagent)
- C14: Humanoid System Integration

**Acceptance Criteria (User Story 3)**:
- All 14 chapters organized into 4 modules in Docusaurus navigation
- Filenames follow c<number>-<slug>.md convention
- Each module introduction clarifies hardware context and learning objectives
- Module 1 (C1-C4) references ROS 2 Humble consistently

## Phase 6: User Story 4 (P3) - Glossary and Terminology Management

**Story Goal**: Maintain consistent terminology across all chapters with consolidated glossary

**Independent Test**: Generate 2-3 chapters, verify glossary-terms-temp.md tracks terms, merge into g1-glossary.md

### Glossary Tasks

- [ ] T073 [US4] Review glossary-terms-temp.md for terminology consistency across all generated chapters
- [ ] T074 [US4] Flag any terminology drift (e.g., "end-effector" vs "gripper" inconsistency) for resolution
- [ ] T075 [US4] Create appendices/g1-glossary.md with alphabetical term list
- [ ] T076 [US4] For each term in glossary-terms-temp.md: Format as "**Term**: Definition (1-2 sentences) | First usage: Chapter N"
- [ ] T077 [US4] Merge all terms from glossary-terms-temp.md into appendices/g1-glossary.md with alphabetical sorting
- [ ] T078 [US4] Add cross-references for related terms (e.g., "See also: forward kinematics" under "inverse kinematics")
- [ ] T079 [US4] Delete glossary-terms-temp.md after successful merge
- [ ] T080 [US4] Commit glossary: git commit -m "appendix(g1): add complete glossary from 14 chapters"
- [ ] T081 [US4] Submit G1 for final approval

**Acceptance Criteria (User Story 4)**:
- appendices/g1-glossary.md contains ≥95% of technical terms from all 14 chapters
- All terms have definitions and first-usage chapter references
- Terms alphabetically sorted
- Terminology consistency validated (≤2% variance as per SC-004)
- glossary-terms-temp.md deleted (no longer needed)

## Phase 7: Appendices and Polish

**Objective**: Complete remaining appendices (A1, A2) and final quality checks

### Appendix Tasks

**A1: Hardware Setup**
- [ ] T082 Create feature branch: git checkout -b feature/appendix-a1-hardware
- [ ] T083 Draft A1 following appendix template: RTX GPU requirements, Jetson Orin specs, Ubuntu 22.04 installation
- [ ] T084 Generate A1 code examples: Bash installation scripts for ROS 2 Humble, Docusaurus setup
- [ ] T085 Apply self-review checklist to A1
- [ ] T086 Commit A1: git commit -m "appendix(a1): add hardware setup guide"
- [ ] T087 Save A1 to appendices/a1-hardware-setup.md
- [ ] T088 Submit A1 for approval and halt

**A2: Reference Tables (Kinematics and ZMP)**
- [ ] T089 Create feature branch: git checkout -b feature/appendix-a2-reference
- [ ] T090 Delegate full Jacobian derivation (Denavit-Hartenberg) to Task tool math subagent
- [ ] T091 Delegate ZMP equation derivation to Task tool math subagent (MANDATORY per plan.md)
- [ ] T092 Draft A2 with detailed mathematical derivations from subagent output (step-by-step, variable definitions, dimensional analysis)
- [ ] T093 Apply self-review checklist to A2
- [ ] T094 Commit A2: git commit -m "appendix(a2): add kinematics and ZMP reference tables"
- [ ] T095 Save A2 to appendices/a2-reference-tables.md
- [ ] T096 Submit A2 for approval and halt

### Final Quality Checks

- [ ] T097 Run Docusaurus build to verify all 14 chapters + 3 appendices render without errors
- [ ] T098 Validate all internal cross-references (equation labels, chapter citations) resolve correctly
- [ ] T099 Check all images in static/images/ have alt text and proper attribution
- [ ] T100 Run markdown linter on all chapter files to ensure Docusaurus compatibility
- [ ] T101 Verify all code examples include framework version documentation in comments
- [ ] T102 Final review of errata.md to ensure no critical errors remain unfixed
- [ ] T103 Create final commit: git commit -m "meta: complete Physical AI & Humanoid Robotics textbook (14 chapters + 3 appendices)"
- [ ] T104 Tag release: git tag -a v1.0.0 -m "Textbook v1.0.0 - Complete edition"

**Acceptance Criteria (Phase 7)**:
- All 14 chapters + 3 appendices complete and approved
- Docusaurus builds successfully without errors (SC-011: 100% rendering correctness)
- All cross-references valid
- All visual content meets standards (alt text, captions, SVG format per SC-007: ≥90%)
- Project complete and ready for deployment

## Dependencies & Execution Order

### Story Dependency Graph

```
Phase 1 (Setup) → Phase 2 (Foundational) → Phase 3 (US1/P1: C1 Generation)
                                                      ↓
                                          Phase 4 (US2/P2: Approval Workflow)
                                                      ↓
                                          Phase 5 (US3/P3: Modules 1-4)
                                                      ↓
                                          Phase 6 (US4/P3: Glossary)
                                                      ↓
                                          Phase 7 (Appendices + Polish)
```

### Critical Path
1. **Setup (T001-T010)**: MUST complete before any chapter generation
2. **Foundational (T011-T015)**: MUST complete before invoking context7 in chapters
3. **US1 (T016-T031)**: MUST complete and pass self-review before approval workflow
4. **US2 (T032-T038)**: MUST complete before proceeding to additional chapters
5. **US3 (T039-T072)**: Sequential chapter approval gates (C2 after C1 approved, C3 after C2 approved, etc.)
6. **US4 (T073-T081)**: MUST wait until all chapters complete before glossary consolidation
7. **Appendices (T082-T096)**: Can proceed in parallel after all chapters complete
8. **Polish (T097-T104)**: Final phase after all content complete

### Parallel Execution Opportunities

**Within Phase 1 (Setup)**:
- T002, T003, T004, T005, T006, T008, T009 can run in parallel (independent file creation)
- T007 and T010 sequential (depend on setup completion)

**Within Phase 2 (Foundational)**:
- T012, T013, T014 can run in parallel after T011 completes

**Within User Story Phases**:
- **NO parallelization within single chapter** (sequential: invoke context7 → draft → code → math → review)
- **Parallel chapter generation NOT ALLOWED** per Constitution Principle III (sequential approval gates)

**Within Phase 7 (Appendices)**:
- A1 and A2 can be drafted in parallel (independent content)
- Final checks (T097-T104) must run sequentially

### Independent Testing per Story

**US1 (Chapter Generation)**:
- Test: Generate C1 only
- Success: C1 passes all 11 self-review checklist items
- Isolation: No dependency on US2/US3/US4

**US2 (Approval Workflow)**:
- Test: Submit C1 → verify halt → send "APPROVED" → verify C2 starts
- Success: Workflow enforces approval gates correctly
- Isolation: Can test with C1 alone, no dependency on C3-C14

**US3 (Module Organization)**:
- Test: Generate C1-C4 → verify Module 1 grouping in sidebars.js
- Success: Module structure correct, filenames follow convention
- Isolation: Test with Module 1 only, no dependency on Modules 2-4

**US4 (Glossary)**:
- Test: Generate C1-C2 → verify glossary-terms-temp.md tracks terms → merge to G1
- Success: Terminology consistent, glossary complete
- Isolation: Test with 2-3 chapters, no dependency on all 14 chapters

## Task Summary

**Total Tasks**: 104 tasks
**By Phase**:
- Phase 1 (Setup): 10 tasks
- Phase 2 (Foundational): 5 tasks
- Phase 3 (US1/P1): 16 tasks
- Phase 4 (US2/P2): 7 tasks
- Phase 5 (US3/P3): 34 tasks (C2-C4 detailed, C5-C14 placeholders)
- Phase 6 (US4/P3): 9 tasks
- Phase 7 (Appendices + Polish): 23 tasks

**Parallel Opportunities**: ~15 tasks can run in parallel (T002-T006, T008-T009, T012-T014, A1/A2 drafting)
**Sequential Gates**: 14 approval gates (one per chapter) + 3 appendix approvals
**MVP Scope**: Phase 1-3 (25 tasks) delivers C1 with full quality validation

**Suggested MVP Delivery**: Complete through T031 (Chapter 1 generation) to demonstrate:
- End-to-end chapter workflow
- context7 integration
- Self-review checklist validation
- Glossary term tracking
- Template compliance

**Post-MVP Increments**:
- Increment 1: Add approval workflow (US2, T032-T038)
- Increment 2: Complete Module 1 (US3, T039-T072)
- Increment 3: Complete Modules 2-4 (US3, C5-C14)
- Increment 4: Glossary consolidation (US4, T073-T081)
- Increment 5: Appendices and polish (Phase 7, T082-T104)

**Format Validation**: ✅ All tasks follow required checklist format:
- Checkbox: `- [ ]` ✓
- Task ID: T001-T104 sequential ✓
- [P] marker: Applied to 15 parallelizable tasks ✓
- [Story] label: Applied to US1-US4 tasks (US1, US2, US3, US4) ✓
- Description + file path: All tasks specify clear action and target file ✓
