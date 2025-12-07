---
id: 0004
title: Physical AI Book Specification
stage: spec
date: 2025-12-01
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: unknown
command: /sp.specify Build the "Physical AI & Humanoid Robotics" book strictly from TXT, with clean Markdown docs, i18n (en/hi/ur), and production-gated local search.
labels: [spec, book, Docusaurus, i18n, search]
links:
  spec: D:/Hackathon/hackathon-project/specs/001-ai-robotics-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
- D:/Hackathon/hackathon-project/my-book1/docs/intro.md
- D:/Hackathon/hackathon-project/my-book1/docs/modules/ros2.md
- D:/Hackathon/hackathon-project/my-book1/docs/modules/simulation.md
- D:/Hackathon/hackathon-project/my-book1/docs/modules/isaac.md
- D:/Hackathon/hackathon-project/my-book1/docs/modules/vla.md
- D:/Hackathon/hackathon-project/my-book1/docs/syllabus.md
- D:/Hackathon/hackathon-project/my-book1/docs/hardware.md
- D:/Hackathon/hackathon-project/my-book1/docs/assessments.md
tests:

---

## Prompt

```text
Specification: Physical AI & Humanoid Robotics book

Scope
- Single source of truth: /D:/Hackathon/hackathon-project/Physical AI & Humanoid Robotics.txt
- No videos, no MDX-only widgets; clean Markdown + tables
- English default, Hindi/Urdu locales supported; translations added incrementally

Information Architecture
- Intro (focus/theme, outcomes, modules, weekly summary, assessments)
- Modules (ROS2, Simulation, Isaac, VLA)
- Syllabus (Weeks 1–13)
- Hardware (Workstation, Edge Kit, Robot Lab, Architecture, Cloud, Jetson kit, Latency)
- Assessments (4 items)

Non-Functional
- i18n: dev serves one locale at a time; prod bundles all
- Search: @easyops-cn/docusaurus-search-local, gated to production builds
- RTL: Urdu direction 'rtl'; logical CSS properties in custom.css
```

## Response snapshot

Defined the book scope and IA strictly from the TXT. Set NFRs for i18n, search, and RTL layout. Documented constraints (dev single-locale behavior) and production expectations (locale switch works after build). Ensured content uses plain Markdown with tables to avoid rendering regressions.

## Outcome

- ✅ Spec finalized per TXT; structure and constraints documented
- 📁 Files: all core docs added under my-book1/docs
- 🔍 Search to be enabled only in production
- 🌐 i18n behavior clarified for dev/prod

## Evaluation notes (flywheel)

- Risks: translation gaps can cause 404 on non-default locales in prod
- Mitigation: create English-first content, add i18n progressively; verify with `npm run build` + `serve`

