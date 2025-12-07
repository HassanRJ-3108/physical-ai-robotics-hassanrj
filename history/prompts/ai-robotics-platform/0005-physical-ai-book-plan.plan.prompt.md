---
id: 0005
title: Physical AI Book Plan
stage: plan
date: 2025-12-01
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-ai-robotics-platform
branch: 001-ai-robotics-platform
user: unknown
command: /sp.plan Rebuild docs from TXT, configure search+i18n, and validate dev/prod flows.
labels: [plan, book, Docusaurus, i18n, search]
links:
  spec: D:/Hackathon/hackathon-project/specs/001-ai-robotics-platform/spec.md
  ticket: null
  adr: null
  pr: null
files:
- D:/Hackathon/hackathon-project/my-book1/docusaurus.config.ts
- D:/Hackathon/hackathon-project/my-book1/sidebars.ts
- D:/Hackathon/hackathon-project/my-book1/src/css/custom.css
tests:

---

## Prompt

```text
Plan
1) Clean docs: remove prior MDX-heavy pages; keep plain Markdown
2) Create new docs from TXT: intro, modules, syllabus, hardware, assessments
3) Navbar: simplify to Docs (docId: intro); auto-generated sidebars
4) i18n: en default; hi (ltr), ur (rtl); dev per-locale servers
5) Search: add @easyops-cn/docusaurus-search-local; gate to production
6) RTL layout fixes: logical properties in custom.css for Urdu
7) Validation: dev on 3000 (en), 3001 (hi), 3002 (ur); prod `build` + `serve`
```

## Response snapshot

Planned incremental rebuild focusing on correctness and simplicity. Decided against fancy MDX widgets to avoid rendering issues. Gated search to production to prevent dev-time module resolution error on Windows.

## Outcome

- ✅ Config updated; search gated; RTL handling set
- 🔧 Docs scaffolded from TXT
- 🧪 Validation path defined for dev/prod

## Evaluation notes (flywheel)

- Windows path/module resolution issues observed with search plugin in dev; gating resolves it
- Next: add translations once English content stabilizes

