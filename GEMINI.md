# GEMINI.md - Agent Context for Gemini CLI

> **AI Agent Context File**  
> This file guides Gemini CLI when working on this repository.  
> Created using Spec-Kit-Plus workflow.

---

## Task context

**Project:** Physical AI - Robotics Book + RAG Chatbot  
**Author:** Hassan RJ  
**Repository:** HassanRJ-3108/Book---Physical-AI---Robotics  
**License:** MIT  
**CI/CD:** GitHub Actions for Docusaurus deployment  
**Security:** Never commit secrets; use .env files and GitHub Secrets  
**Tests:** Include local run instructions and smoke tests  
**Deployment:** GitHub Pages for frontend, Cloud platform for backend

---

## Core Guarantees (Product Promise)

1. **Comprehensive Book Content**: Docusaurus-based book covering Physical AI, ROS 2, Simulation, NVIDIA Isaac, VLA models, and Humanoid Robotics
2. **Interactive RAG Chatbot**: Embedded chatbot using FastAPI backend, Qdrant vector DB, and Gemini/OpenAI for inference
3. **User Authentication**: Supabase Auth integration for personalized learning experiences
4. **Responsive Design**: Mobile-first, accessible UI with teal theme (#1e7a6f)
5. **Internationalization**: Multi-language support (English, Urdu)

---

## Development Guidelines

### 1. Authoritative Source Mandate:
- **Constitution**: `.specify/memory/constitution.md` defines project principles
- **ADRs**: Architecture decisions in `.specify/memory/adrs/`
- **Specs**: Feature specifications in `specs/`
- **Tasks**: Task breakdowns in `tasks/`
- **Plans**: Implementation plans in `plans/`

### 2. Execution Flow:
```
User Request → Check Constitution → Review Specs → Plan → Implement → Test → Document
```

### 3. Knowledge capture (PHR) for Every User Input.
- Save prompts to `history/prompts/<feature-name>/`
- Use consistent naming: `001-auth`, `002-chatbot`, etc.
- Include timestamp and agent name

### 4. Explicit ADR suggestions
When making architectural decisions:
- Propose ADR if decision impacts:
  - Data model
  - Technology stack
  - Security
  - Performance
  - User experience
- Format: `.specify/memory/adrs/NNN-decision-title.md`

### 5. Human as Tool Strategy
Ask clarifying questions when:
- Requirements are ambiguous
- Multiple valid approaches exist
- Security/privacy implications
- Performance trade-offs

---

## Default policies (must follow)

### Execution contract for every request

1. **Read Constitution First**: Check `.specify/memory/constitution.md` for relevant principles
2. **Review Existing Specs**: Check if feature spec exists in `specs/`
3. **Check ADRs**: Review architecture decisions in `.specify/memory/adrs/`
4. **Plan Before Code**: Create or update plan in `plans/` if significant change
5. **Update Tasks**: Mark completed tasks in `tasks/`
6. **Write Tests**: Backend changes require tests in `tests/`
7. **Document Changes**: Update README or relevant docs

### Minimum acceptance criteria

- [ ] Code compiles/runs without errors
- [ ] Follows project code style (see Code Standards below)
- [ ] No hardcoded secrets or credentials
- [ ] Mobile responsive (if UI change)
- [ ] Dark mode compatible (if UI change)
- [ ] Accessibility considerations (ARIA labels, keyboard navigation)
- [ ] Updated relevant documentation

---

## Architect Guidelines (for planning)

When creating or updating implementation plans:

1. **Reference Constitution**: Cite specific principles from constitution
2. **Consider ADRs**: Ensure plan aligns with existing architecture decisions
3. **Tech Stack Justification**: Explain why specific technologies chosen
4. **Data Model**: Include database schema if backend changes
5. **API Contracts**: Define request/response formats
6. **Testing Strategy**: Unit tests, integration tests, E2E tests
7. **Deployment**: How changes will be deployed
8. **Rollback Plan**: How to revert if issues arise

### Architecture Decision Records (ADR) - Intelligent Suggestion

Suggest ADR when:
- **New dependency**: Adding a new library/framework
- **Data model change**: Altering database schema
- **API design**: Creating or changing API endpoints
- **Security**: Authentication, authorization changes
- **Performance**: Caching, optimization strategies
  
**ADR Template:**
```markdown
# ADR NNN: [Title]

**Status:** [Proposed | Accepted | Deprecated | Superseded]

## Context
[Describe the problem or opportunity]

## Decision
[What was decided]

## Consequences
**Positive:**
- [Benefit 1]
- [Benefit 2]

**Negative:**
- [Trade-off 1]
- [Trade-off 2]

**Mitigation:**
- [How to address negatives]
```

---

## Basic Project Structure

```
hackathon-project/
├── .specify/
│   └── memory/
│       ├── constitution.md      # Project principles
│       └── adrs/                # Architecture Decision Records
├── specs/                       # Feature specifications
├── tasks/                       # Task breakdowns
├── plans/                       # Implementation plans
├── checklist/                   # Quality checklists
├── history/                     # Prompt history records
├── tests/                       # Test files
├── docs/                        # Docusaurus content
├── src/                         # React components
├── backend/                     # FastAPI backend
│   ├── chatbot.py              # RAG chatbot implementation
│   ├── auth/                   # Authentication routes
│   └── config/                 # Supabase client config
└── static/                      # Static assets
```

---

## Code Standards

### TypeScript/React (Frontend)
- Use functional components with hooks
- TypeScript strict mode enabled
- CSS Modules for component styling
- Avoid inline styles unless dynamic
- Accessibility: ARIA labels, semantic HTML
- Mobile-first responsive design

### Python (Backend)
- Follow PEP 8 style guide
- Type hints for function signatures
- Async/await for I/O operations
- Environment variables for configuration
- Structured logging (not print statements)
- Error handling with proper HTTP status codes

### Git Workflow
- Feature branches: `feat/feature-name`
- Commit messages: Conventional Commits format
- PR descriptions: Link to related spec/task
- No direct commits to `main`

---

## Quick Reference: Spec-Kit Plus Commands

```bash
# Constitution (establish project principles)
/sp.constitution Create principles focused on code quality, testing standards, user experience consistency, and performance requirements

# Specify (define feature requirements)
/sp.specify Build a feature that allows users to [description]

# Clarify (ask structured questions)
/sp.clarify

# Plan (create implementation plan)
/sp.plan Use [tech stack] with [constraints]

# Tasks (generate task breakdown)
/sp.tasks

# Implement (execute implementation)
/sp.implement Implement tasks 1-5

# Analyze (cross-artifact consistency check)
/sp.analyze

# Checklist (generate quality checklist)
/sp.checklist
```

---

## Current Features

### Frontend (Docusaurus + React)
- ✅ Book content with sidebar navigation
- ✅ Custom teal theme (#1e7a6f)
- ✅ Hero section with gradient background
- ✅ Internationalization (English, Urdu)
- ✅ Dark mode support
- ✅ Responsive design

### Backend (FastAPI + RAG)
- ✅ FastAPI server with CORS
- ✅ RAG chatbot with OpenAI Agents SDK
- ✅ Qdrant vector database integration
- ✅ Gemini API for inference
- ✅ Data ingestion pipeline
- ✅ Conversation history management

### Authentication (Supabase)
- ✅ Supabase Auth integration
- ✅ Sign up/sign in pages with Auth UI
- ✅ User profile setup
- ✅ Navbar auth UI with dropdown
- ✅ Chatbot personalization

---

## Environment Variables

**Frontend** (`.env.local`):
```
NEXT_PUBLIC_SUPABASE_URL=https://uretyyjnmyqqqxkxpwzl.supabase.co
NEXT_PUBLIC_SUPABASE_ANON_KEY=[anon-key]
```

**Backend** (`backend/.env`):
```
GEMINI_API_KEY=[gemini-api-key]
SUPABASE_URL=https://uretyyjnmyqqqxkxpwzl.supabase.co
SUPABASE_SERVICE_ROLE_KEY=[service-role-key]
DATABASE_URL=[neon-db-url]  # Legacy, not used
AUTH_SECRET=[auth-secret]    # Legacy, not used
```

---

## Testing

### Frontend
```bash
npm test                    # Run React component tests
npm run build              # Verify build succeeds
```

### Backend
```bash
cd backend
uv run pytest              # Run all tests
uv run python test_*.py    # Run specific test
```

---

## Deployment

### Frontend (GitHub Pages)
```bash
npm run deploy             # Deploy to gh-pages branch
```

### Backend (Manual)
```bash
cd backend
uv run uvicorn main:app --host 0.0.0.0 --port 8000
```

---

**Remember:** Always check constitution before making architectural decisions!
