# Project Constitution

**Version:** 1.0.0  
**Last Updated:** 2025-12-06  
**Status:** Active

---

## Project Identity

**Name:** Physical AI - Robotics Book & RAG Chatbot  
**Description:** A comprehensive educational platform combining a Docusaurus-based book on Physical AI, ROS 2, Simulation, and Robotics with an intelligent RAG chatbot powered by FastAPI and Gemini API.

**Purpose:** To provide accessible, high-quality education on Physical AI and robotics topics through interactive content and personalized learning experiences.

---

## Core Principles

### 1. Code Quality
**Rationale:** Maintainable, readable code reduces technical debt and enables faster iteration.

**Practices:**
- Write self-documenting code with meaningful variable/function names
- Keep functions focused and under 50 lines when possible
- Document complex algorithms and business logic
- Use TypeScript strict mode for frontend code
- Follow PEP 8 style guide for Python backend

### 2. Testing Standards
**Rationale:** Automated tests prevent regressions and enable confident refactoring.

**Practices:**
- Write unit tests for backend API endpoints
- Test authentication flows end-to-end
- Validate chatbot responses with known inputs
- Monitor test coverage for critical paths
- Include smoke tests in deployment pipeline

### 3. Security First
**Rationale:** Protecting user data and preventing vulnerabilities is non-negotiable.

**Practices:**
- Never commit secrets, API keys, or credentials to version control
- Use environment variables for all sensitive configuration
- Implement Row Level Security (RLS) in Supabase
- Validate and sanitize all user inputs
- Keep dependencies updated to patch security vulnerabilities
- Use HTTPS for all production endpoints

### 4. User Experience Consistency
**Rationale:** Predictable, intuitive UX reduces friction and increases engagement.

**Practices:**
- Maintain consistent teal theme (#1e7a6f) across all pages
- Ensure mobile responsiveness (320px - 1920px viewports)
- Support dark mode throughout the application
- Provide clear feedback for user actions (loading states, errors, success)
- Follow accessibility guidelines (WCAG 2.1 AA)
- Use semantic HTML and ARIA labels

### 5. Performance Requirements
**Rationale:** Fast, responsive applications improve user satisfaction and SEO rankings.

**Practices:**
- Optimize chatbot response times (< 3 seconds for initial response)
- Lazy-load images and code-split React components
- Minimize bundle sizes and use tree-shaking
- Implement caching strategies for frequently accessed data
- Target Lighthouse performance score >90
- Monitor Core Web Vitals (LCP, FID, CLS)

### 6. Accessibility
**Rationale:** Digital accessibility is a human right and legal requirement.

**Practices:**
- Keyboard navigation support for all interactive elements
- Screen reader compatibility
- Sufficient color contrast (4.5:1 for normal text)
- Alt text for all images
- Focus indicators visible at all times
- No reliance on color alone to convey information

---

## Technology Stack Governance

### Frontend
- **Framework:** Docusaurus 3.x (React-based static site generator)
- **Styling:** CSS Modules + custom.css with CSS variables
- **TypeScript:** Strict mode enabled
- **Authentication UI:** Supabase Auth UI React components
- **State Management:** React Context API for global auth state

### Backend
- **Framework:** FastAPI (Python async web framework)
- **AI SDK:** OpenAI Agents SDK with Gemini API
- **Vector Database:** Qdrant Cloud (free tier)
- **Authentication:** Supabase Auth with JWT validation
- **Database:** Supabase PostgreSQL for user profiles

### Infrastructure
- **Frontend Deployment:** GitHub Pages
- **Backend Deployment:** Cloud platform (Railway, Render, etc.)
- **CI/CD:** GitHub Actions for automated deployments
- **Version Control:** Git with conventional commits

---

## Development Workflow

### Branching Strategy
- `main` - Production-ready code
- `feat/*` - Feature development branches
- `fix/*` - Bug fix branches
- `docs/*` - Documentation updates

### Commit Message Format
```
type(scope): description

[optional body]

[optional footer]
```

**Types:** feat, fix, docs, style, refactor, test, chore

### Pull Request Requirements
- Link to related spec in `specs/` folder
- Reference relevant task in `tasks/` folder
- Include screenshots for UI changes
- Pass all CI checks
- At least one review approval

### Code Review Checklist
- [ ] Code follows style guidelines
- [ ] No hardcoded secrets or credentials
- [ ] Tests added/updated
- [ ] Documentation updated
- [ ] Mobile responsive (if UI)
- [ ] Dark mode compatible (if UI)
- [ ] Accessibility considered

---

## Architecture Decision Records (ADRs)

All significant architecture decisions must be documented as ADRs in `.specify/memory/adrs/`.

**When to create an ADR:**
- Adding a new technology or framework
- Changing database schema or data model
- Modifying API contracts or authentication flows
- Performance optimization strategies
- Security implementation choices

**ADR Format:**
- Title: `NNN-kebab-case-description.md`
- Status: Proposed | Accepted | Deprecated | Superseded
- Sections: Context, Decision, Consequences

---

## Quality Gates

### Before Merging to Main
- [ ] All tests pass
- [ ] No TypeScript/Python linting errors
- [ ] Build succeeds without warnings
- [ ] Code review approved
- [ ] Documentation updated

### Before Production Deployment
- [ ] Staging environment tested
- [ ] Performance benchmarks met
- [ ] Security scan passed
- [ ] Environment variables configured
- [ ] Rollback plan documented

---

## Documentation Requirements

### Code Documentation
- Public functions/classes must have docstrings
- Complex logic requires inline comments
- API endpoints documented in OpenAPI/Swagger format

### Project Documentation
- README.md kept up-to-date with setup instructions
- ENV_SETUP.md documents all environment variables
- DEPLOYMENT.md explains deployment process
- Specs in `specs/` folder for all features
- Tasks in `tasks/` folder with progress tracking

---

## Governance

### Constitution Updates
- Major version (X.0.0): Fundamental principle changes
- Minor version (1.X.0): New principles added
- Patch version (1.0.X): Clarifications or formatting

**Amendment Process:**
1. Propose change via GitHub issue
2. Team discussion and consensus
3. Update constitution.md with new version
4. Document rationale in commit message

### Compliance Review
- Monthly review of adherence to principles
- Quarterly assessment of tech stack currency
- Annual constitution relevance check

---

## License

This project is licensed under the MIT License. All code, documentation, and assets are freely available for educational and commercial use with attribution.

---

## Contact

**Project Maintainer:** Hassan RJ  
**Repository:** https://github.com/HassanRJ-3108/Book---Physical-AI---Robotics  
**Issues:** https://github.com/HassanRJ-3108/Book---Physical-AI---Robotics/issues

---

**Remember:** This constitution is a living document. It should evolve with the project's needs while maintaining our core commitment to quality, security, and user-centric design.
