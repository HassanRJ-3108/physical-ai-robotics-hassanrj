# Test: Docusaurus Build and Navigation

**Feature**: 002-frontend-book  
**Type**: Integration Test

## Test Setup

```bash
npm install
npm run build
npm run serve
```

## Test Cases

### TC-001: Homepage Loads

**Steps:**
1. Navigate to http://localhost:3000
2. Verify Hero component renders
3. Verify Sections component renders
4. Check for any console errors

**Expected:**
- Homepage loads without errors
- Hero shows title and CTA buttons
- Sections display learning path cards

**Status:** ✅ PASS

---

### TC-002: Sidebar Navigation

**Steps:**
1. Click on "Chapter 1: Physical AI" in sidebar
2. Verify page loads
3. Click on a subsection
4. Verify navigation updates

**Expected:**
- Sidebar items are clickable
- Pages load instantly (client-side routing)
- Active item highlighted in sidebar

**Status:** ✅ PASS

---

### TC-003: Mobile Responsiveness

**Steps:**
1. Resize browser to 320px width
2. Open sidebar menu (hamburger icon)
3. Navigate to a chapter
4. Verify content is readable

**Expected:**
- Sidebar menu button visible
- Menu opens/closes smoothly
- Content fits screen width
- No horizontal scroll

**Status:** ✅ PASS

---

### TC-004: Dark Mode Toggle

**Steps:**
1. Click theme toggle button
2. Verify colors invert
3. Refresh page
4. Verify dark mode persists

**Expected:**
- Theme toggles instantly
- All text readable in dark mode
- Preference saved in localStorage

**Status:** ✅ PASS

---

### TC-005: Language Switch

**Steps:**
1. Click language dropdown
2. Select "Urdu"
3. Verify UI updates
4. Navigate to different page

**Expected:**
- Language switches to Urdu
- UI labels in Urdu
- Language persists across navigation

**Status:** ✅ PASS

---

### TC-006: Build Production

**Steps:**
1. Run `npm run build`
2. Check build/ directory
3. Verify no errors in build output

**Expected:**
- Build completes successfully
- Static HTML files generated
- No TypeScript errors
- bundle size reasonable (<500KB)

**Status:** ✅ PASS

## Summary

**Total Tests:** 6  
**Passed:** 6  
**Failed:** 0  
**Coverage:** Frontend navigation, responsiveness, theming, i18n, build
