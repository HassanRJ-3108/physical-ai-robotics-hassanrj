# Prompt: Navbar Dropdown Menu

**Date**: 2025-11-29  
**Agent**: Gemini CLI  
**Feature**: 004-auth-system

---

## Request

"Add auth UI to navbar. Guest users see Sign In/Sign Up buttons. Logged in users see avatar with dropdown (Profile Setup, Logout)."

---

## Implementation

### NavbarAuth Component

Created `src/components/NavbarAuth.tsx`:

```typescript
export default function NavbarAuth() {
  const { user, signOut } = useAuth();
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const dropdownRef = useRef<HTMLDivElement>(null);

  // Close dropdown on outside click
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (dropdownRef.current && !dropdownRef.current.contains(event.target as Node)) {
        setIsDropdownOpen(false);
      }
    };
    if (isDropdownOpen) {
      document.addEventListener('mousedown', handleClickOutside);
    }
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, [isDropdownOpen]);

  if (!user) {
    return <SignInSignUpButtons />;
  }

  return <UserDropdown user={user} signOut={signOut} />;
}
```

### Swizzled Navbar

Modified `src/theme/Navbar/Content.tsx`:

```typescript
export default function ContentWrapper(props) {
  return (
    <>
      <Content {...props} />
      <NavbarAuth />
    </>
  );
}
```

---

## Challenges

**Dropdown Not Closing:**
- Used useRef + useEffect
- Check click target containment

**Mobile Positioning:**
- Added `right: 0` for alignment
- Responsive CSS media queries

---

## Testing

✅ Guest buttons visible  
✅ User avatar shows  
✅ Dropdown opens/closes  
✅ Outside click works  
✅ Mobile responsive  
✅ Dark mode compatible

---

**Time:** 75 minutes
