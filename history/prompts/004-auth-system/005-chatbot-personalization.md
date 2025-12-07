# Prompt: Chatbot Personalization

**Date**: 2025-11-29  
**Agent**: Gemini CLI  
**Feature**: 004-auth-system

---

## Request

"Update chatbot to show personalized greeting and user profile info when logged in. Send auth token with chat requests."

---

## Implementation

### Updated ChatBot Component

Modified `src/components/ChatBot.tsx`:

```typescript
const { user, profile } = useAuth();

// Personalized header
{user ? (
  <>
    <h3>Hi, {user.user_metadata?.display_name}!</h3>
    {profile && (
      <div>
        <p>Level: {profile.programming_knowledge}</p>
        <p>Goals: {profile.learning_goals?.join(', ')}</p>
      </div>
    )}
  </>
) : (
  <p>Sign in for personalized learning</p>
)}
```

### Send Auth Token

```typescript
const sendMessage = async () => {
  const token = await supabase.auth.getSession();
  
  const response = await fetch(`${API_URL}/api/chat`, {
    method: 'POST',
    headers: {
      'Authorization': `Bearer ${token.session?.access_token}`,
      'Content-Type': 'application/json'
    },
    body: JSON.stringify({ message, history })
  });
};
```

---

## Testing

✅ Guest sees sign-in prompt  
✅ User sees name and profile  
✅ Token sent with requests  
✅ Backend validates token

---

**Time:** 30 minutes
