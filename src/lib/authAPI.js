/**
 * API Client for Backend Authentication
 * 
 * All auth operations go through FastAPI backend
 * Backend is deployed on Railway
 */

// Backend API URL - Railway deployment
const API_URL = 'https://backend1-production-ccbd.up.railway.app';

class AuthAPI {
    getToken() {
        if (typeof window === 'undefined') return null;
        return localStorage.getItem('auth_token');
    }

    setToken(token) {
        if (typeof window === 'undefined') return;
        localStorage.setItem('auth_token', token);
    }

    removeToken() {
        if (typeof window === 'undefined') return;
        localStorage.removeItem('auth_token');
    }

    async signup(email, password) {
        const response = await fetch(`${API_URL}/api/auth/signup`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({
                email,
                password,
                name: email.split('@')[0], // Use email prefix as default name
                profile: {
                    programming_knowledge: 'beginner',
                    prior_robotics_experience: false,
                    learning_goals: [],
                    preferred_learning_style: 'mixed'
                }
            })
        });

        if (!response.ok) {
            const error = await response.json();
            // Extract meaningful error message
            const errorMsg = error.detail || error.message || 'Signup failed';
            throw new Error(errorMsg);
        }

        const result = await response.json();

        // Check if email confirmation is required
        // If session exists and has access_token, user is logged in
        // Otherwise, email confirmation is required
        if (result.session && result.session.access_token) {
            this.setToken(result.session.access_token);
            return { ...result, email_confirmation_required: false };
        } else {
            // Email confirmation required
            return { ...result, email_confirmation_required: true };
        }
    }

    async signin(email, password) {
        console.log('🔐 Attempting signin:', { email, backend: API_URL });

        const response = await fetch(`${API_URL}/api/auth/signin`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ email, password })
        });

        console.log('📡 Response status:', response.status);

        if (!response.ok) {
            let error;
            try {
                error = await response.json();
                console.error('❌ Backend error:', error);
            } catch (e) {
                console.error('❌ Parse error:', e);
                throw new Error(`Sign in failed (HTTP ${response.status})`);
            }
            // Extract meaningful error message
            const errorMsg = error.detail || error.message || 'Sign in failed';
            throw new Error(errorMsg);
        }

        const result = await response.json();
        console.log('✅ Success:', { hasToken: !!result.access_token });

        // Backend returns { access_token, user, profile }
        if (result.access_token) {
            this.setToken(result.access_token);
        } else {
            throw new Error('No access token received from server');
        }

        return result;
    }

    async signout() {
        await fetch(`${API_URL}/api/auth/signout`, {
            method: 'POST'
        });
        this.removeToken();
    }

    async getCurrentUser() {
        const token = this.getToken();
        if (!token) return null;

        const response = await fetch(`${API_URL}/api/auth/user?token=${token}`);

        if (!response.ok) {
            this.removeToken();
            return null;
        }

        return await response.json();
    }

    async updateProfile(profileData) {
        const token = this.getToken();
        if (!token) throw new Error('Not authenticated');

        const response = await fetch(`${API_URL}/api/auth/profile?token=${token}`, {
            method: 'PUT',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(profileData)
        });

        if (!response.ok) {
            const error = await response.json();
            throw new Error(error.detail || 'Profile update failed');
        }

        const result = await response.json();
        return result.profile;
    }

    isAuthenticated() {
        return !!this.getToken();
    }
}

export const authAPI = new AuthAPI();
