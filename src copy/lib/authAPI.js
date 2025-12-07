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

    async signup(data) {
        const response = await fetch(`${API_URL}/api/auth/signup`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(data)
        });

        if (!response.ok) {
            const error = await response.json();
            throw new Error(error.detail || 'Signup failed');
        }

        const result = await response.json();
        if (result.session?.access_token) {
            this.setToken(result.session.access_token);
        }
        return result;
    }

    async signin(email, password) {
        const response = await fetch(`${API_URL}/api/auth/signin`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ email, password })
        });

        if (!response.ok) {
            const error = await response.json();
            throw new Error(error.detail || 'Sign in failed');
        }

        const result = await response.json();
        if (result.access_token) {
            this.setToken(result.access_token);
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
