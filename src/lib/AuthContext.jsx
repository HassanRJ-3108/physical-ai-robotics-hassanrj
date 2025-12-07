import React, { createContext, useContext, useEffect, useState } from 'react';
import { authAPI } from './authAPI';

const AuthContext = createContext(undefined);

export function AuthProvider({ children }) {
    const [user, setUser] = useState(null);
    const [profile, setProfile] = useState(null);
    const [loading, setLoading] = useState(true);

    // Refresh profile
    const refreshProfile = async () => {
        try {
            const userData = await authAPI.getCurrentUser();
            if (userData) {
                setUser(userData.user);
                setProfile(userData.profile || null);
            } else {
                setUser(null);
                setProfile(null);
            }
        } catch (error) {
            console.error('Error fetching profile:', error);
            setUser(null);
            setProfile(null);
        }
    };

    // Sign out
    const signOut = async () => {
        try {
            await authAPI.signout();
            setUser(null);
            setProfile(null);
        } catch (error) {
            console.error('Error signing out:', error);
        }
    };

    // Load user on mount
    useEffect(() => {
        const loadUser = async () => {
            if (typeof window === 'undefined') {
                console.log('⚠️ SSR mode - skipping auth load');
                setLoading(false);
                return;
            }

            console.log('🔄 Loading user from token...');

            try {
                const userData = await authAPI.getCurrentUser();
                if (userData) {
                    console.log('✅ User loaded:', userData.user.email);
                    setUser(userData.user);
                    setProfile(userData.profile || null);
                } else {
                    console.log('❌ No user data (token invalid or missing)');
                }
            } catch (error) {
                console.error('❌ Error loading user:', error);
            } finally {
                setLoading(false);
                console.log('✅ Auth loading complete');
            }
        };

        loadUser();
    }, []);

    const value = {
        user,
        profile,
        loading,
        signOut,
        refreshProfile,
    };

    return <AuthContext.Provider value={value}>{children}</AuthContext.Provider>;
}

export function useAuth() {
    const context = useContext(AuthContext);
    if (context === undefined) {
        throw new Error('useAuth must be used within an AuthProvider');
    }
    return context;
}
