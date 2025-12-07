import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import { authAPI } from '../lib/authAPI';
import styles from './signin.module.css';

export default function SignIn() {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [error, setError] = useState('');
    const [loading, setLoading] = useState(false);
    const history = useHistory();

    const handleSubmit = async (e) => {
        e.preventDefault();
        console.log('🔵 Form submit event fired!');

        setError('');
        setLoading(true);

        try {
            console.log('📧 Attempting signin...');
            const result = await authAPI.signin(email, password);
            console.log('✅ Signin successful!', result);

            // Successfully signed in, navigate to profile page
            window.location.href = '/profile';
        } catch (err) {
            console.error('❌ Signin failed:', err);
            setError(err.message || 'Sign in failed');
            setLoading(false);
        }
    };

    return (
        <Layout title="Sign In" description="Sign in to your account">
            <div className={styles.container}>
                <div className={styles.authCard}>
                    <div className={styles.header}>
                        <h1>Welcome Back</h1>
                        <p>Sign in to continue your robotics learning journey</p>
                    </div>

                    <form onSubmit={handleSubmit} className={styles.form}>
                        <div className={styles.formGroup}>
                            <label htmlFor="email">Email</label>
                            <input
                                id="email"
                                type="email"
                                value={email}
                                onChange={(e) => setEmail(e.target.value)}
                                placeholder="your@email.com"
                                required
                                disabled={loading}
                            />
                        </div>

                        <div className={styles.formGroup}>
                            <label htmlFor="password">Password</label>
                            <input
                                id="password"
                                type="password"
                                value={password}
                                onChange={(e) => setPassword(e.target.value)}
                                placeholder="••••••••"
                                required
                                disabled={loading}
                            />
                        </div>

                        {error && (
                            <div className={styles.error}>
                                {error}
                            </div>
                        )}

                        <button
                            type="submit"
                            className={styles.submitButton}
                            disabled={loading}
                        >
                            {loading ? 'Signing in...' : 'Sign In'}
                        </button>
                    </form>

                    <div className={styles.footer}>
                        <p>
                            Don't have an account?{' '}
                            <a href="/signup">Sign up</a>
                        </p>
                    </div>
                </div>
            </div>
        </Layout>
    );
}
