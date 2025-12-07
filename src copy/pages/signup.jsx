import React, { useState } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import { authAPI } from '../lib/authAPI';
import styles from './signin.module.css';

export default function SignUp() {
    const [email, setEmail] = useState('');
    const [password, setPassword] = useState('');
    const [name, setName] = useState('');
    const [error, setError] = useState('');
    const [success, setSuccess] = useState('');
    const [loading, setLoading] = useState(false);
    const history = useHistory();

    const handleSubmit = async (e) => {
        e.preventDefault();
        setError('');
        setSuccess('');
        setLoading(true);

        try {
            const result = await authAPI.signup({
                email,
                password,
                name,
                profile: {
                    programming_knowledge: 'beginner',
                    prior_robotics_experience: false,
                    learning_goals: [],
                    preferred_learning_style: 'mixed'
                }
            });

            // Check if email confirmation is required
            if (result.email_confirmation_required) {
                setSuccess('Account created! Please check your email to confirm your account before signing in.');
            } else {
                // Auto login if no confirmation needed
                history.push('/');
                window.location.reload();
            }
        } catch (err) {
            setError(err.message || 'Sign up failed');
        } finally {
            setLoading(false);
        }
    };

    return (
        <Layout title="Sign Up" description="Create your account">
            <div className={styles.container}>
                <div className={styles.authCard}>
                    <div className={styles.header}>
                        <h1>Create Account</h1>
                        <p>Start your robotics learning journey</p>
                    </div>

                    <form onSubmit={handleSubmit} className={styles.form}>
                        <div className={styles.formGroup}>
                            <label htmlFor="name">Full Name</label>
                            <input
                                id="name"
                                type="text"
                                value={name}
                                onChange={(e) => setName(e.target.value)}
                                placeholder="John Doe"
                                required
                                disabled={loading}
                            />
                        </div>

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
                                minLength={8}
                                disabled={loading}
                            />
                            <small style={{ color: '#666', fontSize: '12px' }}>
                                Minimum 8 characters
                            </small>
                        </div>

                        {error && (
                            <div className={styles.error}>
                                {error}
                            </div>
                        )}

                        {success && (
                            <div style={{
                                padding: '12px',
                                background: '#d4edda',
                                color: '#155724',
                                borderRadius: '4px',
                                marginBottom: '16px'
                            }}>
                                {success}
                            </div>
                        )}

                        <button
                            type="submit"
                            className={styles.submitButton}
                            disabled={loading}
                        >
                            {loading ? 'Creating account...' : 'Sign Up'}
                        </button>
                    </form>

                    <div className={styles.footer}>
                        <p>
                            Already have an account?{' '}
                            <a href="/signin">Sign in</a>
                        </p>
                    </div>
                </div>
            </div>
        </Layout>
    );
}
