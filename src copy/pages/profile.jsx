import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { useHistory } from '@docusaurus/router';
import { useAuth } from '../lib/AuthContext';
import { authAPI } from '../lib/authAPI';
import styles from './signin.module.css';

export default function Profile() {
    const { user, profile, refreshProfile } = useAuth();
    const history = useHistory();

    const [programmingKnowledge, setProgrammingKnowledge] = useState('beginner');
    const [priorRoboticsExperience, setPriorRoboticsExperience] = useState(false);
    const [learningGoals, setLearningGoals] = useState([]);
    const [learningStyle, setLearningStyle] = useState('mixed');
    const [error, setError] = useState('');
    const [success, setSuccess] = useState('');
    const [loading, setLoading] = useState(false);

    const goalOptions = ['ros2', 'simulation', 'ai', 'hardware', 'vla-models', 'humanoid-robotics'];

    // Load existing profile data
    useEffect(() => {
        if (profile) {
            setProgrammingKnowledge(profile.programming_knowledge);
            setPriorRoboticsExperience(profile.prior_robotics_experience);
            setLearningGoals(profile.learning_goals || []);
            setLearningStyle(profile.preferred_learning_style);
        }
    }, [profile]);

    // Redirect if not logged in
    useEffect(() => {
        if (!user) {
            history.push('/signin');
        }
    }, [user, history]);

    const handleGoalToggle = (goal) => {
        setLearningGoals(prev =>
            prev.includes(goal) ? prev.filter(g => g !== goal) : [...prev, goal]
        );
    };

    const handleSubmit = async (e) => {
        e.preventDefault();
        setError('');
        setSuccess('');
        setLoading(true);

        try {
            await authAPI.updateProfile({
                id: user.id,
                programming_knowledge: programmingKnowledge,
                prior_robotics_experience: priorRoboticsExperience,
                learning_goals: learningGoals,
                preferred_learning_style: learningStyle
            });

            setSuccess('Profile updated successfully!');
            await refreshProfile(); // Refresh profile in context
        } catch (err) {
            setError(err.message || 'Profile update failed');
        } finally {
            setLoading(false);
        }
    };

    if (!user) {
        return null; // Will redirect
    }

    return (
        <Layout title="Profile Settings" description="Update your learning profile">
            <div className={styles.container}>
                <div className={styles.authCard}>
                    <div className={styles.header}>
                        <h1>Your Learning Profile</h1>
                        <p>Customize your learning experience</p>
                    </div>

                    <form onSubmit={handleSubmit} className={styles.form}>
                        <div className={styles.formGroup}>
                            <label htmlFor="email">Email</label>
                            <input
                                id="email"
                                type="email"
                                value={user.email}
                                disabled
                                style={{ background: '#f5f5f5', cursor: 'not-allowed' }}
                            />
                            <small style={{ color: '#666', fontSize: '12px' }}>
                                Email cannot be changed
                            </small>
                        </div>

                        <div className={styles.formGroup}>
                            <label htmlFor="programming">Programming Knowledge</label>
                            <select
                                id="programming"
                                value={programmingKnowledge}
                                onChange={(e) => setProgrammingKnowledge(e.target.value)}
                                disabled={loading}
                            >
                                <option value="beginner">Beginner</option>
                                <option value="intermediate">Intermediate</option>
                                <option value="advanced">Advanced</option>
                            </select>
                        </div>

                        <div className={styles.formGroup}>
                            <label htmlFor="learningStyle">Preferred Learning Style</label>
                            <select
                                id="learningStyle"
                                value={learningStyle}
                                onChange={(e) => setLearningStyle(e.target.value)}
                                disabled={loading}
                            >
                                <option value="hands-on">Hands-on Practice</option>
                                <option value="theory-first">Theory First</option>
                                <option value="mixed">Mixed Approach</option>
                            </select>
                        </div>

                        <div className={styles.formGroup}>
                            <label>
                                <input
                                    type="checkbox"
                                    checked={priorRoboticsExperience}
                                    onChange={(e) => setPriorRoboticsExperience(e.target.checked)}
                                    disabled={loading}
                                />
                                {' '}I have prior robotics experience
                            </label>
                        </div>

                        <div className={styles.formGroup}>
                            <label>Learning Goals (select all that apply)</label>
                            <div style={{ display: 'flex', flexWrap: 'wrap', gap: '8px', marginTop: '8px' }}>
                                {goalOptions.map(goal => (
                                    <button
                                        key={goal}
                                        type="button"
                                        onClick={() => handleGoalToggle(goal)}
                                        disabled={loading}
                                        style={{
                                            padding: '6px 12px',
                                            borderRadius: '20px',
                                            border: learningGoals.includes(goal) ? '2px solid #1e7a6f' : '2px solid #ccc',
                                            background: learningGoals.includes(goal) ? '#1e7a6f' : 'transparent',
                                            color: learningGoals.includes(goal) ? 'white' : 'inherit',
                                            cursor: 'pointer',
                                            fontSize: '14px',
                                            transition: 'all 0.2s'
                                        }}
                                    >
                                        {goal}
                                    </button>
                                ))}
                            </div>
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
                            {loading ? 'Saving...' : 'Save Profile'}
                        </button>
                    </form>
                </div>
            </div>
        </Layout>
    );
}
