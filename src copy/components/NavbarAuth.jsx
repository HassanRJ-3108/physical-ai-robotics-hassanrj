import React, { useState, useEffect, useRef } from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '@site/src/lib/AuthContext';
import styles from './NavbarAuth.module.css';
import BrowserOnly from '@docusaurus/BrowserOnly';

function NavbarAuthClient() {
    const { user, signOut } = useAuth();
    const [isDropdownOpen, setIsDropdownOpen] = useState(false);
    const dropdownRef = useRef(null);

    useEffect(() => {
        const handleClickOutside = (event) => {
            if (dropdownRef.current && !dropdownRef.current.contains(event.target)) {
                setIsDropdownOpen(false);
            }
        };

        if (isDropdownOpen) {
            document.addEventListener('mousedown', handleClickOutside);
        }

        return () => {
            document.removeEventListener('mousedown', handleClickOutside);
        };
    }, [isDropdownOpen]);

    if (!user) {
        return (
            <div className={styles.authContainer}>
                <Link to="/signin" className={styles.signInButton}>
                    Sign In
                </Link>
                <Link to="/signup" className={styles.signUpButton}>
                    Sign Up
                </Link>
            </div>
        );
    }

    const displayName = user.user_metadata?.display_name || user.email?.split('@')[0] || 'User';

    return (
        <div className={styles.authContainer} ref={dropdownRef}>
            <button
                className={styles.userButton}
                onClick={() => setIsDropdownOpen(!isDropdownOpen)}
                aria-label="User menu"
            >
                <div className={styles.avatar}>
                    {displayName.charAt(0).toUpperCase()}
                </div>
                <span className={styles.userName}>{displayName}</span>
                <svg width="12" height="12" viewBox="0 0 12 12" className={styles.dropdownIcon}>
                    <path d="M2 4L6 8L10 4" stroke="currentColor" strokeWidth="2" strokeLinecap="round" />
                </svg>
            </button>

            {isDropdownOpen && (
                <div className={styles.dropdown}>
                    <Link
                        to="/profile"
                        className={styles.dropdownItem}
                        onClick={() => setIsDropdownOpen(false)}
                    >
                        Profile Settings
                    </Link>
                    <button
                        className={styles.dropdownItem}
                        onClick={async () => {
                            await signOut();
                            setIsDropdownOpen(false);
                        }}
                    >
                        Logout
                    </button>
                </div>
            )}
        </div>
    );
}

export default function NavbarAuth() {
    return (
        <BrowserOnly fallback={<div></div>}>
            {() => <NavbarAuthClient />}
        </BrowserOnly>
    );
}
