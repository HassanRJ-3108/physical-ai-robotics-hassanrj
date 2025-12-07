import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import clsx from 'clsx';

export default function Hero() {
    const { siteConfig } = useDocusaurusContext();

    return (
        <header className="hero-premium">
            <div className="container">
                <h1 className="hero-title">
                    Build the Next Generation of <br />
                    <span style={{ color: 'var(--ifm-color-primary)' }}>Physical AI & Robots</span>
                </h1>
                <p className="hero-subtitle">
                    The comprehensive guide to Embodied Intelligence, ROS 2, Simulation, <br />
                    NVIDIA Isaac, VLA Models, and Humanoid Robotics.
                </p>
                <div className="hero-actions">
                    <Link
                        className="btn-primary"
                        to="/docs/intro">
                        Start Reading
                    </Link>
                    <Link
                        className="btn-ghost"
                        to="/docs/chapter-01-physical-ai">
                        Browse Chapters
                    </Link>
                </div>
            </div>
        </header>
    );
}
