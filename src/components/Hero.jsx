import React from 'react';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Translate from '@docusaurus/Translate';
import Link from '@docusaurus/Link';
import styles from './Hero.module.css';

export default function Hero() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <header className={styles.heroBanner}>
            <div className="container">
                <h1 className={styles.heroTitle}>
                    <Translate id="homepage.hero.title">
                        Physical AI & Humanoid Robotics
                    </Translate>
                </h1>
                <p className={styles.heroSubtitle}>
                    <Translate id="homepage.hero.subtitle">
                        Master the future of embodied intelligence with ROS 2, NVIDIA Isaac, VLA models, and cutting-edge robotics
                    </Translate>
                </p>
                <div className={styles.heroButtons}>
                    <Link
                        className="btn-primary"
                        to="/docs/intro">
                        <Translate id="homepage.hero.getStarted">
                            Get Started
                        </Translate> 🚀
                    </Link>
                    <Link
                        className="btn-ghost"
                        to="/docs/chapter-01-physical-ai">
                        <Translate id="homepage.hero.learnMore">
                            Learn More
                        </Translate>
                    </Link>
                </div>
            </div>
        </header>
    );
}
