import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

export default function Hero() {
    const { siteConfig } = useDocusaurusContext();
    return (
        <header className="hero-premium">
            <div className="container">
                <h1 className="hero-title">
                    اگلی نسل کی فزیکل AI اور روبوٹس بنائیں
                </h1>
                <p className="hero-subtitle">
                    ایمبوڈیڈ انٹیلیجنس، ROS 2، سمیولیشن، NVIDIA Isaac، VLA ماڈلز، اور ہیومنائیڈ روبوٹکس کے لیے جامع گائیڈ۔
                </p>
                <div className="hero-actions">
                    <Link className="btn-primary" to="/docs/intro">
                        پڑھنا شروع کریں
                    </Link>
                    <Link className="btn-secondary" to="/docs/chapter-01-physical-ai">
                        ابواب دیکھیں
                    </Link>
                </div>
            </div>
        </header>
    );
}
