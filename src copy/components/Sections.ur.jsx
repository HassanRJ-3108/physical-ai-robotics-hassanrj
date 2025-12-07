import React from 'react';
import Link from '@docusaurus/Link';

export default function Sections() {
    return (
        <>
            {/* What You Will Learn */}
            <section>
                <h3>آپ کیا سیکھیں گے</h3>
                <div className="features-grid">
                    <div className="feature-card">
                        <div className="feature-icon">🤖</div>
                        <h4>ROS 2 بنیادی باتیں</h4>
                        <p>نوڈز، ٹاپکس، سروسز، اور ایکشنز کو سمجھیں</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">🌐</div>
                        <h4>سمیولیشن کی مہارت</h4>
                        <p>Gazebo، Isaac Sim، اور MuJoCo میں مہارت حاصل کریں</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">🧠</div>
                        <h4>VLA ماڈلز</h4>
                        <p>Vision-Language-Action AI کو روبوٹکس میں شامل کریں</p>
                    </div>
                    <div className="feature-card">
                        <div className="feature-icon">🦾</div>
                        <h4>ہیومنائیڈ روبوٹکس</h4>
                        <p>دو پیروں والے روبوٹس ڈیزائن اور کنٹرول کریں</p>
                    </div>
                </div>
            </section>

            {/* Your Learning Path */}
            <section className="learning-path">
                <h3>آپ کا سیکھنے کا راستہ</h3>
                <div className="path-timeline">
                    <div className="path-step">
                        <span className="step-number">1</span>
                        <h4>بنیادی باتیں سیکھیں</h4>
                        <p>ROS 2 اور Python/C++ کی بنیادیں</p>
                    </div>
                    <div className="path-step">
                        <span className="step-number">2</span>
                        <h4>سمیولیشن میں تجربہ</h4>
                        <p>ورچوئل ماحول میں محفوظ طریقے سے تجربہ کریں</p>
                    </div>
                    <div className="path-step">
                        <span className="step-number">3</span>
                        <h4>AI کو شامل کریں</h4>
                        <p>اپنے روبوٹس میں انٹیلیجنس شامل کریں</p>
                    </div>
                    <div className="path-step">
                        <span className="step-number">4</span>
                        <h4>حقیقی دنیا میں تعینات کریں</h4>
                        <p>اپنی تخلیق کو حقیقت میں لائیں</p>
                    </div>
                </div>
            </section>

            {/* Featured Chapters */}
            <section className="featured-chapters">
                <h3>نمایاں ابواب</h3>
                <div className="chapters-grid">
                    <Link to="/docs/chapter-01-physical-ai" className="chapter-card">
                        <div className="chapter-number">باب 1</div>
                        <h4>فزیکل AI کا تعارف</h4>
                        <p>ایمبوڈیڈ AI اور روبوٹکس کی بنیادیں سمجھیں</p>
                    </Link>
                    <Link to="/docs/chapter-02-ros2" className="chapter-card">
                        <div className="chapter-number">باب 2</div>
                        <h4>ROS 2 Essentials</h4>
                        <p>جدید روبوٹکس فریم ورک میں مہارت حاصل کریں</p>
                    </Link>
                    <Link to="/docs/chapter-03-simulation" className="chapter-card">
                        <div className="chapter-number">باب 3</div>
                        <h4>سمیولیشن ماحول</h4>
                        <p>Gazebo، Isaac اور MuJoCo سیکھیں</p>
                    </Link>
                </div>
            </section>

            {/* CTA Section */}
            <section className="cta-section">
                <h3>آج ہی اپنا سفر شروع کریں</h3>
                <p>عملی رہنمائی، جدید ٹولز، اور حقیقی دنیا کے نمونوں کے ساتھ قدم بہ قدم سیکھیں۔</p>
                <div className="cta-actions">
                    <Link className="btn-primary" to="/docs/intro">
                        ابھی شروع کریں
                    </Link>
                    <Link className="btn-secondary" to="/docs/chapter-01-physical-ai">
                        ابواب دیکھیں
                    </Link>
                </div>
            </section>
        </>
    );
}
