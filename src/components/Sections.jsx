import React from 'react';
import Link from '@docusaurus/Link';

export default function Sections() {
    return (
        <>

            <section style={{ padding: 'var(--space-12) 0' }}>
                <div className="container">
                    <h3 className="text-2xl font-bold">What You Will Learn</h3>
                    <div className="section-grid mt-6">
                        <div className="card">
                            <h3>Core Concepts</h3>
                            <p>Physical AI principles and system design patterns</p>
                            <div className="mt-6 inline-flex gap-4 flex-wrap">
                                <Link className="btn-ghost" to="/docs/chapter-01-physical-ai">Read Chapter</Link>
                            </div>
                        </div>
                        <div className="card">
                            <h3>ROS 2</h3>
                            <p>Architecture, QoS, distributed systems, and best practices</p>
                            <div className="mt-6 inline-flex gap-4 flex-wrap">
                                <Link className="btn-ghost" to="/docs/chapter-02-ros2">Start Module</Link>
                            </div>
                        </div>
                        <div className="card">
                            <h3>Simulation</h3>
                            <p>Gazebo workflows, physics, sensors, and domain randomization</p>
                            <div className="mt-6 inline-flex gap-4 flex-wrap">
                                <Link className="btn-ghost" to="/docs/chapter-03-simulation/gazebo">See Gazebo</Link>
                            </div>
                        </div>
                    </div>
                    <div className="section-grid mt-6">
                        <div className="card">
                            <h3>NVIDIA Isaac</h3>
                            <p>Omniverse‑based robotics simulation and synthetic data</p>
                            <div className="mt-6 inline-flex gap-4 flex-wrap">
                                <Link className="btn-ghost" to="/docs/chapter-04-nvidia-isaac/isaac-sim">Open Isaac</Link>
                            </div>
                        </div>
                        <div className="card">
                            <h3>Vision‑Language‑Action</h3>
                            <p>Bridging perception, language, and action for generalization</p>
                            <div className="mt-6 inline-flex gap-4 flex-wrap">
                                <Link className="btn-ghost" to="/docs/chapter-05-vla">Learn VLA</Link>
                            </div>
                        </div>
                        <div className="card">
                            <h3>Humanoid Robotics</h3>
                            <p>Locomotion, manipulation, and control for human‑scale robots</p>
                            <div className="mt-6 inline-flex gap-4 flex-wrap">
                                <Link className="btn-ghost" to="/docs/chapter-06-humanoid">Visit Chapter</Link>
                            </div>
                        </div>
                    </div>
                </div>
            </section>

            <section style={{ padding: 'var(--space-12) 0' }}>
                <div className="container">
                    <h3 className="text-2xl font-bold">Your Learning Path</h3>
                    <div className="learning-path mt-6">
                        <div className="learning-step">
                            <div className="learning-step-title">1 · Intro</div>
                            <div className="learning-step-desc">Start with fundamentals</div>
                        </div>
                        <span className="learning-arrow">→</span>
                        <div className="learning-step">
                            <div className="learning-step-title">2 · ROS 2</div>
                            <div className="learning-step-desc">Set up and build nodes</div>
                        </div>
                        <span className="learning-arrow">→</span>
                        <div className="learning-step">
                            <div className="learning-step-title">3 · Simulation</div>
                            <div className="learning-step-desc">Test in Gazebo/Isaac</div>
                        </div>
                        <span className="learning-arrow">→</span>
                        <div className="learning-step">
                            <div className="learning-step-title">4 · VLA</div>
                            <div className="learning-step-desc">Integrate perception and language</div>
                        </div>
                        <span className="learning-arrow">→</span>
                        <div className="learning-step">
                            <div className="learning-step-title">5 · Humanoids</div>
                            <div className="learning-step-desc">Control locomotion and manipulation</div>
                        </div>
                        <span className="learning-arrow">→</span>
                        <div className="learning-step">
                            <div className="learning-step-title">6 · Deploy</div>
                            <div className="learning-step-desc">Edge compute on Jetson</div>
                        </div>
                    </div>
                </div>
            </section>

            <section style={{ padding: 'var(--space-12) 0' }}>
                <div className="container">
                    <div className="grid" style={{ gap: 'var(--space-8)' }}>
                        <div className="glass-card p-8 rounded-2xl">
                            <div className="text-xl font-bold">Featured Chapters</div>
                            <div className="grid-responsive-3 mt-6">
                                <Link className="resource-card" to="/docs/chapter-02-ros2/core-architecture">
                                    <div className="resource-title">ROS 2 · Core Architecture</div>
                                    <div className="resource-desc">Distributed nodes, QoS, discovery, performance</div>
                                </Link>
                                <Link className="resource-card" to="/docs/chapter-04-nvidia-isaac/isaac-sim">
                                    <div className="resource-title">NVIDIA Isaac Sim</div>
                                    <div className="resource-desc">PhysX, Omniverse, synthetic data pipelines</div>
                                </Link>
                                <Link className="resource-card" to="/docs/chapter-07-hardware/edge-computing">
                                    <div className="resource-title">Edge Computing</div>
                                    <div className="resource-desc">Jetson platform for onboard AI and control</div>
                                </Link>
                            </div>
                        </div>

                    </div>
                </div>
            </section>

            <section className="bg-gradient-primary" style={{ padding: 'var(--space-16) 0' }}>
                <div className="container text-center">
                    <h3 className="text-3xl font-bold text-white">Build the Next Generation of Physical AI & Robots</h3>
                    <p className="mt-6 max-w-2xl mx-auto text-white">
                        Learn step‑by‑step with practical guidance, modern tooling, and real‑world patterns.
                    </p>
                    <div className="mt-12 inline-flex gap-4 flex-wrap justify-center">
                        <Link className="btn-primary" to="/docs/intro">Start Now</Link>
                        <Link className="btn-ghost" to="/docs/chapter-01-physical-ai" style={{ color: '#ffffff', borderColor: 'rgba(255,255,255,0.5)' }}>Browse Chapters</Link>
                    </div>
                </div>
            </section>
        </>
    );
}
