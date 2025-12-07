import React from 'react';
import clsx from 'clsx';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Hero from '@site/src/components/Hero';
import Sections from '@site/src/components/Sections';

import styles from './index.module.css';

export default function Home() {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={`${siteConfig.title} · Physical AI & Robotics`}
      description="Premium Docusaurus book platform for Physical AI & Humanoid Robotics">
      <Hero />
      <main>
        <section style={{ padding: 'var(--space-12) 0' }}>
          <div className="container">
            <Sections />
          </div>
        </section>
      </main>
    </Layout>
  );
}

