import React, { useState } from 'react';
import { motion } from 'framer-motion';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';

const FloatingCard = ({ children, delay = 0 }) => (
  <motion.div
    initial={{ opacity: 0, y: 30 }}
    whileInView={{ opacity: 1, y: 0 }}
    transition={{ delay, duration: 0.6 }}
    whileHover={{ y: -10, boxShadow: '0 20px 40px rgba(0,0,0,0.2)' }}
    viewport={{ once: true }}
  >
    {children}
  </motion.div>
);

const AnimatedCounter = ({ end, suffix = '' }) => {
  const [count, setCount] = React.useState(0);

  React.useEffect(() => {
    const increment = end / 30;
    const timer = setInterval(() => {
      setCount((prev) => {
        const newVal = prev + increment;
        return newVal > end ? end : newVal;
      });
    }, 30);
    return () => clearInterval(timer);
  }, [end]);

  return <span>{Math.floor(count)}{suffix}</span>;
};

export default function Home() {
  const context = useDocusaurusContext();
  const { siteConfig } = context;

  return (
    <Layout title="Physical AI & Humanoid Robotics" description="Learn to build autonomous humanoid robots with modern AI">
      {/* Hero Section */}
      <section className="hero-section" style={{
        background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        minHeight: '100vh',
        display: 'flex',
        alignItems: 'center',
        overflow: 'hidden',
        position: 'relative',
      }}>
        {/* Animated Background */}
        <motion.div
          animate={{ y: [0, -20, 0] }}
          transition={{ duration: 6, repeat: Infinity }}
          style={{
            position: 'absolute',
            top: 0,
            left: 0,
            right: 0,
            bottom: 0,
            opacity: 0.1,
            fontSize: '200px',
            fontWeight: 'bold',
            color: 'white',
            textAlign: 'center',
            letterSpacing: '2px',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
          }}
        >
          🤖
        </motion.div>

        <div style={{
          maxWidth: '1200px',
          margin: '0 auto',
          padding: '60px 40px',
          position: 'relative',
          zIndex: 2,
          textAlign: 'center',
          color: 'white',
          width: '100%',
        }}>
          <motion.div
            initial={{ opacity: 0, scale: 0.8 }}
            animate={{ opacity: 1, scale: 1 }}
            transition={{ duration: 0.8 }}
            style={{ marginBottom: '30px' }}
          >
            <h1 style={{
              fontSize: '4rem',
              fontWeight: 'bold',
              marginBottom: '20px',
              textShadow: '0 4px 15px rgba(0,0,0,0.3)',
            }}>
              Physical AI & Humanoid Robotics
            </h1>
          </motion.div>

          <motion.div
            initial={{ opacity: 0, y: 30 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.3, duration: 0.8 }}
            style={{ marginBottom: '30px' }}
          >
            <p style={{
              fontSize: '1.5rem',
              opacity: 0.95,
              marginBottom: '20px',
              lineHeight: '1.6',
            }}>
              Build autonomous humanoid robots that listen, understand, and act in the real world
            </p>
          </motion.div>

          <motion.div
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            transition={{ delay: 0.6, duration: 0.8 }}
            style={{ display: 'flex', gap: '20px', justifyContent: 'center', flexWrap: 'wrap' }}
          >
            <Link
              className="button button--primary button--lg"
              style={{
                background: 'white',
                color: '#667eea',
                fontWeight: 'bold',
                padding: '15px 40px',
                fontSize: '1.1rem',
                border: 'none',
              }}
              to="/docs/physical-ai/intro"
            >
              Start Learning 🚀
            </Link>
            <Link
              className="button button--outline button--lg"
              style={{
                color: 'white',
                borderColor: 'white',
                fontWeight: 'bold',
                padding: '15px 40px',
                fontSize: '1.1rem',
              }}
              to="#modules"
            >
              Explore Modules →
            </Link>
          </motion.div>
        </div>
      </section>

      {/* Stats Section */}
      <section style={{
        background: 'linear-gradient(180deg, #f8f9ff 0%, #e8ebff 100%)',
        padding: '80px 40px',
        textAlign: 'center',
      }}>
        <motion.div
          initial={{ opacity: 0 }}
          whileInView={{ opacity: 1 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
        >
          <h2 style={{ fontSize: '2.5rem', marginBottom: '60px', color: '#333' }}>
            By The Numbers
          </h2>
        </motion.div>

        <div style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fit, minmax(250px, 1fr))',
          gap: '40px',
          maxWidth: '1200px',
          margin: '0 auto',
        }}>
          {[
            { number: 8, label: 'Core Chapters', delay: 0 },
            { number: 30, label: 'Code Examples', delay: 0.1 },
            { number: 25, label: 'Thousand Words', delay: 0.2 },
            { number: 1, label: 'Capstone Project', delay: 0.3, suffix: '' },
          ].map((stat, idx) => (
            <FloatingCard key={idx} delay={stat.delay}>
              <div style={{
                padding: '40px',
                background: 'white',
                borderRadius: '15px',
                boxShadow: '0 10px 30px rgba(0,0,0,0.1)',
              }}>
                <div style={{
                  fontSize: '3.5rem',
                  fontWeight: 'bold',
                  color: '#667eea',
                  marginBottom: '15px',
                }}>
                  <AnimatedCounter end={stat.number} suffix={stat.suffix || '+'} />
                </div>
                <p style={{
                  fontSize: '1.2rem',
                  color: '#666',
                  margin: 0,
                }}>
                  {stat.label}
                </p>
              </div>
            </FloatingCard>
          ))}
        </div>
      </section>

      {/* Modules Section */}
      <section id="modules" style={{ padding: '80px 40px' }}>
        <motion.div
          initial={{ opacity: 0 }}
          whileInView={{ opacity: 1 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          style={{ textAlign: 'center', marginBottom: '60px' }}
        >
          <h2 style={{ fontSize: '2.5rem', color: '#333', marginBottom: '20px' }}>
            4 Core Modules
          </h2>
          <p style={{ fontSize: '1.1rem', color: '#666', maxWidth: '600px', margin: '0 auto' }}>
            Master the complete robotics stack from communication to AI perception
          </p>
        </motion.div>

        <div style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))',
          gap: '30px',
          maxWidth: '1200px',
          margin: '0 auto',
        }}>
          {[
            {
              icon: '🧠',
              title: 'ROS 2 & Architecture',
              description: 'The robotic nervous system - learn nodes, topics, services, and communication patterns',
              delay: 0,
            },
            {
              icon: '🎮',
              title: 'Gazebo Simulation',
              description: 'Build digital twins with physics simulation, sensors, and realistic environments',
              delay: 0.1,
            },
            {
              icon: '🤖',
              title: 'NVIDIA Isaac AI',
              description: 'Advanced perception with computer vision, SLAM, and reinforcement learning',
              delay: 0.2,
            },
            {
              icon: '🗣️',
              title: 'Vision-Language-Action',
              description: 'Voice commands to robot actions using GPT-4, Whisper, and CLIP',
              delay: 0.3,
            },
          ].map((module, idx) => (
            <FloatingCard key={idx} delay={module.delay}>
              <div style={{
                padding: '40px',
                background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
                borderRadius: '15px',
                color: 'white',
                height: '100%',
                boxShadow: '0 10px 30px rgba(102, 126, 234, 0.3)',
              }}>
                <div style={{ fontSize: '3rem', marginBottom: '20px' }}>
                  {module.icon}
                </div>
                <h3 style={{ fontSize: '1.4rem', marginBottom: '15px', fontWeight: 'bold' }}>
                  {module.title}
                </h3>
                <p style={{ fontSize: '1rem', opacity: 0.95, lineHeight: '1.6', margin: 0 }}>
                  {module.description}
                </p>
              </div>
            </FloatingCard>
          ))}
        </div>
      </section>

      {/* Capstone Section */}
      <section style={{
        background: 'linear-gradient(135deg, #f093fb 0%, #f5576c 100%)',
        padding: '80px 40px',
        color: 'white',
      }}>
        <motion.div
          initial={{ opacity: 0, y: 30 }}
          whileInView={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          style={{ maxWidth: '800px', margin: '0 auto', textAlign: 'center' }}
        >
          <h2 style={{ fontSize: '2.5rem', marginBottom: '20px' }}>
            🎯 Capstone Project
          </h2>
          <p style={{ fontSize: '1.2rem', marginBottom: '30px', opacity: 0.95 }}>
            Build an autonomous humanoid robot that:
          </p>
          <ul style={{
            listStyle: 'none',
            padding: 0,
            display: 'grid',
            gridTemplateColumns: 'repeat(auto-fit, minmax(200px, 1fr))',
            gap: '20px',
            marginBottom: '40px',
            fontSize: '1.1rem',
          }}>
            {['🎤 Listens to voice', '💭 Understands language', '🤔 Plans actions', '⚙️ Executes tasks', '👁️ Verifies success'].map((item, idx) => (
              <motion.li
                key={idx}
                initial={{ opacity: 0, x: -20 }}
                whileInView={{ opacity: 1, x: 0 }}
                transition={{ delay: idx * 0.1 }}
                viewport={{ once: true }}
              >
                {item}
              </motion.li>
            ))}
          </ul>
          <Link
            className="button button--secondary button--lg"
            style={{
              background: 'white',
              color: '#f5576c',
              fontWeight: 'bold',
              padding: '15px 40px',
              fontSize: '1.1rem',
            }}
            to="/docs/physical-ai/capstone"
          >
            View Capstone Project →
          </Link>
        </motion.div>
      </section>

      {/* Features Section */}
      <section style={{ padding: '80px 40px', background: '#f8f9ff' }}>
        <motion.div
          initial={{ opacity: 0 }}
          whileInView={{ opacity: 1 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
          style={{ textAlign: 'center', marginBottom: '60px' }}
        >
          <h2 style={{ fontSize: '2.5rem', color: '#333', marginBottom: '20px' }}>
            What You'll Learn
          </h2>
        </motion.div>

        <div style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(auto-fit, minmax(280px, 1fr))',
          gap: '30px',
          maxWidth: '1200px',
          margin: '0 auto',
        }}>
          {[
            { icon: '🔧', title: 'Hands-On Coding', desc: '30+ complete Python examples ready to run' },
            { icon: '📚', title: 'Comprehensive Theory', desc: '25,000+ words of detailed explanations' },
            { icon: '🚀', title: 'Industry Standard', desc: 'Learn with ROS 2, Gazebo, NVIDIA Isaac' },
            { icon: '🎓', title: 'Progressive Learning', desc: 'From basics to advanced AI robotics' },
            { icon: '💻', title: 'Production Ready', desc: 'Real-world patterns and best practices' },
            { icon: '🌍', title: 'Global Access', desc: 'Deployed online, accessible anywhere' },
          ].map((feature, idx) => (
            <FloatingCard key={idx} delay={idx * 0.08}>
              <div style={{
                padding: '30px',
                background: 'white',
                borderRadius: '12px',
                textAlign: 'center',
                boxShadow: '0 5px 20px rgba(0,0,0,0.08)',
              }}>
                <div style={{ fontSize: '2.5rem', marginBottom: '15px' }}>
                  {feature.icon}
                </div>
                <h3 style={{ fontSize: '1.2rem', marginBottom: '10px', color: '#333' }}>
                  {feature.title}
                </h3>
                <p style={{ color: '#666', fontSize: '0.95rem', margin: 0 }}>
                  {feature.desc}
                </p>
              </div>
            </FloatingCard>
          ))}
        </div>
      </section>

      {/* CTA Section */}
      <section style={{
        background: 'linear-gradient(135deg, #667eea 0%, #764ba2 100%)',
        padding: '80px 40px',
        textAlign: 'center',
        color: 'white',
      }}>
        <motion.div
          initial={{ opacity: 0, scale: 0.9 }}
          whileInView={{ opacity: 1, scale: 1 }}
          transition={{ duration: 0.8 }}
          viewport={{ once: true }}
        >
          <h2 style={{ fontSize: '2.5rem', marginBottom: '20px' }}>
            Ready to Build the Future?
          </h2>
          <p style={{ fontSize: '1.2rem', marginBottom: '40px', opacity: 0.95 }}>
            Start your journey into Physical AI and autonomous robotics today
          </p>
          <div style={{ display: 'flex', gap: '20px', justifyContent: 'center', flexWrap: 'wrap' }}>
            <Link
              className="button button--lg"
              style={{
                background: 'white',
                color: '#667eea',
                fontWeight: 'bold',
                padding: '15px 40px',
              }}
              to="/docs/physical-ai/intro"
            >
              Start Learning Now
            </Link>
            <Link
              className="button button--outline button--lg"
              style={{
                color: 'white',
                borderColor: 'white',
                fontWeight: 'bold',
                padding: '15px 40px',
              }}
              to="/docs/physical-ai/intro"
            >
              View Resources
            </Link>
          </div>
        </motion.div>
      </section>
    </Layout>
  );
}
