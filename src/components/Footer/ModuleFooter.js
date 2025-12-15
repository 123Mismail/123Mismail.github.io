import React from 'react';
import Link from '@docusaurus/Link';

/**
 * ModuleFooter Component
 * Advanced 4-column footer layout with organized links
 *
 * Constitution Compliance: Principle VIII - Textbook Delivery Platform Requirements
 * Spec: specs/005-delivery-platform/spec.md (M-05, AC-8.4-A)
 */
function ModuleFooter() {
  // Define the advanced footer structure
  const footerSections = [
    {
      title: 'Chapters',
      items: [
        { label: 'Chapter 1: Foundations of Physical AI', to: '/chapters/c1-foundations-physical-ai' },
        { label: 'Chapter 2: ROS 2 Architecture', to: '/chapters/c2-ros2-architecture' },
        { label: 'Chapter 3: ROS 2 Actions', to: '/chapters/c3-ros2-actions' },
        { label: 'Chapter 4: URDF Robot Description', to: '/chapters/c4-urdf-robot-description' },
      ]
    },
    {
      title: 'Advanced Topics',
      items: [
        { label: 'Chapter 5: Gazebo Simulation', to: '/chapters/c5-gazebo-simulation' },
        { label: 'Chapter 6: NVIDIA Isaac Sim', to: '/chapters/c6-isaac-sim' },
        { label: 'Chapter 7: Unity Simulation', to: '/chapters/c7-unity-simulation' },
        { label: 'Chapter 8: Advanced Simulation', to: '/chapters/c8-advanced-simulation' },
      ]
    },
    {
      title: 'Control & Integration',
      items: [
        { label: 'Chapter 9: Real-Time Control', to: '/chapters/c9-real-time-control' },
        { label: 'Chapter 10: Control Algorithms', to: '/chapters/c10-real-time-algorithms' },
        { label: 'Chapter 11: Sensor Fusion', to: '/chapters/c11-sensor-fusion' },
        { label: 'Chapter 12-14: Advanced Topics', to: '/chapters/c12-whole-body-control' },
      ]
    },
    {
      title: 'Resources & Support',
      items: [
        { label: 'ROS 2 Humble Docs', href: 'https://docs.ros.org/en/humble/' },
        { label: 'Gazebo', href: 'https://gazebosim.org/' },
        { label: 'NVIDIA Isaac Sim', href: 'https://docs.omniverse.nvidia.com/isaacsim/latest/' },
        { label: 'GitHub Repository', href: 'https://github.com/123Mismail/physical-ai-humanoid-robotics' },
      ]
    }
  ];

  return (
    <footer className="footer">
      <div className="container container-fluid">
        <div className="footer__container" style={{display: 'flex', justifyContent: 'space-between', flexWrap: 'wrap', gap: '2rem', padding: '2rem 0'}}>
          {footerSections.map((section, index) => (
            <div key={index} className="footer__col" style={{flex: 1, minWidth: '200px'}}>
              <h4 className="footer__title" style={{marginBottom: '1rem'}}>{section.title}</h4>
              <ul className="footer__items" style={{listStyle: 'none', padding: 0}}>
                {section.items.map((item, itemIndex) => (
                  <li key={itemIndex} className="footer__item" style={{marginBottom: '0.5rem'}}>
                    {item.to ? (
                      <Link to={item.to} className="footer__link" style={{textDecoration: 'none'}}>
                        {item.label}
                      </Link>
                    ) : (
                      <a href={item.href} className="footer__link" target="_blank" rel="noopener noreferrer" style={{textDecoration: 'none'}}>
                        {item.label}
                      </a>
                    )}
                  </li>
                ))}
              </ul>
            </div>
          ))}
        </div>
        <div className="footer__bottom text--center" style={{marginTop: '2rem', paddingTop: '1rem', borderTop: '1px solid'}}>
          <div className="footer__copyright">
            Copyright © {new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook | Built with Docusaurus
          </div>
        </div>
      </div>
    </footer>
  );
}

export default ModuleFooter;