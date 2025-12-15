import React, { useEffect } from 'react';
import { useLocation } from '@docusaurus/router';

const RTLHandler = () => {
  const { pathname } = useLocation();

  useEffect(() => {
    // Check if current locale is RTL (for example, Urdu)
    const isRTL = pathname.startsWith('/ur/');

    if (isRTL) {
      // Add RTL class to body for CSS targeting
      document.body.setAttribute('dir', 'rtl');
      document.documentElement.setAttribute('dir', 'rtl');

      // Create and append the RTL CSS link
      const link = document.createElement('link');
      link.rel = 'stylesheet';
      link.href = '/css/custom-rtl.css';
      link.type = 'text/css';
      link.id = 'rtl-stylesheet';

      // Check if the link already exists to avoid duplicates
      if (!document.getElementById('rtl-stylesheet')) {
        document.head.appendChild(link);
      }

      return () => {
        // Clean up: remove the link when component unmounts
        const existingLink = document.getElementById('rtl-stylesheet');
        if (existingLink) {
          document.head.removeChild(existingLink);
        }
      };
    } else {
      // Remove RTL attributes and stylesheet if not RTL locale
      document.body.removeAttribute('dir');
      document.documentElement.removeAttribute('dir');

      const existingLink = document.getElementById('rtl-stylesheet');
      if (existingLink) {
        document.head.removeChild(existingLink);
      }
    }
  }, [pathname]);

  return null; // This component doesn't render anything
};

export default RTLHandler;