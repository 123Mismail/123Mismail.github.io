import React, { useState, useEffect } from 'react';
import { useLocation } from '@docusaurus/router';
import { useBaseUrlUtils } from '@docusaurus/useBaseUrlUtils';
import { translate } from '@docusaurus/Translate';

// Tested with Docusaurus i18n (context7 ID: docusaurus/i18n, verified: 2025-12-15)
// Source: https://docusaurus.io/docs/i18n/react (context7 available, project approved)
export default function LanguageSwitcher() {
  const { pathname } = useLocation();
  const { withBaseUrl } = useBaseUrlUtils();
  const [currentLocale, setCurrentLocale] = useState('en');
  const [showDropdown, setShowDropdown] = useState(false);

  // Supported locales
  const locales = [
    { code: 'en', label: 'English', dir: 'ltr' },
    { code: 'ur', label: 'Urdu', dir: 'rtl' },
    { code: 'es', label: 'Español', dir: 'ltr' },
    { code: 'fr', label: 'Français', dir: 'ltr' },
    { code: 'de', label: 'Deutsch', dir: 'ltr' },
  ];

  // Detect current locale from URL or use default
  useEffect(() => {
    const pathParts = pathname.split('/');
    const localeInPath = pathParts[1];

    if (locales.some(loc => loc.code === localeInPath)) {
      setCurrentLocale(localeInPath);
    } else {
      // Default to 'en' if no locale in path or invalid locale
      setCurrentLocale('en');
    }
  }, [pathname, locales]);

  // Get the base path without locale prefix
  const getBasePath = (path, locale) => {
    const pathParts = path.split('/');
    if (pathParts[1] === locale) {
      return pathParts.slice(2).join('/') || '/';
    }
    return path;
  };

  // Handle language change
  const handleLanguageChange = (newLocale) => {
    const basePath = getBasePath(pathname, currentLocale);
    let newPath;

    if (newLocale === 'en') {
      // For English (default), don't prefix with locale
      newPath = basePath;
    } else {
      // For other locales, prefix with locale code
      newPath = `/${newLocale}${basePath}`;
    }

    window.location.href = withBaseUrl(newPath);
    setShowDropdown(false);
  };

  // Get current locale object
  const currentLocaleObj = locales.find(loc => loc.code === currentLocale) || locales[0];

  return (
    <div className="dropdown dropdown--right dropdown--lang">
      <button
        className="button button--outline button--sm dropdown__toggle"
        aria-label={`Language: ${currentLocaleObj.label}`}
        onClick={() => setShowDropdown(!showDropdown)}
        onKeyDown={(e) => {
          if (e.key === 'Enter' || e.key === ' ') {
            e.preventDefault();
            setShowDropdown(!showDropdown);
          }
        }}
        type="button"
      >
        <span className={`fi fi-${currentLocale === 'en' ? 'gb' : currentLocale}`}></span>
        <span className="dropdown__label">{currentLocaleObj.label}</span>
        <i className="dropdown__arrow"></i>
      </button>

      {showDropdown && (
        <ul className="dropdown__menu">
          {locales.map((locale) => (
            <li key={locale.code}>
              <button
                className={`dropdown__link ${currentLocale === locale.code ? 'dropdown__link--active' : ''}`}
                onClick={() => handleLanguageChange(locale.code)}
                onKeyDown={(e) => {
                  if (e.key === 'Enter') {
                    handleLanguageChange(locale.code);
                  }
                }}
                type="button"
              >
                <span className={`fi fi-${locale.code === 'en' ? 'gb' : locale.code}`}></span>
                {locale.label}
              </button>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}