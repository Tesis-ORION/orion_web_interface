import React, { useEffect, useState } from 'react';
import { MdWbSunny, MdNightlightRound } from 'react-icons/md';

export default function ThemeSwitcher() {
  // Inicializa el estado desde el atributo, si existe (esto se ejecuta en el cliente)
  const [theme, setTheme] = useState(() => {
    return typeof document !== 'undefined'
      ? document.documentElement.dataset.theme || 'light'
      : 'light';
  });

  useEffect(() => {
    // Sincroniza el estado con el valor en localStorage al montar
    const storedTheme = localStorage.getItem("theme");
    if (storedTheme) {
      setTheme(storedTheme);
      document.documentElement.dataset.theme = storedTheme;
    } else {
      document.documentElement.dataset.theme = "light";
    }
  }, []);

  const themes = ["light", "dark"];
  const toggleTheme = () => {
    const currentTheme = document.documentElement.dataset.theme;
    const nextTheme = themes[(themes.indexOf(currentTheme) + 1) % themes.length];
    document.documentElement.dataset.theme = nextTheme;
    localStorage.setItem("theme", nextTheme);
    setTheme(nextTheme);
  };

  return (
    <button 
      id="theme-switcher" 
      type="button" 
      onClick={toggleTheme}
      className="ml-0 origin-[right_center] scale-100 transition-all duration-500"
    >
      {theme === "light" ? (
        <div id="icon-theme-light">
          <MdWbSunny size={24} />
          <span className="sr-only">Use light theme</span>
        </div>
      ) : (
        <div id="icon-theme-dark">
          <MdNightlightRound size={24} />
          <span className="sr-only">Use dark theme</span>
        </div>
      )}
    </button>
  );
}
