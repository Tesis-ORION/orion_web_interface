// tailwind.config.js
import typography from '@tailwindcss/typography';

export default {
  // Ajusta los paths a donde tienes tu código Astro/JSX
  content: ['./src/**/*.{astro,html,js,jsx,ts,tsx}'],
  theme: {
    extend: {},
  },
  plugins: [
    typography,
    // si usas otros plugins, añádelos aquí
  ],
};
